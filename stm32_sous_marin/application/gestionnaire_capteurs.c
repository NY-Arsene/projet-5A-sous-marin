#include "gestionnaire_capteurs.h"
#include <string.h>
#include <math.h>

#define MAG_MAX_AGE_MS 200u     /* au-delà, le magnétomètre n'est plus utilisé pour le cap */

void sensors_default_config(sensors_config_t *cfg)
{
    memset(cfg, 0, sizeof *cfg);
    cfg->baro_model = MS5837_02BA;
    cfg->baro_osr = MS5837_OSR_2048;
    cfg->rho_kg_m3 = DEPTH_RHO_FRESH_WATER;
    cfg->imu_addr = LSM6DSOX_I2C_ADDR_LOW;
    cfg->mag_addr = LIS3MDL_I2C_ADDR_LOW;
    cfg->ina_addr = INA226_I2C_ADDR;
    cfg->shunt_ohm = 0.010f;
    cfg->imu_axis_map[0] = 1; cfg->imu_axis_map[1] = 2; cfg->imu_axis_map[2] = 3;
    cfg->mag_axis_map[0] = 1; cfg->mag_axis_map[1] = 2; cfg->mag_axis_map[2] = 3;
    cfg->batt_cells = 4;                    /* batterie 4S (14 à 16,8 V) : avertissement à 14,0 V, critique à 13,2 V */
    cfg->v_warn_cell = 3.50f;
    cfg->v_crit_cell = 3.30f;
    cfg->leak_thr_on = 2000;
    cfg->leak_thr_off = 1500;
    cfg->leak_active_high = true;
    cfg->limit_active_low = true;
    cfg->max_depth_m = 2.5f;
    cfg->timeout_ms[SENSOR_BARO] = 300;
    cfg->timeout_ms[SENSOR_IMU] = 200;
    cfg->timeout_ms[SENSOR_MAG] = 500;
    cfg->timeout_ms[SENSOR_POWER] = 1500;
    cfg->timeout_ms[SENSOR_TPROBE] = 5000;
    cfg->imu_period_ms = 10;
    cfg->mag_period_ms = 20;
    cfg->power_period_ms = 100;
    cfg->tprobe_period_ms = 100;
    cfg->digital_period_ms = 10;
    cfg->retry_period_ms = 2000;
}

static bool try_init(sensor_manager_t *m, sensor_id_t id, uint32_t now)
{
    const sensors_config_t *c = &m->cfg;
    sens_status_t st = SENS_ERR_PARAM;
    switch (id) {
    case SENSOR_BARO:   st = ms5837_init(&m->baro, c->baro_model, c->baro_osr); break;
    case SENSOR_IMU:    st = lsm6dsox_init(&m->imu, c->imu_addr, LSM6_XL_2G, LSM6_G_500DPS, LSM6_ODR_104HZ); break;
    case SENSOR_MAG:    st = lis3mdl_init(&m->mag, c->mag_addr, LIS3_FS_4G, LIS3_ODR_80HZ); break;
    case SENSOR_POWER:  st = ina226_init(&m->power, c->ina_addr, c->shunt_ohm, INA226_AVG_16); break;
    case SENSOR_TPROBE: st = ds18b20_init(&m->tprobe); break;
    default: break;
    }
    m->present[id] = (st == SENS_OK);
    if (m->present[id]) {
        m->last_progress_ms[id] = now;
        if (id == SENSOR_MAG) {
            m->mag_fresh = false;
        }
    }
    return m->present[id];
}

uint32_t sensors_init(sensor_manager_t *m, const sensors_config_t *cfg, uint32_t now_ms)
{
    memset(m, 0, sizeof *m);
    m->cfg = *cfg;

    depth_init(&m->depth, cfg->rho_kg_m3, 0.3f, 0.1f);
    attitude_init(&m->att, 0.02f, 0.01f);
    mag_cal_init(&m->mag_cal);
    m->mag_cal.n = 0;
    battery_init(&m->batt, cfg->batt_cells, cfg->v_warn_cell, cfg->v_crit_cell);

    uint32_t failed = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (!try_init(m, (sensor_id_t)i, now_ms)) {
            failed |= (1u << i);
        }
        m->last_run_ms[i] = now_ms - 1000u;           /* première exécution immédiate */
    }
    if (leak_init(&m->leak, PORT_ADC_LEAK, cfg->leak_thr_on, cfg->leak_thr_off, 3, cfg->leak_active_high) != SENS_OK) {
        failed |= SENSORS_INIT_LEAK_FAILED;
    }
    for (int i = 0; i < 4; i++) {
        bool raw = port_gpio_read((port_pin_t)i);
        bool active = cfg->limit_active_low ? !raw : raw;
        debounce_init(&m->limit[i], active, 20);
    }
    m->last_digital_ms = now_ms - 1000u;
    m->last_retry_ms = now_ms;
    m->last_att_ms = now_ms;
    m->data.timestamp_ms = now_ms;
    return failed;
}

static bool due(uint32_t now, uint32_t *last, uint32_t period)
{
    if ((uint32_t)(now - *last) >= period) {
        *last = now;
        return true;
    }
    return false;
}

static void mark_ok(sensor_manager_t *m, sensor_id_t id, uint32_t now)
{
    m->ever_ok[id] = true;
    m->last_ok_ms[id] = now;
    m->last_progress_ms[id] = now;
}

sens_status_t sensors_calibrate_gyro(sensor_manager_t *m, uint16_t n_samples)
{
    if (!m->present[SENSOR_IMU]) {
        return SENS_ERR_BUS;
    }
    sens_status_t st = lsm6dsox_calibrate_gyro(&m->imu, n_samples, 2000);
    if (st == SENS_OK) {
        attitude_init(&m->att, m->att.k_acc, m->att.k_mag);   /* repart d'une assiette propre */
    }
    return st;
}

void sensors_tare_depth(sensor_manager_t *m, uint16_t n_samples)
{
    depth_tare_start(&m->depth, n_samples);
}

void sensors_set_mag_cal(sensor_manager_t *m, const mag_cal_t *cal)
{
    m->mag_cal = *cal;
}

void sensors_clear_leak(sensor_manager_t *m)
{
    leak_clear(&m->leak);
    /* Réévaluation IMMÉDIATE : si le capteur est encore mouillé, l'alarme revient tout de suite (sans attendre le prochain
     * passage de sensors_poll), sinon l'appelant verrait un état "sec" fugitif puis une nouvelle alarme. */
    for (uint8_t i = 0; i < m->leak.confirm_n; i++) {
        m->data.leak = leak_poll(&m->leak);
    }
    if (m->data.leak) {
        m->data.failsafe_mask |= FS_LEAK;
    } else {
        m->data.failsafe_mask &= ~FS_LEAK;
    }
}

const sensor_data_t *sensors_data(const sensor_manager_t *m)
{
    return &m->data;
}

static void poll_baro(sensor_manager_t *m, uint32_t now)
{
    sens_status_t st = ms5837_update(&m->baro, now);
    if (st == SENS_OK) {
        bool depth_ok = depth_update(&m->depth, m->baro.pressure_mbar, now);
        m->data.pressure_mbar = m->baro.pressure_mbar;
        m->data.water_temp_c = m->baro.temperature_c;
        if (depth_ok) {
            m->data.depth_m = m->depth.depth_m;
            m->data.depth_rate_m_s = m->depth.rate_m_s;
        }
        m->data.depth_ref_valid = m->depth.ref_valid;
        mark_ok(m, SENSOR_BARO, now);
    }
}

static void poll_imu(sensor_manager_t *m, uint32_t now)
{
    lsm6dsox_data_t d;
    if (lsm6dsox_read(&m->imu, &d) != SENS_OK) {
        return;
    }
    float acc[3], gyr[3];
    attitude_remap_axes(d.acc_g, m->cfg.imu_axis_map, acc);
    attitude_remap_axes(d.gyr_dps, m->cfg.imu_axis_map, gyr);

    float dt = (float)(now - m->last_att_ms) / 1000.0f;
    m->last_att_ms = now;
    if (dt > 0.1f) {
        dt = 0.1f;                         /* après un trou de mesures, on n'intègre pas un long dt */
    }
    bool mag_ok = m->present[SENSOR_MAG] && m->mag_fresh &&
                  (uint32_t)(now - m->last_ok_ms[SENSOR_MAG]) <= MAG_MAX_AGE_MS;
    attitude_update(&m->att, acc, gyr, mag_ok ? m->mag_body : NULL, dt);

    for (int i = 0; i < 3; i++) {
        m->data.acc_g[i] = acc[i];
        m->data.gyro_dps[i] = gyr[i];
    }
    m->data.roll_deg = m->att.roll_deg;
    m->data.pitch_deg = m->att.pitch_deg;
    m->data.yaw_deg = m->att.yaw_deg;
    mark_ok(m, SENSOR_IMU, now);
}

static void poll_mag(sensor_manager_t *m, uint32_t now)
{
    lis3mdl_data_t d;
    if (lis3mdl_read(&m->mag, &d) != SENS_OK) {
        return;
    }
    float cal[3];
    mag_cal_apply(&m->mag_cal, d.gauss, cal);                    /* calibration dans le repère du capteur */
    attitude_remap_axes(cal, m->cfg.mag_axis_map, m->mag_body);  /* puis passage au repère sous-marin     */
    for (int i = 0; i < 3; i++) {
        m->data.mag_gauss[i] = m->mag_body[i];
    }
    m->mag_fresh = true;
    mark_ok(m, SENSOR_MAG, now);
}

static void poll_power(sensor_manager_t *m, uint32_t now)
{
    ina226_data_t d;
    if (ina226_read(&m->power, &d) != SENS_OK) {
        return;
    }
    m->data.vbat_v = d.bus_v;
    m->data.ibat_a = d.current_a;
    m->data.pbat_w = d.power_w;
    m->data.batt_level = battery_update(&m->batt, d.bus_v);
    m->data.soc_pct = m->batt.soc_pct;
    mark_ok(m, SENSOR_POWER, now);
}

static void poll_tprobe(sensor_manager_t *m, uint32_t now)
{
    if (ds18b20_update(&m->tprobe, now) == SENS_OK) {
        m->data.hull_temp_c = m->tprobe.temp_c;
        mark_ok(m, SENSOR_TPROBE, now);
    }
}

void sensors_poll(sensor_manager_t *m, uint32_t now)
{
    const sensors_config_t *c = &m->cfg;

    /* chien de garde : un capteur présent mais muet trop longtemps est considéré perdu (coupure d'alimentation
     * brève => configuration effacée, câble intermittent...) et sera ré-initialisé complètement. */
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (m->present[i] && (uint32_t)(now - m->last_progress_ms[i]) > c->timeout_ms[i] + c->retry_period_ms) {
            m->present[i] = false;
        }
    }
    /* tentative périodique de ré-initialisation des capteurs absents */
    if (due(now, &m->last_retry_ms, c->retry_period_ms)) {
        for (int i = 0; i < SENSOR_COUNT; i++) {
            if (!m->present[i]) {
                try_init(m, (sensor_id_t)i, now);
            }
        }
    }

    if (m->present[SENSOR_BARO]) {
        poll_baro(m, now);                       /* machine à états : appelé à chaque tour */
    }
    if (m->present[SENSOR_IMU] && due(now, &m->last_run_ms[SENSOR_IMU], c->imu_period_ms)) {
        poll_imu(m, now);
    }
    if (m->present[SENSOR_MAG] && due(now, &m->last_run_ms[SENSOR_MAG], c->mag_period_ms)) {
        poll_mag(m, now);
    }
    if (m->present[SENSOR_POWER] && due(now, &m->last_run_ms[SENSOR_POWER], c->power_period_ms)) {
        poll_power(m, now);
    }
    if (m->present[SENSOR_TPROBE] && due(now, &m->last_run_ms[SENSOR_TPROBE], c->tprobe_period_ms)) {
        poll_tprobe(m, now);
    }
    if (due(now, &m->last_digital_ms, c->digital_period_ms)) {
        m->data.leak = leak_poll(&m->leak);
        for (int i = 0; i < 4; i++) {
            bool raw = port_gpio_read((port_pin_t)i);
            m->data.limit[i] = debounce_update(&m->limit[i], c->limit_active_low ? !raw : raw, now);
        }
    }

    /* santé : un capteur est "sain" s'il est présent et a fourni une mesure récemment */
    uint32_t healthy = 0;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (m->present[i] && m->ever_ok[i] && (uint32_t)(now - m->last_ok_ms[i]) <= c->timeout_ms[i]) {
            healthy |= (1u << i);
        }
    }
    m->data.healthy_mask = healthy;

    /* sécurités */
    uint32_t fs = 0;
    if (m->data.leak)                                  fs |= FS_LEAK;
    if (m->data.batt_level == BATT_WARNING)            fs |= FS_BATT_WARN;
    if (m->data.batt_level == BATT_CRITICAL)           fs |= FS_BATT_CRIT;
    if (!(healthy & (1u << SENSOR_BARO)))              fs |= FS_DEPTH_LOST;
    if (!(healthy & (1u << SENSOR_IMU)))               fs |= FS_IMU_LOST;
    if ((healthy & (1u << SENSOR_BARO)) && m->data.depth_m > c->max_depth_m) fs |= FS_OVER_DEPTH;
    m->data.failsafe_mask = fs;
    m->data.timestamp_ms = now;
}
