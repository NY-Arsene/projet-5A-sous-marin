/* Tests d'intégration du gestionnaire de capteurs : sous-marin simulé complet, pannes, sécurités, robustesse. */
#include "mini_tests.h"
#include "gestionnaire_capteurs.h"
#include "faux_composants.h"
#include <math.h>
#include <string.h>
#include "filtres.h"

#define MBAR_PER_M (997.0f * 9.80665f / 100.0f)
#define ALL_SENSORS ((1u << SENSOR_COUNT) - 1u)

static fake_sub_t s_sub;
static sensor_manager_t s_m;
static sensors_config_t s_cfg;
static uint32_t s_failed;

static void set_level_north(void)
{
    const float acc[3] = { 0, 0, -1 }, gyr[3] = { 0, 0, 0 }, mag[3] = { 0.22f, 0.0f, 0.42f };
    fake_imu_set(&s_sub.imu, acc, gyr);
    fake_mag_set(&s_sub.mag, mag);
}

static void rig_setup(void)
{
    fake_sub_setup(&s_sub);
    sensors_default_config(&s_cfg);
    s_failed = sensors_init(&s_m, &s_cfg, fake_time_get());
    set_level_north();                                    /* après init : la pleine échelle est configurée */
}

static void run_ms(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        fake_time_advance(1);
        sensors_poll(&s_m, fake_time_get());
    }
}

static void set_depth(float meters)
{
    fake_ms5837_set_pressure(&s_sub.baro, 1013.0f + meters * MBAR_PER_M);
}

static void nominal(void)
{
    rig_setup();
    sensors_tare_depth(&s_m, 20);
    run_ms(600);
}

/* ------------------------------------------------------------------ fonctionnement nominal */
static void test_init_all_sensors_ok(void)
{
    rig_setup();
    MU_ASSERT_INT(0, s_failed);
    for (int i = 0; i < SENSOR_COUNT; i++) MU_ASSERT(s_m.present[i]);
}

static void test_nominal_run_produces_consistent_data_and_no_failsafe(void)
{
    nominal();
    MU_ASSERT(sensors_data(&s_m)->depth_ref_valid);
    set_depth(1.5f);
    run_ms(1500);
    const sensor_data_t *d = sensors_data(&s_m);
    MU_ASSERT_NEAR(1.5, d->depth_m, 0.03);
    MU_ASSERT_NEAR(0.0, d->depth_rate_m_s, 0.05);
    MU_ASSERT_NEAR(1013.0 + 1.5 * MBAR_PER_M, d->pressure_mbar, 0.3);
    MU_ASSERT_NEAR(20.0, d->water_temp_c, 0.1);
    MU_ASSERT_NEAR(21.5, d->hull_temp_c, 0.1);
    MU_ASSERT_NEAR(16.4, d->vbat_v, 0.01);
    MU_ASSERT_NEAR(0.5, d->ibat_a, 0.01);
    MU_ASSERT(d->soc_pct > 80.0f);
    MU_ASSERT_NEAR(0.0, d->roll_deg, 0.5);
    MU_ASSERT_NEAR(0.0, d->pitch_deg, 0.5);
    MU_ASSERT_NEAR(0.0, fabsf(wrap180(d->yaw_deg)), 2.0);
    MU_ASSERT_NEAR(-1.0, d->acc_g[2], 0.002);
    MU_ASSERT_INT(ALL_SENSORS, d->healthy_mask);
    MU_ASSERT_INT(0, d->failsafe_mask);
    MU_ASSERT(!d->leak);
    for (int i = 0; i < 4; i++) MU_ASSERT(!d->limit[i]);
}

static void test_failsafe_flags_before_first_data_then_clear(void)
{
    rig_setup();
    sensors_poll(&s_m, fake_time_get());
    MU_ASSERT(sensors_data(&s_m)->failsafe_mask & FS_DEPTH_LOST);     /* pas encore de mesure : prudence */
    run_ms(2500);
    MU_ASSERT_INT(0, sensors_data(&s_m)->failsafe_mask);
}

static void test_attitude_follows_tilted_hull(void)
{
    nominal();
    const float acc[3] = { 0.0f, -0.5f, -0.866f }, zero[3] = { 0, 0, 0 };   /* roulis de +30 deg : lecture (0, -sin30, -cos30) */
    fake_imu_set(&s_sub.imu, acc, zero);
    run_ms(2000);
    MU_ASSERT_NEAR(30.0, sensors_data(&s_m)->roll_deg, 1.5);
}

/* ------------------------------------------------------------------ sécurités */
static void test_leak_alarm_is_latched_until_cleared(void)
{
    nominal();
    fake_adc_value[PORT_ADC_LEAK] = 3000;
    run_ms(100);
    MU_ASSERT(sensors_data(&s_m)->failsafe_mask & FS_LEAK);
    fake_adc_value[PORT_ADC_LEAK] = 0;
    run_ms(100);
    MU_ASSERT(sensors_data(&s_m)->failsafe_mask & FS_LEAK);          /* mémorisée même si ça sèche */
    sensors_clear_leak(&s_m);
    run_ms(100);
    MU_ASSERT_INT(0, sensors_data(&s_m)->failsafe_mask & FS_LEAK);
}

static void test_clear_leak_is_effective_immediately_when_dry(void)
{
    nominal();
    fake_adc_value[PORT_ADC_LEAK] = 3000;
    run_ms(100);
    fake_adc_value[PORT_ADC_LEAK] = 0;
    run_ms(100);
    sensors_clear_leak(&s_m);
    MU_ASSERT_INT(0, sensors_data(&s_m)->failsafe_mask & FS_LEAK);      /* sans attendre un passage de sensors_poll */
    MU_ASSERT(!sensors_data(&s_m)->leak);
}

static void test_clear_leak_does_not_hide_a_leak_that_is_still_present(void)
{
    nominal();
    fake_adc_value[PORT_ADC_LEAK] = 3000;
    run_ms(100);
    sensors_clear_leak(&s_m);                                           /* l'opérateur acquitte alors que c'est encore mouillé */
    MU_ASSERT(sensors_data(&s_m)->failsafe_mask & FS_LEAK);             /* l'alarme n'a pas disparu, même un instant */
    MU_ASSERT(sensors_data(&s_m)->leak);
}

static void test_battery_warning_then_critical(void)
{
    nominal();
    fake_ina226_set(&s_sub.ina, 13.6f, 5.0f);                          /* 3,40 V par élément (4S) */
    run_ms(3000);
    uint32_t fs = sensors_data(&s_m)->failsafe_mask;
    MU_ASSERT(fs & FS_BATT_WARN);
    MU_ASSERT_INT(0, fs & FS_BATT_CRIT);
    fake_ina226_set(&s_sub.ina, 12.8f, 5.0f);                          /* 3,20 V par élément (4S) */
    run_ms(4000);
    fs = sensors_data(&s_m)->failsafe_mask;
    MU_ASSERT(fs & FS_BATT_CRIT);
    MU_ASSERT_INT(0, fs & FS_BATT_WARN);
}

static void test_over_depth_failsafe(void)
{
    nominal();
    set_depth(2.0f);
    run_ms(1000);
    MU_ASSERT_INT(0, sensors_data(&s_m)->failsafe_mask & FS_OVER_DEPTH);
    set_depth(3.0f);                                                   /* limite configurée : 2,5 m */
    run_ms(1000);
    MU_ASSERT(sensors_data(&s_m)->failsafe_mask & FS_OVER_DEPTH);
}

static void test_invalid_leak_configuration_fails_safe(void)
{
    fake_sub_setup(&s_sub);
    sensors_default_config(&s_cfg);
    s_cfg.leak_thr_on = 1000;                                          /* incohérent : actif haut exige on > off */
    s_cfg.leak_thr_off = 1500;
    s_failed = sensors_init(&s_m, &s_cfg, fake_time_get());
    MU_ASSERT(s_failed & SENSORS_INIT_LEAK_FAILED);
    set_level_north();
    run_ms(100);
    MU_ASSERT(sensors_data(&s_m)->failsafe_mask & FS_LEAK);            /* en cas de doute : alarme, jamais silence */
}

/* ------------------------------------------------------------------ pannes de capteurs */
static void test_baro_lost_sets_failsafe_and_recovers_automatically(void)
{
    nominal();
    run_ms(2000);
    fake_remove_i2c(MS5837_I2C_ADDR);
    run_ms(400);
    const sensor_data_t *d = sensors_data(&s_m);
    MU_ASSERT(d->failsafe_mask & FS_DEPTH_LOST);
    MU_ASSERT_INT(0, d->healthy_mask & (1u << SENSOR_BARO));
    fake_ms5837_setup(&s_sub.baro, true);                              /* le capteur est rebranché */
    run_ms(6000);                                                      /* chien de garde puis ré-initialisation */
    MU_ASSERT_INT(0, sensors_data(&s_m)->failsafe_mask & FS_DEPTH_LOST);
    MU_ASSERT(sensors_data(&s_m)->healthy_mask & (1u << SENSOR_BARO));
}

static void test_imu_lost_sets_failsafe(void)
{
    nominal();
    run_ms(500);
    fake_remove_i2c(0x6A);
    run_ms(300);
    MU_ASSERT(sensors_data(&s_m)->failsafe_mask & FS_IMU_LOST);
}

static float s_status_ready;
static void imu_status_hook(fake_regmap_t *m, uint8_t reg)
{
    if (reg == 0x1E) {                                                 /* ODR à 0 = composant en veille = jamais de donnée */
        m->regs[0x1E] = ((m->regs[0x10] & 0xF0) && (m->regs[0x11] & 0xF0)) ? 0x03 : 0x00;
    }
    (void)s_status_ready;
}

static void test_imu_brownout_config_loss_is_detected_and_repaired(void)
{
    nominal();
    s_sub.imu.on_read_reg = imu_status_hook;
    run_ms(500);
    MU_ASSERT(sensors_data(&s_m)->healthy_mask & (1u << SENSOR_IMU));
    s_sub.imu.regs[0x10] = 0; s_sub.imu.regs[0x11] = 0; s_sub.imu.regs[0x12] = 0;   /* coupure d'alimentation brève : registres remis à 0 */
    run_ms(400);
    MU_ASSERT(sensors_data(&s_m)->failsafe_mask & FS_IMU_LOST);        /* plus aucune donnée */
    run_ms(6000);                                                      /* chien de garde => ré-initialisation complète */
    set_level_north();
    run_ms(500);
    MU_ASSERT_INT(0, sensors_data(&s_m)->failsafe_mask & FS_IMU_LOST);
    MU_ASSERT_INT(0x40, s_sub.imu.regs[0x10]);                         /* configuration réécrite */
}

static void test_missing_magnetometer_at_boot_is_not_fatal(void)
{
    fake_sub_setup(&s_sub);
    fake_remove_i2c(0x1C);
    sensors_default_config(&s_cfg);
    s_failed = sensors_init(&s_m, &s_cfg, fake_time_get());
    MU_ASSERT_INT(1u << SENSOR_MAG, s_failed);
    set_level_north();
    run_ms(2500);
    const sensor_data_t *d = sensors_data(&s_m);
    MU_ASSERT_INT(ALL_SENSORS & ~(1u << SENSOR_MAG), d->healthy_mask);
    MU_ASSERT_INT(0, d->failsafe_mask);                                /* le cap seul est dégradé : pas de remontée d'urgence */
    MU_ASSERT(!s_m.att.yaw_from_mag);
}

static void test_magnetometer_dropout_attitude_keeps_integrating_gyro(void)
{
    nominal();
    run_ms(500);
    float start = sensors_data(&s_m)->yaw_deg;
    fake_remove_i2c(0x1C);
    const float acc[3] = { 0, 0, -1 }, gz[3] = { 0, 0, 30 };
    fake_imu_set(&s_sub.imu, acc, gz);                                 /* rotation de 30 deg/s en lacet */
    run_ms(1000);
    float yaw = sensors_data(&s_m)->yaw_deg;
    float diff = yaw - start;
    while (diff < -180.0f) diff += 360.0f;
    while (diff > 180.0f) diff -= 360.0f;
    MU_ASSERT_NEAR(30.0, diff, 3.0);
}

static void test_limit_switches_are_debounced(void)
{
    nominal();
    fake_gpio_level[PORT_PIN_LIMIT_FWD_TOP] = false;                   /* appui (actif bas) */
    run_ms(5);
    fake_gpio_level[PORT_PIN_LIMIT_FWD_TOP] = true;                    /* rebond court */
    run_ms(100);
    MU_ASSERT(!sensors_data(&s_m)->limit[0]);
    fake_gpio_level[PORT_PIN_LIMIT_AFT_BOTTOM] = false;
    run_ms(100);
    MU_ASSERT(sensors_data(&s_m)->limit[3]);
    MU_ASSERT(!sensors_data(&s_m)->limit[0]);
    fake_gpio_level[PORT_PIN_LIMIT_AFT_BOTTOM] = true;
    run_ms(100);
    MU_ASSERT(!sensors_data(&s_m)->limit[3]);
}

static void test_manager_gyro_calibration_removes_bias_from_output(void)
{
    nominal();
    const float acc[3] = { 0, 0, -1 }, bias[3] = { 1.5f, -0.8f, 0.4f };
    fake_imu_set(&s_sub.imu, acc, bias);
    run_ms(300);
    MU_ASSERT_NEAR(1.5, sensors_data(&s_m)->gyro_dps[0], 0.05);          /* avant : le biais est visible */
    MU_ASSERT_STATUS(SENS_OK, sensors_calibrate_gyro(&s_m, 100));
    run_ms(300);
    MU_ASSERT_NEAR(0.0, sensors_data(&s_m)->gyro_dps[0], 0.05);          /* après : au repos la sortie est nulle */
    MU_ASSERT_NEAR(0.0, sensors_data(&s_m)->gyro_dps[1], 0.05);
    MU_ASSERT_NEAR(0.0, sensors_data(&s_m)->gyro_dps[2], 0.05);
}

static void test_manager_gyro_calibration_without_imu_reports_error(void)
{
    fake_sub_setup(&s_sub);
    fake_remove_i2c(0x6A);
    sensors_default_config(&s_cfg);
    s_failed = sensors_init(&s_m, &s_cfg, fake_time_get());
    MU_ASSERT(s_failed & (1u << SENSOR_IMU));
    MU_ASSERT_STATUS(SENS_ERR_BUS, sensors_calibrate_gyro(&s_m, 100));
}

static void test_manager_applies_hard_iron_calibration_to_heading(void)
{
    nominal();
    const float biased[3] = { 0.22f + 0.15f, 0.08f, 0.42f - 0.10f };    /* champ nord + fer dur de (0,15 ; 0,08 ; -0,10) G */
    fake_mag_set(&s_sub.mag, biased);
    run_ms(2500);
    float err_before = fabsf(wrap180(sensors_data(&s_m)->yaw_deg));
    MU_ASSERT(err_before > 10.0f);                                       /* sans calibration le cap est franchement faux */
    mag_cal_t cal;
    mag_cal_init(&cal);
    cal.offset[0] = 0.15f; cal.offset[1] = 0.08f; cal.offset[2] = -0.10f;
    cal.valid = true;
    sensors_set_mag_cal(&s_m, &cal);
    run_ms(2500);
    MU_ASSERT_NEAR(0.0, fabsf(wrap180(sensors_data(&s_m)->yaw_deg)), 2.0);   /* avec : cap nord retrouvé */
}

static void test_manager_remaps_imu_axes(void)
{
    fake_sub_setup(&s_sub);
    sensors_default_config(&s_cfg);
    s_cfg.imu_axis_map[0] = 2; s_cfg.imu_axis_map[1] = -1; s_cfg.imu_axis_map[2] = 3;   /* module monté tourné de 90 deg */
    s_failed = sensors_init(&s_m, &s_cfg, fake_time_get());
    const float acc[3] = { 0.30f, -0.40f, -0.866f }, zero[3] = { 0, 0, 0 };   /* lecture brute dans le repère du capteur */
    fake_imu_set(&s_sub.imu, acc, zero);
    run_ms(200);
    const sensor_data_t *d = sensors_data(&s_m);
    MU_ASSERT_NEAR(-0.40, d->acc_g[0], 0.002);                           /* x_sous-marin = y_capteur  */
    MU_ASSERT_NEAR(-0.30, d->acc_g[1], 0.002);                           /* y_sous-marin = -x_capteur */
    MU_ASSERT_NEAR(-0.866, d->acc_g[2], 0.002);
}

static void test_manager_does_not_integrate_gyro_over_a_long_gap(void)
{
    nominal();
    run_ms(500);
    fake_remove_i2c(0x1C);                                             /* pas de magnétomètre : le cap vient du gyro seul */
    const float acc[3] = { 0, 0, -1 }, gz[3] = { 0, 0, 30 };
    fake_imu_set(&s_sub.imu, acc, gz);
    run_ms(200);                                                       /* laisse le magnétomètre devenir périmé */
    float before = sensors_data(&s_m)->yaw_deg;
    fake_time_advance(2000);                                           /* la boucle principale est bloquée 2 s (flash, ISR...) */
    sensors_poll(&s_m, fake_time_get());
    float diff = sensors_data(&s_m)->yaw_deg - before;
    while (diff < -180.0f) diff += 360.0f;
    while (diff > 180.0f) diff -= 360.0f;
    /* sans plafond du dt : 30 deg/s * 2 s = 60 deg ; avec plafond à 0,1 s : environ 3 deg */
    MU_ASSERT(diff < 8.0f);
}

/* ------------------------------------------------------------------ robustesse (fuzzing déterministe) */
static uint32_t s_rng = 12345;
static uint32_t rnd(void)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    return s_rng >> 8;
}

static bool data_is_finite(const sensor_data_t *d)
{
    float v[] = { d->depth_m, d->depth_rate_m_s, d->pressure_mbar, d->water_temp_c, d->hull_temp_c,
                  d->roll_deg, d->pitch_deg, d->yaw_deg, d->vbat_v, d->ibat_a, d->pbat_w, d->soc_pct,
                  d->acc_g[0], d->acc_g[1], d->acc_g[2], d->gyro_dps[0], d->gyro_dps[1], d->gyro_dps[2],
                  d->mag_gauss[0], d->mag_gauss[1], d->mag_gauss[2] };
    for (unsigned i = 0; i < sizeof v / sizeof v[0]; i++) {
        if (!isfinite(v[i])) return false;
    }
    return true;
}

static void test_fuzz_random_register_garbage_and_bus_failures_never_crash_nor_yield_nan(void)
{
    nominal();
    s_rng = 987654321u;
    int bad = 0;
    for (int ms = 0; ms < 30000; ms++) {
        if (rnd() % 10 == 0) {                                          /* données capteurs aléatoires */
            for (int r = 0x22; r <= 0x2D; r++) s_sub.imu.regs[r] = (uint8_t)rnd();
            for (int r = 0x28; r <= 0x2D; r++) s_sub.mag.regs[r] = (uint8_t)rnd();
            s_sub.ina.regs[0x01] = (uint16_t)rnd();
            s_sub.ina.regs[0x02] = (uint16_t)rnd();
            s_sub.baro.d1 = rnd() & 0xFFFFFF;
            s_sub.baro.d2 = rnd() & 0xFFFFFF;
            s_sub.temp.raw = (int16_t)rnd();
        }
        fake_adc_value[PORT_ADC_LEAK] = (uint16_t)(rnd() & 0xFFF);
        for (int p = 0; p < PORT_PIN_COUNT; p++) fake_gpio_level[p] = (rnd() & 1) != 0;
        fake_i2c_set_fail(rnd() % 200 == 0);                            /* pannes de bus ponctuelles */
        if (rnd() % 1000 == 0) s_sub.baro.nack_next = 2;
        if (rnd() % 3000 == 0) s_sub.baro.dead_adc = !s_sub.baro.dead_adc;
        fake_time_advance(1);
        sensors_poll(&s_m, fake_time_get());
        if (!data_is_finite(sensors_data(&s_m))) bad++;
    }
    MU_ASSERT_INT(0, bad);
}

int main(void)
{
    MU_RUN(test_init_all_sensors_ok);
    MU_RUN(test_nominal_run_produces_consistent_data_and_no_failsafe);
    MU_RUN(test_failsafe_flags_before_first_data_then_clear);
    MU_RUN(test_attitude_follows_tilted_hull);
    MU_RUN(test_leak_alarm_is_latched_until_cleared);
    MU_RUN(test_clear_leak_is_effective_immediately_when_dry);
    MU_RUN(test_clear_leak_does_not_hide_a_leak_that_is_still_present);
    MU_RUN(test_battery_warning_then_critical);
    MU_RUN(test_over_depth_failsafe);
    MU_RUN(test_invalid_leak_configuration_fails_safe);
    MU_RUN(test_baro_lost_sets_failsafe_and_recovers_automatically);
    MU_RUN(test_imu_lost_sets_failsafe);
    MU_RUN(test_imu_brownout_config_loss_is_detected_and_repaired);
    MU_RUN(test_missing_magnetometer_at_boot_is_not_fatal);
    MU_RUN(test_magnetometer_dropout_attitude_keeps_integrating_gyro);
    MU_RUN(test_limit_switches_are_debounced);
    MU_RUN(test_manager_gyro_calibration_removes_bias_from_output);
    MU_RUN(test_manager_gyro_calibration_without_imu_reports_error);
    MU_RUN(test_manager_applies_hard_iron_calibration_to_heading);
    MU_RUN(test_manager_remaps_imu_axes);
    MU_RUN(test_manager_does_not_integrate_gyro_over_a_long_gap);
    MU_RUN(test_fuzz_random_register_garbage_and_bus_failures_never_crash_nor_yield_nan);
    return MU_SUMMARY();
}
