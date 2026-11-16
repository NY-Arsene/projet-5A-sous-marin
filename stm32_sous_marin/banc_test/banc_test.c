#include "banc_test.h"
#include "materiel.h"
#include "gestionnaire_capteurs.h"
#include <stdio.h>
#include <math.h>

#ifndef BENCH_BARO_MODEL
#define BENCH_BARO_MODEL MS5837_02BA      /* MS5837_30BA si vous gardez le Bar30 */
#endif

static int g_fails;

#define CHECK(label, cond) do { \
    if (cond) { printf("  [PASS] %s\n", label); } \
    else      { printf("  [FAIL] %s\n", label); g_fails++; } } while (0)

typedef struct { double n, sum, sumsq, min, max; } stats_t;

static void st_init(stats_t *s) { s->n = 0; s->sum = 0; s->sumsq = 0; s->min = 1e30; s->max = -1e30; }
static void st_add(stats_t *s, double v)
{
    s->n++; s->sum += v; s->sumsq += v * v;
    if (v < s->min) s->min = v;
    if (v > s->max) s->max = v;
}
static double st_mean(const stats_t *s) { return s->n > 0 ? s->sum / s->n : 0.0; }
static double st_std(const stats_t *s)
{
    if (s->n < 2) return 0.0;
    double m = st_mean(s), v = s->sumsq / s->n - m * m;
    return v > 0 ? sqrt(v) : 0.0;
}

static void title(const char *t) { printf("\n=== %s ===\n", t); }

/* ----------------------------------------------------------------------------- scan I2C */
int bench_i2c_scan(void)
{
    title("Scan I2C");
    g_fails = 0;
    int found_imu = 0, found_mag = 0, found_ina = 0, found_baro = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (port_i2c_probe(a)) {
            printf("  adresse 0x%02X repond\n", a);
            if (a == 0x6A || a == 0x6B) found_imu = 1;
            if (a == 0x1C || a == 0x1E) found_mag = 1;
            if (a == 0x40) found_ina = 1;
            if (a == 0x76) found_baro = 1;
        }
    }
    CHECK("LSM6DSOX present (0x6A/0x6B)", found_imu);
    CHECK("LIS3MDL present (0x1C/0x1E)", found_mag);
    CHECK("INA226 present (0x40)", found_ina);
    CHECK("MS5837 present (0x76)", found_baro);
    return g_fails;
}

/* ----------------------------------------------------------------------------- MS5837 */
int bench_baro(void)
{
    title("MS5837 pression / temperature");
    g_fails = 0;
    ms5837_t s;
    sens_status_t st = ms5837_init(&s, BENCH_BARO_MODEL, MS5837_OSR_4096);
    printf("  init : %s\n", sens_status_str(st));
    CHECK("init + CRC de la PROM", st == SENS_OK);
    if (st != SENS_OK) {
        return g_fails;
    }
    stats_t p, t;
    st_init(&p); st_init(&t);
    uint32_t t0 = port_millis();
    int errors = 0;
    for (int i = 0; i < 50; i++) {
        ms5837_result_t r;
        if (ms5837_read_blocking(&s, &r) == SENS_OK) {
            st_add(&p, r.pressure_mbar);
            st_add(&t, r.temperature_c);
        } else {
            errors++;
        }
    }
    uint32_t dt = port_millis() - t0;
    printf("  pression : moy %.2f mbar, ecart-type %.3f mbar (= %.1f mm d'eau), min %.2f, max %.2f\n",
           st_mean(&p), st_std(&p), st_std(&p) * 10.23, p.min, p.max);
    printf("  temperature : moy %.2f C ; 50 mesures en %lu ms ; erreurs %d\n",
           st_mean(&t), (unsigned long)dt, errors);
    CHECK("aucune erreur de lecture", errors == 0);
    CHECK("pression atmospherique plausible (900-1100 mbar)", st_mean(&p) > 900.0 && st_mean(&p) < 1100.0);
    CHECK("bruit < 0.5 mbar (5 mm d'eau)", st_std(&p) < 0.5);
    CHECK("temperature plausible (0-50 C)", st_mean(&t) > 0.0 && st_mean(&t) < 50.0);
    return g_fails;
}

/* ----------------------------------------------------------------------------- LSM6DSOX */
static bool imu_sample(lsm6dsox_t *s, lsm6dsox_data_t *d)
{
    uint32_t t0 = port_millis();
    while ((uint32_t)(port_millis() - t0) < 100u) {
        sens_status_t st = lsm6dsox_read(s, d);
        if (st == SENS_OK) return true;
        if (st != SENS_ERR_NOT_READY) return false;
        port_delay_ms(1);
    }
    return false;
}

int bench_imu(void)
{
    title("LSM6DSOX accelerometre / gyroscope (laisser le module IMMOBILE et a plat)");
    g_fails = 0;
    lsm6dsox_t s;
    sens_status_t st = lsm6dsox_init(&s, LSM6DSOX_I2C_ADDR_LOW, LSM6_XL_2G, LSM6_G_500DPS, LSM6_ODR_104HZ);
    printf("  init : %s\n", sens_status_str(st));
    CHECK("init + WHO_AM_I = 0x6C", st == SENS_OK);
    if (st != SENS_OK) return g_fails;

    stats_t ax, ay, az, an, gx, gy, gz;
    st_init(&ax); st_init(&ay); st_init(&az); st_init(&an); st_init(&gx); st_init(&gy); st_init(&gz);
    int errors = 0;
    for (int i = 0; i < 100; i++) {
        lsm6dsox_data_t d;
        if (!imu_sample(&s, &d)) { errors++; continue; }
        st_add(&ax, d.acc_g[0]); st_add(&ay, d.acc_g[1]); st_add(&az, d.acc_g[2]);
        st_add(&an, sqrt((double)d.acc_g[0] * d.acc_g[0] + (double)d.acc_g[1] * d.acc_g[1] + (double)d.acc_g[2] * d.acc_g[2]));
        st_add(&gx, d.gyr_dps[0]); st_add(&gy, d.gyr_dps[1]); st_add(&gz, d.gyr_dps[2]);
    }
    printf("  accel moy (g) : %.3f %.3f %.3f  | norme %.3f g\n", st_mean(&ax), st_mean(&ay), st_mean(&az), st_mean(&an));
    printf("  gyro moy (dps) : %.2f %.2f %.2f  | ecart-type %.3f %.3f %.3f\n",
           st_mean(&gx), st_mean(&gy), st_mean(&gz), st_std(&gx), st_std(&gy), st_std(&gz));
    CHECK("aucune erreur de lecture", errors == 0);
    CHECK("norme de l'acceleration = 1 g +-0.05", fabs(st_mean(&an) - 1.0) < 0.05);
    CHECK("bruit accelerometre < 0.02 g", st_std(&ax) < 0.02 && st_std(&ay) < 0.02 && st_std(&az) < 0.02);
    CHECK("biais gyro < 5 dps", fabs(st_mean(&gx)) < 5 && fabs(st_mean(&gy)) < 5 && fabs(st_mean(&gz)) < 5);
    CHECK("bruit gyro < 0.5 dps", st_std(&gx) < 0.5 && st_std(&gy) < 0.5 && st_std(&gz) < 0.5);

    st = lsm6dsox_calibrate_gyro(&s, 200, 3000);
    printf("  calibration gyro : %s ; biais = %.3f %.3f %.3f dps\n", sens_status_str(st),
           s.gyro_bias_dps[0], s.gyro_bias_dps[1], s.gyro_bias_dps[2]);
    CHECK("calibration du biais gyro (module immobile)", st == SENS_OK);
    return g_fails;
}

/* ----------------------------------------------------------------------------- LIS3MDL */
int bench_mag(void)
{
    title("LIS3MDL magnetometre (loin de tout moteur ou aimant)");
    g_fails = 0;
    lis3mdl_t s;
    sens_status_t st = lis3mdl_init(&s, LIS3MDL_I2C_ADDR_LOW, LIS3_FS_4G, LIS3_ODR_80HZ);
    printf("  init : %s\n", sens_status_str(st));
    CHECK("init + WHO_AM_I = 0x3D", st == SENS_OK);
    if (st != SENS_OK) return g_fails;

    stats_t n;
    st_init(&n);
    int errors = 0;
    lis3mdl_data_t d = { {0}, {0, 0, 0} };
    for (int i = 0; i < 100; i++) {
        uint32_t t0 = port_millis();
        sens_status_t r = SENS_ERR_NOT_READY;
        while (r == SENS_ERR_NOT_READY && (uint32_t)(port_millis() - t0) < 100u) {
            r = lis3mdl_read(&s, &d);
            if (r == SENS_ERR_NOT_READY) port_delay_ms(1);
        }
        if (r != SENS_OK) { errors++; continue; }
        st_add(&n, sqrt((double)d.gauss[0] * d.gauss[0] + (double)d.gauss[1] * d.gauss[1] + (double)d.gauss[2] * d.gauss[2]));
    }
    printf("  derniere mesure : %.3f %.3f %.3f gauss ; norme moy %.3f G (champ terrestre : 0.25 a 0.65 G)\n",
           d.gauss[0], d.gauss[1], d.gauss[2], st_mean(&n));
    CHECK("aucune erreur de lecture", errors == 0);
    CHECK("norme du champ plausible (0.15-0.80 G)", st_mean(&n) > 0.15 && st_mean(&n) < 0.80);
    CHECK("bruit sur la norme < 0.02 G", st_std(&n) < 0.02);
    return g_fails;
}

/* ----------------------------------------------------------------------------- INA226 */
int bench_power(void)
{
    title("INA226 batterie");
    g_fails = 0;
    ina226_t s;
    sens_status_t st = ina226_init(&s, INA226_I2C_ADDR, 0.010f, INA226_AVG_16);
    printf("  init : %s\n", sens_status_str(st));
    CHECK("init + identifiants TI (0x5449 / 0x2260)", st == SENS_OK);
    if (st != SENS_OK) return g_fails;
    port_delay_ms(20);                          /* premier cycle de moyenne */
    stats_t v, i;
    st_init(&v); st_init(&i);
    int errors = 0;
    ina226_data_t d = { 0, 0, 0, 0, false };
    for (int k = 0; k < 20; k++) {
        if (ina226_read(&s, &d) != SENS_OK) { errors++; continue; }
        st_add(&v, d.bus_v); st_add(&i, d.current_a);
        port_delay_ms(5);
    }
    printf("  tension %.3f V ; courant %.3f A ; puissance %.2f W ; saturation : %s\n",
           st_mean(&v), st_mean(&i), d.power_w, d.saturated ? "OUI" : "non");
    CHECK("aucune erreur de lecture", errors == 0);
    CHECK("tension batterie plausible (5-17,5 V)", st_mean(&v) > 5.0 && st_mean(&v) < 17.5);
    CHECK("courant plausible (|I| < 8 A)", fabs(st_mean(&i)) < 8.0 && !d.saturated);
    return g_fails;
}

/* ----------------------------------------------------------------------------- DS18B20 */
int bench_tprobe(void)
{
    title("DS18B20 sonde de temperature");
    g_fails = 0;
    ds18b20_t s;
    sens_status_t st = ds18b20_init(&s);
    CHECK("sonde presente (impulsion de presence 1-Wire)", st == SENS_OK);
    if (st != SENS_OK) return g_fails;
    sens_status_t r = SENS_ERR_NOT_READY;
    uint32_t t0 = port_millis();
    while ((uint32_t)(port_millis() - t0) < 3000u) {
        r = ds18b20_update(&s, port_millis());
        if (r != SENS_ERR_NOT_READY) break;
        port_delay_ms(10);
    }
    printf("  lecture : %s ; %.2f C ; erreurs %lu\n", sens_status_str(r), s.temp_c, (unsigned long)s.error_count);
    CHECK("lecture valide (CRC correct) en moins de 3 s", r == SENS_OK);
    CHECK("temperature plausible (-10 a 60 C)", r == SENS_OK && s.temp_c > -10.0f && s.temp_c < 60.0f);
    return g_fails;
}

/* ----------------------------------------------------------------------------- fuite */
int bench_leak(void)
{
    title("Detecteur de fuite (electrodes SECHES)");
    g_fails = 0;
    leak_t l;
    sensors_config_t cfg;
    sensors_default_config(&cfg);
    CHECK("configuration des seuils valide",
          leak_init(&l, PORT_ADC_LEAK, cfg.leak_thr_on, cfg.leak_thr_off, 3, cfg.leak_active_high) == SENS_OK);
    uint16_t mn = 0xFFFF, mx = 0;
    bool alarm = false;
    for (int i = 0; i < 50; i++) {
        alarm = leak_poll(&l);
        if (l.last_raw < mn) mn = l.last_raw;
        if (l.last_raw > mx) mx = l.last_raw;
        port_delay_ms(5);
    }
    printf("  ADC a sec : min %u, max %u (seuil d'alarme %u)\n", mn, mx, cfg.leak_thr_on);
    CHECK("pas de fausse alarme a sec", !alarm);
    CHECK("marge de bruit confortable (valeur max < 75 % du seuil)", mx < (cfg.leak_thr_on * 3u) / 4u);
    return g_fails;
}

int bench_leak_wet_test(uint32_t timeout_ms)
{
    title("Detecteur de fuite : MOUILLEZ les electrodes maintenant");
    g_fails = 0;
    leak_t l;
    sensors_config_t cfg;
    sensors_default_config(&cfg);
    leak_init(&l, PORT_ADC_LEAK, cfg.leak_thr_on, cfg.leak_thr_off, 3, cfg.leak_active_high);
    uint32_t t0 = port_millis();
    bool alarm = false;
    while (!alarm && (uint32_t)(port_millis() - t0) < timeout_ms) {
        alarm = leak_poll(&l);
        port_delay_ms(10);
    }
    printf("  derniere valeur ADC : %u\n", l.last_raw);
    CHECK("alarme declenchee au contact de l'eau", alarm);
    return g_fails;
}

/* ----------------------------------------------------------------------------- intégration */
int bench_manager(uint32_t duration_ms)
{
    title("Integration : gestionnaire de capteurs");
    g_fails = 0;
    sensor_manager_t m;
    sensors_config_t cfg;
    sensors_default_config(&cfg);
    cfg.baro_model = BENCH_BARO_MODEL;
    uint32_t failed = sensors_init(&m, &cfg, port_millis());
    printf("  capteurs en echec a l'init (masque) : 0x%lX\n", (unsigned long)failed);
    CHECK("tous les capteurs initialises", failed == 0);
    uint32_t t0 = port_millis();
    while ((uint32_t)(port_millis() - t0) < duration_ms) {
        sensors_poll(&m, port_millis());
        port_delay_ms(1);
    }
    const sensor_data_t *d = sensors_data(&m);
    printf("  P %.1f mbar | profondeur %.3f m | T eau %.1f C | T coque %.1f C\n",
           d->pressure_mbar, d->depth_m, d->water_temp_c, d->hull_temp_c);
    printf("  roulis %.1f | tangage %.1f | cap %.1f deg | batterie %.2f V %.2f A (%.0f %%)\n",
           d->roll_deg, d->pitch_deg, d->yaw_deg, d->vbat_v, d->ibat_a, d->soc_pct);
    printf("  sains : 0x%lX | securites : 0x%lX\n", (unsigned long)d->healthy_mask, (unsigned long)d->failsafe_mask);
    CHECK("tous les capteurs sains", d->healthy_mask == ((1u << SENSOR_COUNT) - 1u));
    CHECK("aucune securite active", d->failsafe_mask == 0);
    return g_fails;
}

int bench_run_all(void)
{
    int total = 0;
    total += bench_i2c_scan();
    total += bench_baro();
    total += bench_imu();
    total += bench_mag();
    total += bench_power();
    total += bench_tprobe();
    total += bench_leak();
    total += bench_manager(2000);
    printf("\n==== BILAN : %s (%d critere(s) en echec) ====\n", total == 0 ? "TOUT EST BON" : "ECHEC", total);
    return total;
}

/* ----------------------------------------------------------------------------- interactifs */
void bench_limits(uint32_t duration_ms)
{
    title("Fins de course (appuyez dessus)");
    bool prev[PORT_PIN_COUNT];
    for (int i = 0; i < PORT_PIN_COUNT; i++) prev[i] = port_gpio_read((port_pin_t)i);
    uint32_t t0 = port_millis();
    while ((uint32_t)(port_millis() - t0) < duration_ms) {
        for (int i = 0; i < PORT_PIN_COUNT; i++) {
            bool v = port_gpio_read((port_pin_t)i);
            if (v != prev[i]) {
                printf("  fin de course %d : %s\n", i, v ? "relache (niveau haut)" : "ENFONCE (niveau bas)");
                prev[i] = v;
            }
        }
        port_delay_ms(5);
    }
}

void bench_attitude_live(uint32_t duration_ms)
{
    title("Assiette en direct (inclinez : nez en haut => tangage +, tribord en bas => roulis +)");
    sensor_manager_t m;
    sensors_config_t cfg;
    sensors_default_config(&cfg);
    sensors_init(&m, &cfg, port_millis());
    sensors_calibrate_gyro(&m, 100);
    uint32_t t0 = port_millis(), last = 0;
    while ((uint32_t)(port_millis() - t0) < duration_ms) {
        uint32_t now = port_millis();
        sensors_poll(&m, now);
        if ((uint32_t)(now - last) >= 250u) {
            last = now;
            const sensor_data_t *d = sensors_data(&m);
            printf("  roulis %7.1f | tangage %7.1f | cap %6.1f\n", d->roll_deg, d->pitch_deg, d->yaw_deg);
        }
        port_delay_ms(1);
    }
}

int bench_mag_calibration(uint32_t duration_ms)
{
    title("Calibration magnetometre : tournez le module dans TOUTES les orientations");
    g_fails = 0;
    lis3mdl_t s;
    if (lis3mdl_init(&s, LIS3MDL_I2C_ADDR_LOW, LIS3_FS_4G, LIS3_ODR_80HZ) != SENS_OK) {
        printf("  init impossible\n");
        return 1;
    }
    mag_cal_t cal;
    mag_cal_init(&cal);
    uint32_t t0 = port_millis();
    while ((uint32_t)(port_millis() - t0) < duration_ms) {
        lis3mdl_data_t d;
        if (lis3mdl_read(&s, &d) == SENS_OK) {
            mag_cal_feed(&cal, d.gauss);
        }
        port_delay_ms(5);
    }
    sens_status_t st = mag_cal_finish(&cal, 0.3f);
    printf("  %lu echantillons ; resultat : %s\n", (unsigned long)cal.n, sens_status_str(st));
    CHECK("couverture suffisante des orientations", st == SENS_OK);
    if (st == SENS_OK) {
        printf("  offset (G) : %.3f %.3f %.3f ; echelle : %.3f %.3f %.3f\n",
               cal.offset[0], cal.offset[1], cal.offset[2], cal.scale[0], cal.scale[1], cal.scale[2]);
        printf("  => a copier dans le firmware (sensors_set_mag_cal)\n");
    }
    return g_fails;
}

void bench_depth_demo(void)
{
    title("Profondeur : tare a la surface, puis plongez le capteur de 20 cm");
    ms5837_t s;
    if (ms5837_init(&s, BENCH_BARO_MODEL, MS5837_OSR_4096) != SENS_OK) {
        printf("  init MS5837 impossible\n");
        return;
    }
    depth_t d;
    depth_init(&d, DEPTH_RHO_FRESH_WATER, 0.3f, 0.1f);
    depth_tare_start(&d, 30);
    ms5837_result_t r;
    while (depth_is_taring(&d)) {
        if (ms5837_read_blocking(&s, &r) == SENS_OK) depth_update(&d, r.pressure_mbar, port_millis());
    }
    printf("  reference : %.2f mbar. Plongez le capteur de 20 cm puis appuyez sur une touche (ou attendez 10 s)...\n", d.p_ref_mbar);
    (void)port_console_getc(10000);
    for (int i = 0; i < 40; i++) {
        if (ms5837_read_blocking(&s, &r) == SENS_OK) depth_update(&d, r.pressure_mbar, port_millis());
    }
    printf("  profondeur mesuree : %.3f m (attendu : ~0.20 m a la regle)\n", d.depth_m);
}

void bench_menu(void)
{
    for (;;) {
        printf("\n--- BANC DE TEST CAPTEURS ---\n"
               " s scan I2C | p pression | i IMU | m magneto | e batterie | t temperature | l fuite (sec)\n"
               " w fuite (mouillee) | f fins de course | o assiette live | c calibration magneto\n"
               " d profondeur | g gestionnaire | a TOUT | q quitter\n> ");
        int c = port_console_getc(60000);
        if (c < 0 || c == 'q') { printf("\nfin du banc de test\n"); return; }
        switch (c) {
        case 's': bench_i2c_scan(); break;
        case 'p': bench_baro(); break;
        case 'i': bench_imu(); break;
        case 'm': bench_mag(); break;
        case 'e': bench_power(); break;
        case 't': bench_tprobe(); break;
        case 'l': bench_leak(); break;
        case 'w': bench_leak_wet_test(15000); break;
        case 'f': bench_limits(15000); break;
        case 'o': bench_attitude_live(20000); break;
        case 'c': bench_mag_calibration(25000); break;
        case 'd': bench_depth_demo(); break;
        case 'g': bench_manager(3000); break;
        case 'a': bench_run_all(); break;
        default: break;
        }
    }
}
