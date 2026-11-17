/* Tests unitaires du driver MS5837 : calcul pur, CRC, initialisation, machine à états, pannes. */
#include "mini_tests.h"
#include "ms5837.h"
#include "faux_composants.h"
#include <string.h>

static const uint16_t C02[7] = { 0x0150, 46372, 43981, 29059, 27842, 31553, 28165 };
static const uint16_t C30[7] = { 0x0150, 34982, 36352, 20328, 22354, 26646, 26146 };

/* Appelle update() toutes les ms jusqu'à un résultat autre que NOT_READY. */
static sens_status_t run_until_result(ms5837_t *s, int max_ms)
{
    for (int i = 0; i < max_ms; i++) {
        sens_status_t st = ms5837_update(s, fake_time_get());
        if (st != SENS_ERR_NOT_READY) {
            return st;
        }
        fake_time_advance(1);
    }
    return SENS_ERR_TIMEOUT;
}

/* ------------------------------------------------------------------ calcul pur */
static void test_crc4_matches_independent_reference(void)
{
    /* valeurs calculées par une implémentation Python indépendante */
    MU_ASSERT_INT(7, ms5837_crc4(C02));
    MU_ASSERT_INT(13, ms5837_crc4(C30));
}

static void test_prom_valid_detects_every_single_bit_flip(void)
{
    uint16_t p[7];
    memcpy(p, C02, sizeof p);
    p[0] = (uint16_t)((p[0] & 0x0FFF) | (7u << 12));
    MU_ASSERT(ms5837_prom_valid(p));
    for (int w = 0; w < 7; w++) {
        for (int b = 0; b < 16; b++) {
            uint16_t q[7];
            memcpy(q, p, sizeof q);
            q[w] ^= (uint16_t)(1u << b);
            if (ms5837_prom_valid(q)) {
                printf("    un bit inverse (mot %d, bit %d) n'est pas detecte\n", w, b);
                mu_cur_failed = 1;
            }
            mu_checks++;
        }
    }
}

typedef struct { ms5837_model_t model; uint32_t d1, d2; int temp_centi; int p_units; } vec_t;

static void test_compute_against_reference_vectors(void)
{
    /* p_units : 0,01 mbar (02BA) ou 0,1 mbar (30BA) ; temp en 0,01 degC (valeurs de référence Python) */
    static const vec_t v[] = {
        { MS5837_02BA, 6239267, 8077568,  2000, 100000 },   /* 20 degC : branche haute temp.           */
        { MS5837_02BA, 7369880, 7630811,   438, 144845 },   /* 5 degC  : 2e ordre basse température    */
        { MS5837_02BA, 6691512, 8524325,  3499, 123944 },   /* 35 degC                                  */
        { MS5837_02BA, 6268663, 7184054, -1254,  94106 },   /* -10 degC (température négative)         */
        { MS5837_30BA, 4508439, 6821376,  2000,   9999 },
        { MS5837_30BA, 4733250, 6340120,   420,  24755 },
        { MS5837_30BA, 4538414, 7142213,  2998,  11898 },
        { MS5837_30BA, 4808186, 5538027, -2575,  28902 },   /* < -15 degC : branche "very low temp"    */
    };
    for (unsigned i = 0; i < sizeof v / sizeof v[0]; i++) {
        ms5837_result_t r;
        const uint16_t *c = (v[i].model == MS5837_02BA) ? C02 : C30;
        ms5837_compute(v[i].model, c, v[i].d1, v[i].d2, &r);
        double p_expected = (v[i].model == MS5837_02BA) ? v[i].p_units / 100.0 : v[i].p_units / 10.0;
        MU_ASSERT_NEAR(v[i].temp_centi / 100.0, r.temperature_c, 0.006);
        MU_ASSERT_NEAR(p_expected, r.pressure_mbar, 0.06);
    }
}

static void test_conv_times_are_increasing(void)
{
    uint32_t prev = 0;
    for (int o = MS5837_OSR_256; o <= MS5837_OSR_8192; o++) {
        uint32_t t = ms5837_conv_time_ms((ms5837_osr_t)o);
        MU_ASSERT(t > prev);
        prev = t;
    }
    MU_ASSERT(ms5837_conv_time_ms(MS5837_OSR_8192) >= 18);   /* 17,2 ms max selon la fiche */
}

/* ------------------------------------------------------------------ initialisation */
static void test_init_ok_reads_prom_and_resets_once(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, true);
    MU_ASSERT_STATUS(SENS_OK, ms5837_init(&s, MS5837_02BA, MS5837_OSR_2048));
    MU_ASSERT_INT(1, f.resets);
    MU_ASSERT_INT(46372, s.prom[1]);
    MU_ASSERT_INT(28165, s.prom[6]);
}

static void test_init_bus_failure(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, true);
    fake_i2c_set_fail(true);
    MU_ASSERT_STATUS(SENS_ERR_BUS, ms5837_init(&s, MS5837_02BA, MS5837_OSR_2048));
}

static void test_init_no_device_on_bus(void)
{
    ms5837_t s;
    MU_ASSERT_STATUS(SENS_ERR_BUS, ms5837_init(&s, MS5837_02BA, MS5837_OSR_2048));
}

static void test_init_rejects_idle_bus_all_ones_or_zeros(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, true);
    for (int i = 0; i < 7; i++) f.prom[i] = 0xFFFF;
    MU_ASSERT_STATUS(SENS_ERR_ID, ms5837_init(&s, MS5837_02BA, MS5837_OSR_2048));
    for (int i = 0; i < 7; i++) f.prom[i] = 0x0000;
    MU_ASSERT_STATUS(SENS_ERR_ID, ms5837_init(&s, MS5837_02BA, MS5837_OSR_2048));
}

static void test_init_detects_corrupted_prom(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, true);
    f.prom[3] ^= 0x0100;
    MU_ASSERT_STATUS(SENS_ERR_CRC, ms5837_init(&s, MS5837_02BA, MS5837_OSR_2048));
}

static void test_init_rejects_invalid_parameters(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, true);
    MU_ASSERT_STATUS(SENS_ERR_PARAM, ms5837_init(NULL, MS5837_02BA, MS5837_OSR_2048));
    MU_ASSERT_STATUS(SENS_ERR_PARAM, ms5837_init(&s, (ms5837_model_t)5, MS5837_OSR_2048));
    MU_ASSERT_STATUS(SENS_ERR_PARAM, ms5837_init(&s, MS5837_02BA, (ms5837_osr_t)9));
}

/* ------------------------------------------------------------------ machine à états */
static void test_update_returns_correct_pressure_and_temperature(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, true);
    fake_ms5837_set_pressure(&f, 1013.0f);
    MU_ASSERT_STATUS(SENS_OK, ms5837_init(&s, MS5837_02BA, MS5837_OSR_2048));
    MU_ASSERT_STATUS(SENS_OK, run_until_result(&s, 100));
    MU_ASSERT_NEAR(1013.0, s.pressure_mbar, 0.05);
    MU_ASSERT_NEAR(20.0, s.temperature_c, 0.05);
    MU_ASSERT_INT(1, s.sample_count);
    MU_ASSERT_INT(1, f.conv_d2_count);          /* la 1re mesure inclut la température */
}

static void test_update_does_not_touch_bus_while_converting(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, true);
    ms5837_init(&s, MS5837_02BA, MS5837_OSR_2048);          /* conversion de 5 ms */
    MU_ASSERT_STATUS(SENS_ERR_NOT_READY, ms5837_update(&s, fake_time_get()));   /* lance D1 */
    uint32_t n = fake_i2c_transactions;
    fake_time_advance(2);
    MU_ASSERT_STATUS(SENS_ERR_NOT_READY, ms5837_update(&s, fake_time_get()));
    MU_ASSERT_INT(n, fake_i2c_transactions);                 /* aucune transaction I2C pendant l'attente */
}

static void test_temperature_is_refreshed_only_every_n_samples(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, true);
    ms5837_init(&s, MS5837_02BA, MS5837_OSR_1024);
    int ok = 0;
    for (int i = 0; i < 4000 && ok < 40; i++) {
        if (ms5837_update(&s, fake_time_get()) == SENS_OK) ok++;
        fake_time_advance(1);
    }
    MU_ASSERT_INT(40, ok);
    MU_ASSERT_INT(5, f.conv_d2_count);          /* échantillons 1, 9, 17, 25, 33 */
    MU_ASSERT_INT(40, f.conv_d1_count);
}

static void test_update_tracks_pressure_and_temperature_changes(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, true);
    ms5837_init(&s, MS5837_02BA, MS5837_OSR_1024);
    s.temp_every = 1;                            /* température à chaque cycle pour ce test */
    fake_ms5837_set_pressure(&f, 1013.0f);
    run_until_result(&s, 100);
    fake_ms5837_set_pressure(&f, 1113.0f);
    fake_ms5837_set_temp(&f, 25.0f);
    for (int i = 0; i < 3; i++) run_until_result(&s, 100);
    MU_ASSERT_NEAR(1113.0, s.pressure_mbar, 0.5);
    MU_ASSERT_NEAR(25.0, s.temperature_c, 0.1);
}

static void test_works_with_30ba_model(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, false);
    fake_ms5837_set_pressure(&f, 1500.0f);
    MU_ASSERT_STATUS(SENS_OK, ms5837_init(&s, MS5837_30BA, MS5837_OSR_4096));
    MU_ASSERT_STATUS(SENS_OK, run_until_result(&s, 100));
    MU_ASSERT_NEAR(1500.0, s.pressure_mbar, 0.2);
}

/* ------------------------------------------------------------------ pannes */
static void test_dead_adc_leads_to_timeout_then_recovers_cycle(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, true);
    ms5837_init(&s, MS5837_02BA, MS5837_OSR_2048);
    f.dead_adc = true;
    MU_ASSERT_STATUS(SENS_ERR_TIMEOUT, run_until_result(&s, 500));
    MU_ASSERT(s.error_count >= 1);
    f.dead_adc = false;                          /* le capteur "revient" */
    MU_ASSERT_STATUS(SENS_OK, run_until_result(&s, 500));
}

static void test_bus_error_midrun_is_reported_and_recovers(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, true);
    ms5837_init(&s, MS5837_02BA, MS5837_OSR_2048);
    MU_ASSERT_STATUS(SENS_OK, run_until_result(&s, 100));
    fake_i2c_set_fail(true);
    MU_ASSERT_STATUS(SENS_ERR_BUS, run_until_result(&s, 100));
    fake_i2c_set_fail(false);
    MU_ASSERT_STATUS(SENS_OK, run_until_result(&s, 100));
    MU_ASSERT(s.error_count >= 1);
}

static void test_isolated_nack_does_not_corrupt_next_sample(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, true);
    fake_ms5837_set_pressure(&f, 1013.0f);
    ms5837_init(&s, MS5837_02BA, MS5837_OSR_2048);
    run_until_result(&s, 100);
    f.nack_next = 1;
    ms5837_update(&s, fake_time_get());                      /* démarre D1 -> NACK */
    sens_status_t st = run_until_result(&s, 100);
    MU_ASSERT_STATUS(SENS_OK, (st == SENS_ERR_BUS) ? run_until_result(&s, 100) : st);
    MU_ASSERT_NEAR(1013.0, s.pressure_mbar, 0.05);
}

static void test_out_of_range_pressure_is_rejected_and_not_published(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, true);
    fake_ms5837_set_pressure(&f, 1013.0f);
    ms5837_init(&s, MS5837_02BA, MS5837_OSR_1024);
    s.temp_every = 1;
    run_until_result(&s, 100);
    float good = s.pressure_mbar;
    fake_ms5837_set_pressure(&f, 5000.0f);                   /* hors plage du 02BA (2 bar) */
    MU_ASSERT_STATUS(SENS_ERR_RANGE, run_until_result(&s, 100));
    MU_ASSERT_NEAR(good, s.pressure_mbar, 0.0001);           /* ancienne valeur conservée */
}

static void test_read_blocking_ok_and_timeout(void)
{
    fake_ms5837_t f; ms5837_t s; ms5837_result_t r;
    fake_ms5837_setup(&f, true);
    ms5837_init(&s, MS5837_02BA, MS5837_OSR_4096);
    MU_ASSERT_STATUS(SENS_OK, ms5837_read_blocking(&s, &r));
    MU_ASSERT_NEAR(1013.0, r.pressure_mbar, 0.05);
    f.dead_adc = true;
    MU_ASSERT(ms5837_read_blocking(&s, &r) != SENS_OK);
}

static void test_failure_when_starting_temperature_conversion_restarts_cleanly(void)
{
    fake_ms5837_t f; ms5837_t s;
    fake_ms5837_setup(&f, true);
    ms5837_init(&s, MS5837_02BA, MS5837_OSR_2048);
    ms5837_update(&s, fake_time_get());                      /* transaction 1 : commande D1 */
    fake_time_advance(10);
    f.nack_skip = 2;                                         /* lecture ADC (2 transactions) OK ... */
    f.nack_next = 1;                                         /* ... puis la commande D2 est refusée */
    MU_ASSERT_STATUS(SENS_ERR_BUS, ms5837_update(&s, fake_time_get()));
    MU_ASSERT(s.error_count >= 1);
    MU_ASSERT_STATUS(SENS_OK, run_until_result(&s, 200));    /* le cycle suivant réussit */
    MU_ASSERT_NEAR(1013.0, s.pressure_mbar, 0.05);
}

static void test_blocking_read_reports_bus_error_immediately(void)
{
    fake_ms5837_t f; ms5837_t s; ms5837_result_t r;
    fake_ms5837_setup(&f, true);
    ms5837_init(&s, MS5837_02BA, MS5837_OSR_4096);
    fake_i2c_set_fail(true);
    uint32_t t0 = fake_time_get();
    MU_ASSERT_STATUS(SENS_ERR_BUS, ms5837_read_blocking(&s, &r));
    MU_ASSERT(fake_time_get() - t0 < 5);                     /* pas d'attente inutile quand le bus est mort */
}

int main(void)
{
    MU_RUN(test_crc4_matches_independent_reference);
    MU_RUN(test_prom_valid_detects_every_single_bit_flip);
    MU_RUN(test_compute_against_reference_vectors);
    MU_RUN(test_conv_times_are_increasing);
    MU_RUN(test_init_ok_reads_prom_and_resets_once);
    MU_RUN(test_init_bus_failure);
    MU_RUN(test_init_no_device_on_bus);
    MU_RUN(test_init_rejects_idle_bus_all_ones_or_zeros);
    MU_RUN(test_init_detects_corrupted_prom);
    MU_RUN(test_init_rejects_invalid_parameters);
    MU_RUN(test_update_returns_correct_pressure_and_temperature);
    MU_RUN(test_update_does_not_touch_bus_while_converting);
    MU_RUN(test_temperature_is_refreshed_only_every_n_samples);
    MU_RUN(test_update_tracks_pressure_and_temperature_changes);
    MU_RUN(test_works_with_30ba_model);
    MU_RUN(test_dead_adc_leads_to_timeout_then_recovers_cycle);
    MU_RUN(test_bus_error_midrun_is_reported_and_recovers);
    MU_RUN(test_isolated_nack_does_not_corrupt_next_sample);
    MU_RUN(test_out_of_range_pressure_is_rejected_and_not_published);
    MU_RUN(test_read_blocking_ok_and_timeout);
    MU_RUN(test_failure_when_starting_temperature_conversion_restarts_cleanly);
    MU_RUN(test_blocking_read_reports_bus_error_immediately);
    return MU_SUMMARY();
}
