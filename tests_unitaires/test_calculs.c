/* Tests unitaires de la bibliothèque pure : filtres, profondeur, batterie, anti-rebond, fuite, calibration magnétomètre. */
#include "mini_tests.h"
#include "filtres.h"
#include "profondeur.h"
#include "batterie.h"
#include "anti_rebond.h"
#include "detecteur_fuite.h"
#include "calibration_magneto.h"
#include <math.h>

/* ================================================================== filtres */
static void test_median3_all_orderings(void)
{
    MU_ASSERT_NEAR(2, filt_median3(1, 2, 3), 0);
    MU_ASSERT_NEAR(2, filt_median3(1, 3, 2), 0);
    MU_ASSERT_NEAR(2, filt_median3(2, 1, 3), 0);
    MU_ASSERT_NEAR(2, filt_median3(2, 3, 1), 0);
    MU_ASSERT_NEAR(2, filt_median3(3, 1, 2), 0);
    MU_ASSERT_NEAR(2, filt_median3(3, 2, 1), 0);
    MU_ASSERT_NEAR(2, filt_median3(2, 2, 5), 0);            /* doublons */
    MU_ASSERT_NEAR(10.1, filt_median3(10.0f, 10.1f, 500.0f), 1e-4);   /* pic isolé éliminé */
}

static void test_ema_first_sample_converges_and_ignores_nan(void)
{
    ema_t e;
    ema_init(&e, 0.5f);
    MU_ASSERT_NEAR(10, ema_update(&e, 10), 1e-6);           /* 1re valeur = initialisation */
    MU_ASSERT_NEAR(15, ema_update(&e, 20), 1e-6);
    MU_ASSERT_NEAR(15, ema_update(&e, NAN), 1e-6);          /* NaN ignoré */
    MU_ASSERT_NEAR(15, ema_update(&e, INFINITY), 1e-6);
    for (int i = 0; i < 40; i++) ema_update(&e, 20);
    MU_ASSERT_NEAR(20, e.y, 1e-3);
    ema_reset(&e);
    MU_ASSERT_NEAR(7, ema_update(&e, 7), 1e-6);
}

static void test_ema_alpha_from_cutoff(void)
{
    /* fc = 1 Hz, dt = 10 ms : alpha = dt / (dt + 1/(2*pi*fc)) = 0.0591 */
    MU_ASSERT_NEAR(0.0591, ema_alpha_from_fc(1.0f, 0.01f), 1e-4);
    MU_ASSERT(ema_alpha_from_fc(100.0f, 0.01f) > ema_alpha_from_fc(1.0f, 0.01f));   /* plus de coupure => plus réactif */
}

static void test_angle_wrapping(void)
{
    MU_ASSERT_NEAR(-170, wrap180(190), 1e-3);
    MU_ASSERT_NEAR(170, wrap180(-190), 1e-3);
    MU_ASSERT_NEAR(-180, wrap180(180), 1e-3);
    MU_ASSERT_NEAR(-180, wrap180(540), 1e-3);
    MU_ASSERT_NEAR(0, wrap180(720), 1e-3);
    MU_ASSERT_NEAR(350, wrap360(-10), 1e-3);
    MU_ASSERT_NEAR(0, wrap360(720), 1e-3);
    MU_ASSERT_NEAR(10, wrap360(370), 1e-3);
}

/* ================================================================== profondeur */
#define MBAR_PER_METER (997.0f * 9.80665f / 100.0f)         /* 97,77 mbar par mètre d'eau douce */

static void test_depth_from_pressure_physics(void)
{
    MU_ASSERT_NEAR(1.0, depth_from_pressure(1013.25f + MBAR_PER_METER, 1013.25f, 997.0f), 1e-3);
    MU_ASSERT_NEAR(0.0, depth_from_pressure(1013.25f, 1013.25f, 997.0f), 1e-6);
    MU_ASSERT_NEAR(-0.5, depth_from_pressure(1013.25f - 0.5f * MBAR_PER_METER, 1013.25f, 997.0f), 1e-3);
    /* eau de mer : plus dense, donc moins profond pour la même pression */
    MU_ASSERT(depth_from_pressure(1100.0f, 1013.25f, 1025.0f) < depth_from_pressure(1100.0f, 1013.25f, 997.0f));
}

static void test_depth_tare_averages_reference(void)
{
    depth_t d;
    depth_init(&d, 997.0f, 0.3f, 0.1f);
    MU_ASSERT(!d.ref_valid);
    depth_tare_start(&d, 10);
    MU_ASSERT(depth_is_taring(&d));
    for (int i = 0; i < 10; i++) {
        MU_ASSERT(!depth_update(&d, (i % 2) ? 1020.2f : 1019.8f, (uint32_t)i * 10));   /* bruit autour de 1020 */
    }
    MU_ASSERT(!depth_is_taring(&d));
    MU_ASSERT(d.ref_valid);
    MU_ASSERT_NEAR(1020.0, d.p_ref_mbar, 1e-2);
}

static void test_depth_converges_and_reference_removes_weather_offset(void)
{
    depth_t d;
    depth_init(&d, 997.0f, 0.3f, 0.1f);
    depth_set_reference(&d, 995.0f);                        /* basse pression : ne doit pas fausser la profondeur */
    bool ok = false;
    for (int i = 0; i < 60; i++) ok = depth_update(&d, 995.0f + 2.0f * MBAR_PER_METER, (uint32_t)i * 10);
    MU_ASSERT(ok);
    MU_ASSERT_NEAR(2.0, d.depth_m, 0.01);
}

static void test_depth_median_rejects_single_spike(void)
{
    depth_t d;
    depth_init(&d, 997.0f, 0.3f, 0.1f);
    depth_set_reference(&d, 1013.25f);
    for (int i = 0; i < 20; i++) depth_update(&d, 1013.25f + MBAR_PER_METER, (uint32_t)i * 10);
    MU_ASSERT_NEAR(1.0, d.depth_m, 0.01);
    depth_update(&d, 1013.25f + 5.0f * MBAR_PER_METER, 200);       /* pic parasite à 5 m */
    MU_ASSERT(d.depth_m < 1.05f);
    depth_update(&d, 1013.25f + MBAR_PER_METER, 210);
    MU_ASSERT_NEAR(1.0, d.depth_m, 0.02);
}

static void test_depth_rate_sign_and_value(void)
{
    depth_t d;
    depth_init(&d, 997.0f, 0.3f, 0.1f);
    depth_set_reference(&d, 1013.25f);
    for (int i = 0; i < 300; i++) {                          /* descente à 0,5 m/s, mesures à 100 Hz */
        float depth = 0.5f * (float)i * 0.01f;
        depth_update(&d, 1013.25f + depth * MBAR_PER_METER, (uint32_t)i * 10);
    }
    MU_ASSERT_NEAR(0.5, d.rate_m_s, 0.03);
    for (int i = 300; i < 700; i++) {                        /* remontée à 0,25 m/s */
        float depth = 1.5f - 0.25f * (float)(i - 300) * 0.01f;
        depth_update(&d, 1013.25f + depth * MBAR_PER_METER, (uint32_t)i * 10);
    }
    MU_ASSERT_NEAR(-0.25, d.rate_m_s, 0.03);
}

static void test_depth_untared_flag_and_nan_pressure(void)
{
    depth_t d;
    depth_init(&d, 997.0f, 0.3f, 0.1f);
    MU_ASSERT(depth_update(&d, 1013.25f, 0));                /* utilise la référence par défaut */
    MU_ASSERT(!d.ref_valid);                                 /* mais signale qu'elle n'est pas mesurée */
    MU_ASSERT(!depth_update(&d, NAN, 10));
    MU_ASSERT(isfinite(d.depth_m));
}

/* ================================================================== batterie */
static void test_soc_table(void)
{
    MU_ASSERT_NEAR(100, battery_soc_from_cell_v(4.20f), 1e-3);
    MU_ASSERT_NEAR(100, battery_soc_from_cell_v(4.50f), 1e-3);       /* saturation */
    MU_ASSERT_NEAR(0, battery_soc_from_cell_v(3.00f), 1e-3);
    MU_ASSERT_NEAR(0, battery_soc_from_cell_v(2.50f), 1e-3);
    MU_ASSERT_NEAR(35, battery_soc_from_cell_v(3.70f), 1e-3);
    MU_ASSERT_NEAR(68.5, battery_soc_from_cell_v(3.95f), 0.1);       /* interpolation 62 -> 75 */
    MU_ASSERT_NEAR(0, battery_soc_from_cell_v(NAN), 1e-3);
    float prev = -1;
    for (float v = 3.0f; v <= 4.2f; v += 0.01f) {                    /* monotone croissante */
        float s = battery_soc_from_cell_v(v);
        MU_ASSERT(s >= prev);
        prev = s;
    }
}

static batt_level_t feed(battery_t *b, float v, int n)
{
    batt_level_t l = b->level;
    for (int i = 0; i < n; i++) l = battery_update(b, v);
    return l;
}

static void test_battery_levels_hysteresis_and_confirmation(void)
{
    battery_t b;
    battery_init(&b, 3, 3.50f, 3.30f);
    MU_ASSERT_INT(BATT_OK, feed(&b, 12.3f, 20));
    MU_ASSERT_INT(BATT_OK, battery_update(&b, 9.0f));                 /* un seul creux de tension : ignoré */
    MU_ASSERT_INT(BATT_OK, feed(&b, 12.3f, 30));
    MU_ASSERT_INT(BATT_WARNING, feed(&b, 10.2f, 60));                 /* 3,40 V/élément */
    MU_ASSERT_INT(BATT_WARNING, feed(&b, 10.6f, 60));                 /* 3,53 : au-dessus du seuil mais dans l'hystérésis */
    MU_ASSERT_INT(BATT_OK, feed(&b, 11.1f, 60));                      /* 3,70 : franchement remonté */
    MU_ASSERT_INT(BATT_CRITICAL, feed(&b, 9.6f, 80));                 /* 3,20 V/élément */
    MU_ASSERT_INT(BATT_WARNING, feed(&b, 10.5f, 80));                 /* 3,50 : sort du critique mais pas de l'alerte */
    MU_ASSERT_INT(BATT_OK, feed(&b, 12.0f, 80));
}

static void test_battery_ignores_nan_and_zero_cells(void)
{
    battery_t b;
    battery_init(&b, 0, 3.5f, 3.3f);                                  /* 0 élément => corrigé à 1 */
    MU_ASSERT_INT(1, b.cells);
    battery_init(&b, 3, 3.5f, 3.3f);
    feed(&b, 12.3f, 10);
    MU_ASSERT_INT(BATT_OK, battery_update(&b, NAN));
    MU_ASSERT(isfinite(b.v_filt));
}

/* ================================================================== anti-rebond */
static void test_debounce_filters_bounces_and_reports_single_edge(void)
{
    debounce_t d;
    debounce_init(&d, false, 20);
    uint32_t t = 100;
    int edges = 0;
    for (int i = 0; i < 6; i++, t++) {                               /* rebonds : 6 ms d'alternance */
        MU_ASSERT(!debounce_update(&d, (i % 2) == 0, t));
    }
    bool stable = false;
    int t_stable = -1;
    for (int i = 0; i < 60; i++, t++) {                               /* puis état haut franc */
        stable = debounce_update(&d, true, t);
        edges += (d.edge == 1);
        if (stable && t_stable < 0) t_stable = i;
    }
    MU_ASSERT(stable);
    MU_ASSERT_INT(1, edges);
    MU_ASSERT(t_stable >= 18 && t_stable <= 21);                      /* ~20 ms après le dernier changement */
}

static void test_debounce_ignores_short_glitch_and_handles_timer_wraparound(void)
{
    debounce_t d;
    debounce_init(&d, true, 20);
    uint32_t t = 0xFFFFFFF0u;                                         /* le compteur de ms va déborder */
    for (int i = 0; i < 5; i++, t++) MU_ASSERT(debounce_update(&d, false, t));   /* glitch de 5 ms */
    for (int i = 0; i < 30; i++, t++) MU_ASSERT(debounce_update(&d, true, t));
    bool s = true;
    int falling = 0;
    for (int i = 0; i < 40; i++, t++) {                                           /* vrai relâchement, à cheval sur le débordement */
        s = debounce_update(&d, false, t);
        falling += (d.edge == -1);
    }
    MU_ASSERT(!s);
    MU_ASSERT_INT(1, falling);                                                    /* un seul front descendant signalé */
}

/* ================================================================== fuite */
static void test_leak_init_validates_thresholds(void)
{
    leak_t l;
    MU_ASSERT_STATUS(SENS_OK, leak_init(&l, PORT_ADC_LEAK, 2000, 1500, 3, true));
    MU_ASSERT_STATUS(SENS_ERR_PARAM, leak_init(&l, PORT_ADC_LEAK, 1500, 2000, 3, true));   /* actif haut : on > off */
    MU_ASSERT_STATUS(SENS_ERR_PARAM, leak_init(&l, PORT_ADC_LEAK, 2000, 2000, 3, true));
    MU_ASSERT_STATUS(SENS_OK, leak_init(&l, PORT_ADC_LEAK, 1000, 1500, 3, false));          /* actif bas : on < off */
    MU_ASSERT_STATUS(SENS_ERR_PARAM, leak_init(&l, PORT_ADC_LEAK, 2000, 1500, 3, false));
    MU_ASSERT_STATUS(SENS_ERR_PARAM, leak_init(&l, PORT_ADC_LEAK, 2000, 1500, 0, true));    /* confirmation nulle interdite */
}

static void test_leak_needs_consecutive_confirmations_and_latches(void)
{
    leak_t l;
    leak_init(&l, PORT_ADC_LEAK, 2000, 1500, 3, true);
    for (int i = 0; i < 10; i++) MU_ASSERT(!leak_process(&l, 100));        /* sec */
    MU_ASSERT(!leak_process(&l, 3000));                                    /* 1 lecture mouillée : pas d'alarme */
    MU_ASSERT(!leak_process(&l, 3000));
    MU_ASSERT(!leak_process(&l, 100));                                     /* interrompu => compteur remis à 0 */
    MU_ASSERT(!leak_process(&l, 3000));
    MU_ASSERT(!leak_process(&l, 3000));
    MU_ASSERT(leak_process(&l, 3000));                                     /* 3 consécutives : ALARME */
    for (int i = 0; i < 10; i++) MU_ASSERT(leak_process(&l, 50));          /* mémorisée même si ça sèche */
    leak_clear(&l);
    MU_ASSERT(!leak_process(&l, 50));
}

static void test_leak_hysteresis_band(void)
{
    leak_t l;
    leak_init(&l, PORT_ADC_LEAK, 2000, 1500, 2, true);
    for (int i = 0; i < 10; i++) MU_ASSERT(!leak_process(&l, 1800));       /* zone intermédiaire depuis le sec : reste sec */
    leak_process(&l, 2500);                                                /* mouillé... */
    MU_ASSERT(leak_process(&l, 1800));                                     /* ...reste mouillé dans la zone d'hystérésis => confirme */
}

static void test_leak_active_low_and_poll_reads_adc(void)
{
    leak_t l;
    leak_init(&l, PORT_ADC_LEAK, 1000, 1500, 2, false);
    fake_adc_value[PORT_ADC_LEAK] = 3000;                                  /* sec : tension haute */
    MU_ASSERT(!leak_poll(&l));
    fake_adc_value[PORT_ADC_LEAK] = 200;                                   /* mouillé : tension basse */
    MU_ASSERT(!leak_poll(&l));
    MU_ASSERT(leak_poll(&l));
    MU_ASSERT_INT(200, l.last_raw);
}

/* ================================================================== calibration magnétomètre */
/* Ellipsoïde alignée sur les axes (fer doux) décalée (fer dur) : cas typique d'un capteur dans un sous-marin. */
static void fill_ellipsoid(mag_cal_t *c, const float off[3], const float sc[3], float radius)
{
    for (int lat = -90; lat <= 90; lat += 15) {
        for (int lon = 0; lon < 360; lon += 15) {
            float la = (float)lat * 3.14159265f / 180.0f, lo = (float)lon * 3.14159265f / 180.0f;
            float m[3] = {
                off[0] + sc[0] * radius * cosf(la) * cosf(lo),
                off[1] + sc[1] * radius * cosf(la) * sinf(lo),
                off[2] + sc[2] * radius * sinf(la),
            };
            mag_cal_feed(c, m);
        }
    }
}

static void test_mag_cal_removes_hard_and_soft_iron(void)
{
    mag_cal_t c;
    mag_cal_init(&c);
    const float off[3] = { 0.12f, -0.05f, 0.20f };
    const float sc[3] = { 1.0f, 1.3f, 0.8f };
    fill_ellipsoid(&c, off, sc, 0.45f);
    MU_ASSERT_STATUS(SENS_OK, mag_cal_finish(&c, 0.3f));
    MU_ASSERT_NEAR(0.12, c.offset[0], 1e-3);
    MU_ASSERT_NEAR(-0.05, c.offset[1], 1e-3);
    MU_ASSERT_NEAR(0.20, c.offset[2], 1e-3);
    /* après correction, tous les points doivent être à la même distance de l'origine */
    double mn = 1e9, mx = 0;
    for (int lat = -75; lat <= 75; lat += 30) {
        for (int lon = 0; lon < 360; lon += 40) {
            float la = (float)lat * 3.14159265f / 180.0f, lo = (float)lon * 3.14159265f / 180.0f;
            float m[3] = { off[0] + sc[0] * 0.45f * cosf(la) * cosf(lo), off[1] + sc[1] * 0.45f * cosf(la) * sinf(lo),
                           off[2] + sc[2] * 0.45f * sinf(la) };
            float o[3];
            mag_cal_apply(&c, m, o);
            double n = sqrt((double)o[0] * o[0] + (double)o[1] * o[1] + (double)o[2] * o[2]);
            if (n < mn) mn = n;
            if (n > mx) mx = n;
        }
    }
    MU_ASSERT((mx - mn) / ((mx + mn) / 2) < 0.02);                         /* dispersion de rayon < 2 % */
}

static void test_mag_cal_rejects_insufficient_data(void)
{
    mag_cal_t c;
    mag_cal_init(&c);
    float m[3] = { 0.1f, 0.1f, 0.1f };
    for (int i = 0; i < 10; i++) mag_cal_feed(&c, m);
    MU_ASSERT_STATUS(SENS_ERR_NOT_READY, mag_cal_finish(&c, 0.3f));        /* trop peu d'échantillons */

    mag_cal_init(&c);
    for (int i = 0; i < 200; i++) {                                        /* rotation à plat uniquement : Z n'a pas bougé */
        float a = (float)i * 0.1f;
        float p[3] = { 0.4f * cosf(a), 0.4f * sinf(a), 0.30f };
        mag_cal_feed(&c, p);
    }
    MU_ASSERT_STATUS(SENS_ERR_RANGE, mag_cal_finish(&c, 0.3f));
    MU_ASSERT(!c.valid);
}

static void test_mag_cal_ignores_nan_samples_and_identity_when_not_calibrated(void)
{
    mag_cal_t c;
    mag_cal_init(&c);
    float bad[3] = { NAN, 0.1f, 0.1f };
    mag_cal_feed(&c, bad);
    MU_ASSERT_INT(0, c.n);
    float in[3] = { 0.1f, 0.2f, 0.3f }, out[3];
    mag_cal_apply(&c, in, out);                                            /* non calibré : identité */
    MU_ASSERT_NEAR(0.1, out[0], 1e-6);
    MU_ASSERT_NEAR(0.3, out[2], 1e-6);
}

static void test_depth_rate_not_updated_after_a_measurement_gap(void)
{
    depth_t d;
    depth_init(&d, 997.0f, 0.3f, 0.1f);
    depth_set_reference(&d, 1013.25f);
    for (int i = 0; i < 300; i++) {                                    /* descente régulière à 0,5 m/s */
        float depth = 0.5f * (float)i * 0.01f;
        depth_update(&d, 1013.25f + depth * MBAR_PER_METER, (uint32_t)i * 10);
    }
    float rate_before = d.rate_m_s;
    /* silence de 2 s, puis une profondeur très différente : le dt est trop grand pour dériver une vitesse fiable */
    depth_update(&d, 1013.25f + 4.0f * MBAR_PER_METER, 2000u + 2990u);
    MU_ASSERT_NEAR(rate_before, d.rate_m_s, 1e-6);
}

int main(void)
{
    MU_RUN(test_median3_all_orderings);
    MU_RUN(test_ema_first_sample_converges_and_ignores_nan);
    MU_RUN(test_ema_alpha_from_cutoff);
    MU_RUN(test_angle_wrapping);
    MU_RUN(test_depth_from_pressure_physics);
    MU_RUN(test_depth_tare_averages_reference);
    MU_RUN(test_depth_converges_and_reference_removes_weather_offset);
    MU_RUN(test_depth_median_rejects_single_spike);
    MU_RUN(test_depth_rate_sign_and_value);
    MU_RUN(test_depth_untared_flag_and_nan_pressure);
    MU_RUN(test_depth_rate_not_updated_after_a_measurement_gap);
    MU_RUN(test_soc_table);
    MU_RUN(test_battery_levels_hysteresis_and_confirmation);
    MU_RUN(test_battery_ignores_nan_and_zero_cells);
    MU_RUN(test_debounce_filters_bounces_and_reports_single_edge);
    MU_RUN(test_debounce_ignores_short_glitch_and_handles_timer_wraparound);
    MU_RUN(test_leak_init_validates_thresholds);
    MU_RUN(test_leak_needs_consecutive_confirmations_and_latches);
    MU_RUN(test_leak_hysteresis_band);
    MU_RUN(test_leak_active_low_and_poll_reads_adc);
    MU_RUN(test_mag_cal_removes_hard_and_soft_iron);
    MU_RUN(test_mag_cal_rejects_insufficient_data);
    MU_RUN(test_mag_cal_ignores_nan_samples_and_identity_when_not_calibrated);
    return MU_SUMMARY();
}
