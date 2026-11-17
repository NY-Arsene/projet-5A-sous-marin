/* Tests de l'estimation d'assiette : on SIMULE des rotations connues (matrice de rotation) et on vérifie les angles retrouvés. */
#include "mini_tests.h"
#include "assiette.h"
#include "filtres.h"
#include <math.h>

#define D2R (3.14159265358979f / 180.0f)

/* Lecture d'un accéléromètre et d'un magnétomètre pour une orientation (roulis phi, tangage theta, cap psi),
 * repère avant-tribord-bas, champ terrestre (nord 0,22 ; bas 0,42 gauss) : matrice DCM Z-Y-X. */
static void simulate(float phi_d, float th_d, float psi_d, float acc[3], float mag[3])
{
    float p = phi_d * D2R, t = th_d * D2R, s = psi_d * D2R;
    float sp = sinf(p), cp = cosf(p), st = sinf(t), ct = cosf(t), ss = sinf(s), cs = cosf(s);
    float R[3][3] = {
        { ct * cs,                ct * ss,                -st     },
        { sp * st * cs - cp * ss, sp * st * ss + cp * cs, sp * ct },
        { cp * st * cs + sp * ss, cp * st * ss - sp * cs, cp * ct },
    };
    /* gravité (vecteur vers le bas) exprimée dans le repère du corps, l'accéléromètre lit son opposé */
    acc[0] = -R[0][2];
    acc[1] = -R[1][2];
    acc[2] = -R[2][2];
    const float bn = 0.22f, bd = 0.42f;
    mag[0] = R[0][0] * bn + R[0][2] * bd;
    mag[1] = R[1][0] * bn + R[1][2] * bd;
    mag[2] = R[2][0] * bn + R[2][2] * bd;
}

static void test_heading_is_tilt_compensated_over_a_grid_of_orientations(void)
{
    const float rolls[] = { -30, 0, 25 }, pitches[] = { -20, 0, 15 };
    const float yaws[] = { 0, 45, 90, 135, 180, 270, 350 };
    for (unsigned r = 0; r < 3; r++) for (unsigned p = 0; p < 3; p++) for (unsigned y = 0; y < 7; y++) {
        float acc[3], mag[3];
        simulate(rolls[r], pitches[p], yaws[y], acc, mag);
        float h = attitude_heading_deg(rolls[r], pitches[p], mag);
        float err = fabsf(wrap180(h - yaws[y]));
        if (err > 0.05f) printf("    cap faux : roulis %.0f tangage %.0f cap %.0f -> %.2f\n", rolls[r], pitches[p], yaws[y], h);
        MU_ASSERT(err <= 0.05f);
    }
}

static void test_first_update_initialises_from_accelerometer_and_magnetometer(void)
{
    const float rolls[] = { -40, -10, 0, 20, 60 }, pitches[] = { -30, 0, 25 };
    for (unsigned r = 0; r < 5; r++) for (unsigned p = 0; p < 3; p++) {
        attitude_t a;
        attitude_init(&a, 0.02f, 0.01f);
        float acc[3], mag[3], gyr[3] = { 0, 0, 0 };
        simulate(rolls[r], pitches[p], 123.0f, acc, mag);
        attitude_update(&a, acc, gyr, mag, 0.01f);
        MU_ASSERT_NEAR(rolls[r], a.roll_deg, 0.1);
        MU_ASSERT_NEAR(pitches[p], a.pitch_deg, 0.1);
        MU_ASSERT_NEAR(0.0, fabsf(wrap180(a.yaw_deg - 123.0f)), 0.1);
    }
}

static void test_gyro_integration_yaw_without_magnetometer(void)
{
    attitude_t a;
    attitude_init(&a, 0.02f, 0.01f);
    const float level[3] = { 0, 0, -1 }, gyr[3] = { 0, 0, 90 };      /* rotation de 90 deg/s autour de Z */
    attitude_update(&a, level, gyr, NULL, 0.01f);                     /* initialisation */
    MU_ASSERT(!a.yaw_from_mag);
    for (int i = 0; i < 100; i++) attitude_update(&a, level, gyr, NULL, 0.01f);
    MU_ASSERT_NEAR(90.0, a.yaw_deg, 0.5);
    MU_ASSERT_NEAR(0.0, a.roll_deg, 0.2);
    MU_ASSERT_NEAR(0.0, a.pitch_deg, 0.2);
}

static void test_gyro_integration_roll_and_pitch_signs(void)
{
    attitude_t a;
    attitude_init(&a, 0.0f, 0.0f);                                    /* sans correction : intégration pure */
    const float level[3] = { 0, 0, -1 };
    float gx[3] = { 20, 0, 0 }, gy[3] = { 0, 15, 0 };
    attitude_update(&a, level, gx, NULL, 0.01f);
    for (int i = 0; i < 100; i++) attitude_update(&a, level, gx, NULL, 0.01f);
    MU_ASSERT_NEAR(20.0, a.roll_deg, 0.3);                            /* p > 0 => roulis > 0 */
    attitude_init(&a, 0.0f, 0.0f);                                    /* nouvelle expérience : tangage seul */
    attitude_update(&a, level, gy, NULL, 0.01f);
    for (int i = 0; i < 100; i++) attitude_update(&a, level, gy, NULL, 0.01f);
    MU_ASSERT_NEAR(15.0, a.pitch_deg, 0.3);                           /* q > 0 => tangage > 0 (nez en haut) */
    MU_ASSERT_NEAR(0.0, a.roll_deg, 0.1);
}

static void test_accelerometer_corrects_initial_error_and_gyro_bias(void)
{
    attitude_t a;
    attitude_init(&a, 0.02f, 0.0f);
    float acc[3], mag[3], zero[3] = { 0, 0, 0 };
    simulate(0, 0, 0, acc, mag);
    attitude_update(&a, acc, zero, NULL, 0.01f);                      /* démarre à plat */
    simulate(30, 0, 0, acc, mag);                                     /* le sous-marin est en fait incliné de 30 deg */
    for (int i = 0; i < 1000; i++) attitude_update(&a, acc, zero, NULL, 0.01f);
    MU_ASSERT_NEAR(30.0, a.roll_deg, 0.3);

    float biased[3] = { 0.5f, 0, 0 };                                 /* biais gyro résiduel de 0,5 deg/s */
    for (int i = 0; i < 3000; i++) attitude_update(&a, acc, biased, NULL, 0.01f);
    MU_ASSERT_NEAR(30.0, a.roll_deg, 1.0);                            /* l'accéléromètre empêche la dérive */
}

static void test_strong_acceleration_disables_accelerometer_correction(void)
{
    attitude_t a;
    attitude_init(&a, 0.05f, 0.0f);
    float level[3] = { 0, 0, -1 }, zero[3] = { 0, 0, 0 };
    attitude_update(&a, level, zero, NULL, 0.01f);
    float shock[3] = { 1.2f, 0, -1.6f };                              /* norme 2,0 g : choc, gravité non mesurable */
    for (int i = 0; i < 50; i++) attitude_update(&a, shock, zero, NULL, 0.01f);
    MU_ASSERT_NEAR(0.0, a.roll_deg, 0.001);
    MU_ASSERT_NEAR(0.0, a.pitch_deg, 0.001);
}

static void test_magnetometer_pulls_heading_and_takes_the_short_way_around_north(void)
{
    attitude_t a;
    attitude_init(&a, 0.02f, 0.02f);
    float acc[3], mag[3], zero[3] = { 0, 0, 0 };
    simulate(0, 0, 359.0f, acc, mag);
    attitude_update(&a, acc, zero, mag, 0.01f);
    MU_ASSERT_NEAR(0.0, fabsf(wrap180(a.yaw_deg - 359.0f)), 0.1);
    simulate(0, 0, 1.0f, acc, mag);                                   /* cap réel 1 deg : 2 deg plus loin, en passant par le nord */
    for (int i = 0; i < 300; i++) {
        attitude_update(&a, acc, zero, mag, 0.01f);
        MU_ASSERT(a.yaw_deg > 350.0f || a.yaw_deg < 10.0f);           /* ne repasse JAMAIS par 180 */
    }
    MU_ASSERT_NEAR(0.0, fabsf(wrap180(a.yaw_deg - 1.0f)), 0.2);

    simulate(0, 0, 60.0f, acc, mag);                                  /* grand écart : convergence progressive */
    for (int i = 0; i < 600; i++) attitude_update(&a, acc, zero, mag, 0.01f);
    MU_ASSERT_NEAR(0.0, fabsf(wrap180(a.yaw_deg - 60.0f)), 0.5);
}

static void test_magnetometer_dropout_keeps_integrating_gyro(void)
{
    attitude_t a;
    attitude_init(&a, 0.02f, 0.02f);
    float acc[3], mag[3], zero[3] = { 0, 0, 0 }, gz[3] = { 0, 0, 30 };
    simulate(0, 0, 100.0f, acc, mag);
    attitude_update(&a, acc, zero, mag, 0.01f);
    for (int i = 0; i < 100; i++) attitude_update(&a, acc, gz, NULL, 0.01f);   /* magnétomètre perdu pendant 1 s */
    MU_ASSERT_NEAR(130.0, a.yaw_deg, 0.5);
    float bad[3] = { NAN, 0, 0 };
    float before = a.yaw_deg;
    attitude_update(&a, acc, zero, bad, 0.01f);                        /* mesure corrompue : ignorée */
    MU_ASSERT(isfinite(a.yaw_deg));
    MU_ASSERT_NEAR(before, a.yaw_deg, 0.01);
}

static void test_invalid_dt_and_extreme_pitch_never_produce_nan(void)
{
    attitude_t a;
    attitude_init(&a, 0.02f, 0.01f);
    float acc[3], mag[3], g[3] = { 10, 200, -50 };
    simulate(0, 89.0f, 0, acc, mag);                                   /* presque vertical : singularité du tangage */
    attitude_update(&a, acc, g, mag, 0.01f);
    float r0 = a.roll_deg;
    attitude_update(&a, acc, g, mag, 0.0f);                            /* dt nul */
    attitude_update(&a, acc, g, mag, -1.0f);                           /* dt négatif */
    attitude_update(&a, acc, g, mag, NAN);
    MU_ASSERT_NEAR(r0, a.roll_deg, 1e-6);
    for (int i = 0; i < 2000; i++) attitude_update(&a, acc, g, mag, 0.01f);
    MU_ASSERT(isfinite(a.roll_deg) && isfinite(a.pitch_deg) && isfinite(a.yaw_deg));
    MU_ASSERT(a.roll_deg >= -180 && a.roll_deg < 180);
    MU_ASSERT(a.yaw_deg >= 0 && a.yaw_deg < 360);
}

static void test_axis_remap(void)
{
    const float in[3] = { 1, 2, 3 };
    float out[3];
    const int8_t m1[3] = { 2, -1, 3 };
    MU_ASSERT(attitude_remap_axes(in, m1, out));
    MU_ASSERT_NEAR(2, out[0], 0); MU_ASSERT_NEAR(-1, out[1], 0); MU_ASSERT_NEAR(3, out[2], 0);
    const int8_t id[3] = { 1, 2, 3 };
    MU_ASSERT(attitude_remap_axes(in, id, out));
    MU_ASSERT_NEAR(1, out[0], 0); MU_ASSERT_NEAR(3, out[2], 0);
    const int8_t bad1[3] = { 0, 1, 2 }, bad2[3] = { 4, 1, 2 }, bad3[3] = { 1, -5, 2 };
    MU_ASSERT(!attitude_remap_axes(in, bad1, out)); MU_ASSERT_NEAR(0, out[1], 0);
    MU_ASSERT(!attitude_remap_axes(in, bad2, out));
    MU_ASSERT(!attitude_remap_axes(in, bad3, out));
    float x[3] = { 1, 2, 3 };
    MU_ASSERT(attitude_remap_axes(x, m1, x));                          /* entrée = sortie autorisée */
    MU_ASSERT_NEAR(2, x[0], 0); MU_ASSERT_NEAR(-1, x[1], 0);
}

static void test_gate_is_progressive_between_0_1_and_0_3_g(void)
{
    /* norme 1,2 g : correction à moitié de son poids ; norme 1,0 g : plein poids ; norme 1,35 g : nulle */
    float zero[3] = { 0, 0, 0 };
    float dev_roll[3];
    const float norms[3] = { 1.0f, 1.2f, 1.35f };
    float shift[3];
    for (int k = 0; k < 3; k++) {
        attitude_t a;
        attitude_init(&a, 0.1f, 0.0f);
        const float level[3] = { 0, 0, -1 };
        attitude_update(&a, level, zero, NULL, 0.01f);
        /* vecteur incliné de 20 deg de roulis, de norme choisie */
        dev_roll[0] = 0.0f;
        dev_roll[1] = -norms[k] * sinf(20.0f * D2R);
        dev_roll[2] = -norms[k] * cosf(20.0f * D2R);
        attitude_update(&a, dev_roll, zero, NULL, 0.01f);
        shift[k] = a.roll_deg;                                         /* déplacement obtenu en une étape */
    }
    MU_ASSERT_NEAR(2.0, shift[0], 0.05);                               /* 0,1 * 20 deg */
    MU_ASSERT_NEAR(1.0, shift[1], 0.1);                                /* gate = 0,5 */
    MU_ASSERT_NEAR(0.0, shift[2], 1e-3);                               /* au-delà de 0,3 g d'écart : aucune correction */
}

static void test_gyro_integration_step_is_bounded_near_vertical_pitch(void)
{
    attitude_t a;
    attitude_init(&a, 0.0f, 0.0f);
    float acc[3], mag[3];
    simulate(0, 89.0f, 0, acc, mag);                                   /* tangage 89 deg : tan(theta) = 57 */
    float zero[3] = { 0, 0, 0 };
    attitude_update(&a, acc, zero, NULL, 0.01f);                       /* initialisation depuis l'accéléromètre */
    float r0 = a.roll_deg;
    float gyr[3] = { 0, 0, 100.0f };                                   /* 100 deg/s en lacet */
    attitude_update(&a, acc, gyr, NULL, 0.01f);
    /* sans protection : roulis += 100 * 57 * 0,01 = 57 deg en un pas ; avec protection : au plus ~5 deg */
    MU_ASSERT(fabsf(wrap180(a.roll_deg - r0)) < 6.0f);
}

int main(void)
{
    MU_RUN(test_heading_is_tilt_compensated_over_a_grid_of_orientations);
    MU_RUN(test_first_update_initialises_from_accelerometer_and_magnetometer);
    MU_RUN(test_gyro_integration_yaw_without_magnetometer);
    MU_RUN(test_gyro_integration_roll_and_pitch_signs);
    MU_RUN(test_accelerometer_corrects_initial_error_and_gyro_bias);
    MU_RUN(test_strong_acceleration_disables_accelerometer_correction);
    MU_RUN(test_magnetometer_pulls_heading_and_takes_the_short_way_around_north);
    MU_RUN(test_magnetometer_dropout_keeps_integrating_gyro);
    MU_RUN(test_invalid_dt_and_extreme_pitch_never_produce_nan);
    MU_RUN(test_axis_remap);
    MU_RUN(test_gate_is_progressive_between_0_1_and_0_3_g);
    MU_RUN(test_gyro_integration_step_is_bounded_near_vertical_pitch);
    return MU_SUMMARY();
}
