/* Tests de la régulation profondeur / tangage : sens des corrections, saturation, rampe, transfert sans à-coup. */
#include "mini_tests.h"
#include "regulation_profondeur.h"

static depth_ctrl_t dc;
static depth_ctrl_cfg_t cfg;

static void setup(void)
{
    depth_ctrl_default_config(&cfg);
    MU_ASSERT(depth_ctrl_init(&dc, &cfg));
    depth_ctrl_start(&dc, 0.0f, cfg.v_neutral_ml, cfg.v_neutral_ml);
}

static void run(int n, float depth, float rate, float pitch, float prate, bool pvalid)
{
    for (int i = 0; i < n; i++) depth_ctrl_update(&dc, depth, rate, pitch, prate, pvalid, 0.01f);
}

static void test_init_validation(void)
{
    depth_ctrl_cfg_t c; depth_ctrl_default_config(&c);
    depth_ctrl_t d;
    MU_ASSERT(depth_ctrl_init(&d, &c));
    depth_ctrl_cfg_t b = c; b.v_neutral_ml = 150; MU_ASSERT(!depth_ctrl_init(&d, &b));        /* neutre hors course */
    b = c; b.v_neutral_ml = -1;                    MU_ASSERT(!depth_ctrl_init(&d, &b));
    b = c; b.max_volume_ml = 0;                    MU_ASSERT(!depth_ctrl_init(&d, &b));
    b = c; b.pitch_sign = 0.5f;                    MU_ASSERT(!depth_ctrl_init(&d, &b));
    b = c; b.setpoint_rate_m_s = NAN;              MU_ASSERT(!depth_ctrl_init(&d, &b));
    b = c; b.kp_depth = -1;                        MU_ASSERT(!depth_ctrl_init(&d, &b));
    MU_ASSERT(!depth_ctrl_init(NULL, &c));
}

static void test_neutral_when_on_target_and_level(void)
{
    setup();
    run(100, 0.0f, 0, 0, 0, true);
    MU_ASSERT_NEAR(cfg.v_neutral_ml, dc.front_ml, 0.01);
    MU_ASSERT_NEAR(cfg.v_neutral_ml, dc.aft_ml, 0.01);
}

static void test_too_shallow_floods_ballasts_to_descend(void)
{
    setup();
    MU_ASSERT(depth_ctrl_set_depth(&dc, 1.0f));
    run(500, 0.0f, 0, 0, 0, true);                         /* 5 s : consigne rampée, sous-marin toujours en surface */
    MU_ASSERT(dc.front_ml > cfg.v_neutral_ml + 1.0f);
    MU_ASSERT(dc.aft_ml > cfg.v_neutral_ml + 1.0f);
    MU_ASSERT_NEAR(dc.front_ml, dc.aft_ml, 0.01);          /* à plat : même volume des deux côtés */
}

static void test_too_deep_empties_ballasts_to_rise(void)
{
    setup();
    depth_ctrl_start(&dc, 1.5f, cfg.v_neutral_ml, cfg.v_neutral_ml);
    MU_ASSERT(depth_ctrl_set_depth(&dc, 0.5f));
    run(500, 1.5f, 0, 0, 0, true);
    MU_ASSERT(dc.front_ml < cfg.v_neutral_ml - 1.0f);
    MU_ASSERT(dc.aft_ml < cfg.v_neutral_ml - 1.0f);
}

static void test_descending_fast_brakes_via_derivative(void)
{
    setup();
    run(1, 0.0f, 0.0f, 0, 0, true);
    float no_rate = dc.front_ml;
    setup();
    run(1, 0.0f, 0.3f, 0, 0, true);                        /* même profondeur mais ça descend déjà à 0,3 m/s */
    MU_ASSERT(dc.front_ml < no_rate - 10.0f);              /* on vide pour freiner */
}

static void test_volumes_stay_within_stroke(void)
{
    setup();
    MU_ASSERT(depth_ctrl_set_depth(&dc, 2.0f));
    run(20000, 0.0f, 0, 0, 0, true);                       /* erreur énorme pendant 200 s */
    MU_ASSERT(dc.front_ml <= cfg.max_volume_ml + 1e-3);
    MU_ASSERT(dc.aft_ml <= cfg.max_volume_ml + 1e-3);
    depth_ctrl_start(&dc, 2.0f, cfg.v_neutral_ml, cfg.v_neutral_ml);
    MU_ASSERT(depth_ctrl_set_depth(&dc, 0.0f));
    run(20000, 2.0f, 0, 0, 0, true);
    MU_ASSERT(dc.front_ml >= -1e-3);
    MU_ASSERT(dc.aft_ml >= -1e-3);
}

static void test_nose_up_floods_front_more(void)
{
    setup();
    run(100, 0.0f, 0, 10.0f, 0, true);                     /* nez à +10 degrés */
    MU_ASSERT(dc.front_ml > dc.aft_ml + 3.0f);             /* avant plus lourd : le nez redescend */
    run(100, 0.0f, 0, -10.0f, 0, true);                    /* nez vers le bas : l'inverse */
    MU_ASSERT(dc.front_ml < dc.aft_ml);
}

static void test_pitch_sign_inverts_correction(void)
{
    depth_ctrl_default_config(&cfg);
    cfg.pitch_sign = -1.0f;
    MU_ASSERT(depth_ctrl_init(&dc, &cfg));
    depth_ctrl_start(&dc, 0.0f, cfg.v_neutral_ml, cfg.v_neutral_ml);
    run(100, 0.0f, 0, 10.0f, 0, true);
    MU_ASSERT(dc.front_ml < dc.aft_ml - 3.0f);
}

static void test_pitch_difference_is_limited(void)
{
    setup();
    run(3000, 0.0f, 0, 80.0f, 0, true);
    MU_ASSERT(dc.front_ml - dc.aft_ml <= cfg.max_diff_ml + 0.01);
}

static void test_invalid_imu_disables_pitch_correction_but_depth_continues(void)
{
    setup();
    run(100, 0.0f, 0, 10.0f, 0, true);
    MU_ASSERT(dc.front_ml - dc.aft_ml > 3.0f);
    run(10, 0.0f, 0, 10.0f, 0, false);                     /* IMU perdue */
    MU_ASSERT_NEAR(dc.front_ml, dc.aft_ml, 0.01);
    MU_ASSERT(depth_ctrl_set_depth(&dc, 1.0f));
    run(500, 0.0f, 0, 10.0f, 0, false);
    MU_ASSERT(dc.front_ml > cfg.v_neutral_ml);             /* la profondeur est toujours régulée */
}

static void test_imu_loss_clears_pitch_integral(void)
{
    setup();
    run(300, 0.0f, 0, 10.0f, 0, true);                     /* tangage +10 degres pendant 3 s : l'integrale de tangage s'accumule */
    MU_ASSERT(dc.front_ml - dc.aft_ml > 3.0f);
    run(1, 0.0f, 0, 10.0f, 0, false);                      /* IMU perdue un instant */
    run(1, 0.0f, 0, 0.0f, 0, true);                        /* revenue, tangage nul : plus aucune correction heritee */
    MU_ASSERT_NEAR(dc.front_ml, dc.aft_ml, 0.01);
}

static void test_invalid_depth_keeps_previous_volumes(void)
{
    setup();
    MU_ASSERT(depth_ctrl_set_depth(&dc, 1.0f));
    run(300, 0.0f, 0, 0, 0, true);
    float f = dc.front_ml, a = dc.aft_ml;
    depth_ctrl_update(&dc, NAN, 0, 0, 0, true, 0.01f);
    depth_ctrl_update(&dc, 0.0f, INFINITY, 0, 0, true, 0.01f);
    depth_ctrl_update(&dc, 0.0f, 0, 0, 0, true, 0.0f);
    MU_ASSERT_NEAR(f, dc.front_ml, 1e-9);
    MU_ASSERT_NEAR(a, dc.aft_ml, 1e-9);
}

static void test_setpoint_is_validated(void)
{
    setup();
    MU_ASSERT(depth_ctrl_set_depth(&dc, 1.2f));
    MU_ASSERT(!depth_ctrl_set_depth(&dc, -0.1f));
    MU_ASSERT(!depth_ctrl_set_depth(&dc, cfg.max_setpoint_m + 0.01f));
    MU_ASSERT(!depth_ctrl_set_depth(&dc, NAN));
    MU_ASSERT_NEAR(1.2, dc.setpoint_m, 1e-6);              /* l'ancienne consigne reste */
}

static void test_setpoint_is_ramped(void)
{
    setup();
    MU_ASSERT(depth_ctrl_set_depth(&dc, 1.5f));
    run(100, 0.0f, 0, 0, 0, true);                         /* 1 s à 0,1 m/s */
    MU_ASSERT_NEAR(0.1, dc.setpoint_eff_m, 0.002);
    run(1400, 0.0f, 0, 0, 0, true);
    MU_ASSERT_NEAR(1.5, dc.setpoint_eff_m, 1e-4);          /* et elle finit par l'atteindre exactement */
}

static void test_start_is_bumpless(void)
{
    setup();
    depth_ctrl_start(&dc, 1.0f, 70.0f, 70.0f);             /* on reprend la main avec les ballasts à 70 mL, à 1,0 m */
    run(1, 1.0f, 0, 0, 0, true);
    MU_ASSERT_NEAR(70.0, dc.front_ml, 0.5);                /* pas de saut : on reste au volume actuel */
    MU_ASSERT_NEAR(1.0, dc.setpoint_m, 1e-9);              /* et on tient la profondeur actuelle */
}

static void test_start_from_surface_begins_at_neutral_not_empty(void)
{
    setup();
    depth_ctrl_start(&dc, 0.0f, 0.0f, 0.0f);               /* en surface, ballasts vides (IDLE) */
    run(1, 0.0f, 0, 0, 0, true);
    MU_ASSERT_NEAR(cfg.v_neutral_ml, dc.front_ml, 0.5);    /* la consigne de volume vise la flottabilité neutre, pas 0 */
    MU_ASSERT_NEAR(0.0, dc.setpoint_m, 1e-9);
}

static void test_set_gains_validates_and_selects_controller(void)
{
    setup();
    MU_ASSERT(depth_ctrl_set_gains(&dc, 0, 10, 1, 5));
    MU_ASSERT_NEAR(10.0, dc.depth_pid.kp, 1e-9);
    MU_ASSERT(depth_ctrl_set_gains(&dc, 1, 2, 0, 1));
    MU_ASSERT_NEAR(2.0, dc.pitch_pid.kp, 1e-9);
    MU_ASSERT(!depth_ctrl_set_gains(&dc, 2, 1, 1, 1));
    MU_ASSERT(!depth_ctrl_set_gains(&dc, 0, -1, 1, 1));
    MU_ASSERT_NEAR(10.0, dc.depth_pid.kp, 1e-9);
}

int main(void)
{
    MU_RUN(test_init_validation);
    MU_RUN(test_neutral_when_on_target_and_level);
    MU_RUN(test_too_shallow_floods_ballasts_to_descend);
    MU_RUN(test_too_deep_empties_ballasts_to_rise);
    MU_RUN(test_descending_fast_brakes_via_derivative);
    MU_RUN(test_volumes_stay_within_stroke);
    MU_RUN(test_nose_up_floods_front_more);
    MU_RUN(test_pitch_sign_inverts_correction);
    MU_RUN(test_pitch_difference_is_limited);
    MU_RUN(test_invalid_imu_disables_pitch_correction_but_depth_continues);
    MU_RUN(test_imu_loss_clears_pitch_integral);
    MU_RUN(test_invalid_depth_keeps_previous_volumes);
    MU_RUN(test_setpoint_is_validated);
    MU_RUN(test_setpoint_is_ramped);
    MU_RUN(test_start_is_bumpless);
    MU_RUN(test_start_from_surface_begins_at_neutral_not_empty);
    MU_RUN(test_set_gains_validates_and_selects_controller);
    return MU_SUMMARY();
}
