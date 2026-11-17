/* Tests du générateur d'impulsions : précision de position, rampe, inversion, butées, arrêt, robustesse. */
#include "mini_tests.h"
#include "moteur_pas_a_pas.h"

#define TICK_HZ 20000u

static stepper_t   s_st;
static stepper_cfg_t s_cfg;

static void setup(void)
{
    s_cfg = (stepper_cfg_t){ .tick_hz = TICK_HZ, .max_speed_sps = 3000, .min_speed_sps = 100,
                             .accel_sps2 = 6000, .min_pos = -10000, .max_pos = 10000 };
    MU_ASSERT(stepper_init(&s_st, PORT_MOTOR_FWD, &s_cfg));
    stepper_enable(&s_st, true);
}

/** Fait tourner le tick jusqu'à l'arrêt ; renvoie le nombre de ticks (ou max_ticks si jamais arrêté). */
static uint32_t run_until_idle(uint32_t max_ticks)
{
    uint32_t n = 0;
    while (n < max_ticks && stepper_is_moving(&s_st)) {
        stepper_tick(&s_st);
        n++;
    }
    return n;
}

static void test_init_rejects_invalid_config(void)
{
    stepper_t s;
    stepper_cfg_t c = { TICK_HZ, 3000, 100, 6000, -10, 10 };
    MU_ASSERT(stepper_init(&s, PORT_MOTOR_FWD, &c));

    stepper_cfg_t bad = c; bad.max_speed_sps = (int32_t)(TICK_HZ / 2u + 1);   /* plus d'une impulsion tous les 2 ticks */
    MU_ASSERT(!stepper_init(&s, PORT_MOTOR_FWD, &bad));
    bad = c; bad.min_speed_sps = 0;           MU_ASSERT(!stepper_init(&s, PORT_MOTOR_FWD, &bad));
    bad = c; bad.min_speed_sps = 4000;        MU_ASSERT(!stepper_init(&s, PORT_MOTOR_FWD, &bad));
    bad = c; bad.accel_sps2 = 0;              MU_ASSERT(!stepper_init(&s, PORT_MOTOR_FWD, &bad));
    bad = c; bad.accel_sps2 = 10;             MU_ASSERT(!stepper_init(&s, PORT_MOTOR_FWD, &bad));   /* < 1 pas/s par tick */
    bad = c; bad.min_pos = 11;                MU_ASSERT(!stepper_init(&s, PORT_MOTOR_FWD, &bad));
    bad = c; bad.tick_hz = 0;                 MU_ASSERT(!stepper_init(&s, PORT_MOTOR_FWD, &bad));
    MU_ASSERT(!stepper_init(NULL, PORT_MOTOR_FWD, &c));
    MU_ASSERT(!stepper_init(&s, PORT_MOTOR_FWD, NULL));
    MU_ASSERT(!stepper_init(&s, PORT_MOTOR_COUNT, &c));
}

static void test_long_move_reaches_exact_position(void)
{
    setup();
    stepper_move_to(&s_st, 4000);
    uint32_t ticks = run_until_idle(200000);
    MU_ASSERT_INT(4000, stepper_position(&s_st));
    MU_ASSERT_INT(4000, fake_motor_phys[PORT_MOTOR_FWD]);        /* le "moteur" a vraiment bougé de 4000 pas */
    MU_ASSERT_INT(4000, fake_motor_pulses[PORT_MOTOR_FWD]);      /* ni pas perdu, ni pas en trop */
    /* temps théorique : d/vmax + vmax/a = 4000/3000 + 3000/6000 = 1,83 s = 36 700 ticks ; mesuré : 35 729.
     * Borne haute serrée (+5 %) : un freinage qui démarre trop tôt (décélération réelle a/2) donnerait ~41 700. */
    MU_ASSERT(ticks > 34000 && ticks < 37500);
    MU_ASSERT(!stepper_is_moving(&s_st));
}

/** La vitesse d'approche (min_speed) est tenue jusqu'à la dernière impulsion : le moteur ne rampe pas vers zéro. */
static void test_final_approach_keeps_minimum_speed(void)
{
    fake_reset();
    s_cfg = (stepper_cfg_t){ .tick_hz = TICK_HZ, .max_speed_sps = 3000, .min_speed_sps = 1000,
                             .accel_sps2 = 6000, .min_pos = -10000, .max_pos = 10000 };
    MU_ASSERT(stepper_init(&s_st, PORT_MOTOR_FWD, &s_cfg));
    stepper_enable(&s_st, true);
    stepper_move_to(&s_st, 2000);

    uint32_t tick = 0, last_pulse_tick = 0, last_interval = 0, pulses_seen = 0;
    while (tick < 400000 && stepper_is_moving(&s_st)) {
        stepper_tick(&s_st);
        tick++;
        if (fake_motor_pulses[PORT_MOTOR_FWD] != pulses_seen) {
            pulses_seen = fake_motor_pulses[PORT_MOTOR_FWD];
            if (last_pulse_tick != 0) {
                last_interval = tick - last_pulse_tick;
            }
            last_pulse_tick = tick;
        }
    }
    MU_ASSERT_INT(2000, stepper_position(&s_st));
    /* 1000 pas/s à 20 kHz = une impulsion tous les 20 ticks. Sans le plancher, l'intervalle final dépasse 50 ticks. */
    MU_ASSERT(last_interval >= 19 && last_interval <= 22);
}

static void test_speed_never_exceeds_max(void)
{
    setup();
    stepper_move_to(&s_st, 6000);
    uint32_t window_start = 0, window_pulses = 0, max_window = 0;
    for (uint32_t t = 0; t < 200000 && stepper_is_moving(&s_st); t++) {
        stepper_tick(&s_st);
        if (t - window_start >= TICK_HZ / 10) {                  /* fenêtre de 100 ms */
            uint32_t p = fake_motor_pulses[PORT_MOTOR_FWD];
            if (p - window_pulses > max_window) max_window = p - window_pulses;
            window_pulses = p;
            window_start = t;
        }
    }
    MU_ASSERT(max_window <= 301);          /* 3000 pas/s x 0,1 s, +1 d'arrondi */
    MU_ASSERT(max_window >= 290);          /* et la vitesse maximale est bien atteinte */
}

static void test_short_move_never_overshoots(void)
{
    for (int32_t d = 1; d <= 60; d++) {
        fake_reset();
        setup();
        stepper_move_to(&s_st, d);
        run_until_idle(100000);
        MU_ASSERT_INT(d, stepper_position(&s_st));
        MU_ASSERT_INT(d, fake_motor_pulses[PORT_MOTOR_FWD]);     /* aucun dépassement suivi d'un retour */
    }
}

static void test_negative_move_and_direction_pin(void)
{
    setup();
    stepper_move_to(&s_st, -500);
    run_until_idle(100000);
    MU_ASSERT_INT(-500, stepper_position(&s_st));
    MU_ASSERT(!fake_motor_dir_fill[PORT_MOTOR_FWD]);
    stepper_move_to(&s_st, 0);
    run_until_idle(100000);
    MU_ASSERT_INT(0, stepper_position(&s_st));
    MU_ASSERT(fake_motor_dir_fill[PORT_MOTOR_FWD]);
    MU_ASSERT_INT(1000, fake_motor_pulses[PORT_MOTOR_FWD]);
}

static void test_reversal_during_motion_brakes_then_returns(void)
{
    setup();
    stepper_move_to(&s_st, 5000);
    for (int i = 0; i < 20000; i++) stepper_tick(&s_st);          /* 1 s : en pleine vitesse */
    int32_t at_reversal = stepper_position(&s_st);
    MU_ASSERT(at_reversal > 1000 && at_reversal < 5000);
    stepper_move_to(&s_st, 0);
    run_until_idle(200000);
    MU_ASSERT_INT(0, stepper_position(&s_st));
    MU_ASSERT_INT(fake_motor_phys[PORT_MOTOR_FWD], stepper_position(&s_st));
    /* il a dû freiner (continuer un peu dans l'ancien sens) avant de revenir : plus d'impulsions que 2 x at_reversal */
    MU_ASSERT(fake_motor_pulses[PORT_MOTOR_FWD] > (uint32_t)(2 * at_reversal));
}

static void test_software_limits_clamp_target(void)
{
    setup();
    stepper_move_to(&s_st, 50000);
    MU_ASSERT_INT(10000, stepper_target(&s_st));
    stepper_move_to(&s_st, -50000);
    MU_ASSERT_INT(-10000, stepper_target(&s_st));
    stepper_set_limits(&s_st, 0, 100);
    MU_ASSERT_INT(0, stepper_target(&s_st));
    stepper_move_to(&s_st, 500);
    MU_ASSERT_INT(100, stepper_target(&s_st));
    stepper_set_limits(&s_st, 50, 10);                           /* incohérent : ignoré */
    MU_ASSERT_INT(100, stepper_target(&s_st));
}

static void test_stop_now_halts_without_ramp(void)
{
    setup();
    stepper_move_to(&s_st, 8000);
    for (int i = 0; i < 15000; i++) stepper_tick(&s_st);
    uint32_t before = fake_motor_pulses[PORT_MOTOR_FWD];
    int32_t pos = stepper_position(&s_st);
    stepper_stop_now(&s_st);
    for (int i = 0; i < 5000; i++) stepper_tick(&s_st);
    MU_ASSERT_INT(before, fake_motor_pulses[PORT_MOTOR_FWD]);    /* plus aucune impulsion */
    MU_ASSERT_INT(pos, stepper_position(&s_st));
    MU_ASSERT(!stepper_is_moving(&s_st));
}

static void test_disabled_driver_emits_nothing_and_forgets_order(void)
{
    setup();
    stepper_enable(&s_st, false);
    stepper_move_to(&s_st, 100);
    for (int i = 0; i < 5000; i++) stepper_tick(&s_st);
    MU_ASSERT_INT(0, fake_motor_pulses[PORT_MOTOR_FWD]);
    MU_ASSERT(!stepper_is_moving(&s_st));

    stepper_enable(&s_st, true);                                 /* se réactiver ne rejoue pas l'ancien ordre : */
    stepper_move_to(&s_st, 100);
    stepper_enable(&s_st, false);
    stepper_enable(&s_st, true);
    for (int i = 0; i < 5000; i++) stepper_tick(&s_st);
    MU_ASSERT_INT(0, fake_motor_pulses[PORT_MOTOR_FWD]);
    MU_ASSERT_INT(0, fake_motor_pulses_disabled[PORT_MOTOR_FWD]);
}

static void test_set_position_redefines_origin(void)
{
    setup();
    stepper_move_to(&s_st, 300);
    run_until_idle(100000);
    stepper_set_position(&s_st, 0);                              /* référencement sur une butée */
    MU_ASSERT_INT(0, stepper_position(&s_st));
    MU_ASSERT_INT(0, stepper_target(&s_st));
    stepper_move_to(&s_st, 100);
    run_until_idle(100000);
    MU_ASSERT_INT(100, stepper_position(&s_st));
    MU_ASSERT_INT(400, fake_motor_phys[PORT_MOTOR_FWD]);         /* le moteur n'a pas bougé lors du set_position */
}

static void test_set_max_speed_validates(void)
{
    setup();
    MU_ASSERT(stepper_set_max_speed(&s_st, 500));
    MU_ASSERT(!stepper_set_max_speed(&s_st, 50));                /* sous la vitesse d'approche */
    MU_ASSERT(!stepper_set_max_speed(&s_st, (int32_t)(TICK_HZ / 2u + 1)));
    stepper_move_to(&s_st, 2000);
    uint32_t ticks = run_until_idle(400000);
    MU_ASSERT(ticks > 2000 * TICK_HZ / 500);                     /* plus lent : au moins d/vmax secondes */
}

static void test_two_motors_are_independent(void)
{
    setup();
    stepper_t aft;
    MU_ASSERT(stepper_init(&aft, PORT_MOTOR_AFT, &s_cfg));
    stepper_enable(&aft, true);
    stepper_move_to(&s_st, 300);
    stepper_move_to(&aft, -200);
    for (int i = 0; i < 100000; i++) { stepper_tick(&s_st); stepper_tick(&aft); }
    MU_ASSERT_INT(300, fake_motor_phys[PORT_MOTOR_FWD]);
    MU_ASSERT_INT(-200, fake_motor_phys[PORT_MOTOR_AFT]);
}

int main(void)
{
    MU_RUN(test_init_rejects_invalid_config);
    MU_RUN(test_long_move_reaches_exact_position);
    MU_RUN(test_final_approach_keeps_minimum_speed);
    MU_RUN(test_speed_never_exceeds_max);
    MU_RUN(test_short_move_never_overshoots);
    MU_RUN(test_negative_move_and_direction_pin);
    MU_RUN(test_reversal_during_motion_brakes_then_returns);
    MU_RUN(test_software_limits_clamp_target);
    MU_RUN(test_stop_now_halts_without_ramp);
    MU_RUN(test_disabled_driver_emits_nothing_and_forgets_order);
    MU_RUN(test_set_position_redefines_origin);
    MU_RUN(test_set_max_speed_validates);
    MU_RUN(test_two_motors_are_independent);
    return MU_SUMMARY();
}
