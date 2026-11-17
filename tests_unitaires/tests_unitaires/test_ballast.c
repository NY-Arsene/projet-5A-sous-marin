/* Tests d'un ballast : référencement, conversion mL/pas, butées, recalage sur pas perdus, urgence. */
#include "mini_tests.h"
#include "ballast.h"
#include <stdlib.h>

#define EMPTY_IDX 0
#define FULL_IDX  1
#define MAX_STEPS 56600            /* 100 mL x 566 pas/mL */

static ballast_t     s_b;
static ballast_cfg_t s_cfg;
static bool          s_limit[4];
static bool          s_empty_stuck_on, s_empty_dead;      /* défauts de butée simulés */

/* Modèle physique des butées : actives à l'approche du bout de course, relâchées 50 pas plus loin (hystérésis). */
static void update_switches(void)
{
    int32_t p = fake_motor_phys[PORT_MOTOR_FWD];
    if (p <= 0) s_limit[EMPTY_IDX] = true; else if (p >= 50) s_limit[EMPTY_IDX] = false;
    if (p >= MAX_STEPS) s_limit[FULL_IDX] = true; else if (p <= MAX_STEPS - 50) s_limit[FULL_IDX] = false;
    if (s_empty_stuck_on) s_limit[EMPTY_IDX] = true;
    if (s_empty_dead)     s_limit[EMPTY_IDX] = false;
}

static uint32_t s_now;

static void setup_with_phys(int32_t start_phys)
{
    s_cfg = (ballast_cfg_t){0};
    ballast_default_config(&s_cfg);
    MU_ASSERT(ballast_init(&s_b, PORT_MOTOR_FWD, EMPTY_IDX, FULL_IDX, &s_cfg));
    fake_motor_phys[PORT_MOTOR_FWD] = start_phys;
    fake_motor_phys_min[PORT_MOTOR_FWD] = -300;                 /* butée mécanique juste derrière l'interrupteur */
    fake_motor_phys_max[PORT_MOTOR_FWD] = MAX_STEPS + 300;
    for (int i = 0; i < 4; i++) s_limit[i] = false;
    s_empty_stuck_on = s_empty_dead = false;
    s_now = 1000;
    update_switches();
}

static void run_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        for (int t = 0; t < 20; t++) ballast_tick(&s_b);        /* 20 kHz */
        update_switches();
        s_now++;
        if (s_now % 10 == 0) ballast_update(&s_b, s_limit, s_now);   /* 100 Hz */
    }
}

static void test_init_validation(void)
{
    ballast_t b; ballast_cfg_t c; ballast_default_config(&c);
    MU_ASSERT(ballast_init(&b, PORT_MOTOR_FWD, 0, 1, &c));
    MU_ASSERT(!ballast_init(&b, PORT_MOTOR_FWD, 4, 1, &c));
    MU_ASSERT(!ballast_init(&b, PORT_MOTOR_FWD, 0, 4, &c));
    MU_ASSERT(!ballast_init(&b, PORT_MOTOR_FWD, 2, 2, &c));
    ballast_cfg_t bad = c; bad.steps_per_ml = 0;                  MU_ASSERT(!ballast_init(&b, PORT_MOTOR_FWD, 0, 1, &bad));
    bad = c; bad.steps_per_ml = NAN;                              MU_ASSERT(!ballast_init(&b, PORT_MOTOR_FWD, 0, 1, &bad));
    bad = c; bad.max_volume_ml = -1;                              MU_ASSERT(!ballast_init(&b, PORT_MOTOR_FWD, 0, 1, &bad));
    bad = c; bad.homing_speed_sps = c.max_speed_sps + 1;          MU_ASSERT(!ballast_init(&b, PORT_MOTOR_FWD, 0, 1, &bad));
    bad = c; bad.homing_timeout_ms = 0;                           MU_ASSERT(!ballast_init(&b, PORT_MOTOR_FWD, 0, 1, &bad));
    bad = c; bad.steps_per_ml = 1.0e9f;                           MU_ASSERT(!ballast_init(&b, PORT_MOTOR_FWD, 0, 1, &bad));
    MU_ASSERT(!ballast_init(NULL, PORT_MOTOR_FWD, 0, 1, &c));
    MU_ASSERT(!ballast_init(&b, PORT_MOTOR_FWD, 0, 1, NULL));
}

static void test_unhomed_refuses_volume_commands(void)
{
    setup_with_phys(5000);
    MU_ASSERT_INT(BALLAST_UNHOMED, ballast_state(&s_b));
    MU_ASSERT(!ballast_set_volume_ml(&s_b, 10.0f));
    run_ms(500);
    MU_ASSERT_INT(0, fake_motor_pulses[PORT_MOTOR_FWD]);          /* et le moteur ne bouge pas tout seul */
}

static void test_homing_from_mid_travel(void)
{
    setup_with_phys(5000);
    ballast_home_start(&s_b, s_now);
    MU_ASSERT_INT(BALLAST_HOMING, ballast_state(&s_b));
    run_ms(15000);
    MU_ASSERT_INT(BALLAST_READY, ballast_state(&s_b));
    MU_ASSERT_NEAR(0.0, ballast_volume_ml(&s_b), 0.05);
    MU_ASSERT(abs(fake_motor_phys[PORT_MOTOR_FWD]) < 30);         /* arrêté au contact de la butée */
    MU_ASSERT(!ballast_is_moving(&s_b));
    MU_ASSERT_INT(0, fake_motor_pulses_disabled[PORT_MOTOR_FWD]);
}

static void test_homing_when_already_on_the_switch_backs_off_first(void)
{
    setup_with_phys(-100);                                        /* déjà dans la zone active de la butée */
    update_switches();
    MU_ASSERT(s_limit[EMPTY_IDX]);
    ballast_home_start(&s_b, s_now);
    run_ms(3000);
    MU_ASSERT_INT(BALLAST_READY, ballast_state(&s_b));
    MU_ASSERT(abs(fake_motor_phys[PORT_MOTOR_FWD]) < 30);         /* référence prise sur le front d'approche */
    MU_ASSERT_NEAR(0.0, ballast_volume_ml(&s_b), 0.05);
}

static void test_homing_fails_if_switch_never_triggers(void)
{
    setup_with_phys(2000);
    s_cfg.homing_timeout_ms = 5000;
    MU_ASSERT(ballast_init(&s_b, PORT_MOTOR_FWD, EMPTY_IDX, FULL_IDX, &s_cfg));
    fake_motor_phys[PORT_MOTOR_FWD] = 2000;
    s_empty_dead = true;                                          /* butée débranchée : ne s'active jamais */
    ballast_home_start(&s_b, s_now);
    run_ms(4000);
    MU_ASSERT_INT(BALLAST_HOMING, ballast_state(&s_b));
    run_ms(2500);
    MU_ASSERT_INT(BALLAST_FAULT, ballast_state(&s_b));
    MU_ASSERT(!fake_motor_enabled[PORT_MOTOR_FWD]);               /* moteur coupé après l'échec */
    MU_ASSERT(!ballast_set_volume_ml(&s_b, 10.0f));
}

static void test_homing_fails_if_switch_stuck_active(void)
{
    setup_with_phys(1000);
    s_empty_stuck_on = true;                                      /* butée collée : jamais relâchée pendant le recul */
    ballast_home_start(&s_b, s_now);
    run_ms(3000);
    MU_ASSERT_INT(BALLAST_FAULT, ballast_state(&s_b));
}

static void homed(void)
{
    setup_with_phys(3000);
    ballast_home_start(&s_b, s_now);
    run_ms(8000);
    MU_ASSERT_INT(BALLAST_READY, ballast_state(&s_b));
}

static void test_volume_conversion_and_accuracy(void)
{
    homed();
    int32_t base = fake_motor_phys[PORT_MOTOR_FWD];
    MU_ASSERT(ballast_set_volume_ml(&s_b, 50.0f));
    MU_ASSERT_NEAR(50.0, ballast_target_ml(&s_b), 0.01);
    run_ms(8000);
    MU_ASSERT_NEAR(50.0, ballast_volume_ml(&s_b), 0.01);
    MU_ASSERT_INT(28300, fake_motor_phys[PORT_MOTOR_FWD] - base);  /* 50 mL x 566 pas/mL */
    MU_ASSERT(!ballast_is_moving(&s_b));
}

static void test_volume_is_clamped_and_bad_values_refused(void)
{
    homed();
    MU_ASSERT(ballast_set_volume_ml(&s_b, -5.0f));
    MU_ASSERT_NEAR(0.0, ballast_target_ml(&s_b), 1e-3);
    MU_ASSERT(ballast_set_volume_ml(&s_b, 250.0f));
    MU_ASSERT_NEAR(100.0, ballast_target_ml(&s_b), 1e-3);
    /* Valeurs énormes : doivent être bornées AVANT la conversion en pas (sinon débordement de l'entier). */
    const float huge[] = { 1.0e4f, 1.0e7f, 1.0e9f, 1.0e12f, 1.0e30f };
    for (unsigned i = 0; i < sizeof huge / sizeof huge[0]; i++) {
        MU_ASSERT(ballast_set_volume_ml(&s_b, huge[i]));
        MU_ASSERT_NEAR(100.0, ballast_target_ml(&s_b), 1e-3);
        MU_ASSERT(ballast_set_volume_ml(&s_b, -huge[i]));
        MU_ASSERT_NEAR(0.0, ballast_target_ml(&s_b), 1e-3);
    }
    MU_ASSERT(ballast_set_volume_ml(&s_b, 250.0f));
    float before = ballast_target_ml(&s_b);
    MU_ASSERT(!ballast_set_volume_ml(&s_b, NAN));
    MU_ASSERT(!ballast_set_volume_ml(&s_b, INFINITY));
    MU_ASSERT_NEAR(before, ballast_target_ml(&s_b), 1e-3);        /* cible inchangée */
}

static void test_leaving_and_returning_to_empty_causes_no_false_resync(void)
{
    homed();
    MU_ASSERT(ballast_set_volume_ml(&s_b, 10.0f));
    run_ms(4000);
    MU_ASSERT(ballast_set_volume_ml(&s_b, 0.0f));
    run_ms(4000);
    MU_ASSERT_NEAR(0.0, ballast_volume_ml(&s_b), 0.05);
    MU_ASSERT_INT(0, s_b.resync_count);
}

static void test_lost_steps_are_detected_by_full_switch(void)
{
    homed();
    MU_ASSERT(ballast_set_volume_ml(&s_b, 20.0f));
    run_ms(4000);
    fake_motor_phys[PORT_MOTOR_FWD] += 1500;                      /* le piston a glissé : l'estimation est fausse de 1500 pas */
    MU_ASSERT(ballast_set_volume_ml(&s_b, 100.0f));
    run_ms(15000);
    MU_ASSERT_INT(1, s_b.resync_count);                           /* la butée "plein" a contredit l'estimation */
    MU_ASSERT_NEAR(100.0, ballast_volume_ml(&s_b), 0.05);         /* position recalée sur la réalité */
    MU_ASSERT(!ballast_is_moving(&s_b));
    MU_ASSERT(fake_motor_phys[PORT_MOTOR_FWD] < MAX_STEPS + 100); /* et on n'a pas forcé contre la butée mécanique */
}

static void test_emergency_empty_from_ready(void)
{
    homed();
    MU_ASSERT(ballast_set_volume_ml(&s_b, 80.0f));
    run_ms(5000);
    ballast_emergency_empty(&s_b, s_now);
    run_ms(12000);
    MU_ASSERT_NEAR(0.0, ballast_volume_ml(&s_b), 0.05);
    MU_ASSERT(ballast_is_ready(&s_b));
}

static void test_emergency_empty_when_position_unknown_rehomes(void)
{
    setup_with_phys(30000);
    MU_ASSERT_INT(BALLAST_UNHOMED, ballast_state(&s_b));
    for (int i = 0; i < 3; i++) {
        ballast_emergency_empty(&s_b, s_now);                     /* appels répétés à chaque cycle : sans effet de bord */
        run_ms(10);
    }
    uint32_t start = s_b.homing_start_ms;
    run_ms(2000);
    ballast_emergency_empty(&s_b, s_now);
    MU_ASSERT_INT(start, s_b.homing_start_ms);                    /* le délai de référencement n'est pas relancé */
    run_ms(40000);
    MU_ASSERT_INT(BALLAST_READY, ballast_state(&s_b));
    MU_ASSERT(abs(fake_motor_phys[PORT_MOTOR_FWD]) < 30);
}

static void test_emergency_after_fault_retries_homing(void)
{
    setup_with_phys(2000);
    s_cfg.homing_timeout_ms = 2000;
    MU_ASSERT(ballast_init(&s_b, PORT_MOTOR_FWD, EMPTY_IDX, FULL_IDX, &s_cfg));
    fake_motor_phys[PORT_MOTOR_FWD] = 2000;
    s_empty_dead = true;
    ballast_home_start(&s_b, s_now);
    run_ms(3000);
    MU_ASSERT_INT(BALLAST_FAULT, ballast_state(&s_b));
    s_empty_dead = false;                                         /* réparé */
    ballast_emergency_empty(&s_b, s_now);
    MU_ASSERT_INT(BALLAST_HOMING, ballast_state(&s_b));
    run_ms(10000);
    MU_ASSERT_INT(BALLAST_READY, ballast_state(&s_b));
}

int main(void)
{
    MU_RUN(test_init_validation);
    MU_RUN(test_unhomed_refuses_volume_commands);
    MU_RUN(test_homing_from_mid_travel);
    MU_RUN(test_homing_when_already_on_the_switch_backs_off_first);
    MU_RUN(test_homing_fails_if_switch_never_triggers);
    MU_RUN(test_homing_fails_if_switch_stuck_active);
    MU_RUN(test_volume_conversion_and_accuracy);
    MU_RUN(test_volume_is_clamped_and_bad_values_refused);
    MU_RUN(test_leaving_and_returning_to_empty_causes_no_false_resync);
    MU_RUN(test_lost_steps_are_detected_by_full_switch);
    MU_RUN(test_emergency_empty_from_ready);
    MU_RUN(test_emergency_empty_when_position_unknown_rehomes);
    MU_RUN(test_emergency_after_fault_retries_homing);
    return MU_SUMMARY();
}
