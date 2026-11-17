/* Tests de la machine d'états de sécurité : transitions, causes d'urgence, mémorisation, acquittement. */
#include "mini_tests.h"
#include "securite.h"

static safety_t s;
static safety_cfg_t cfg;
static safety_inputs_t in;

static void setup(void)
{
    safety_default_config(&cfg);
    safety_init(&s, &cfg, 1000);
    in = (safety_inputs_t){ .failsafe_mask = 0, .depth_ref_valid = true, .depth_m = 0.0f, .ballasts_ready = true,
                            .ballast_fault = false, .homing_done = true, .now_ms = 1000 };
}

static void step(uint32_t dt_ms)
{
    in.now_ms += dt_ms;
    safety_update(&s, &in);
}

static void go_idle(void)  { setup(); step(10); MU_ASSERT_INT(MODE_IDLE, safety_mode(&s)); }

static void go_manual(void)
{
    go_idle();
    safety_link_activity(&s, in.now_ms);
    MU_ASSERT_INT(SAFE_REQ_OK, safety_request_mode(&s, OP_MANUAL, &in));
}

static void go_hold(void)
{
    go_idle();
    safety_link_activity(&s, in.now_ms);
    MU_ASSERT_INT(SAFE_REQ_OK, safety_request_mode(&s, OP_DEPTH_HOLD, &in));
}

static void test_init_waits_for_homing_and_tare(void)
{
    setup();
    in.homing_done = false;
    step(100);
    MU_ASSERT_INT(MODE_INIT, safety_mode(&s));
    in.homing_done = true; in.depth_ref_valid = false;
    step(100);
    MU_ASSERT_INT(MODE_INIT, safety_mode(&s));                  /* tare pas encore faite */
    in.depth_ref_valid = true;
    step(100);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s));
}

static void test_init_times_out_even_if_things_failed(void)
{
    setup();
    in.homing_done = false; in.depth_ref_valid = false;
    step(cfg.init_timeout_ms - 10);
    MU_ASSERT_INT(MODE_INIT, safety_mode(&s));
    step(20);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s));                  /* IDLE, mais plongée impossible : voir les rejets ci-dessous */
    in.ballasts_ready = false;
    MU_ASSERT_INT(SAFE_REQ_REJECTED, safety_request_mode(&s, OP_MANUAL, &in));
}

static void test_requests_in_init_are_rejected(void)
{
    setup();
    MU_ASSERT_INT(SAFE_REQ_REJECTED, safety_request_mode(&s, OP_MANUAL, &in));
}

static void test_mode_requests_rules(void)
{
    go_idle();
    MU_ASSERT_INT(SAFE_REQ_INVALID, safety_request_mode(&s, 3, &in));
    MU_ASSERT_INT(SAFE_REQ_INVALID, safety_request_mode(&s, -1, &in));
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s));

    in.ballasts_ready = false;
    MU_ASSERT_INT(SAFE_REQ_REJECTED, safety_request_mode(&s, OP_MANUAL, &in));
    MU_ASSERT_INT(SAFE_REQ_REJECTED, safety_request_mode(&s, OP_DEPTH_HOLD, &in));
    in.ballasts_ready = true;

    in.depth_ref_valid = false;
    MU_ASSERT_INT(SAFE_REQ_REJECTED, safety_request_mode(&s, OP_DEPTH_HOLD, &in));   /* pas de tare : pas de régulation */
    MU_ASSERT_INT(SAFE_REQ_OK, safety_request_mode(&s, OP_MANUAL, &in));             /* mais le manuel reste possible */
    in.depth_ref_valid = true;

    in.failsafe_mask = FS_DEPTH_LOST;
    MU_ASSERT_INT(SAFE_REQ_REJECTED, safety_request_mode(&s, OP_DEPTH_HOLD, &in));
    in.failsafe_mask = 0;
    in.depth_m = NAN;
    MU_ASSERT_INT(SAFE_REQ_REJECTED, safety_request_mode(&s, OP_DEPTH_HOLD, &in));
    in.depth_m = 0.0f;

    in.failsafe_mask = FS_LEAK;                                 /* une cause d'urgence présente : aucune plongée */
    MU_ASSERT_INT(SAFE_REQ_REJECTED, safety_request_mode(&s, OP_MANUAL, &in));
    in.failsafe_mask = FS_BATT_WARN;                            /* batterie faible : on peut encore piloter */
    MU_ASSERT_INT(SAFE_REQ_OK, safety_request_mode(&s, OP_DEPTH_HOLD, &in));
    MU_ASSERT_INT(MODE_DEPTH_HOLD, safety_mode(&s));
    MU_ASSERT_INT(SAFE_REQ_OK, safety_request_mode(&s, OP_MANUAL, &in));
    MU_ASSERT_INT(SAFE_REQ_OK, safety_request_mode(&s, OP_IDLE, &in));
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s));
}

static void test_hard_causes_trigger_emergency_in_any_mode(void)
{
    const uint32_t causes[] = { FS_LEAK, FS_BATT_CRIT, FS_OVER_DEPTH };
    for (unsigned i = 0; i < 3; i++) {
        go_idle();                                              /* même en IDLE : on mémorise l'alarme */
        in.failsafe_mask = causes[i];
        step(10);
        MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));
        MU_ASSERT_INT(causes[i], safety_reasons(&s));
    }
    setup();                                                    /* et pendant l'initialisation */
    in.failsafe_mask = FS_LEAK; in.homing_done = false;
    step(10);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));
}

static void test_warnings_do_not_trigger_emergency(void)
{
    go_hold();
    in.failsafe_mask = FS_BATT_WARN | FS_IMU_LOST;
    step(10);
    MU_ASSERT_INT(MODE_DEPTH_HOLD, safety_mode(&s));
    MU_ASSERT_INT(0, safety_reasons(&s));
}

static void test_depth_lost_matters_only_when_actively_diving(void)
{
    go_idle();
    in.failsafe_mask = FS_DEPTH_LOST;
    step(10);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s));                  /* ballasts vides : déjà dans l'état le plus sûr */
    go_hold();
    in.failsafe_mask = FS_DEPTH_LOST;
    step(10);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));
    go_manual();
    in.failsafe_mask = FS_DEPTH_LOST;
    step(10);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));
}

static void test_link_loss_when_submerged_triggers_emergency(void)
{
    go_hold();
    in.depth_m = 1.0f;
    safety_link_activity(&s, in.now_ms);
    step(cfg.link_timeout_ms);                                  /* exactement le délai : pas encore */
    MU_ASSERT_INT(MODE_DEPTH_HOLD, safety_mode(&s));
    step(1);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));
    MU_ASSERT(safety_reasons(&s) & SAFE_LINK_LOST);
}

static void test_link_activity_resets_the_timeout(void)
{
    go_hold();
    in.depth_m = 1.0f;
    for (int i = 0; i < 20; i++) {
        step(1000);
        safety_link_activity(&s, in.now_ms);                    /* un message par seconde : jamais de coupure */
    }
    MU_ASSERT_INT(MODE_DEPTH_HOLD, safety_mode(&s));
}

static void test_link_loss_at_surface_is_not_an_emergency(void)
{
    go_manual();
    in.depth_m = 0.05f;
    step(60000);
    MU_ASSERT_INT(MODE_MANUAL, safety_mode(&s));
    in.depth_m = NAN;                                           /* profondeur inconnue : on suppose le pire */
    step(10);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));
}

static void test_link_loss_is_irrelevant_in_idle(void)
{
    go_idle();
    in.depth_m = 2.0f;
    step(60000);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s));
}

static void test_link_timeout_survives_clock_wraparound(void)
{
    setup();
    in.now_ms = 0xFFFFFF00u;
    s.start_ms = in.now_ms; s.last_link_ms = in.now_ms;
    step(10);
    safety_link_activity(&s, in.now_ms);
    MU_ASSERT_INT(SAFE_REQ_OK, safety_request_mode(&s, OP_MANUAL, &in));
    in.depth_m = 1.0f;
    step(2000);                                                 /* l'horloge passe par zéro : 2 s écoulées, pas 49 jours */
    MU_ASSERT_INT(MODE_MANUAL, safety_mode(&s));
    step(1500);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));
}

static void test_ballast_fault_matters_when_diving(void)
{
    go_idle();
    in.ballast_fault = true;
    step(10);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s));
    go_manual();
    in.ballast_fault = true;
    step(10);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));
    MU_ASSERT(safety_reasons(&s) & SAFE_BALLAST_FAULT);
}

static void test_emergency_is_latched_until_cleared(void)
{
    go_hold();
    in.failsafe_mask = FS_BATT_CRIT;
    step(10);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));
    in.failsafe_mask = 0;                                       /* la cause disparaît (batterie remise, contact...) */
    step(10000);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));             /* mais on reste en urgence */
    MU_ASSERT_INT(SAFE_REQ_REJECTED, safety_request_mode(&s, OP_IDLE, &in));
    MU_ASSERT_INT(SAFE_REQ_REJECTED, safety_request_mode(&s, OP_MANUAL, &in));
    MU_ASSERT_INT(SAFE_REQ_INVALID, safety_request_mode(&s, 3, &in));    /* un mode inexistant est invalide meme en urgence */
    MU_ASSERT_INT(SAFE_REQ_INVALID, safety_request_mode(&s, -1, &in));
    safety_clear_fault(&s);
    step(10);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s));
    MU_ASSERT_INT(0, safety_reasons(&s));
}

static void test_clear_is_refused_while_cause_persists(void)
{
    go_idle();
    in.failsafe_mask = FS_LEAK;
    step(10);
    safety_clear_fault(&s);
    step(10);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));             /* toujours de l'eau : on ne sort pas */
    MU_ASSERT_INT(FS_LEAK, safety_reasons(&s));
    in.failsafe_mask = 0;
    safety_clear_fault(&s);
    step(10);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s));
}

static void test_clear_request_does_not_persist(void)
{
    go_idle();
    safety_clear_fault(&s);                                     /* acquittement sans urgence : ignoré... */
    step(10);
    in.failsafe_mask = FS_LEAK;                                 /* ... et il ne doit pas annuler une urgence future */
    step(10);
    in.failsafe_mask = 0;
    step(10);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));
}

static void test_operator_emergency_stop(void)
{
    go_hold();
    safety_emergency_stop(&s);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));             /* immédiat, sans attendre le cycle */
    step(10);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));
    MU_ASSERT(safety_reasons(&s) & SAFE_OPERATOR);
    safety_clear_fault(&s);
    step(10);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s));
}

static void test_link_lost_reason_not_kept_after_reconnect_and_clear(void)
{
    go_hold();
    in.depth_m = 1.0f;
    step(cfg.link_timeout_ms + 100);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s));
    safety_link_activity(&s, in.now_ms);                        /* la liaison revient */
    safety_clear_fault(&s);
    step(10);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s));
}

static void test_mode_names(void)
{
    MU_ASSERT(safety_mode_name(MODE_EMERGENCY)[0] == 'E');
    MU_ASSERT(safety_mode_name((sub_mode_t)99)[0] == '?');
}

int main(void)
{
    MU_RUN(test_init_waits_for_homing_and_tare);
    MU_RUN(test_init_times_out_even_if_things_failed);
    MU_RUN(test_requests_in_init_are_rejected);
    MU_RUN(test_mode_requests_rules);
    MU_RUN(test_hard_causes_trigger_emergency_in_any_mode);
    MU_RUN(test_warnings_do_not_trigger_emergency);
    MU_RUN(test_depth_lost_matters_only_when_actively_diving);
    MU_RUN(test_link_loss_when_submerged_triggers_emergency);
    MU_RUN(test_link_activity_resets_the_timeout);
    MU_RUN(test_link_loss_at_surface_is_not_an_emergency);
    MU_RUN(test_link_loss_is_irrelevant_in_idle);
    MU_RUN(test_link_timeout_survives_clock_wraparound);
    MU_RUN(test_ballast_fault_matters_when_diving);
    MU_RUN(test_emergency_is_latched_until_cleared);
    MU_RUN(test_clear_is_refused_while_cause_persists);
    MU_RUN(test_clear_request_does_not_persist);
    MU_RUN(test_operator_emergency_stop);
    MU_RUN(test_link_lost_reason_not_kept_after_reconnect_and_clear);
    MU_RUN(test_mode_names);
    return MU_SUMMARY();
}
