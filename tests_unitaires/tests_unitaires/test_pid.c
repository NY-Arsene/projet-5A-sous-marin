/* Tests du régulateur PID : proportionnel, intégral, dérivé, saturation, anti-emballement, entrées invalides. */
#include "mini_tests.h"
#include "pid.h"

static pid_ctrl_t p;

static void test_init_validation(void)
{
    MU_ASSERT(pid_init(&p, 1, 0.1f, 0.5f, -10, 10));
    MU_ASSERT(!pid_init(&p, -1, 0, 0, -10, 10));
    MU_ASSERT(!pid_init(&p, NAN, 0, 0, -10, 10));
    MU_ASSERT(!pid_init(&p, 1, INFINITY, 0, -10, 10));
    MU_ASSERT(!pid_init(&p, 1, 0, 0, 10, -10));
    MU_ASSERT(!pid_init(&p, 1, 0, 0, 5, 5));
    MU_ASSERT(!pid_init(&p, 1, 0, 0, NAN, 5));
    MU_ASSERT(!pid_init(NULL, 1, 0, 0, -1, 1));
    MU_ASSERT(pid_init(&p, 1, 0.1f, 0.5f, -10, 10));
    MU_ASSERT(!pid_set_gains(&p, -2, 0, 0));
    MU_ASSERT_NEAR(1.0, p.kp, 1e-9);                       /* gains inchangés après un refus */
    MU_ASSERT(pid_set_gains(&p, 2, 0, 0));
    MU_ASSERT_NEAR(2.0, p.kp, 1e-9);
}

static void test_proportional_only(void)
{
    pid_init(&p, 4.0f, 0, 0, -100, 100);
    MU_ASSERT_NEAR(8.0, pid_update(&p, 3.0f, 1.0f, 0, 0.01f), 1e-5);      /* 4 x (3-1) */
    MU_ASSERT_NEAR(-8.0, pid_update(&p, 1.0f, 3.0f, 0, 0.01f), 1e-5);
    MU_ASSERT_NEAR(0.0, pid_update(&p, 2.0f, 2.0f, 0, 0.01f), 1e-5);
}

static void test_derivative_on_measured_rate_opposes_motion(void)
{
    pid_init(&p, 0, 0, 10.0f, -100, 100);
    MU_ASSERT_NEAR(-5.0, pid_update(&p, 0.0f, 0.0f, 0.5f, 0.01f), 1e-5);  /* descend à 0,5 m/s : on freine */
    MU_ASSERT_NEAR(5.0, pid_update(&p, 0.0f, 0.0f, -0.5f, 0.01f), 1e-5);
}

static void test_setpoint_step_causes_no_derivative_kick(void)
{
    pid_init(&p, 2.0f, 0, 50.0f, -1000, 1000);
    float a = pid_update(&p, 0.0f, 0.0f, 0.0f, 0.01f);
    float b = pid_update(&p, 1.0f, 0.0f, 0.0f, 0.01f);                    /* saut de consigne, vitesse mesurée nulle */
    MU_ASSERT_NEAR(0.0, a, 1e-5);
    MU_ASSERT_NEAR(2.0, b, 1e-5);                                         /* P seul : pas de pic dû à D */
}

static void test_integral_accumulates_and_removes_steady_error(void)
{
    pid_init(&p, 0, 2.0f, 0, -100, 100);
    float out = 0;
    for (int i = 0; i < 100; i++) out = pid_update(&p, 1.0f, 0.0f, 0, 0.01f);   /* erreur 1 pendant 1 s */
    MU_ASSERT_NEAR(2.0, out, 1e-3);                                       /* ki x erreur x temps */
    for (int i = 0; i < 100; i++) out = pid_update(&p, 0.0f, 1.0f, 0, 0.01f);   /* erreur -1 pendant 1 s */
    MU_ASSERT_NEAR(0.0, out, 1e-3);
}

static void test_integral_only_acts_inside_its_zone(void)
{
    pid_init(&p, 0, 1.0f, 0, -100, 100);
    MU_ASSERT(pid_set_i_zone(&p, 0.5f));
    for (int i = 0; i < 500; i++) pid_update(&p, 2.0f, 0.0f, 0, 0.01f);        /* erreur 2 : hors zone, rien ne s'accumule */
    MU_ASSERT_NEAR(0.0, p.i_term, 1e-9);
    for (int i = 0; i < 100; i++) pid_update(&p, 0.4f, 0.0f, 0, 0.01f);        /* erreur 0,4 : dans la zone */
    MU_ASSERT_NEAR(0.4, p.i_term, 1e-3);
    MU_ASSERT(!pid_set_i_zone(&p, -1.0f));
    MU_ASSERT(!pid_set_i_zone(&p, NAN));
    MU_ASSERT(pid_set_i_zone(&p, 0.0f));                                       /* 0 : toujours active */
    for (int i = 0; i < 100; i++) pid_update(&p, 2.0f, 0.0f, 0, 0.01f);
    MU_ASSERT(p.i_term > 2.0f);
}

static void test_output_is_clamped(void)
{
    pid_init(&p, 100.0f, 0, 0, -5, 7);
    MU_ASSERT_NEAR(7.0, pid_update(&p, 10.0f, 0.0f, 0, 0.01f), 1e-6);
    MU_ASSERT_NEAR(-5.0, pid_update(&p, -10.0f, 0.0f, 0, 0.01f), 1e-6);
}

static void test_anti_windup_recovers_quickly_after_saturation(void)
{
    pid_init(&p, 1.0f, 1.0f, 0, -10, 10);
    for (int i = 0; i < 6000; i++) pid_update(&p, 100.0f, 0.0f, 0, 0.01f);     /* 60 s collé à la saturation */
    MU_ASSERT(p.i_term <= 10.0f + 1e-6);                                       /* l'intégrale n'a pas explosé */
    int n = 0;
    float out = 10;
    while (out > 0.0f && n < 3000) { out = pid_update(&p, -1.0f, 0.0f, 0, 0.01f); n++; }
    MU_ASSERT(n < 200);                                                         /* repasse sous zéro en moins de 2 s */
}

static void test_derivative_term_counts_in_the_saturation_check(void)
{
    pid_init(&p, 1.0f, 1.0f, 10.0f, -10, 10);
    /* erreur +5, le terme dérivé pousse la sortie à +25 (> 10) : saturée du côté de l'erreur, l'intégrale ne doit pas grossir */
    pid_update(&p, 5.0f, 0.0f, -2.0f, 0.01f);
    MU_ASSERT_NEAR(0.0, p.i_term, 1e-9);
}

static void test_integral_stays_bounded_even_if_derivative_hides_saturation(void)
{
    pid_init(&p, 0.0f, 1.0f, 100.0f, -10, 10);
    /* -kd x vitesse = -100 : la sortie reste dans la plage, l'anti-emballement ne bloque pas ; seule la borne de l'intégrale protège */
    for (int i = 0; i < 3000; i++) pid_update(&p, 1.0f, 0.0f, 1.0f, 0.01f);   /* l'intégrale voudrait atteindre 30 */
    MU_ASSERT(p.i_term <= 10.0f + 1e-6f);
}

static void test_invalid_inputs_keep_previous_output_and_state(void)
{
    pid_init(&p, 2.0f, 1.0f, 0, -100, 100);
    float good = pid_update(&p, 5.0f, 0.0f, 0, 0.01f);
    float i_before = p.i_term;
    MU_ASSERT_NEAR(good, pid_update(&p, NAN, 0.0f, 0, 0.01f), 1e-9);
    MU_ASSERT_NEAR(good, pid_update(&p, 5.0f, INFINITY, 0, 0.01f), 1e-9);
    MU_ASSERT_NEAR(good, pid_update(&p, 5.0f, 0.0f, NAN, 0.01f), 1e-9);
    MU_ASSERT_NEAR(good, pid_update(&p, 5.0f, 0.0f, 0, 0.0f), 1e-9);          /* dt nul */
    MU_ASSERT_NEAR(good, pid_update(&p, 5.0f, 0.0f, 0, -0.01f), 1e-9);
    MU_ASSERT_NEAR(good, pid_update(&p, 5.0f, 0.0f, 0, 5.0f), 1e-9);          /* dt énorme : la boucle a été bloquée */
    MU_ASSERT_NEAR(i_before, p.i_term, 1e-9);                                  /* l'état n'a pas bougé */
}

static void test_reset_sets_integral_clamped(void)
{
    pid_init(&p, 1.0f, 1.0f, 0, -10, 10);
    pid_reset(&p, 4.0f);
    MU_ASSERT_NEAR(4.0, p.i_term, 1e-9);
    MU_ASSERT_NEAR(4.0, pid_update(&p, 0.0f, 0.0f, 0, 0.01f), 1e-9);          /* sortie = valeur de départ : pas d'à-coup */
    pid_reset(&p, 50.0f);
    MU_ASSERT_NEAR(10.0, p.i_term, 1e-9);
    pid_reset(&p, NAN);
    MU_ASSERT_NEAR(0.0, p.i_term, 1e-9);
}

int main(void)
{
    MU_RUN(test_init_validation);
    MU_RUN(test_proportional_only);
    MU_RUN(test_derivative_on_measured_rate_opposes_motion);
    MU_RUN(test_setpoint_step_causes_no_derivative_kick);
    MU_RUN(test_integral_accumulates_and_removes_steady_error);
    MU_RUN(test_integral_only_acts_inside_its_zone);
    MU_RUN(test_output_is_clamped);
    MU_RUN(test_anti_windup_recovers_quickly_after_saturation);
    MU_RUN(test_derivative_term_counts_in_the_saturation_check);
    MU_RUN(test_integral_stays_bounded_even_if_derivative_hides_saturation);
    MU_RUN(test_invalid_inputs_keep_previous_output_and_state);
    MU_RUN(test_reset_sets_integral_clamped);
    return MU_SUMMARY();
}
