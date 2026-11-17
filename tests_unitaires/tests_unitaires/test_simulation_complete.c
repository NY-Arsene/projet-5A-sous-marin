/*
 * Tests de l'application complète sur un SOUS-MARIN SIMULÉ en boucle fermée.
 *
 * Le simulateur modélise : la poussée due au volume d'eau dans les deux ballasts (position PHYSIQUE des pistons,
 * pas l'estimation du firmware), la traînée quadratique, la surface, le tangage, les fins de course mécaniques,
 * les capteurs (pression, IMU) alimentés par les faux circuits, et la Raspberry Pi (commandes + battement de coeur).
 * Le firmware complet (app_init / app_step / app_tick) tourne tel quel, sans aucune modification pour le test.
 *
 * Les valeurs du modèle (masse, traînée) sont plausibles pour un petit sous-marin mais INVENTÉES : ces tests
 * prouvent que la logique est cohérente (signes, sécurités, séquencement), pas que les gains conviennent au vrai bateau.
 */
#include "mini_tests.h"
#include "programme_principal.h"
#include "faux_composants.h"
#include <stdlib.h>
#include <string.h>

#define MBAR_PER_M   (997.0f * 9.80665f / 100.0f)
#define MAX_STEPS    56600                         /* 100 mL x 566 pas/mL */

/* ------------------------------------------------------------------ simulateur */
typedef struct {
    double depth, vel;                 /* m, m/s (+ vers le bas) */
    double pitch_deg;
    double neutral_total_ml;           /* somme des deux ballasts qui donne la flottabilité neutre */
    double pitch_bias_deg;             /* assiette imposée par un poids mal placé */
    bool   leak;
    bool   dead_empty_switch[2];       /* butée "vide" débranchée (ne s'active jamais) */
} sim_t;

static fake_sub_t s_sub;
static app_t      s_app;
static sim_t      s_sim;
static bool       s_heartbeat;
static uint32_t   s_next_hb;

/* ce que la "Raspberry Pi" voit */
typedef struct { uint8_t cmd[16]; uint8_t res[16]; int n_ack; telemetry_t tlm; int n_tlm; } pc_t;
static pc_t     s_pc;
static link_rx_t s_pc_rx;

static void pc_on_frame(void *ctx, uint8_t type, const uint8_t *p, uint8_t len)
{
    pc_t *pc = ctx;
    if (type == MSG_ACK && len == 2 && pc->n_ack < 16) {
        pc->cmd[pc->n_ack] = p[0];
        pc->res[pc->n_ack] = p[1];
        pc->n_ack++;
    } else if (type == MSG_TELEMETRY) {
        if (telemetry_decode(p, len, &pc->tlm)) pc->n_tlm++;
    }
}

static void pc_drain(void)
{
    uint8_t buf[256];
    uint16_t n;
    while ((n = fake_link_tx_take(buf, sizeof buf)) > 0) link_rx_feed(&s_pc_rx, buf, n);
}

static void send_cmd(uint8_t type, const uint8_t *pl, uint8_t len)
{
    uint8_t f[LINK_MAX_FRAME];
    uint16_t n = link_encode(f, sizeof f, type, pl, len);
    fake_link_feed(f, n);
}

static void send_mode(uint8_t m)      { send_cmd(MSG_SET_MODE, &m, 1); }
static void send_depth(float d)       { uint8_t p[4]; link_put_f32(p, d); send_cmd(MSG_SET_DEPTH, p, 4); }
static void send_ballast(float f, float a) { uint8_t p[8]; link_put_f32(p, f); link_put_f32(p + 4, a); send_cmd(MSG_SET_BALLAST, p, 8); }

static float phys_ml(int m) { return (float)fake_motor_phys[m] / 566.0f; }

static void sim_physics_1ms(void)
{
    const double k_n_per_ml = 0.00981, m_eff = 7.5, dt = 0.001;
    double excess = (double)phys_ml(PORT_MOTOR_FWD) + (double)phys_ml(PORT_MOTOR_AFT) - s_sim.neutral_total_ml;
    double force = k_n_per_ml * excess - (2.0 * s_sim.vel + 20.0 * fabs(s_sim.vel) * s_sim.vel);
    s_sim.vel += force / m_eff * dt;
    s_sim.depth += s_sim.vel * dt;
    if (s_sim.depth < 0.0) { s_sim.depth = 0.0; if (s_sim.vel < 0.0) s_sim.vel = 0.0; }   /* la surface */

    double diff_ml = (double)phys_ml(PORT_MOTOR_FWD) - (double)phys_ml(PORT_MOTOR_AFT);
    double target = s_sim.pitch_bias_deg - 0.5 * diff_ml;       /* avant plus plein => nez vers le bas */
    s_sim.pitch_deg += (target - s_sim.pitch_deg) / 2.0 * dt;   /* constante de temps 2 s */
}

static void sim_sensors(void)
{
    fake_ms5837_set_pressure(&s_sub.baro, 1013.0f + (float)s_sim.depth * MBAR_PER_M);
    float th = (float)(s_sim.pitch_deg * 3.14159265358979 / 180.0);
    const float acc[3] = { sinf(th), 0.0f, -cosf(th) };       /* lecture = -(gravité) : nez en haut => x > 0 (voir assiette.c) */
    const float gyr[3] = { 0.0f, 0.0f, 0.0f };
    fake_imu_set(&s_sub.imu, acc, gyr);
    fake_adc_value[PORT_ADC_LEAK] = s_sim.leak ? 3000 : 100;
    /* fins de course : actives (niveau bas) au bout de course, avec 50 pas d'hystérésis */
    static bool empty[2], full[2];
    for (int m = 0; m < 2; m++) {
        int32_t p = fake_motor_phys[m];
        if (p <= 0) empty[m] = true; else if (p >= 50) empty[m] = false;
        if (p >= MAX_STEPS) full[m] = true; else if (p <= MAX_STEPS - 50) full[m] = false;
        fake_gpio_level[2 * m]     = s_sim.dead_empty_switch[m] ? true : !empty[m];     /* PORT_PIN_LIMIT_*_TOP = butée "vide" */
        fake_gpio_level[2 * m + 1] = !full[m];      /* PORT_PIN_LIMIT_*_BOTTOM = butée "plein" */
    }
}

static void run_ms(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        for (int t = 0; t < 20; t++) app_tick(&s_app);
        sim_physics_1ms();
        fake_time_advance(1);
        if ((i & 1u) == 0) sim_sensors();
        uint32_t now = fake_time_get();
        if (s_heartbeat && (int32_t)(now - s_next_hb) >= 0) {
            send_cmd(MSG_HEARTBEAT, NULL, 0);
            s_next_hb = now + 500;
        }
        app_step(&s_app, now);
        if ((i % 20u) == 0) pc_drain();
    }
    pc_drain();
}

static void boot_ex(double neutral_total, bool dead_empty_switch)
{
    (void)dead_empty_switch;
    fake_sub_setup(&s_sub);
    memset(&s_sim, 0, sizeof s_sim);
    s_sim.neutral_total_ml = neutral_total;
    memset(&s_pc, 0, sizeof s_pc);
    link_rx_init(&s_pc_rx, pc_on_frame, &s_pc);
    for (int m = 0; m < PORT_MOTOR_COUNT; m++) {
        fake_motor_phys[m] = 1500;                          /* pistons à mi-course au démarrage : position inconnue du firmware */
        fake_motor_phys_min[m] = -300;
        fake_motor_phys_max[m] = MAX_STEPS + 300;
    }
    s_heartbeat = true;
    s_next_hb = fake_time_get() + 500;
    app_init(&s_app, fake_time_get());
    const float mag[3] = { 0.22f, 0.0f, 0.42f };
    fake_mag_set(&s_sub.mag, mag);
    sim_sensors();
}

static void boot(void) { boot_ex(100.0, false); }

static void boot_to_idle(void)
{
    boot();
    run_ms(8000);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s_app.safety));
}

static int last_ack(uint8_t cmd)
{
    for (int i = s_pc.n_ack - 1; i >= 0; i--) if (s_pc.cmd[i] == cmd) return s_pc.res[i];
    return -1;
}

static void clear_acks(void) { s_pc.n_ack = 0; }

/** Plonge à la profondeur voulue en DEPTH_HOLD (suppose le sous-marin en IDLE). */
static void dive_to(float depth, uint32_t settle_ms)
{
    send_mode(OP_DEPTH_HOLD);
    run_ms(50);
    send_depth(depth);
    run_ms(settle_ms);
}

/* ------------------------------------------------------------------ tests */
static void test_boot_homes_ballasts_and_reaches_idle(void)
{
    boot();
    MU_ASSERT_INT(MODE_INIT, safety_mode(&s_app.safety));
    run_ms(8000);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s_app.safety));
    MU_ASSERT(ballast_is_ready(&s_app.ballast[0]) && ballast_is_ready(&s_app.ballast[1]));
    MU_ASSERT_NEAR(0.0, ballast_volume_ml(&s_app.ballast[0]), 0.1);
    MU_ASSERT_NEAR(0.0, phys_ml(PORT_MOTOR_FWD), 0.2);            /* et le piston PHYSIQUE est vraiment arrivé à "vide" */
    MU_ASSERT_NEAR(0.0, phys_ml(PORT_MOTOR_AFT), 0.2);
    MU_ASSERT_NEAR(0.0, s_sim.depth, 0.01);                       /* le sous-marin flotte */
    MU_ASSERT_INT(0, fake_motor_pulses_disabled[0] + fake_motor_pulses_disabled[1]);
    MU_ASSERT(s_app.init_ok);
    MU_ASSERT_INT(0, s_app.sensors_init_failed);
}

static void test_telemetry_stream_is_10hz_and_consistent(void)
{
    boot_to_idle();
    int before = s_pc.n_tlm;
    run_ms(2000);
    MU_ASSERT(s_pc.n_tlm - before >= 19 && s_pc.n_tlm - before <= 21);
    MU_ASSERT_INT(MODE_IDLE, s_pc.tlm.mode);
    MU_ASSERT_NEAR(-1.0, s_pc.tlm.depth_setpoint_m, 1e-6);         /* pas de régulation : consigne "absente" */
    MU_ASSERT_NEAR(16.4, s_pc.tlm.vbat_v, 0.1);
    MU_ASSERT_INT(0, s_pc.tlm.reasons);
    MU_ASSERT_INT(0x22, s_pc.tlm.ballast_state);                   /* READY (2) des deux côtés */
    MU_ASSERT_INT(0, s_app.tx_overflow);
}

static void test_depth_hold_reaches_and_holds_one_meter(void)
{
    boot_to_idle();
    dive_to(1.0f, 100000);
    MU_ASSERT_INT(MODE_DEPTH_HOLD, safety_mode(&s_app.safety));
    MU_ASSERT_INT(ACK_OK, last_ack(MSG_SET_MODE));
    MU_ASSERT_INT(ACK_OK, last_ack(MSG_SET_DEPTH));
    MU_ASSERT_NEAR(1.0, s_sim.depth, 0.04);
    MU_ASSERT_NEAR(0.0, s_sim.vel, 0.01);
    MU_ASSERT_NEAR(1.0, s_pc.tlm.depth_m, 0.05);                   /* ce que mesure le firmware est cohérent avec la réalité simulée */
    MU_ASSERT_NEAR(1.0, s_pc.tlm.depth_setpoint_m, 1e-3);
    /* à l'équilibre le sous-marin est neutre : chaque ballast près de 50 mL */
    MU_ASSERT_NEAR(50.0, s_pc.tlm.vol_front_ml, 6.0);
    MU_ASSERT_INT(0, s_pc.tlm.reasons);
}

static void test_depth_hold_has_limited_overshoot_and_no_oscillation(void)
{
    boot_to_idle();
    send_mode(OP_DEPTH_HOLD);
    run_ms(50);
    send_depth(1.0f);
    double max_depth = 0.0;
    for (int s = 0; s < 120; s++) {
        run_ms(1000);
        if (s_sim.depth > max_depth) max_depth = s_sim.depth;
    }
    MU_ASSERT(max_depth < 1.12);                                   /* dépassement < 12 % */
    /* pas d'oscillation entretenue : sur les 20 dernières secondes la profondeur ne varie presque plus */
    double lo = 9, hi = -9;
    for (int s = 0; s < 20; s++) {
        run_ms(1000);
        if (s_sim.depth < lo) lo = s_sim.depth;
        if (s_sim.depth > hi) hi = s_sim.depth;
    }
    MU_ASSERT(hi - lo < 0.03);
}

static void test_depth_hold_rejects_bad_setpoints_and_keeps_current(void)
{
    boot_to_idle();
    dive_to(0.5f, 40000);
    clear_acks();
    send_depth(5.0f);   run_ms(50);                                 /* trop profond */
    MU_ASSERT_INT(ACK_INVALID, last_ack(MSG_SET_DEPTH));
    send_depth(-1.0f);  run_ms(50);
    MU_ASSERT_INT(ACK_INVALID, last_ack(MSG_SET_DEPTH));
    send_depth(NAN);    run_ms(50);
    MU_ASSERT_INT(ACK_INVALID, last_ack(MSG_SET_DEPTH));
    run_ms(30000);
    MU_ASSERT_NEAR(0.5, s_sim.depth, 0.04);                         /* la consigne précédente est toujours tenue */
}

static void test_depth_hold_changes_depth_both_ways(void)
{
    boot_to_idle();
    dive_to(1.2f, 90000);
    MU_ASSERT_NEAR(1.2, s_sim.depth, 0.05);
    send_depth(0.4f);
    run_ms(90000);
    MU_ASSERT_NEAR(0.4, s_sim.depth, 0.05);
}

static void test_depth_hold_compensates_wrong_neutral_buoyancy(void)
{
    boot_ex(108.0, false);                                          /* le sous-marin est plus lourd que prévu : 4 mL de plus par ballast */
    run_ms(8000);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s_app.safety));
    dive_to(1.0f, 200000);
    MU_ASSERT_NEAR(1.0, s_sim.depth, 0.03);                         /* l'intégrale a rattrapé l'erreur de réglage */
}

static void test_depth_hold_corrects_a_pitch_disturbance(void)
{
    boot_to_idle();
    s_sim.pitch_bias_deg = 8.0;                                     /* un poids mal placé cabre le sous-marin */
    dive_to(1.0f, 120000);
    MU_ASSERT_NEAR(1.0, s_sim.depth, 0.05);
    MU_ASSERT(fabs(s_sim.pitch_deg) < 1.5);                         /* l'assiette est rattrapée par la différence de volume */
    MU_ASSERT(s_pc.tlm.vol_front_ml > s_pc.tlm.vol_aft_ml + 5.0f);  /* avant plus plein, comme attendu */
}

static void test_depth_hold_survives_imu_loss_without_emergency(void)
{
    boot_to_idle();
    dive_to(0.8f, 60000);
    fake_remove_i2c(LSM6DSOX_I2C_ADDR_LOW);                         /* la centrale inertielle ne répond plus */
    run_ms(20000);
    MU_ASSERT_INT(MODE_DEPTH_HOLD, safety_mode(&s_app.safety));     /* pas d'urgence : seul le tangage n'est plus corrigé */
    MU_ASSERT(s_pc.tlm.failsafe_mask & FS_IMU_LOST);
    MU_ASSERT_NEAR(0.8, s_sim.depth, 0.05);
}

static void test_leak_triggers_emergency_ascent_and_latches(void)
{
    boot_to_idle();
    dive_to(1.0f, 90000);
    MU_ASSERT_INT(MODE_DEPTH_HOLD, safety_mode(&s_app.safety));
    s_sim.leak = true;
    run_ms(300);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s_app.safety));      /* réaction en moins de 300 ms */
    MU_ASSERT(s_pc.tlm.reasons & FS_LEAK);
    run_ms(60000);
    MU_ASSERT_NEAR(0.0, phys_ml(PORT_MOTOR_FWD), 0.3);              /* ballasts vidés */
    MU_ASSERT_NEAR(0.0, phys_ml(PORT_MOTOR_AFT), 0.3);
    MU_ASSERT(s_sim.depth < 0.05);                                  /* et le sous-marin est remonté */

    s_sim.leak = false;                                             /* le capteur sèche */
    run_ms(5000);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s_app.safety));      /* mais l'alarme reste mémorisée */
    clear_acks();
    send_mode(OP_DEPTH_HOLD); run_ms(100);
    MU_ASSERT_INT(ACK_REJECTED, last_ack(MSG_SET_MODE));            /* pas de nouvelle plongée tant que non acquitté */
    send_cmd(MSG_CLEAR_FAULT, NULL, 0);
    run_ms(500);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s_app.safety));
    MU_ASSERT_INT(0, s_pc.tlm.reasons);
}

static void test_clear_is_refused_while_still_wet(void)
{
    boot_to_idle();
    s_sim.leak = true;
    run_ms(500);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s_app.safety));
    send_cmd(MSG_CLEAR_FAULT, NULL, 0);
    run_ms(500);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s_app.safety));      /* toujours de l'eau : acquittement sans effet */
    s_sim.leak = false;
    send_cmd(MSG_CLEAR_FAULT, NULL, 0);
    run_ms(500);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s_app.safety));
}

static void test_link_loss_while_submerged_triggers_ascent(void)
{
    boot_to_idle();
    dive_to(1.0f, 90000);
    s_heartbeat = false;                                            /* la Raspberry Pi ne répond plus */
    run_ms(2500);
    MU_ASSERT_INT(MODE_DEPTH_HOLD, safety_mode(&s_app.safety));     /* pas avant 3 s */
    run_ms(1500);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s_app.safety));
    MU_ASSERT(s_pc.tlm.reasons & SAFE_LINK_LOST);
    run_ms(60000);
    MU_ASSERT(s_sim.depth < 0.05);
}

static void test_depth_sensor_loss_while_diving_triggers_ascent(void)
{
    boot_to_idle();
    dive_to(1.0f, 90000);
    fake_remove_i2c(0x76);                                          /* capteur de pression débranché */
    run_ms(1500);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s_app.safety));
    MU_ASSERT(s_pc.tlm.reasons & FS_DEPTH_LOST);
    run_ms(60000);
    MU_ASSERT(s_sim.depth < 0.05);                                  /* la remontée se fait ballasts vides, sans avoir besoin de la profondeur */
}

static void test_over_depth_triggers_emergency(void)
{
    boot_to_idle();
    dive_to(1.0f, 60000);
    s_sim.depth = 2.7;                                              /* poussé trop profond (courant, erreur de pilotage) */
    run_ms(1500);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s_app.safety));
    MU_ASSERT(s_pc.tlm.reasons & FS_OVER_DEPTH);
}

static void test_operator_emergency_command(void)
{
    boot_to_idle();
    dive_to(1.0f, 60000);
    send_cmd(MSG_EMERGENCY, NULL, 0);
    run_ms(200);
    MU_ASSERT_INT(MODE_EMERGENCY, safety_mode(&s_app.safety));
    MU_ASSERT(s_pc.tlm.reasons & SAFE_OPERATOR);
    run_ms(60000);
    MU_ASSERT(s_sim.depth < 0.05);
}

static void test_manual_mode_sets_ballast_volumes(void)
{
    boot_to_idle();
    clear_acks();
    send_ballast(30.0f, 20.0f); run_ms(50);
    MU_ASSERT_INT(ACK_REJECTED, last_ack(MSG_SET_BALLAST));         /* pas en IDLE */
    send_mode(OP_MANUAL); run_ms(50);
    MU_ASSERT_INT(ACK_OK, last_ack(MSG_SET_MODE));
    send_ballast(30.0f, 20.0f); run_ms(10000);
    MU_ASSERT_INT(ACK_OK, last_ack(MSG_SET_BALLAST));
    MU_ASSERT_NEAR(30.0, phys_ml(PORT_MOTOR_FWD), 0.2);
    MU_ASSERT_NEAR(20.0, phys_ml(PORT_MOTOR_AFT), 0.2);
    send_ballast(150.0f, 0.0f); run_ms(100);                        /* hors course */
    MU_ASSERT_INT(ACK_INVALID, last_ack(MSG_SET_BALLAST));
    send_ballast(0.0f, 150.0f); run_ms(100);                        /* le ballast arrière aussi est contrôlé */
    MU_ASSERT_INT(ACK_INVALID, last_ack(MSG_SET_BALLAST));
    send_ballast(-1.0f, 0.0f); run_ms(100);
    MU_ASSERT_INT(ACK_INVALID, last_ack(MSG_SET_BALLAST));
    send_ballast(NAN, 0.0f); run_ms(100);
    MU_ASSERT_INT(ACK_INVALID, last_ack(MSG_SET_BALLAST));
    MU_ASSERT_NEAR(30.0, phys_ml(PORT_MOTOR_FWD), 0.2);             /* rien n'a bougé */
    send_mode(OP_IDLE); run_ms(15000);
    MU_ASSERT_NEAR(0.0, phys_ml(PORT_MOTOR_FWD), 0.2);              /* retour en IDLE : ballasts vidés */
}

static void test_tare_only_allowed_when_idle_and_shallow(void)
{
    boot_to_idle();
    clear_acks();
    send_cmd(MSG_TARE_DEPTH, NULL, 0); run_ms(2000);
    MU_ASSERT_INT(ACK_OK, last_ack(MSG_TARE_DEPTH));
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s_app.safety));
    send_mode(OP_DEPTH_HOLD); run_ms(100);
    MU_ASSERT_INT(ACK_OK, last_ack(MSG_SET_MODE));                  /* la tare est finie : plongée à nouveau permise */
    send_depth(1.0f); run_ms(90000);
    clear_acks();
    send_cmd(MSG_TARE_DEPTH, NULL, 0); run_ms(100);
    MU_ASSERT_INT(ACK_REJECTED, last_ack(MSG_TARE_DEPTH));          /* en plongée : refusée (sinon la profondeur serait fausse) */
}

static void test_set_depth_refused_outside_depth_hold(void)
{
    boot_to_idle();
    clear_acks();
    send_depth(1.0f); run_ms(50);
    MU_ASSERT_INT(ACK_REJECTED, last_ack(MSG_SET_DEPTH));           /* IDLE : pas de regulation */
    send_mode(OP_MANUAL); run_ms(50);
    send_depth(1.0f); run_ms(50);
    MU_ASSERT_INT(ACK_REJECTED, last_ack(MSG_SET_DEPTH));           /* MANUAL aussi */
    send_mode(OP_DEPTH_HOLD); run_ms(50);
    send_depth(0.3f); run_ms(50);
    MU_ASSERT_INT(ACK_OK, last_ack(MSG_SET_DEPTH));                 /* et accepte en regulation */
}

static void test_tare_refused_when_not_at_surface(void)
{
    boot_to_idle();
    s_sim.depth = 1.2;                                              /* en IDLE mais encore immerge (apres un retour en IDLE) */
    run_ms(300);
    MU_ASSERT(s_app.sm.data.depth_m > 0.5f);
    clear_acks();
    send_cmd(MSG_TARE_DEPTH, NULL, 0); run_ms(100);
    MU_ASSERT_INT(ACK_REJECTED, last_ack(MSG_TARE_DEPTH));          /* la pression mesuree ici n'est pas celle de la surface */
}

static void test_dive_refused_while_tare_is_running(void)
{
    boot_to_idle();
    clear_acks();
    send_cmd(MSG_TARE_DEPTH, NULL, 0); run_ms(30);
    MU_ASSERT_INT(ACK_OK, last_ack(MSG_TARE_DEPTH));
    send_mode(OP_DEPTH_HOLD); run_ms(30);
    MU_ASSERT_INT(ACK_REJECTED, last_ack(MSG_SET_MODE));            /* tare en cours : la profondeur de reference n'est pas prete */
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s_app.safety));
    run_ms(3000);
    send_mode(OP_DEPTH_HOLD); run_ms(50);
    MU_ASSERT_INT(ACK_OK, last_ack(MSG_SET_MODE));                  /* tare terminee : permis */
}

static void test_set_pid_only_in_idle_and_validated(void)
{
    boot_to_idle();
    uint8_t p[13]; p[0] = 0;
    link_put_f32(p + 1, 12.0f); link_put_f32(p + 5, 0.5f); link_put_f32(p + 9, 100.0f);
    clear_acks();
    send_cmd(MSG_SET_PID, p, 13); run_ms(50);
    MU_ASSERT_INT(ACK_OK, last_ack(MSG_SET_PID));
    MU_ASSERT_NEAR(12.0, s_app.dc.depth_pid.kp, 1e-6);
    link_put_f32(p + 1, -3.0f);
    send_cmd(MSG_SET_PID, p, 13); run_ms(50);
    MU_ASSERT_INT(ACK_INVALID, last_ack(MSG_SET_PID));
    link_put_f32(p + 1, 5000.0f);
    send_cmd(MSG_SET_PID, p, 13); run_ms(50);
    MU_ASSERT_INT(ACK_INVALID, last_ack(MSG_SET_PID));
    p[0] = 7; link_put_f32(p + 1, 5.0f);
    send_cmd(MSG_SET_PID, p, 13); run_ms(50);
    MU_ASSERT_INT(ACK_INVALID, last_ack(MSG_SET_PID));
    MU_ASSERT_NEAR(12.0, s_app.dc.depth_pid.kp, 1e-6);              /* gains inchangés après les refus */
    send_cmd(MSG_SET_PID, p, 5); run_ms(50);                        /* longueur fausse */
    MU_ASSERT_INT(ACK_INVALID, last_ack(MSG_SET_PID));
    p[0] = 0;
    send_mode(OP_MANUAL); run_ms(50);
    send_cmd(MSG_SET_PID, p, 13); run_ms(50);
    MU_ASSERT_INT(ACK_REJECTED, last_ack(MSG_SET_PID));             /* pas pendant un pilotage */
}

static void test_corrupted_and_unknown_commands_are_harmless(void)
{
    boot_to_idle();
    uint8_t f[LINK_MAX_FRAME];
    uint8_t m = OP_DEPTH_HOLD;
    uint16_t n = link_encode(f, sizeof f, MSG_SET_MODE, &m, 1);
    f[3] ^= 0x01;                                                   /* trame altérée */
    fake_link_feed(f, n);
    uint8_t noise[100];
    for (unsigned i = 0; i < sizeof noise; i++) noise[i] = (uint8_t)(i * 37u + 11u);
    fake_link_feed(noise, sizeof noise);
    run_ms(200);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s_app.safety));           /* la commande altérée n'a pas été exécutée */
    clear_acks();
    send_cmd(0x66, NULL, 0); run_ms(50);
    MU_ASSERT_INT(ACK_UNKNOWN, last_ack(0x66));
    uint8_t two[2] = { 1, 2 };
    send_cmd(MSG_SET_MODE, two, 2); run_ms(50);                     /* longueur fausse */
    MU_ASSERT_INT(ACK_INVALID, last_ack(MSG_SET_MODE));
    uint8_t bad = 9;
    send_cmd(MSG_SET_MODE, &bad, 1); run_ms(50);                    /* mode inexistant */
    MU_ASSERT_INT(ACK_INVALID, last_ack(MSG_SET_MODE));
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s_app.safety));
}

static void test_command_flood_does_not_freeze_the_loop(void)
{
    boot_to_idle();
    uint8_t blob[512];
    memset(blob, 0xA5, sizeof blob);
    for (int i = 0; i < 6; i++) fake_link_feed(blob, sizeof blob);  /* 3 Ko de faux débuts de trame d'un coup */
    int before = s_pc.n_tlm;
    run_ms(1000);
    MU_ASSERT(s_pc.n_tlm - before >= 9);                            /* la télémétrie continue */
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s_app.safety));
}

static void test_failed_homing_leaves_idle_with_diving_disabled(void)
{
    boot();
    s_sim.dead_empty_switch[BALLAST_FRONT] = true;       /* butée "vide" de l'avant coupée : ce ballast ne sera jamais référencé */
    run_ms(70000);
    MU_ASSERT_INT(MODE_IDLE, safety_mode(&s_app.safety));
    MU_ASSERT_INT(BALLAST_FAULT, ballast_state(&s_app.ballast[BALLAST_FRONT]));
    MU_ASSERT_INT(BALLAST_READY, ballast_state(&s_app.ballast[BALLAST_AFT]));
    clear_acks();
    send_mode(OP_MANUAL); run_ms(100);
    MU_ASSERT_INT(ACK_REJECTED, last_ack(MSG_SET_MODE));
    send_mode(OP_DEPTH_HOLD); run_ms(100);
    MU_ASSERT_INT(ACK_REJECTED, last_ack(MSG_SET_MODE));
}

int main(void)
{
    MU_RUN(test_boot_homes_ballasts_and_reaches_idle);
    MU_RUN(test_telemetry_stream_is_10hz_and_consistent);
    MU_RUN(test_depth_hold_reaches_and_holds_one_meter);
    MU_RUN(test_depth_hold_has_limited_overshoot_and_no_oscillation);
    MU_RUN(test_depth_hold_rejects_bad_setpoints_and_keeps_current);
    MU_RUN(test_depth_hold_changes_depth_both_ways);
    MU_RUN(test_depth_hold_compensates_wrong_neutral_buoyancy);
    MU_RUN(test_depth_hold_corrects_a_pitch_disturbance);
    MU_RUN(test_depth_hold_survives_imu_loss_without_emergency);
    MU_RUN(test_leak_triggers_emergency_ascent_and_latches);
    MU_RUN(test_clear_is_refused_while_still_wet);
    MU_RUN(test_link_loss_while_submerged_triggers_ascent);
    MU_RUN(test_depth_sensor_loss_while_diving_triggers_ascent);
    MU_RUN(test_over_depth_triggers_emergency);
    MU_RUN(test_operator_emergency_command);
    MU_RUN(test_manual_mode_sets_ballast_volumes);
    MU_RUN(test_tare_only_allowed_when_idle_and_shallow);
    MU_RUN(test_set_depth_refused_outside_depth_hold);
    MU_RUN(test_tare_refused_when_not_at_surface);
    MU_RUN(test_dive_refused_while_tare_is_running);
    MU_RUN(test_set_pid_only_in_idle_and_validated);
    MU_RUN(test_corrupted_and_unknown_commands_are_harmless);
    MU_RUN(test_command_flood_does_not_freeze_the_loop);
    MU_RUN(test_failed_homing_leaves_idle_with_diving_disabled);
    return MU_SUMMARY();
}
