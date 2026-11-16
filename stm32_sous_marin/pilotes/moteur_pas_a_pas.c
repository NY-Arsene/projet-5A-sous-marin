#include "moteur_pas_a_pas.h"
#include <stddef.h>

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

bool stepper_init(stepper_t *s, port_motor_t motor, const stepper_cfg_t *cfg)
{
    if (s == NULL || cfg == NULL || (unsigned)motor >= (unsigned)PORT_MOTOR_COUNT) {
        return false;
    }
    if (cfg->tick_hz < 1000u || cfg->tick_hz > 200000u ||
        cfg->max_speed_sps < 1 || (uint32_t)cfg->max_speed_sps > cfg->tick_hz / 2u ||
        cfg->min_speed_sps < 1 || cfg->min_speed_sps > cfg->max_speed_sps ||
        cfg->accel_sps2 < 1 || cfg->min_pos > cfg->max_pos) {
        return false;
    }
    int32_t dv = (int32_t)(((int64_t)cfg->accel_sps2 * 256) / (int64_t)cfg->tick_hz);
    if (dv < 1) {
        return false;                       /* accélération trop faible pour cette cadence de tick */
    }
    s->cfg = *cfg;
    s->motor = motor;
    s->dv_q8 = dv;
    s->tick_q8 = (int32_t)(cfg->tick_hz * 256u);
    s->position = 0;
    s->target = 0;
    s->v_q8 = 0;
    s->phase_q8 = 0;
    s->dir = 1;
    s->enabled = false;
    s->pulses = 0;
    port_motor_dir(motor, true);
    port_motor_enable(motor, false);
    return true;
}

void stepper_enable(stepper_t *s, bool on)
{
    if (!on) {
        s->target = s->position;            /* on ne reprend pas un ancien ordre en se réactivant */
        s->v_q8 = 0;
        s->phase_q8 = 0;
    }
    s->enabled = on;
    port_motor_enable(s->motor, on);
}

void stepper_move_to(stepper_t *s, int32_t target)
{
    s->target = clamp_i32(target, s->cfg.min_pos, s->cfg.max_pos);
}

void stepper_stop_now(stepper_t *s)
{
    s->target = s->position;
    s->v_q8 = 0;
    s->phase_q8 = 0;
}

void stepper_set_position(stepper_t *s, int32_t pos)
{
    s->position = pos;
    stepper_stop_now(s);
}

void stepper_set_limits(stepper_t *s, int32_t min_pos, int32_t max_pos)
{
    if (min_pos <= max_pos) {
        s->cfg.min_pos = min_pos;
        s->cfg.max_pos = max_pos;
        s->target = clamp_i32(s->target, min_pos, max_pos);
    }
}

bool stepper_set_max_speed(stepper_t *s, int32_t sps)
{
    if (sps < s->cfg.min_speed_sps || (uint32_t)sps > s->cfg.tick_hz / 2u) {
        return false;
    }
    s->cfg.max_speed_sps = sps;
    return true;
}

int32_t stepper_position(const stepper_t *s)  { return s->position; }
int32_t stepper_target(const stepper_t *s)    { return s->target; }
bool    stepper_is_moving(const stepper_t *s) { return s->enabled && (s->position != s->target || s->v_q8 > 0); }

void stepper_tick(stepper_t *s)
{
    if (!s->enabled) {
        return;
    }
    int32_t dist = s->target - s->position;
    if (dist == 0) {                        /* arrivé : arrêt net */
        s->v_q8 = 0;
        s->phase_q8 = 0;
        return;
    }
    int8_t want = (dist > 0) ? 1 : -1;

    if (s->v_q8 > 0 && s->dir != want) {
        /* La cible est derrière nous : on freine jusqu'à l'arrêt avant de changer de sens. */
        s->v_q8 -= s->dv_q8;
        if (s->v_q8 < 0) {
            s->v_q8 = 0;
        }
    } else {
        if (s->v_q8 == 0 && s->dir != want) {
            s->dir = want;
            port_motor_dir(s->motor, want > 0);   /* la broche DIR change bien avant la prochaine impulsion */
        }
        int64_t v_sps = (int64_t)(s->v_q8 >> 8);
        int64_t adist = (dist > 0) ? (int64_t)dist : -(int64_t)dist;
        if (v_sps * v_sps >= 2 * (int64_t)s->cfg.accel_sps2 * adist) {
            /* Distance de freinage v²/(2a) atteinte : on décélère, sans descendre sous la vitesse d'approche. */
            s->v_q8 -= s->dv_q8;
            int32_t vmin = s->cfg.min_speed_sps * 256;
            if (s->v_q8 < vmin) {
                s->v_q8 = vmin;
            }
        } else {
            s->v_q8 += s->dv_q8;
            int32_t vmax = s->cfg.max_speed_sps * 256;
            if (s->v_q8 > vmax) {
                s->v_q8 = vmax;
            }
        }
    }

    s->phase_q8 += s->v_q8;
    if (s->phase_q8 >= s->tick_q8) {
        s->phase_q8 -= s->tick_q8;
        s->position += s->dir;
        s->pulses++;
        port_motor_step(s->motor);
    }
}
