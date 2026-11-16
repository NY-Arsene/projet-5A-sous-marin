#include "pid.h"
#include <math.h>
#include <stddef.h>

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static bool gains_ok(float kp, float ki, float kd)
{
    return isfinite(kp) && isfinite(ki) && isfinite(kd) && kp >= 0.0f && ki >= 0.0f && kd >= 0.0f;
}

bool pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float out_min, float out_max)
{
    if (p == NULL || !gains_ok(kp, ki, kd) || !isfinite(out_min) || !isfinite(out_max) || !(out_min < out_max)) {
        return false;
    }
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_min = out_min;
    p->out_max = out_max;
    p->i_zone = 0.0f;
    p->i_term = 0.0f;
    p->last_out = clampf(0.0f, out_min, out_max);
    return true;
}

bool pid_set_gains(pid_ctrl_t *p, float kp, float ki, float kd)
{
    if (p == NULL || !gains_ok(kp, ki, kd)) {
        return false;
    }
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    return true;
}

bool pid_set_i_zone(pid_ctrl_t *p, float zone)
{
    if (p == NULL || !isfinite(zone) || zone < 0.0f) {
        return false;
    }
    p->i_zone = zone;
    return true;
}

void pid_reset(pid_ctrl_t *p, float i_term)
{
    p->i_term = isfinite(i_term) ? clampf(i_term, p->out_min, p->out_max) : 0.0f;
    p->last_out = clampf(p->i_term, p->out_min, p->out_max);
}

float pid_update(pid_ctrl_t *p, float setpoint, float meas, float meas_rate, float dt_s)
{
    if (!isfinite(setpoint) || !isfinite(meas) || !isfinite(meas_rate) || !isfinite(dt_s) ||
        !(dt_s > 0.0f) || dt_s > 1.0f) {
        return p->last_out;                      /* donnée invalide : on garde la commande précédente */
    }
    float err = setpoint - meas;
    float unsat = p->kp * err + p->i_term - p->kd * meas_rate;

    /* Anti-windup : l'intégrale ne grossit pas si la sortie est déjà saturée du côté de l'erreur. */
    bool pushing_into_limit = (unsat > p->out_max && err > 0.0f) || (unsat < p->out_min && err < 0.0f);
    bool in_zone = (p->i_zone <= 0.0f) || (fabsf(err) <= p->i_zone);
    if (!pushing_into_limit && in_zone) {
        p->i_term = clampf(p->i_term + p->ki * err * dt_s, p->out_min, p->out_max);
    }

    float out = clampf(p->kp * err + p->i_term - p->kd * meas_rate, p->out_min, p->out_max);
    p->last_out = out;
    return out;
}
