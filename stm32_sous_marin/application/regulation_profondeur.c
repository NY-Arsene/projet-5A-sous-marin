#include "regulation_profondeur.h"
#include <math.h>
#include <stddef.h>

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void depth_ctrl_default_config(depth_ctrl_cfg_t *cfg)
{
    cfg->kp_depth = 35.0f;   cfg->ki_depth = 0.4f;   cfg->kd_depth = 300.0f;
    cfg->kp_pitch = 3.0f;    cfg->ki_pitch = 0.2f;   cfg->kd_pitch = 1.0f;
    cfg->v_neutral_ml = 50.0f;           /* à régler : volume qui rend le sous-marin neutre (essai en bassin) */
    cfg->max_volume_ml = 100.0f;
    cfg->max_diff_ml = 30.0f;
    cfg->max_setpoint_m = 2.0f;
    cfg->setpoint_rate_m_s = 0.10f;
    cfg->pitch_sign = 1.0f;
    cfg->i_zone_depth_m = 0.3f;
    cfg->bumpless_min_depth_m = 0.3f;
}

bool depth_ctrl_init(depth_ctrl_t *dc, const depth_ctrl_cfg_t *cfg)
{
    if (dc == NULL || cfg == NULL) {
        return false;
    }
    if (!isfinite(cfg->v_neutral_ml) || !isfinite(cfg->max_volume_ml) || !isfinite(cfg->max_diff_ml) ||
        !isfinite(cfg->max_setpoint_m) || !isfinite(cfg->setpoint_rate_m_s) || !isfinite(cfg->pitch_sign) || !isfinite(cfg->bumpless_min_depth_m) || !isfinite(cfg->i_zone_depth_m) || cfg->i_zone_depth_m < 0.0f ||
        !(cfg->max_volume_ml > 0.0f) || cfg->v_neutral_ml < 0.0f || cfg->v_neutral_ml > cfg->max_volume_ml ||
        !(cfg->max_diff_ml > 0.0f) || !(cfg->max_setpoint_m > 0.0f) || !(cfg->setpoint_rate_m_s > 0.0f) ||
        (cfg->pitch_sign != 1.0f && cfg->pitch_sign != -1.0f)) {
        return false;
    }
    /* La sortie du régulateur de profondeur est un décalage autour du volume neutre : sa plage garantit
     * que le volume final reste dans [0 ; max_volume_ml] (et donc que l'intégrale ne dépasse jamais). */
    if (!pid_init(&dc->depth_pid, cfg->kp_depth, cfg->ki_depth, cfg->kd_depth,
                  -cfg->v_neutral_ml - 1.0e-3f, cfg->max_volume_ml - cfg->v_neutral_ml + 1.0e-3f) ||
        !pid_init(&dc->pitch_pid, cfg->kp_pitch, cfg->ki_pitch, cfg->kd_pitch,
                  -cfg->max_diff_ml, cfg->max_diff_ml)) {
        return false;
    }
    (void)pid_set_i_zone(&dc->depth_pid, cfg->i_zone_depth_m);
    dc->cfg = *cfg;
    dc->setpoint_m = dc->setpoint_eff_m = 0.0f;
    dc->front_ml = dc->aft_ml = cfg->v_neutral_ml;
    return true;
}

void depth_ctrl_start(depth_ctrl_t *dc, float depth_m, float front_ml, float aft_ml)
{
    if (!isfinite(depth_m)) {
        depth_m = 0.0f;
    }
    if (!isfinite(front_ml)) front_ml = dc->cfg.v_neutral_ml;
    if (!isfinite(aft_ml))   aft_ml = dc->cfg.v_neutral_ml;
    dc->setpoint_m = dc->setpoint_eff_m = clampf(depth_m, 0.0f, dc->cfg.max_setpoint_m);
    dc->front_ml = front_ml;
    dc->aft_ml = aft_ml;
    if (depth_m >= dc->cfg.bumpless_min_depth_m) {
        pid_reset(&dc->depth_pid, 0.5f * (front_ml + aft_ml) - dc->cfg.v_neutral_ml);
        pid_reset(&dc->pitch_pid, -dc->cfg.pitch_sign * (front_ml - aft_ml));
    } else {
        pid_reset(&dc->depth_pid, 0.0f);
        pid_reset(&dc->pitch_pid, 0.0f);
    }
}

bool depth_ctrl_set_depth(depth_ctrl_t *dc, float depth_m)
{
    if (!isfinite(depth_m) || depth_m < 0.0f || depth_m > dc->cfg.max_setpoint_m) {
        return false;
    }
    dc->setpoint_m = depth_m;
    return true;
}

bool depth_ctrl_set_gains(depth_ctrl_t *dc, int which, float kp, float ki, float kd)
{
    if (which == 0) {
        return pid_set_gains(&dc->depth_pid, kp, ki, kd);
    }
    if (which == 1) {
        return pid_set_gains(&dc->pitch_pid, kp, ki, kd);
    }
    return false;
}

void depth_ctrl_update(depth_ctrl_t *dc, float depth_m, float depth_rate_m_s,
                       float pitch_deg, float pitch_rate_dps, bool pitch_valid, float dt_s)
{
    if (!isfinite(depth_m) || !isfinite(depth_rate_m_s) || !isfinite(dt_s) || !(dt_s > 0.0f) || dt_s > 1.0f) {
        return;                                   /* on garde les volumes précédents */
    }
    /* consigne rampée */
    float max_step = dc->cfg.setpoint_rate_m_s * dt_s;
    dc->setpoint_eff_m += clampf(dc->setpoint_m - dc->setpoint_eff_m, -max_step, max_step);

    float offset = pid_update(&dc->depth_pid, dc->setpoint_eff_m, depth_m, depth_rate_m_s, dt_s);
    float common = dc->cfg.v_neutral_ml + offset;

    float diff = 0.0f;
    if (pitch_valid && isfinite(pitch_deg) && isfinite(pitch_rate_dps)) {
        diff = -dc->cfg.pitch_sign * pid_update(&dc->pitch_pid, 0.0f, pitch_deg, pitch_rate_dps, dt_s);
    } else {
        pid_reset(&dc->pitch_pid, 0.0f);          /* sans mesure fiable, pas de correction de tangage */
    }

    dc->front_ml = clampf(common + 0.5f * diff, 0.0f, dc->cfg.max_volume_ml);
    dc->aft_ml   = clampf(common - 0.5f * diff, 0.0f, dc->cfg.max_volume_ml);
}
