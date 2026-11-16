#include "securite.h"
#include <math.h>
#include <stddef.h>

#define HARD_CAUSES (FS_LEAK | FS_BATT_CRIT | FS_OVER_DEPTH)

void safety_default_config(safety_cfg_t *cfg)
{
    cfg->link_timeout_ms = 3000u;
    cfg->submerged_depth_m = 0.30f;
    cfg->init_timeout_ms = 90000u;      /* un peu plus que le délai de référencement d'un ballast (60 s) */
}

void safety_init(safety_t *s, const safety_cfg_t *cfg, uint32_t now_ms)
{
    s->cfg = *cfg;
    s->mode = MODE_INIT;
    s->reasons = 0;
    s->start_ms = now_ms;
    s->last_link_ms = now_ms;
    s->clear_requested = false;
}

void safety_link_activity(safety_t *s, uint32_t now_ms)
{
    s->last_link_ms = now_ms;
}

static bool is_active_mode(sub_mode_t m)
{
    return m == MODE_MANUAL || m == MODE_DEPTH_HOLD;
}

safe_req_t safety_request_mode(safety_t *s, int op_mode, const safety_inputs_t *in)
{
    if (op_mode < (int)OP_IDLE || op_mode > (int)OP_DEPTH_HOLD) {
        return SAFE_REQ_INVALID;
    }
    if (s->mode == MODE_INIT || s->mode == MODE_EMERGENCY || (in->failsafe_mask & HARD_CAUSES)) {
        return SAFE_REQ_REJECTED;
    }
    switch ((op_mode_t)op_mode) {
    case OP_IDLE:
        s->mode = MODE_IDLE;
        return SAFE_REQ_OK;
    case OP_MANUAL:
        if (!in->ballasts_ready) {
            return SAFE_REQ_REJECTED;
        }
        s->mode = MODE_MANUAL;
        return SAFE_REQ_OK;
    case OP_DEPTH_HOLD:
        if (!in->ballasts_ready || !in->depth_ref_valid || (in->failsafe_mask & FS_DEPTH_LOST) || !isfinite(in->depth_m)) {
            return SAFE_REQ_REJECTED;
        }
        s->mode = MODE_DEPTH_HOLD;
        return SAFE_REQ_OK;
    default:
        return SAFE_REQ_INVALID;
    }
}

void safety_emergency_stop(safety_t *s)
{
    s->reasons |= SAFE_OPERATOR;
    s->mode = MODE_EMERGENCY;
}

void safety_clear_fault(safety_t *s)
{
    s->clear_requested = true;
}

void safety_update(safety_t *s, const safety_inputs_t *in)
{
    uint32_t now_reasons = in->failsafe_mask & HARD_CAUSES;
    if (is_active_mode(s->mode)) {
        if (in->failsafe_mask & FS_DEPTH_LOST) {
            now_reasons |= FS_DEPTH_LOST;
        }
        bool submerged = !isfinite(in->depth_m) || in->depth_m > s->cfg.submerged_depth_m;
        if (submerged && (uint32_t)(in->now_ms - s->last_link_ms) > s->cfg.link_timeout_ms) {
            now_reasons |= SAFE_LINK_LOST;
        }
        if (in->ballast_fault) {
            now_reasons |= SAFE_BALLAST_FAULT;
        }
    }

    if (s->mode == MODE_INIT) {
        uint32_t elapsed = (uint32_t)(in->now_ms - s->start_ms);
        bool ready = in->homing_done && (in->depth_ref_valid || elapsed > s->cfg.init_timeout_ms);
        if (ready || elapsed > s->cfg.init_timeout_ms) {
            s->mode = MODE_IDLE;
        }
    }

    if (s->mode == MODE_EMERGENCY) {
        s->reasons |= now_reasons;
        if (s->clear_requested) {
            s->reasons = now_reasons;           /* on oublie les anciennes causes, on garde celles encore actives */
            if (s->reasons == 0) {
                s->mode = MODE_IDLE;
            }
        }
    } else if (now_reasons != 0) {
        s->reasons |= now_reasons;
        s->mode = MODE_EMERGENCY;
    }
    s->clear_requested = false;
}

sub_mode_t safety_mode(const safety_t *s)    { return s->mode; }
uint32_t   safety_reasons(const safety_t *s) { return s->reasons; }

const char *safety_mode_name(sub_mode_t m)
{
    switch (m) {
    case MODE_INIT:       return "INIT";
    case MODE_IDLE:       return "IDLE";
    case MODE_MANUAL:     return "MANUAL";
    case MODE_DEPTH_HOLD: return "DEPTH_HOLD";
    case MODE_EMERGENCY:  return "EMERGENCY";
    default:              return "?";
    }
}
