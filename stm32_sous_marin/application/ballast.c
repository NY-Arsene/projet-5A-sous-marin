#include "ballast.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

/** Écart toléré entre la position estimée et un contact de butée avant de le compter comme "pas perdus". */
#define RESYNC_TOLERANCE_STEPS 100

void ballast_default_config(ballast_cfg_t *cfg)
{
    /* Moteur 17HS19-2004S1 : 200 pas/tour, micropas 1/16 -> 3200 pas/tour ; vis T8 pas 8 mm/tour -> 400 pas/mm.
     * Seringue 100 mL de diamètre intérieur ~ 30 mm : 0,707 mL/mm -> ~ 566 pas/mL.  (valeurs à mesurer sur le montage réel) */
    cfg->steps_per_ml      = 566.0f;
    cfg->max_volume_ml     = 100.0f;
    cfg->tick_hz           = 20000u;
    cfg->max_speed_sps     = 4000;         /* 4000/400 = 10 mm/s environ */
    cfg->min_speed_sps     = 150;
    cfg->accel_sps2        = 8000;
    cfg->homing_speed_sps  = 800;
    cfg->homing_timeout_ms = 60000u;       /* 60 s : course complète à vitesse de référencement ~ 2x la marge */
    cfg->homing_backoff_steps = 800;       /* 2 mm */
}

bool ballast_init(ballast_t *b, port_motor_t motor, uint8_t empty_limit_idx, uint8_t full_limit_idx,
                  const ballast_cfg_t *cfg)
{
    if (b == NULL || cfg == NULL || empty_limit_idx > 3u || full_limit_idx > 3u || empty_limit_idx == full_limit_idx) {
        return false;
    }
    if (!(cfg->steps_per_ml > 0.0f) || !(cfg->max_volume_ml > 0.0f) || cfg->homing_speed_sps < 1 ||
        cfg->homing_speed_sps > cfg->max_speed_sps || cfg->homing_timeout_ms == 0u ||
        cfg->homing_backoff_steps < 1) {
        return false;
    }
    float steps = cfg->steps_per_ml * cfg->max_volume_ml;
    if (!(steps >= 1.0f) || steps > 1.0e8f) {
        return false;
    }
    int32_t max_steps = (int32_t)lroundf(steps);

    /* Butées logicielles [0, max]. Pendant le référencement (position encore inconnue), ballast_home_start les élargit
     * à [-2 max, max] pour pouvoir reculer jusqu'à la vraie butée "vide" ; finish_homing les remet à [0, max]. */
    stepper_cfg_t sc = {
        .tick_hz = cfg->tick_hz, .max_speed_sps = cfg->max_speed_sps, .min_speed_sps = cfg->min_speed_sps,
        .accel_sps2 = cfg->accel_sps2, .min_pos = 0, .max_pos = max_steps
    };
    if (!stepper_init(&b->st, motor, &sc)) {
        return false;
    }
    b->cfg = *cfg;
    b->empty_limit = empty_limit_idx;
    b->full_limit = full_limit_idx;
    b->max_steps = max_steps;
    b->state = BALLAST_UNHOMED;
    b->homing_start_ms = 0;
    b->homing_first = false;
    b->homing_backoff = false;
    b->prev_empty = b->prev_full = false;
    b->resync_count = 0;
    stepper_enable(&b->st, false);
    return true;
}

void ballast_home_start(ballast_t *b, uint32_t now_ms)
{
    stepper_set_limits(&b->st, -2 * b->max_steps, b->max_steps);
    stepper_set_max_speed(&b->st, b->cfg.homing_speed_sps);
    stepper_enable(&b->st, true);
    stepper_move_to(&b->st, stepper_position(&b->st) - 2 * b->max_steps);   /* loin vers "vide" : la butée arrêtera avant */
    b->homing_start_ms = now_ms;
    b->homing_first = true;
    b->homing_backoff = false;
    b->state = BALLAST_HOMING;
}

static void finish_homing(ballast_t *b)
{
    stepper_stop_now(&b->st);
    stepper_set_position(&b->st, 0);
    stepper_set_limits(&b->st, 0, b->max_steps);
    stepper_set_max_speed(&b->st, b->cfg.max_speed_sps);
    b->state = BALLAST_READY;
}

void ballast_update(ballast_t *b, const bool limit[4], uint32_t now_ms)
{
    bool empty_hit = limit[b->empty_limit];
    bool full_hit  = limit[b->full_limit];

    switch (b->state) {
    case BALLAST_HOMING:
        if (b->homing_first) {
            b->homing_first = false;
            if (empty_hit) {                   /* déjà sur la butée : on s'en éloigne d'abord pour la rapprocher proprement */
                b->homing_backoff = true;
                stepper_move_to(&b->st, stepper_position(&b->st) + b->cfg.homing_backoff_steps);
            }
        }
        if (b->homing_backoff) {
            if (!empty_hit) {                  /* butée relâchée : on repart vers "vide" */
                b->homing_backoff = false;
                stepper_move_to(&b->st, stepper_position(&b->st) - 2 * b->max_steps);
            } else if (!stepper_is_moving(&b->st)) {   /* recul terminé et butée toujours active : coincée ou mal câblée */
                stepper_enable(&b->st, false);
                b->state = BALLAST_FAULT;
            }
        } else if (empty_hit) {
            finish_homing(b);
        }
        if (b->state == BALLAST_HOMING && (uint32_t)(now_ms - b->homing_start_ms) > b->cfg.homing_timeout_ms) {
            stepper_stop_now(&b->st);
            stepper_enable(&b->st, false);
            b->state = BALLAST_FAULT;
        }
        break;

    case BALLAST_READY:
        /* Un FRONT MONTANT de butée est un contact réel : la position vraie est connue, on recale l'estimation.
         * (Le niveau, lui, reste actif quelques pas en s'éloignant : il ne prouve rien.) */
        if (empty_hit && !b->prev_empty) {
            if (labs((long)stepper_position(&b->st)) > RESYNC_TOLERANCE_STEPS) {
                b->resync_count++;
            }
            stepper_set_position(&b->st, 0);
        } else if (full_hit && !b->prev_full) {
            if (labs((long)stepper_position(&b->st) - (long)b->max_steps) > RESYNC_TOLERANCE_STEPS) {
                b->resync_count++;
            }
            stepper_set_position(&b->st, b->max_steps);
        }
        break;

    case BALLAST_UNHOMED:
    case BALLAST_FAULT:
    default:
        break;
    }
    /* Mémorisé à chaque appel, y compris à la fin du référencement : la butée "vide" active à cet instant
     * n'est alors pas vue comme un nouveau front. */
    b->prev_empty = empty_hit;
    b->prev_full = full_hit;
}

void ballast_tick(ballast_t *b)
{
    stepper_tick(&b->st);
}

bool ballast_set_volume_ml(ballast_t *b, float ml)
{
    if (b->state != BALLAST_READY || !isfinite(ml)) {
        return false;
    }
    if (ml < 0.0f) {
        ml = 0.0f;
    }
    if (ml > b->cfg.max_volume_ml) {
        ml = b->cfg.max_volume_ml;
    }
    stepper_move_to(&b->st, (int32_t)lroundf(ml * b->cfg.steps_per_ml));
    return true;
}

void ballast_emergency_empty(ballast_t *b, uint32_t now_ms)
{
    if (b->state == BALLAST_READY) {
        stepper_set_max_speed(&b->st, b->cfg.max_speed_sps);
        stepper_move_to(&b->st, 0);
    } else if (b->state != BALLAST_HOMING) {
        ballast_home_start(b, now_ms);          /* position inconnue : la marche vers "vide" est justement ce qu'on veut */
    }
}

float ballast_volume_ml(const ballast_t *b)
{
    return (b->cfg.steps_per_ml > 0.0f) ? (float)stepper_position(&b->st) / b->cfg.steps_per_ml : 0.0f;
}

float ballast_target_ml(const ballast_t *b)
{
    return (b->cfg.steps_per_ml > 0.0f) ? (float)stepper_target(&b->st) / b->cfg.steps_per_ml : 0.0f;
}

bool ballast_is_ready(const ballast_t *b)           { return b->state == BALLAST_READY; }
bool ballast_is_moving(const ballast_t *b)          { return stepper_is_moving(&b->st); }
ballast_state_t ballast_state(const ballast_t *b)   { return b->state; }
