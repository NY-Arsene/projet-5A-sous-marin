#include "profondeur.h"
#include "types_capteurs.h"
#include <string.h>
#include <math.h>

float depth_from_pressure(float p_mbar, float p_ref_mbar, float rho_kg_m3)
{
    return (p_mbar - p_ref_mbar) * 100.0f / (rho_kg_m3 * SENS_G_MS2);
}

void depth_init(depth_t *d, float rho_kg_m3, float alpha_depth, float alpha_rate)
{
    memset(d, 0, sizeof *d);
    d->rho_kg_m3 = rho_kg_m3;
    d->p_ref_mbar = 1013.25f;      /* valeur par défaut tant qu'il n'y a pas de tare */
    d->ref_valid = false;
    ema_init(&d->ema_depth, alpha_depth);
    ema_init(&d->ema_rate, alpha_rate);
}

void depth_tare_start(depth_t *d, uint16_t n_samples)
{
    d->tare_target = n_samples;
    d->tare_count = 0;
    d->tare_sum = 0.0;
}

bool depth_is_taring(const depth_t *d)
{
    return d->tare_target > 0;
}

static void reset_filters(depth_t *d)
{
    d->hist_n = 0;
    d->have_last = false;
    ema_reset(&d->ema_depth);
    ema_reset(&d->ema_rate);
    d->depth_m = 0.0f;
    d->rate_m_s = 0.0f;
}

void depth_set_reference(depth_t *d, float p_ref_mbar)
{
    d->p_ref_mbar = p_ref_mbar;
    d->ref_valid = true;
    reset_filters(d);
}

bool depth_update(depth_t *d, float p_mbar, uint32_t now_ms)
{
    if (!isfinite(p_mbar)) {
        return false;
    }
    if (d->tare_target > 0) {
        d->tare_sum += (double)p_mbar;
        d->tare_count++;
        if (d->tare_count >= d->tare_target) {
            depth_set_reference(d, (float)(d->tare_sum / (double)d->tare_count));
            d->tare_target = 0;
        }
        return false;
    }

    float raw = depth_from_pressure(p_mbar, d->p_ref_mbar, d->rho_kg_m3);

    /* médiane glissante sur 3 points puis passe-bas */
    if (d->hist_n < 3) {
        d->hist[d->hist_n++] = raw;
    } else {
        d->hist[0] = d->hist[1];
        d->hist[1] = d->hist[2];
        d->hist[2] = raw;
    }
    float med = (d->hist_n >= 3) ? filt_median3(d->hist[0], d->hist[1], d->hist[2]) : raw;
    d->depth_m = ema_update(&d->ema_depth, med);

    if (d->have_last) {
        float dt = (float)(now_ms - d->last_ms) / 1000.0f;
        if (dt > 0.0f && dt < 1.0f) {
            d->rate_m_s = ema_update(&d->ema_rate, (d->depth_m - d->last_depth) / dt);
        }
    }
    d->last_depth = d->depth_m;
    d->last_ms = now_ms;
    d->have_last = true;
    return true;
}
