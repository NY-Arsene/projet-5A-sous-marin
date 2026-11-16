#include "filtres.h"
#include "types_capteurs.h"
#include <math.h>

float filt_median3(float a, float b, float c)
{
    float lo = (a < b) ? a : b;
    float hi = (a < b) ? b : a;
    float m  = (hi < c) ? hi : c;
    return (lo > m) ? lo : m;
}

void ema_init(ema_t *e, float alpha)
{
    e->alpha = alpha;
    e->y = 0.0f;
    e->init = false;
}

void ema_reset(ema_t *e)
{
    e->y = 0.0f;
    e->init = false;
}

float ema_update(ema_t *e, float x)
{
    if (!isfinite(x)) {
        return e->y;
    }
    if (!e->init) {
        e->y = x;
        e->init = true;
    } else {
        e->y += e->alpha * (x - e->y);
    }
    return e->y;
}

float ema_alpha_from_fc(float fc_hz, float dt_s)
{
    float rc = 1.0f / (2.0f * SENS_PI * fc_hz);
    return dt_s / (dt_s + rc);
}

float wrap180(float deg)
{
    float x = fmodf(deg + 180.0f, 360.0f);
    if (x < 0.0f) {
        x += 360.0f;
    }
    return x - 180.0f;
}

float wrap360(float deg)
{
    float x = fmodf(deg, 360.0f);
    if (x < 0.0f) {
        x += 360.0f;
    }
    return x;
}
