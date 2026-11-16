#include "calibration_magneto.h"
#include <float.h>
#include <math.h>

#define MAG_CAL_MIN_SAMPLES 50u

void mag_cal_init(mag_cal_t *c)
{
    for (int i = 0; i < 3; i++) {
        c->min[i] = FLT_MAX;
        c->max[i] = -FLT_MAX;
        c->offset[i] = 0.0f;
        c->scale[i] = 1.0f;
    }
    c->n = 0;
    c->valid = false;
}

void mag_cal_feed(mag_cal_t *c, const float m[3])
{
    for (int i = 0; i < 3; i++) {
        if (!isfinite(m[i])) {
            return;                         /* échantillon corrompu : ignoré en entier */
        }
    }
    for (int i = 0; i < 3; i++) {
        if (m[i] < c->min[i]) c->min[i] = m[i];
        if (m[i] > c->max[i]) c->max[i] = m[i];
    }
    c->n++;
}

sens_status_t mag_cal_finish(mag_cal_t *c, float min_span)
{
    if (c->n < MAG_CAL_MIN_SAMPLES) {
        return SENS_ERR_NOT_READY;
    }
    float r[3];
    for (int i = 0; i < 3; i++) {
        float span = c->max[i] - c->min[i];
        if (span < min_span) {
            return SENS_ERR_RANGE;
        }
        r[i] = span * 0.5f;
    }
    float avg = (r[0] + r[1] + r[2]) / 3.0f;
    for (int i = 0; i < 3; i++) {
        c->offset[i] = (c->max[i] + c->min[i]) * 0.5f;
        c->scale[i] = avg / r[i];
    }
    c->valid = true;
    return SENS_OK;
}

void mag_cal_apply(const mag_cal_t *c, const float in[3], float out[3])
{
    for (int i = 0; i < 3; i++) {
        out[i] = (in[i] - c->offset[i]) * c->scale[i];
    }
}
