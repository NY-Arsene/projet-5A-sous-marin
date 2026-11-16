#include "assiette.h"
#include "filtres.h"
#include "types_capteurs.h"
#include <math.h>
#include <stddef.h>

static float norm3(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void attitude_init(attitude_t *a, float k_acc, float k_mag)
{
    a->roll_deg = 0.0f;
    a->pitch_deg = 0.0f;
    a->yaw_deg = 0.0f;
    a->init = false;
    a->yaw_from_mag = false;
    a->k_acc = k_acc;
    a->k_mag = k_mag;
}

float attitude_heading_deg(float roll_deg, float pitch_deg, const float m[3])
{
    float phi = roll_deg * SENS_DEG2RAD;
    float th  = pitch_deg * SENS_DEG2RAD;
    float sphi = sinf(phi), cphi = cosf(phi);
    float sth = sinf(th),  cth = cosf(th);
    float bfx = m[0] * cth + m[1] * sth * sphi + m[2] * sth * cphi;
    float bfy = m[1] * cphi - m[2] * sphi;
    return wrap360(atan2f(-bfy, bfx) * SENS_RAD2DEG);
}

/* Angles d'inclinaison "gravité" : g_b = -(lecture de l'accéléromètre) */
static void tilt_from_acc(const float acc_g[3], float *roll_deg, float *pitch_deg)
{
    float gx = -acc_g[0], gy = -acc_g[1], gz = -acc_g[2];
    *roll_deg  = atan2f(gy, gz) * SENS_RAD2DEG;
    *pitch_deg = atan2f(-gx, sqrtf(gy * gy + gz * gz)) * SENS_RAD2DEG;
}

static bool mag_usable(const float *mag)
{
    return (mag != NULL) && isfinite(mag[0]) && isfinite(mag[1]) && isfinite(mag[2])
           && (norm3(mag) > 1e-3f);
}

void attitude_update(attitude_t *a, const float acc_g[3], const float gyro_dps[3],
                     const float mag[3], float dt_s)
{
    float acc_n = norm3(acc_g);
    bool acc_ok = isfinite(acc_n) && (acc_n > 0.05f);
    bool mag_ok = mag_usable(mag);

    if (!a->init) {
        if (acc_ok) {
            tilt_from_acc(acc_g, &a->roll_deg, &a->pitch_deg);
        }
        a->yaw_deg = mag_ok ? attitude_heading_deg(a->roll_deg, a->pitch_deg, mag) : 0.0f;
        a->yaw_from_mag = mag_ok;
        a->init = true;
        return;
    }
    if (!(dt_s > 0.0f) || !isfinite(dt_s)) {
        return;
    }

    /* 1) Prédiction : intégration des vitesses angulaires (équations d'Euler) */
    float phi = a->roll_deg * SENS_DEG2RAD;
    float th  = a->pitch_deg * SENS_DEG2RAD;
    float p = gyro_dps[0] * SENS_DEG2RAD;
    float q = gyro_dps[1] * SENS_DEG2RAD;
    float r = gyro_dps[2] * SENS_DEG2RAD;
    float sphi = sinf(phi), cphi = cosf(phi);
    float cth = cosf(th);
    if (fabsf(cth) < 0.2f) {                 /* évite la singularité à +-90 deg de tangage */
        cth = (cth < 0.0f) ? -0.2f : 0.2f;
    }
    float tth = sinf(th) / cth;
    float phi_dot = p + (q * sphi + r * cphi) * tth;
    float th_dot  = q * cphi - r * sphi;
    float psi_dot = (q * sphi + r * cphi) / cth;

    a->roll_deg  += phi_dot * SENS_RAD2DEG * dt_s;
    a->pitch_deg += th_dot  * SENS_RAD2DEG * dt_s;
    a->yaw_deg   += psi_dot * SENS_RAD2DEG * dt_s;

    /* 2) Correction par l'accéléromètre, désactivée en cas d'accélération dynamique */
    if (acc_ok) {
        /* poids de la correction : 1 si la norme vaut 1 g +-0,1 ; décroît linéairement jusqu'à 0 à +-0,3 g */
        float gate = (0.3f - fabsf(acc_n - 1.0f)) / 0.2f;
        if (gate > 1.0f) gate = 1.0f;
        if (gate > 0.0f) {
            float r_acc, p_acc;
            tilt_from_acc(acc_g, &r_acc, &p_acc);
            a->roll_deg  += a->k_acc * gate * wrap180(r_acc - a->roll_deg);
            a->pitch_deg += a->k_acc * gate * wrap180(p_acc - a->pitch_deg);
        }
    }

    /* 3) Correction du cap par le magnétomètre (si disponible) */
    if (mag_ok) {
        float h = attitude_heading_deg(a->roll_deg, a->pitch_deg, mag);
        if (!a->yaw_from_mag) {
            a->yaw_deg = h;
            a->yaw_from_mag = true;
        } else {
            a->yaw_deg += a->k_mag * wrap180(h - a->yaw_deg);
        }
    }

    a->roll_deg  = wrap180(a->roll_deg);
    a->pitch_deg = wrap180(a->pitch_deg);
    a->yaw_deg   = wrap360(a->yaw_deg);
}

bool attitude_remap_axes(const float in[3], const int8_t map[3], float out[3])
{
    float tmp[3];
    bool ok = true;
    for (int i = 0; i < 3; i++) {
        int m = map[i];
        int idx = (m < 0) ? -m : m;
        if (idx < 1 || idx > 3) {
            ok = false;
            break;
        }
        tmp[i] = (m < 0 ? -1.0f : 1.0f) * in[idx - 1];
    }
    for (int i = 0; i < 3; i++) {
        out[i] = ok ? tmp[i] : 0.0f;
    }
    return ok;
}
