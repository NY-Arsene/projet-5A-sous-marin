/**
 * @file calibration_magneto.h
 * @brief Calibration du magnétomètre : fer dur (offset) + fer doux simplifié (échelle par axe).
 *
 * Procédure : alimenter le sous-marin comme en mission, tourner le module dans toutes les
 * orientations pendant ~20 s en appelant mag_cal_feed(), puis mag_cal_finish().
 */
#ifndef CALIBRATION_MAGNETO_H
#define CALIBRATION_MAGNETO_H

#include <stdint.h>
#include <stdbool.h>
#include "types_capteurs.h"

typedef struct {
    float    min[3];
    float    max[3];
    uint32_t n;
    float    offset[3];   /**< fer dur, même unité que la mesure */
    float    scale[3];    /**< fer doux (échelle par axe)         */
    bool     valid;
} mag_cal_t;

void mag_cal_init(mag_cal_t *c);
void mag_cal_feed(mag_cal_t *c, const float m[3]);
/**
 * @param min_span  étendue minimale exigée sur chaque axe (même unité que la mesure).
 *                  Garantit que l'on a bien tourné dans toutes les directions.
 * @return SENS_OK, SENS_ERR_NOT_READY (trop peu d'échantillons) ou SENS_ERR_RANGE (couverture insuffisante)
 */
sens_status_t mag_cal_finish(mag_cal_t *c, float min_span);
void mag_cal_apply(const mag_cal_t *c, const float in[3], float out[3]);

#endif
