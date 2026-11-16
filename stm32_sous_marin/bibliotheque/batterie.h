/**
 * @file batterie.h
 * @brief Niveau de batterie (accu Li-Ion / LiPo, nombre d'éléments configurable, 4S par défaut) : tension filtrée, seuils avec hystérésis, SoC indicatif.
 *
 * Le SoC déduit de la tension est INDICATIF (la tension chute en charge) ; ce sont les seuils
 * confirmés qui déclenchent le mode de sécurité.
 */
#ifndef BATTERIE_H
#define BATTERIE_H

#include <stdint.h>
#include "filtres.h"

typedef enum { BATT_OK = 0, BATT_WARNING, BATT_CRITICAL } batt_level_t;

typedef struct {
    uint8_t      cells;
    float        v_warn_cell;    /**< ex. 3.50 V/élément */
    float        v_crit_cell;    /**< ex. 3.30 V/élément */
    float        hyst_cell;      /**< ex. 0.10 V         */
    uint16_t     confirm_n;      /**< nb d'appels consécutifs avant changement de niveau */
    uint16_t     cnt;
    ema_t        ema_v;
    float        v_filt;
    float        soc_pct;
    batt_level_t level;
} battery_t;

/** SoC (%) d'un élément Li-Ion selon sa tension à vide (interpolation linéaire, saturée). */
float battery_soc_from_cell_v(float v_cell);

void battery_init(battery_t *b, uint8_t cells, float v_warn_cell, float v_crit_cell);
/** Appeler à chaque mesure de tension. Renvoie le niveau courant. */
batt_level_t battery_update(battery_t *b, float v_bus);

#endif
