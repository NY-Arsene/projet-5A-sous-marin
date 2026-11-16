/**
 * @file profondeur.h
 * @brief Pression absolue (mbar) -> profondeur (m) filtrée + vitesse verticale.
 *
 * Profondeur positive vers le bas. La référence (pression de surface) est mesurée
 * par un "tare" au lieu d'être supposée égale à 1013,25 mbar : la météo la fait
 * varier de +-30 mbar, soit +-30 cm d'eau !
 */
#ifndef PROFONDEUR_H
#define PROFONDEUR_H

#include <stdbool.h>
#include <stdint.h>
#include "filtres.h"

#define DEPTH_RHO_FRESH_WATER 997.0f
#define DEPTH_RHO_SEA_WATER   1025.0f

typedef struct {
    float    rho_kg_m3;
    float    p_ref_mbar;      /**< pression de surface de référence          */
    bool     ref_valid;       /**< false tant qu'aucun tare n'a abouti       */
    uint16_t tare_target;     /**< nb d'échantillons demandés (0 = pas de tare) */
    uint16_t tare_count;
    double   tare_sum;
    float    hist[3];         /**< 3 dernières profondeurs brutes (médiane)  */
    uint8_t  hist_n;
    ema_t    ema_depth;
    ema_t    ema_rate;
    float    depth_m;         /**< profondeur filtrée                        */
    float    rate_m_s;        /**< vitesse verticale filtrée (+ = descend)   */
    bool     have_last;
    uint32_t last_ms;
    float    last_depth;
} depth_t;

/** Conversion pure (testable) : profondeur = (P - Pref) / (rho * g). */
float depth_from_pressure(float p_mbar, float p_ref_mbar, float rho_kg_m3);

void depth_init(depth_t *d, float rho_kg_m3, float alpha_depth, float alpha_rate);
/** Lance la mesure de la référence sur n échantillons (capteur hors de l'eau / à la surface). */
void depth_tare_start(depth_t *d, uint16_t n_samples);
bool depth_is_taring(const depth_t *d);
void depth_set_reference(depth_t *d, float p_ref_mbar);
/** À appeler à chaque nouvel échantillon de pression. Renvoie false pendant le tare. */
bool depth_update(depth_t *d, float p_mbar, uint32_t now_ms);

#endif
