/**
 * @file ballast.h
 * @brief Un ballast à seringue : moteur pas à pas + vis T8 + piston, piloté en VOLUME (mL).
 *
 * Conventions (à respecter au câblage) :
 *   - position 0 pas = ballast VIDE (piston rentré, 0 mL d'eau, sous-marin plus léger) ;
 *   - la position augmente quand le ballast se REMPLIT (sous-marin plus lourd, il descend) ;
 *   - fin de course "VIDE"  : actif quand le piston est à 0 mL (broches ..._TOP de materiel.h) ;
 *   - fin de course "PLEIN" : actif quand le piston est à la course maximale (broches ..._BOTTOM).
 *
 * Cycle de vie : UNHOMED -> (ballast_home_start) -> HOMING -> READY, ou FAULT si la butée "vide" n'est
 * jamais atteinte dans le délai (ou reste bloquée). Le référencement avance vers "vide" jusqu'au contact
 * de la butée ; si elle est déjà active au départ, il recule d'abord pour la relâcher.
 *
 * Le zéro est défini au POINT D'ACTIONNEMENT de la butée "vide" (front montant en s'en approchant). Ensuite,
 * chaque front montant d'une butée recale la position estimée : on détecte ainsi les pas perdus.
 */
#ifndef BALLAST_H
#define BALLAST_H

#include <stdint.h>
#include <stdbool.h>
#include "moteur_pas_a_pas.h"

typedef enum {
    BALLAST_UNHOMED = 0,   /**< position inconnue : seule la mise à zéro est permise      */
    BALLAST_HOMING,        /**< marche vers la butée "vide"                              */
    BALLAST_READY,         /**< position connue, pilotable en volume                     */
    BALLAST_FAULT          /**< référencement échoué : à réparer avant plongée           */
} ballast_state_t;

typedef struct {
    float    steps_per_ml;         /**< pas moteur par mL : (pas/tour x micropas) / (pas de vis mm/tour) x (mm de course par mL) */
    float    max_volume_ml;        /**< volume utile de la seringue (course entre les deux butées)  */
    uint32_t tick_hz;              /**< fréquence d'appel de ballast_tick()                         */
    int32_t  max_speed_sps;        /**< vitesse normale (pas/s)                                      */
    int32_t  min_speed_sps;
    int32_t  accel_sps2;
    int32_t  homing_speed_sps;     /**< vitesse lente de référencement (pas/s)                       */
    uint32_t homing_timeout_ms;    /**< délai max pour trouver la butée "vide"                       */
    int32_t  homing_backoff_steps; /**< si la butée "vide" est déjà active au départ : recul avant de la rapprocher */
} ballast_cfg_t;

typedef struct {
    ballast_cfg_t   cfg;
    stepper_t       st;
    uint8_t         empty_limit;   /**< indice dans sensor_data_t.limit[] de la butée "vide"  */
    uint8_t         full_limit;    /**< indice de la butée "plein"                            */
    ballast_state_t state;
    int32_t         max_steps;
    uint32_t        homing_start_ms;
    bool            homing_first;    /**< true jusqu'au premier ballast_update() après le démarrage du référencement */
    bool            homing_backoff;  /**< true pendant le recul hors de la butée "vide" */
    bool            prev_empty, prev_full;   /**< états précédents des butées (détection de front) */
    uint32_t        resync_count;    /**< nb de fois où une butée a contredit la position estimée (pas perdus ?) */
} ballast_t;

/** Valeurs de départ pour une seringue 100 mL, moteur 1,8°/pas en 1/16 de pas, vis T8 (pas 8 mm/tour) : à CALIBRER. */
void ballast_default_config(ballast_cfg_t *cfg);

/** @return false si la configuration est incohérente ou si les indices de fin de course sont hors plage. */
bool ballast_init(ballast_t *b, port_motor_t motor, uint8_t empty_limit_idx, uint8_t full_limit_idx,
                  const ballast_cfg_t *cfg);

/** Démarre (ou redémarre après une panne) le référencement sur la butée "vide". */
void ballast_home_start(ballast_t *b, uint32_t now_ms);

/**
 * À appeler régulièrement (>= 50 Hz) depuis le programme principal.
 * @param limit  états anti-rebond des 4 fins de course (sensor_data_t.limit), true = contact
 */
void ballast_update(ballast_t *b, const bool limit[4], uint32_t now_ms);

/** Interruption timer : fait avancer le moteur. */
void ballast_tick(ballast_t *b);

/** Vise un volume (mL, borné à [0, max]). Refusé (false) si le ballast n'est pas READY ou si la valeur est NaN. */
bool ballast_set_volume_ml(ballast_t *b, float ml);

/** Sécurité : vide le ballast le plus vite possible. Si la position est inconnue ou en défaut, relance le
 *  référencement (qui avance justement vers "vide"). Peut être appelée à chaque cycle sans effet de bord. */
void ballast_emergency_empty(ballast_t *b, uint32_t now_ms);

float           ballast_volume_ml(const ballast_t *b);    /**< volume actuel estimé (mL) */
float           ballast_target_ml(const ballast_t *b);
bool            ballast_is_ready(const ballast_t *b);
bool            ballast_is_moving(const ballast_t *b);
ballast_state_t ballast_state(const ballast_t *b);

#endif /* BALLAST_H */
