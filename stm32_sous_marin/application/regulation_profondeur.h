/**
 * @file regulation_profondeur.h
 * @brief Régulation de profondeur et d'assiette par deux ballasts (avant / arrière).
 *
 *  - Régulateur de PROFONDEUR : l'erreur de profondeur donne un volume COMMUN aux deux ballasts
 *    (plus d'eau = plus lourd = on descend). Flottabilité neutre = v_neutral_ml dans chaque ballast.
 *  - Régulateur de TANGAGE : l'erreur de tangage donne une DIFFÉRENCE de volume avant - arrière
 *    (ballast avant plus plein = nez vers le bas).
 *  - La consigne de profondeur est rampée (vitesse limitée) pour éviter les à-coups.
 *
 * Les gains par défaut sont des points de départ issus d'un modèle simplifié : ils DOIVENT être réglés
 * au bassin (voir le message SET_PID du protocole, qui permet de les changer sans recompiler).
 */
#ifndef REGULATION_PROFONDEUR_H
#define REGULATION_PROFONDEUR_H

#include <stdbool.h>
#include "pid.h"

typedef struct {
    float kp_depth, ki_depth, kd_depth;   /**< mL / m,  mL / (m.s),  mL / (m/s)          */
    float kp_pitch, ki_pitch, kd_pitch;   /**< mL / deg, mL / (deg.s), mL / (deg/s)      */
    float v_neutral_ml;                   /**< volume de chaque ballast à flottabilité neutre */
    float max_volume_ml;                  /**< course de chaque ballast                  */
    float max_diff_ml;                    /**< écart maximal avant - arrière             */
    float max_setpoint_m;                 /**< consigne de profondeur maximale acceptée  */
    float setpoint_rate_m_s;              /**< vitesse maximale de variation de la consigne */
    float pitch_sign;                     /**< +1 : ballast avant plus plein => nez vers le bas ; -1 si le montage est inversé */
    float i_zone_depth_m;                 /**< l'intégrale de profondeur n'agit que si |erreur| <= cette valeur (m) */
    float bumpless_min_depth_m;           /**< en dessous de cette profondeur on part de la flottabilité neutre, sinon des volumes actuels */
} depth_ctrl_cfg_t;

typedef struct {
    depth_ctrl_cfg_t cfg;
    pid_ctrl_t depth_pid;
    pid_ctrl_t pitch_pid;
    float setpoint_m;        /**< consigne demandée                         */
    float setpoint_eff_m;    /**< consigne après limitation de vitesse      */
    float front_ml, aft_ml;  /**< volumes à envoyer aux ballasts            */
} depth_ctrl_t;

void depth_ctrl_default_config(depth_ctrl_cfg_t *cfg);
bool depth_ctrl_init(depth_ctrl_t *dc, const depth_ctrl_cfg_t *cfg);

/**
 * Démarre la régulation : consigne = profondeur actuelle.
 *  - sous-marin déjà immergé (profondeur >= bumpless_min_depth_m) : l'intégrale reprend les volumes actuels (SANS À-COUP) ;
 *  - sous-marin en surface, ballasts vides : l'intégrale part de la flottabilité neutre. Reprendre les volumes actuels
 *    (0 mL) forcerait l'intégrale tout en bas et la plongée mettrait plus de deux minutes à démarrer. Le saut de consigne
 *    de volume n'est pas brutal pour autant : les moteurs sont limités en vitesse.
 */
void depth_ctrl_start(depth_ctrl_t *dc, float depth_m, float front_ml, float aft_ml);

/** Nouvelle consigne (m). Refusée (false) si non finie ou hors de [0 ; max_setpoint_m]. */
bool depth_ctrl_set_depth(depth_ctrl_t *dc, float depth_m);

/** Modifie les gains d'un régulateur : which = 0 profondeur, 1 tangage. */
bool depth_ctrl_set_gains(depth_ctrl_t *dc, int which, float kp, float ki, float kd);

/**
 * Un cycle de régulation (typiquement 100 Hz).
 * Profondeur invalide (NaN) : les volumes précédents sont conservés (la sécurité décide quoi faire).
 * Tangage invalide : la correction de tangage est neutralisée (différence ramenée à 0).
 */
void depth_ctrl_update(depth_ctrl_t *dc, float depth_m, float depth_rate_m_s,
                       float pitch_deg, float pitch_rate_dps, bool pitch_valid, float dt_s);

#endif
