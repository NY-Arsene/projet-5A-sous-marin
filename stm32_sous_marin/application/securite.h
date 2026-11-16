/**
 * @file securite.h
 * @brief Machine d'états du sous-marin et logique de sécurité (remontée d'urgence).
 *
 *   INIT ──(référencement ballasts + tare de profondeur terminés)──> IDLE
 *   IDLE        : ballasts vides, le sous-marin flotte. Aucune régulation.
 *   MANUAL      : l'opérateur commande directement le volume des ballasts.
 *   DEPTH_HOLD  : régulation automatique de profondeur et d'assiette.
 *   EMERGENCY   : ballasts vidés au maximum -> remontée. MÉMORISÉ : on n'en sort que par un
 *                 acquittement explicite de l'opérateur ET lorsque les causes ont disparu.
 *
 * Causes d'urgence (mémorisées dans reasons) :
 *   toujours      : fuite (FS_LEAK), batterie critique (FS_BATT_CRIT), profondeur maximale dépassée (FS_OVER_DEPTH),
 *                   arrêt d'urgence demandé par l'opérateur (SAFE_OPERATOR) ;
 *   en MANUAL / DEPTH_HOLD seulement : capteur de pression perdu (FS_DEPTH_LOST), liaison perdue alors que le
 *                   sous-marin est immergé (SAFE_LINK_LOST), ballast en défaut (SAFE_BALLAST_FAULT).
 * En IDLE les ballasts sont déjà vides : le sous-marin est dans l'état le plus sûr, ces trois causes n'y comptent pas.
 * Batterie faible (FS_BATT_WARN) et IMU perdue (FS_IMU_LOST) ne déclenchent PAS d'urgence : elles sont signalées.
 */
#ifndef SECURITE_H
#define SECURITE_H

#include <stdint.h>
#include <stdbool.h>
#include "gestionnaire_capteurs.h"      /* bits FS_* */

#define SAFE_LINK_LOST      (1u << 8)
#define SAFE_BALLAST_FAULT  (1u << 9)
#define SAFE_OPERATOR       (1u << 10)

typedef enum { MODE_INIT = 0, MODE_IDLE, MODE_MANUAL, MODE_DEPTH_HOLD, MODE_EMERGENCY } sub_mode_t;

/** Modes demandables par l'opérateur : valeurs identiques à celles du protocole. */
typedef enum { OP_IDLE = 0, OP_MANUAL = 1, OP_DEPTH_HOLD = 2 } op_mode_t;

typedef enum { SAFE_REQ_OK = 0, SAFE_REQ_REJECTED = 1, SAFE_REQ_INVALID = 2 } safe_req_t;

typedef struct {
    uint32_t link_timeout_ms;     /**< sans trame valide de la Raspberry Pi depuis ce délai : liaison perdue */
    float    submerged_depth_m;   /**< en dessous de cette profondeur on est "immergé"                         */
    uint32_t init_timeout_ms;     /**< au-delà, on quitte INIT même si le référencement ou la tare ont échoué  */
} safety_cfg_t;

typedef struct {
    uint32_t failsafe_mask;       /**< FS_* du gestionnaire de capteurs               */
    bool     depth_ref_valid;
    float    depth_m;             /**< NaN : profondeur inconnue => considérée immergée */
    bool     ballasts_ready;      /**< les deux ballasts sont référencés (READY)      */
    bool     ballast_fault;       /**< au moins un ballast en FAULT                   */
    bool     homing_done;         /**< aucun ballast n'est encore en train de se référencer */
    uint32_t now_ms;
} safety_inputs_t;

typedef struct {
    safety_cfg_t cfg;
    sub_mode_t   mode;
    uint32_t     reasons;         /**< causes mémorisées (FS_* | SAFE_*) */
    uint32_t     start_ms;
    uint32_t     last_link_ms;
    bool         clear_requested;
} safety_t;

void safety_default_config(safety_cfg_t *cfg);
void safety_init(safety_t *s, const safety_cfg_t *cfg, uint32_t now_ms);

/** À appeler pour chaque trame valide reçue de la Raspberry Pi. */
void safety_link_activity(safety_t *s, uint32_t now_ms);

/** Demande de mode par l'opérateur ; la décision est immédiate. */
safe_req_t safety_request_mode(safety_t *s, int op_mode, const safety_inputs_t *in);

/** Arrêt d'urgence demandé par l'opérateur : passe tout de suite en EMERGENCY. */
void safety_emergency_stop(safety_t *s);

/** Acquittement : sortie d'EMERGENCY au prochain safety_update(), si plus aucune cause n'est active. */
void safety_clear_fault(safety_t *s);

/** Un cycle (>= 20 Hz) : évalue les causes, met à jour le mode. */
void safety_update(safety_t *s, const safety_inputs_t *in);

sub_mode_t  safety_mode(const safety_t *s);
uint32_t    safety_reasons(const safety_t *s);
const char *safety_mode_name(sub_mode_t m);

#endif
