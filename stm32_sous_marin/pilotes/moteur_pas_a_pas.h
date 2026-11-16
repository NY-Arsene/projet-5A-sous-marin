/**
 * @file moteur_pas_a_pas.h
 * @brief Générateur d'impulsions STEP pour un moteur pas à pas, avec rampe d'accélération.
 *
 * Principe : stepper_tick() est appelée à fréquence FIXE (ex. 20 kHz) par l'interruption d'un timer.
 * À chaque appel, la vitesse monte, descend ou reste constante (profil trapézoïdal), puis un
 * accumulateur de phase décide s'il faut émettre une impulsion STEP à ce tick. Aucun calcul flottant,
 * aucune boucle d'attente : le coût par appel est de quelques dizaines de cycles.
 *
 * Le programme principal ne fait que stepper_move_to() ; l'interruption fait le reste.
 */
#ifndef MOTEUR_PAS_A_PAS_H
#define MOTEUR_PAS_A_PAS_H

#include <stdint.h>
#include <stdbool.h>
#include "materiel.h"

typedef struct {
    uint32_t tick_hz;         /**< fréquence d'appel de stepper_tick()                          */
    int32_t  max_speed_sps;   /**< vitesse maximale en pas/s (doit être <= tick_hz / 2)         */
    int32_t  min_speed_sps;   /**< vitesse de fin de course d'approche (évite de caler/s'arrêter avant la cible) */
    int32_t  accel_sps2;      /**< accélération en pas/s²                                       */
    int32_t  min_pos;         /**< butée logicielle basse (pas)                                 */
    int32_t  max_pos;         /**< butée logicielle haute (pas)                                 */
} stepper_cfg_t;

typedef struct {
    stepper_cfg_t     cfg;
    port_motor_t      motor;
    int32_t           dv_q8;       /**< incrément de vitesse par tick (pas/s x 256)             */
    int32_t           tick_q8;     /**< tick_hz x 256 : seuil de l'accumulateur de phase        */
    volatile int32_t  position;    /**< position courante en pas (modifiée dans l'interruption) */
    volatile int32_t  target;      /**< position visée en pas                                   */
    int32_t           v_q8;        /**< vitesse courante (pas/s x 256), toujours >= 0           */
    int32_t           phase_q8;
    int8_t            dir;         /**< +1 : la position augmente, -1 : elle diminue            */
    volatile bool     enabled;
    uint32_t          pulses;      /**< nombre total d'impulsions émises (diagnostic)           */
} stepper_t;

/** Valide la configuration et prépare l'état. Renvoie false (et ne touche à rien) si elle est incohérente. */
bool stepper_init(stepper_t *s, port_motor_t motor, const stepper_cfg_t *cfg);

/** Alimente ou coupe le driver. Désactivé : stepper_tick() ne fait rien. */
void stepper_enable(stepper_t *s, bool on);

/** Vise une position (bornée aux butées logicielles). */
void stepper_move_to(stepper_t *s, int32_t target);

/** Arrêt immédiat à la position courante (sans rampe) : à utiliser quand une butée matérielle est atteinte. */
void stepper_stop_now(stepper_t *s);

/** Redéfinit la position courante (après référencement sur une butée). Arrête aussi le mouvement. */
void stepper_set_position(stepper_t *s, int32_t pos);

/** Change les butées logicielles. */
void stepper_set_limits(stepper_t *s, int32_t min_pos, int32_t max_pos);

/** Change la vitesse maximale (pas/s) ; renvoie false si hors plage. */
bool stepper_set_max_speed(stepper_t *s, int32_t sps);

int32_t stepper_position(const stepper_t *s);
int32_t stepper_target(const stepper_t *s);
bool    stepper_is_moving(const stepper_t *s);

/** À appeler à fréquence fixe cfg.tick_hz depuis une interruption de timer. */
void stepper_tick(stepper_t *s);

#endif /* MOTEUR_PAS_A_PAS_H */
