/**
 * @file pid.h
 * @brief Régulateur PID avec saturation, anti-emballement de l'intégrale et dérivée sur la MESURE.
 *
 * Particularités voulues pour un sous-marin :
 *  - la dérivée n'est pas calculée par différence (bruitée) : l'appelant fournit la vitesse mesurée
 *    (vitesse verticale du capteur de pression, vitesse angulaire du gyroscope) ;
 *  - la dérivée agit sur la mesure et non sur l'erreur : pas de "coup" quand la consigne change ;
 *  - l'intégrale est bornée par la plage de sortie et n'augmente plus tant que la sortie est saturée
 *    dans le sens de l'erreur (anti-windup).
 *
 *  sortie = kp * (consigne - mesure) + I - kd * vitesse_mesurée
 */
#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    float kp, ki, kd;
    float out_min, out_max;
    float i_zone;          /**< l'intégrale n'agit que si |erreur| <= i_zone (0 = toujours) : évite l'accumulation pendant les grandes manoeuvres */
    float i_term;
    float last_out;
} pid_ctrl_t;

/** Renvoie false (régulateur inchangé) si un paramètre n'est pas fini, si un gain est négatif ou si out_min >= out_max. */
bool pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float out_min, float out_max);

/** Change les gains sans toucher à l'état. Mêmes règles de validation que pid_init. */
bool pid_set_gains(pid_ctrl_t *p, float kp, float ki, float kd);

/** Zone d'action de l'intégrale (même unité que l'erreur). 0 la désactive ; refusé si négatif ou non fini. */
bool pid_set_i_zone(pid_ctrl_t *p, float zone);

/** Remet l'intégrale à une valeur donnée (transfert sans à-coup : y mettre la sortie actuelle de l'actionneur). */
void pid_reset(pid_ctrl_t *p, float i_term);

/**
 * @param meas_rate  vitesse mesurée de la grandeur régulée (même unité que mesure / seconde)
 * @param dt_s       pas de temps. Hors de ]0 ; 1] s ou entrée non finie : rien n'est mis à jour, la dernière sortie est renvoyée.
 */
float pid_update(pid_ctrl_t *p, float setpoint, float meas, float meas_rate, float dt_s);

#endif
