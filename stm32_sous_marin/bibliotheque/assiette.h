/**
 * @file assiette.h
 * @brief Estimation d'assiette (roulis, tangage) et de cap (lacet) par filtre complémentaire.
 *
 * CONVENTION DES AXES (repère du sous-marin, type aéronautique) :
 *   X vers l'avant, Y vers tribord (droite), Z vers le bas.
 *   Roulis > 0 : tribord plonge. Tangage > 0 : nez vers le haut. Cap : 0 = nord, 90 = est (sens horaire).
 *   L'accéléromètre mesure la force spécifique : à plat et immobile, sa lecture est (0, 0, -1) g.
 *   Les capteurs réels sont à remapper dans ce repère avec attitude_remap_axes().
 */
#ifndef ASSIETTE_H
#define ASSIETTE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;      /**< cap magnétique [0,360[ si magnétomètre fourni, sinon intégré du gyro */
    bool  init;
    bool  yaw_from_mag; /**< true si le cap est recalé par le magnétomètre */
    float k_acc;        /**< gain de correction accéléromètre par échantillon (ex. 0.02) */
    float k_mag;        /**< gain de correction magnétomètre par échantillon (ex. 0.01)  */
} attitude_t;

void attitude_init(attitude_t *a, float k_acc, float k_mag);

/**
 * @param acc_g     accéléromètre en g (repère sous-marin), 3 valeurs
 * @param gyro_dps  gyroscope en deg/s (repère sous-marin), 3 valeurs, biais déjà retiré
 * @param mag       magnétomètre (toute unité, repère sous-marin) ou NULL si indisponible
 * @param dt_s      pas de temps entre deux appels, en secondes
 */
void attitude_update(attitude_t *a, const float acc_g[3], const float gyro_dps[3],
                     const float mag[3], float dt_s);

/** Cap magnétique compensé de l'inclinaison (fonction pure, testable). */
float attitude_heading_deg(float roll_deg, float pitch_deg, const float mag[3]);

/**
 * Remappe les axes d'un capteur vers le repère sous-marin.
 * map[i] = +-(1..3) : out[i] = signe * in[|map[i]|-1]. Ex. {2,-1,3} : out = (in.y, -in.x, in.z).
 * Renvoie false (et sortie à 0) si la table est invalide.
 */
bool attitude_remap_axes(const float in[3], const int8_t map[3], float out[3]);

#endif
