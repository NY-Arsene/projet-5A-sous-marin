/**
 * @file types_capteurs.h
 * @brief Types et constantes communs à tout le code capteurs du sous-marin.
 */
#ifndef TYPES_CAPTEURS_H
#define TYPES_CAPTEURS_H

#include <stdint.h>
#include <stdbool.h>

#define SENS_PI       3.14159265358979323846f
#define SENS_G_MS2    9.80665f
#define SENS_DEG2RAD  (SENS_PI / 180.0f)
#define SENS_RAD2DEG  (180.0f / SENS_PI)

/** Code retour unique pour tous les drivers : jamais d'exception, toujours un statut explicite. */
typedef enum {
    SENS_OK = 0,
    SENS_ERR_BUS,        /**< NACK / timeout I2C / 1-Wire absent            */
    SENS_ERR_ID,         /**< WHO_AM_I ou ID constructeur incorrect         */
    SENS_ERR_CRC,        /**< CRC PROM / scratchpad invalide                */
    SENS_ERR_PARAM,      /**< paramètre invalide (pointeur NULL, enum...)   */
    SENS_ERR_NOT_READY,  /**< pas de nouvelle donnée (ce n'est PAS une panne) */
    SENS_ERR_RANGE,      /**< valeur physiquement impossible / hors plage   */
    SENS_ERR_TIMEOUT     /**< le capteur ne répond plus dans le temps imparti */
} sens_status_t;

/** Texte lisible d'un statut (pour les traces du banc de test). */
const char *sens_status_str(sens_status_t s);

#endif /* TYPES_CAPTEURS_H */
