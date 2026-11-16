/**
 * @file ms5837.h
 * @brief Driver du capteur de pression/température MS5837 (30BA ou 02BA), I2C 0x76.
 *
 * Deux modes d'utilisation :
 *  - EXPLOITABLE DANS LE SOUS-MARIN : ms5837_update() non bloquant (machine à états),
 *    à appeler le plus souvent possible depuis la boucle principale.
 *  - BANC DE TEST : ms5837_read_blocking() (une mesure complète, bloquante).
 */
#ifndef MS5837_H
#define MS5837_H

#include <stdint.h>
#include <stdbool.h>
#include "types_capteurs.h"

#define MS5837_I2C_ADDR 0x76

typedef enum { MS5837_30BA = 0, MS5837_02BA = 1 } ms5837_model_t;
typedef enum {
    MS5837_OSR_256 = 0, MS5837_OSR_512, MS5837_OSR_1024,
    MS5837_OSR_2048, MS5837_OSR_4096, MS5837_OSR_8192
} ms5837_osr_t;

typedef struct {
    float pressure_mbar;     /**< pression absolue compensée (2e ordre) */
    float temperature_c;
} ms5837_result_t;

typedef struct {
    ms5837_model_t model;
    ms5837_osr_t   osr;
    uint16_t       prom[7];          /**< coefficients d'étalonnage C0..C6 */
    uint8_t        state;
    uint32_t       t_start_ms;
    uint32_t       d1, d2;           /**< dernières valeurs brutes pression / température */
    bool           have_d2;
    uint8_t        temp_every;       /**< 1 mesure de température toutes les N pressions (défaut 8) */
    uint8_t        temp_cnt;
    float          pressure_mbar;
    float          temperature_c;
    uint32_t       last_sample_ms;
    uint32_t       sample_count;
    uint32_t       error_count;
} ms5837_t;

/** CRC4 des coefficients (algorithme de la fiche technique). */
uint8_t ms5837_crc4(const uint16_t prom[7]);
/** true si le CRC4 stocké (4 bits de poids fort de C0) correspond aux coefficients. */
bool ms5837_prom_valid(const uint16_t prom[7]);
/** Temps maximal de conversion (ms, arrondi par excès) pour un OSR donné. */
uint32_t ms5837_conv_time_ms(ms5837_osr_t osr);
/** Calcul pur (sans bus) : compensation du 1er et du 2e ordre. Testable avec des vecteurs. */
void ms5837_compute(ms5837_model_t model, const uint16_t prom[7], uint32_t d1, uint32_t d2,
                    ms5837_result_t *out);

/** Reset, lecture et vérification de la PROM. Erreurs : BUS, ID (bus mort), CRC, PARAM. */
sens_status_t ms5837_init(ms5837_t *s, ms5837_model_t model, ms5837_osr_t osr);

/**
 * Non bloquant : une transaction I2C au plus par appel.
 * @return SENS_OK            une nouvelle pression/température est disponible (s->pressure_mbar...)
 *         SENS_ERR_NOT_READY conversion en cours, rien de nouveau (NORMAL)
 *         autre              erreur (BUS, TIMEOUT, RANGE) ; la machine redémarre un cycle proprement
 */
sens_status_t ms5837_update(ms5837_t *s, uint32_t now_ms);

/** Banc de test : bloque jusqu'à une mesure complète (timeout 200 ms). */
sens_status_t ms5837_read_blocking(ms5837_t *s, ms5837_result_t *out);

#endif
