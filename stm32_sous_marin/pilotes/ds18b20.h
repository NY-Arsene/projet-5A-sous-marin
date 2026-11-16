/**
 * @file ds18b20.h
 * @brief Sonde de température DS18B20 (1-Wire), UN SEUL composant sur le bus (commande Skip ROM).
 *
 * Mode non bloquant : une conversion 12 bits dure 750 ms ; ds18b20_update() lance la conversion
 * puis lit le résultat plus tard. Les accès 1-Wire (bit-banging) bloquent ~1 ms (reset) et ~6 ms
 * (lecture du scratchpad) : à appeler depuis une tâche de basse priorité, PAS depuis la boucle PID.
 */
#ifndef DS18B20_H
#define DS18B20_H

#include <stdint.h>
#include <stdbool.h>
#include "types_capteurs.h"

typedef struct {
    uint8_t  state;
    uint32_t t_start_ms;
    float    temp_c;
    bool     valid;
    bool     seen_good;      /**< au moins une mesure valide déjà reçue */
    uint32_t last_ok_ms;
    uint32_t error_count;
} ds18b20_t;

/** CRC-8 Dallas/Maxim (polynôme x^8+x^5+x^4+1) sur len octets. Le CRC d'un bloc + son CRC vaut 0. */
uint8_t ds18b20_crc8(const uint8_t *data, uint8_t len);
/** Conversion de la valeur brute 16 bits (1/16 degC par LSB, signée). */
float ds18b20_raw_to_c(int16_t raw);

/** Vérifie la présence de la sonde (impulsion de présence). */
sens_status_t ds18b20_init(ds18b20_t *s);
/**
 * Non bloquant. SENS_OK : nouvelle température dans s->temp_c.
 * SENS_ERR_NOT_READY : conversion en cours. BUS / CRC / RANGE : erreur (le cycle redémarre).
 */
sens_status_t ds18b20_update(ds18b20_t *s, uint32_t now_ms);

#endif
