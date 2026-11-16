#include "ds18b20.h"
#include "materiel.h"
#include <stddef.h>
#include <string.h>

#define CMD_SKIP_ROM    0xCC
#define CMD_CONVERT_T   0x44
#define CMD_READ_SCRATCH 0xBE
#define CONV_TIME_MS    800u      /* 750 ms max en 12 bits + marge */
#define RAW_POWER_ON    0x0550    /* 85,0 degC : valeur du registre avant toute conversion */

enum { ST_IDLE = 0, ST_CONVERTING };

uint8_t ds18b20_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        for (int b = 0; b < 8; b++) {
            uint8_t mix = (uint8_t)((crc ^ byte) & 0x01);
            crc >>= 1;
            if (mix) {
                crc ^= 0x8C;
            }
            byte >>= 1;
        }
    }
    return crc;
}

float ds18b20_raw_to_c(int16_t raw)
{
    return (float)raw / 16.0f;
}

sens_status_t ds18b20_init(ds18b20_t *s)
{
    if (s == NULL) {
        return SENS_ERR_PARAM;
    }
    memset(s, 0, sizeof *s);
    return port_ow_reset() ? SENS_OK : SENS_ERR_BUS;
}

sens_status_t ds18b20_update(ds18b20_t *s, uint32_t now_ms)
{
    if (s->state == ST_IDLE) {
        if (!port_ow_reset()) {
            s->valid = false;
            s->error_count++;
            return SENS_ERR_BUS;
        }
        port_ow_write_byte(CMD_SKIP_ROM);
        port_ow_write_byte(CMD_CONVERT_T);
        s->t_start_ms = now_ms;
        s->state = ST_CONVERTING;
        return SENS_ERR_NOT_READY;
    }

    if ((uint32_t)(now_ms - s->t_start_ms) < CONV_TIME_MS) {
        return SENS_ERR_NOT_READY;
    }
    s->state = ST_IDLE;                    /* quoi qu'il arrive, le prochain appel relance un cycle */

    if (!port_ow_reset()) {
        s->valid = false;
        s->error_count++;
        return SENS_ERR_BUS;
    }
    port_ow_write_byte(CMD_SKIP_ROM);
    port_ow_write_byte(CMD_READ_SCRATCH);
    uint8_t sp[9];
    for (int i = 0; i < 9; i++) {
        sp[i] = port_ow_read_byte();
    }
    if (ds18b20_crc8(sp, 9) != 0) {        /* CRC sur 9 octets (CRC inclus) = 0 si tout va bien */
        s->valid = false;
        s->error_count++;
        return SENS_ERR_CRC;
    }
    int16_t raw = (int16_t)(uint16_t)((uint16_t)sp[0] | ((uint16_t)sp[1] << 8));
    if (raw == RAW_POWER_ON && !s->seen_good) {
        return SENS_ERR_NOT_READY;         /* conversion pas encore faite : valeur de démarrage ignorée */
    }
    float t = ds18b20_raw_to_c(raw);
    if (t < -55.0f || t > 125.0f) {
        s->valid = false;
        s->error_count++;
        return SENS_ERR_RANGE;
    }
    s->temp_c = t;
    s->valid = true;
    s->seen_good = true;
    s->last_ok_ms = now_ms;
    return SENS_OK;
}
