#include "ms5837.h"
#include "materiel.h"
#include <string.h>

#define CMD_RESET        0x1E
#define CMD_ADC_READ     0x00
#define CMD_PROM_BASE    0xA0
#define CMD_CONV_D1      0x40
#define CMD_CONV_D2      0x50
#define TIMEOUT_MARGIN   50u      /* ms au-delà du temps de conversion avant de déclarer une panne */

enum { ST_START_D1 = 0, ST_WAIT_D1, ST_WAIT_D2 };

static const uint8_t k_conv_ms[6] = { 1, 2, 3, 5, 10, 19 };

uint32_t ms5837_conv_time_ms(ms5837_osr_t osr)
{
    return ((unsigned)osr <= (unsigned)MS5837_OSR_8192) ? k_conv_ms[osr] : 19u;
}

uint8_t ms5837_crc4(const uint16_t prom[7])
{
    uint16_t n[8];
    for (int i = 0; i < 7; i++) {
        n[i] = prom[i];
    }
    n[0] &= 0x0FFF;                    /* le CRC lui-même est exclu du calcul */
    n[7] = 0;
    uint16_t rem = 0;
    for (int i = 0; i < 16; i++) {
        if (i & 1) {
            rem ^= (uint16_t)(n[i >> 1] & 0x00FF);
        } else {
            rem ^= (uint16_t)(n[i >> 1] >> 8);
        }
        for (int b = 8; b > 0; b--) {
            if (rem & 0x8000) {
                rem = (uint16_t)((rem << 1) ^ 0x3000);
            } else {
                rem = (uint16_t)(rem << 1);
            }
        }
    }
    return (uint8_t)((rem >> 12) & 0x0F);
}

bool ms5837_prom_valid(const uint16_t prom[7])
{
    return (uint8_t)(prom[0] >> 12) == ms5837_crc4(prom);
}

void ms5837_compute(ms5837_model_t model, const uint16_t prom[7], uint32_t d1, uint32_t d2,
                    ms5837_result_t *out)
{
    const int64_t C1 = prom[1], C2 = prom[2], C3 = prom[3], C4 = prom[4], C5 = prom[5], C6 = prom[6];
    const int64_t D1 = (int64_t)d1;
    int64_t dT = (int64_t)d2 - C5 * 256;
    int64_t SENS, OFF;
    int64_t TEMP = 2000 + (dT * C6) / 8388608LL;

    if (model == MS5837_02BA) {
        SENS = C1 * 65536 + (C3 * dT) / 128;
        OFF  = C2 * 131072 + (C4 * dT) / 64;
    } else {
        SENS = C1 * 32768 + (C3 * dT) / 256;
        OFF  = C2 * 65536 + (C4 * dT) / 128;
    }

    /* compensation du second ordre (basses et hautes températures) */
    int64_t Ti = 0, OFFi = 0, SENSi = 0;
    int64_t t2 = TEMP - 2000;
    if (model == MS5837_02BA) {
        if ((TEMP / 100) < 20) {
            Ti    = (11 * dT * dT) / 34359738368LL;
            OFFi  = (31 * t2 * t2) / 8;
            SENSi = (63 * t2 * t2) / 32;
        }
    } else {
        if ((TEMP / 100) < 20) {
            Ti    = (3 * dT * dT) / 8589934592LL;
            OFFi  = (3 * t2 * t2) / 2;
            SENSi = (5 * t2 * t2) / 8;
            if ((TEMP / 100) < -15) {
                int64_t t3 = TEMP + 1500;
                OFFi  += 7 * t3 * t3;
                SENSi += 4 * t3 * t3;
            }
        } else {
            Ti    = (2 * dT * dT) / 137438953472LL;
            OFFi  = (t2 * t2) / 16;
            SENSi = 0;
        }
    }
    int64_t OFF2 = OFF - OFFi;
    int64_t SENS2 = SENS - SENSi;
    int64_t TEMPc = TEMP - Ti;
    int64_t P;

    if (model == MS5837_02BA) {
        P = ((D1 * SENS2) / 2097152LL - OFF2) / 32768LL;      /* unité : 0,01 mbar */
        out->pressure_mbar = (float)P / 100.0f;
    } else {
        P = ((D1 * SENS2) / 2097152LL - OFF2) / 8192LL;       /* unité : 0,1 mbar  */
        out->pressure_mbar = (float)P / 10.0f;
    }
    out->temperature_c = (float)TEMPc / 100.0f;
}

static bool send_cmd(uint8_t cmd)
{
    return port_i2c_write(MS5837_I2C_ADDR, &cmd, 1);
}

/* Lecture ADC : commande 0x00 puis 3 octets. Un résultat nul = conversion non terminée. */
static sens_status_t read_adc(uint32_t *out)
{
    uint8_t cmd = CMD_ADC_READ;
    uint8_t rx[3];
    if (!port_i2c_write(MS5837_I2C_ADDR, &cmd, 1) || !port_i2c_read(MS5837_I2C_ADDR, rx, 3)) {
        return SENS_ERR_BUS;
    }
    *out = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
    return (*out == 0) ? SENS_ERR_NOT_READY : SENS_OK;
}

sens_status_t ms5837_init(ms5837_t *s, ms5837_model_t model, ms5837_osr_t osr)
{
    if (s == NULL || (unsigned)model > (unsigned)MS5837_02BA || (unsigned)osr > (unsigned)MS5837_OSR_8192) {
        return SENS_ERR_PARAM;
    }
    memset(s, 0, sizeof *s);
    s->model = model;
    s->osr = osr;
    s->temp_every = 8;

    if (!send_cmd(CMD_RESET)) {
        return SENS_ERR_BUS;
    }
    port_delay_ms(10);

    bool all0 = true, allF = true;
    for (int i = 0; i < 7; i++) {
        uint8_t cmd = (uint8_t)(CMD_PROM_BASE + 2 * i);
        uint8_t rx[2];
        if (!port_i2c_write(MS5837_I2C_ADDR, &cmd, 1) || !port_i2c_read(MS5837_I2C_ADDR, rx, 2)) {
            return SENS_ERR_BUS;
        }
        s->prom[i] = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
        if (s->prom[i] != 0x0000) all0 = false;
        if (s->prom[i] != 0xFFFF) allF = false;
    }
    if (all0 || allF) {
        return SENS_ERR_ID;          /* bus tiré à 0/1 : aucun vrai capteur n'a répondu */
    }
    if (!ms5837_prom_valid(s->prom)) {
        return SENS_ERR_CRC;
    }
    s->state = ST_START_D1;
    return SENS_OK;
}

static sens_status_t finish_sample(ms5837_t *s, uint32_t now_ms)
{
    ms5837_result_t r;
    ms5837_compute(s->model, s->prom, s->d1, s->d2, &r);
    float p_max = (s->model == MS5837_02BA) ? 2000.0f : 30000.0f;
    if (!(r.pressure_mbar >= 300.0f && r.pressure_mbar <= p_max) ||
        !(r.temperature_c >= -40.0f && r.temperature_c <= 85.0f)) {
        s->error_count++;
        return SENS_ERR_RANGE;       /* on ne publie pas une valeur absurde */
    }
    s->pressure_mbar = r.pressure_mbar;
    s->temperature_c = r.temperature_c;
    s->last_sample_ms = now_ms;
    s->sample_count++;
    if (s->temp_cnt < 255) s->temp_cnt++;
    return SENS_OK;
}

sens_status_t ms5837_update(ms5837_t *s, uint32_t now_ms)
{
    uint32_t conv = ms5837_conv_time_ms(s->osr);

    switch (s->state) {
    case ST_START_D1:
        if (!send_cmd((uint8_t)(CMD_CONV_D1 + 2 * (int)s->osr))) {
            s->error_count++;
            return SENS_ERR_BUS;
        }
        s->t_start_ms = now_ms;
        s->state = ST_WAIT_D1;
        return SENS_ERR_NOT_READY;

    case ST_WAIT_D1:
    case ST_WAIT_D2: {
        uint32_t waited = now_ms - s->t_start_ms;
        if (waited < conv) {
            return SENS_ERR_NOT_READY;
        }
        uint32_t v = 0;
        sens_status_t st = read_adc(&v);
        if (st == SENS_ERR_NOT_READY) {
            if (waited > conv + TIMEOUT_MARGIN) {
                s->state = ST_START_D1;
                s->error_count++;
                return SENS_ERR_TIMEOUT;
            }
            return SENS_ERR_NOT_READY;
        }
        if (st != SENS_OK) {
            s->state = ST_START_D1;
            s->error_count++;
            return st;
        }
        if (s->state == ST_WAIT_D1) {
            s->d1 = v;
            if (!s->have_d2 || s->temp_cnt >= s->temp_every) {
                if (!send_cmd((uint8_t)(CMD_CONV_D2 + 2 * (int)s->osr))) {
                    s->state = ST_START_D1;
                    s->error_count++;
                    return SENS_ERR_BUS;
                }
                s->t_start_ms = now_ms;
                s->state = ST_WAIT_D2;
                return SENS_ERR_NOT_READY;
            }
        } else {
            s->d2 = v;
            s->have_d2 = true;
            s->temp_cnt = 0;
        }
        s->state = ST_START_D1;
        return finish_sample(s, now_ms);
    }
    default:
        s->state = ST_START_D1;
        return SENS_ERR_PARAM;
    }
}

sens_status_t ms5837_read_blocking(ms5837_t *s, ms5837_result_t *out)
{
    uint32_t t0 = port_millis();
    for (;;) {
        uint32_t now = port_millis();
        sens_status_t st = ms5837_update(s, now);
        if (st == SENS_OK) {
            out->pressure_mbar = s->pressure_mbar;
            out->temperature_c = s->temperature_c;
            return SENS_OK;
        }
        if (st != SENS_ERR_NOT_READY) {
            return st;
        }
        if ((uint32_t)(now - t0) > 200u) {
            return SENS_ERR_TIMEOUT;
        }
        port_delay_ms(1);
    }
}
