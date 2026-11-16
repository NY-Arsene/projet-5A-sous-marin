#include "ina226.h"
#include "materiel.h"
#include <stddef.h>

#define REG_CONFIG    0x00
#define REG_SHUNT     0x01
#define REG_BUS       0x02
#define REG_MANUF_ID  0xFE
#define REG_DIE_ID    0xFF
#define MANUF_ID      0x5449      /* "TI" */
#define DIE_ID        0x2260

static bool reg_read16(uint8_t addr, uint8_t reg, uint16_t *v)
{
    uint8_t rx[2];
    if (!port_i2c_write_read(addr, &reg, 1, rx, 2)) {
        return false;
    }
    *v = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);      /* poids fort en premier (big-endian) */
    return true;
}

sens_status_t ina226_init(ina226_t *s, uint8_t addr, float shunt_ohm, ina226_avg_t avg)
{
    if (s == NULL || !(shunt_ohm > 0.0f) || (unsigned)avg > 7u) {
        return SENS_ERR_PARAM;
    }
    s->addr = addr;
    s->shunt_ohm = shunt_ohm;

    uint16_t manuf = 0, die = 0;
    if (!reg_read16(addr, REG_MANUF_ID, &manuf) || !reg_read16(addr, REG_DIE_ID, &die)) {
        return SENS_ERR_BUS;
    }
    if (manuf != MANUF_ID || die != DIE_ID) {
        return SENS_ERR_ID;
    }
    /* 0x4127 = défaut (conversions 1,1 ms, mode continu shunt+bus) ; AVG dans les bits 11:9 */
    uint16_t cfg = (uint16_t)(0x4127u | ((unsigned)avg << 9));
    uint8_t b[3] = { REG_CONFIG, (uint8_t)(cfg >> 8), (uint8_t)(cfg & 0xFF) };
    if (!port_i2c_write(addr, b, 3)) {
        return SENS_ERR_BUS;
    }
    return SENS_OK;
}

sens_status_t ina226_read(ina226_t *s, ina226_data_t *out)
{
    uint16_t shunt_u = 0, bus_u = 0;
    if (!reg_read16(s->addr, REG_SHUNT, &shunt_u) || !reg_read16(s->addr, REG_BUS, &bus_u)) {
        return SENS_ERR_BUS;
    }
    int16_t shunt = (int16_t)shunt_u;
    out->shunt_mv = (float)shunt * 0.0025f;                  /* 2,5 uV/LSB */
    out->bus_v = (float)(bus_u & 0x7FFF) * 0.00125f;         /* 1,25 mV/LSB */
    out->current_a = (out->shunt_mv / 1000.0f) / s->shunt_ohm;
    out->power_w = out->bus_v * out->current_a;
    out->saturated = (shunt == 32767 || shunt == -32768);
    return SENS_OK;
}
