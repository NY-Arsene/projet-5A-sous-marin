#include "lis3mdl.h"
#include "materiel.h"
#include <stddef.h>
#include <string.h>

#define REG_WHO_AM_I   0x0F
#define REG_CTRL1      0x20
#define REG_CTRL2      0x21
#define REG_CTRL3      0x22
#define REG_CTRL4      0x23
#define REG_CTRL5      0x24
#define REG_STATUS     0x27
#define REG_OUT_X_L    0x28
#define AUTO_INC       0x80      /* bit 7 de l'adresse : incrément automatique (OBLIGATOIRE en I2C multi-octets) */

static bool reg_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t n)
{
    return port_i2c_write_read(addr, &reg, 1, buf, n);
}

static bool reg_write(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return port_i2c_write(addr, b, 2);
}

sens_status_t lis3mdl_init(lis3mdl_t *s, uint8_t addr, lis3mdl_fs_t fs, lis3mdl_odr_t odr)
{
    if (s == NULL || (addr != LIS3MDL_I2C_ADDR_LOW && addr != LIS3MDL_I2C_ADDR_HIGH) ||
        (unsigned)fs > 3u || (unsigned)odr < 4u || (unsigned)odr > 7u) {
        return SENS_ERR_PARAM;
    }
    memset(s, 0, sizeof *s);
    s->addr = addr;

    uint8_t id = 0;
    if (!reg_read(addr, REG_WHO_AM_I, &id, 1)) {
        return SENS_ERR_BUS;
    }
    if (id != LIS3MDL_WHO_AM_I_VALUE) {
        return SENS_ERR_ID;
    }

    if (!reg_write(addr, REG_CTRL2, 0x04)) {           /* SOFT_RST */
        return SENS_ERR_BUS;
    }
    port_delay_ms(10);

    if (!reg_write(addr, REG_CTRL1, (uint8_t)(0x60 | ((unsigned)odr << 2))) ||   /* XY ultra-haute perf., ODR */
        !reg_write(addr, REG_CTRL2, (uint8_t)((unsigned)fs << 5)) ||             /* pleine échelle            */
        !reg_write(addr, REG_CTRL3, 0x00) ||                                     /* mode continu              */
        !reg_write(addr, REG_CTRL4, 0x0C) ||                                     /* Z ultra-haute perf.       */
        !reg_write(addr, REG_CTRL5, 0x40)) {                                     /* BDU                       */
        return SENS_ERR_BUS;
    }

    static const float lsb_per_gauss[4] = { 6842.0f, 3421.0f, 2281.0f, 1711.0f };
    s->gauss_per_lsb = 1.0f / lsb_per_gauss[fs];
    return SENS_OK;
}

sens_status_t lis3mdl_read(lis3mdl_t *s, lis3mdl_data_t *out)
{
    uint8_t status = 0;
    if (!reg_read(s->addr, REG_STATUS, &status, 1)) {
        return SENS_ERR_BUS;
    }
    if ((status & 0x08) == 0) {                         /* ZYXDA */
        return SENS_ERR_NOT_READY;
    }
    uint8_t b[6];
    if (!reg_read(s->addr, REG_OUT_X_L | AUTO_INC, b, 6)) {
        return SENS_ERR_BUS;
    }
    for (int i = 0; i < 3; i++) {
        out->raw[i] = (int16_t)(uint16_t)((uint16_t)b[2 * i] | ((uint16_t)b[2 * i + 1] << 8));
        out->gauss[i] = (float)out->raw[i] * s->gauss_per_lsb;
    }
    return SENS_OK;
}
