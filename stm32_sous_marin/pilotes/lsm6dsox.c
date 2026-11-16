#include "lsm6dsox.h"
#include "materiel.h"
#include <math.h>
#include <string.h>

#define REG_WHO_AM_I   0x0F
#define REG_CTRL1_XL   0x10
#define REG_CTRL2_G    0x11
#define REG_CTRL3_C    0x12
#define REG_STATUS     0x1E
#define REG_OUTX_L_G   0x22      /* gyro X,Y,Z puis accel X,Y,Z : 12 octets consécutifs */

#define CTRL3_SW_RESET 0x01
#define CTRL3_BDU_IFINC 0x44     /* BDU=1 (lecture cohérente L/H) + IF_INC=1 (adresse auto-incrémentée) */

static bool reg_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t n)
{
    return port_i2c_write_read(addr, &reg, 1, buf, n);
}

static bool reg_write(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return port_i2c_write(addr, b, 2);
}

sens_status_t lsm6dsox_init(lsm6dsox_t *s, uint8_t addr, lsm6dsox_acc_fs_t acc_fs,
                            lsm6dsox_gyr_fs_t gyr_fs, lsm6dsox_odr_t odr)
{
    if (s == NULL || (addr != LSM6DSOX_I2C_ADDR_LOW && addr != LSM6DSOX_I2C_ADDR_HIGH) ||
        (unsigned)acc_fs > 3u || (unsigned)gyr_fs > 3u || (unsigned)odr < 1u || (unsigned)odr > 7u) {
        return SENS_ERR_PARAM;
    }
    memset(s, 0, sizeof *s);
    s->addr = addr;

    uint8_t id = 0;
    if (!reg_read(addr, REG_WHO_AM_I, &id, 1)) {
        return SENS_ERR_BUS;
    }
    if (id != LSM6DSOX_WHO_AM_I_VALUE) {
        return SENS_ERR_ID;
    }

    if (!reg_write(addr, REG_CTRL3_C, CTRL3_SW_RESET)) {
        return SENS_ERR_BUS;
    }
    uint8_t c3 = CTRL3_SW_RESET;
    for (int i = 0; i < 20 && (c3 & CTRL3_SW_RESET); i++) {
        port_delay_ms(1);
        if (!reg_read(addr, REG_CTRL3_C, &c3, 1)) {
            return SENS_ERR_BUS;
        }
    }
    if (c3 & CTRL3_SW_RESET) {
        return SENS_ERR_TIMEOUT;     /* le reset ne se termine jamais : composant défectueux */
    }

    if (!reg_write(addr, REG_CTRL3_C, CTRL3_BDU_IFINC) ||
        !reg_write(addr, REG_CTRL1_XL, (uint8_t)(((unsigned)odr << 4) | ((unsigned)acc_fs << 2))) ||
        !reg_write(addr, REG_CTRL2_G,  (uint8_t)(((unsigned)odr << 4) | ((unsigned)gyr_fs << 2)))) {
        return SENS_ERR_BUS;
    }

    static const float acc_mg[4]   = { 0.061f, 0.488f, 0.122f, 0.244f };   /* indexé par le code FS_XL */
    static const float gyr_mdps[4] = { 8.75f, 17.5f, 35.0f, 70.0f };       /* indexé par le code FS_G  */
    s->acc_g_per_lsb = acc_mg[acc_fs] / 1000.0f;
    s->gyr_dps_per_lsb = gyr_mdps[gyr_fs] / 1000.0f;
    return SENS_OK;
}

static int16_t le16(const uint8_t *b)
{
    return (int16_t)(uint16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

sens_status_t lsm6dsox_read(lsm6dsox_t *s, lsm6dsox_data_t *out)
{
    uint8_t status = 0;
    if (!reg_read(s->addr, REG_STATUS, &status, 1)) {
        return SENS_ERR_BUS;
    }
    if ((status & 0x03) != 0x03) {            /* XLDA et GDA : accel ET gyro prêts */
        return SENS_ERR_NOT_READY;
    }
    uint8_t b[12];
    if (!reg_read(s->addr, REG_OUTX_L_G, b, 12)) {
        return SENS_ERR_BUS;
    }
    for (int i = 0; i < 3; i++) {
        out->gyr_raw[i] = le16(&b[2 * i]);
        out->acc_raw[i] = le16(&b[6 + 2 * i]);
        out->gyr_dps[i] = (float)out->gyr_raw[i] * s->gyr_dps_per_lsb - s->gyro_bias_dps[i];
        out->acc_g[i]   = (float)out->acc_raw[i] * s->acc_g_per_lsb;
    }
    return SENS_OK;
}

sens_status_t lsm6dsox_calibrate_gyro(lsm6dsox_t *s, uint16_t n_samples, uint32_t timeout_ms)
{
    if (n_samples < 2) {
        return SENS_ERR_PARAM;
    }
    double sum[3] = { 0, 0, 0 }, sumsq[3] = { 0, 0, 0 };
    uint16_t got = 0;
    uint32_t t0 = port_millis();
    while (got < n_samples) {
        lsm6dsox_data_t d;
        sens_status_t st = lsm6dsox_read(s, &d);
        if (st == SENS_OK) {
            for (int i = 0; i < 3; i++) {
                double v = (double)d.gyr_raw[i] * (double)s->gyr_dps_per_lsb;   /* brut, sans ancien biais */
                sum[i] += v;
                sumsq[i] += v * v;
            }
            got++;
        } else if (st == SENS_ERR_NOT_READY) {
            if ((uint32_t)(port_millis() - t0) > timeout_ms) {
                return SENS_ERR_TIMEOUT;
            }
            port_delay_ms(1);
        } else {
            return st;
        }
    }
    float mean[3];
    for (int i = 0; i < 3; i++) {
        double m = sum[i] / got;
        double var = sumsq[i] / got - m * m;
        if (var < 0.0) var = 0.0;
        if (sqrt(var) > 1.0) {                 /* > 1 deg/s d'écart-type : le sous-marin bouge */
            return SENS_ERR_RANGE;
        }
        mean[i] = (float)m;
    }
    for (int i = 0; i < 3; i++) {
        s->gyro_bias_dps[i] = mean[i];
    }
    return SENS_OK;
}
