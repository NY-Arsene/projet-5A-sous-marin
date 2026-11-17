#include "faux_composants.h"
#include "materiel_faux.h"
#include "ms5837.h"
#include <math.h>
#include <string.h>

/* ================================================================== MS5837 */
static const uint32_t k_real_conv_ms[6] = { 1, 2, 3, 5, 9, 18 };   /* un peu plus court que les délais du driver */

static bool ms_should_nack(fake_ms5837_t *f)
{
    if (f->nack_skip > 0) {
        f->nack_skip--;
        return false;
    }
    if (f->nack_next > 0) {
        f->nack_next--;
        return true;
    }
    return false;
}

static bool ms_write(void *ctx, const uint8_t *tx, uint16_t len)
{
    fake_ms5837_t *f = ctx;
    if (ms_should_nack(f)) {
        return false;
    }
    if (len != 1) {
        return false;
    }
    uint8_t cmd = tx[0];
    if (cmd == 0x1E) {
        f->resets++;
        f->mode = 0;
        f->conv_kind = 0;
    } else if (cmd >= 0xA0 && cmd <= 0xAC && ((cmd & 1) == 0)) {
        f->prom_idx = (uint8_t)((cmd - 0xA0) / 2);
        f->mode = 1;
    } else if (cmd >= 0x40 && cmd <= 0x4A && ((cmd & 1) == 0)) {
        f->conv_kind = 1;
        f->ready_at = fake_time_get() + k_real_conv_ms[(cmd - 0x40) / 2];
        f->conv_d1_count++;
        f->mode = 0;
    } else if (cmd >= 0x50 && cmd <= 0x5A && ((cmd & 1) == 0)) {
        f->conv_kind = 2;
        f->ready_at = fake_time_get() + k_real_conv_ms[(cmd - 0x50) / 2];
        f->conv_d2_count++;
        f->mode = 0;
    } else if (cmd == 0x00) {
        f->mode = 2;
    } else {
        return false;
    }
    return true;
}

static bool ms_read(void *ctx, uint8_t *rx, uint16_t len)
{
    fake_ms5837_t *f = ctx;
    if (ms_should_nack(f)) {
        return false;
    }
    if (f->mode == 1 && len == 2) {
        rx[0] = (uint8_t)(f->prom[f->prom_idx] >> 8);
        rx[1] = (uint8_t)(f->prom[f->prom_idx] & 0xFF);
        return true;
    }
    if (f->mode == 2 && len == 3) {
        uint32_t v = 0;
        bool done = (f->conv_kind != 0) && !f->dead_adc && (fake_time_get() >= f->ready_at);
        if (done) {
            v = (f->conv_kind == 1) ? f->d1 : f->d2;
        }
        rx[0] = (uint8_t)(v >> 16);
        rx[1] = (uint8_t)(v >> 8);
        rx[2] = (uint8_t)v;
        return true;
    }
    return false;
}

void fake_ms5837_setup(fake_ms5837_t *f, bool is_02ba)
{
    memset(f, 0, sizeof *f);
    /* coefficients typiques (C1..C6) ; C0 = CRC(4 bits) + code usine */
    static const uint16_t c02[7] = { 0x0000, 46372, 43981, 29059, 27842, 31553, 28165 };
    static const uint16_t c30[7] = { 0x0000, 34982, 36352, 20328, 22354, 26646, 26146 };
    memcpy(f->prom, is_02ba ? c02 : c30, sizeof f->prom);
    f->prom[0] = 0x0000 | 0x0150;                         /* code usine arbitraire */
    f->prom[0] = (uint16_t)((f->prom[0] & 0x0FFF) | ((uint16_t)ms5837_crc4(f->prom) << 12));
    f->d2 = (uint32_t)f->prom[5] * 256u;                  /* dT = 0  =>  20,00 degC */
    fake_ms5837_set_pressure(f, 1013.0f);
    fake_add_i2c(MS5837_I2C_ADDR, ms_write, ms_read, f);
    /* NB : la classe de capteur (02BA/30BA) est choisie à la création par les coefficients */
    (void)is_02ba;
}

/* Recherche (dichotomie) du D1 qui donne la pression visée avec la compensation complète. */
static void recalc_d1(fake_ms5837_t *f)
{
    ms5837_model_t model = (f->prom[1] == 46372) ? MS5837_02BA : MS5837_30BA;   /* C1 distingue nos 2 jeux de test */
    uint32_t lo = 0, hi = 0xFFFFFF;
    for (int i = 0; i < 32; i++) {
        uint32_t mid = lo + (hi - lo) / 2;
        ms5837_result_t r;
        ms5837_compute(model, f->prom, mid, f->d2, &r);
        if (r.pressure_mbar < f->target_p) lo = mid; else hi = mid;
    }
    f->d1 = hi;
}

void fake_ms5837_set_pressure(fake_ms5837_t *f, float mbar)
{
    f->target_p = mbar;
    recalc_d1(f);
}

void fake_ms5837_set_temp(fake_ms5837_t *f, float temp_c)
{
    double dT = ((double)temp_c - 20.0) * 100.0 * 8388608.0 / (double)f->prom[6];
    f->d2 = (uint32_t)llround((double)f->prom[5] * 256.0 + dT);
    recalc_d1(f);
}

/* ================================================================== INA226 */
static bool ina_write(void *ctx, const uint8_t *tx, uint16_t len)
{
    fake_ina226_t *f = ctx;
    if (len == 0) return true;
    f->ptr = tx[0];
    if (len == 3) {
        f->regs[f->ptr] = (uint16_t)(((uint16_t)tx[1] << 8) | tx[2]);
        if (f->ptr == 0x00) f->last_config = f->regs[0];
    }
    return true;
}

static bool ina_read(void *ctx, uint8_t *rx, uint16_t len)
{
    fake_ina226_t *f = ctx;
    uint8_t p = f->ptr;
    for (uint16_t i = 0; i + 1 < len + 1 && i < len; i += 2) {
        rx[i] = (uint8_t)(f->regs[p] >> 8);
        if (i + 1 < len) rx[i + 1] = (uint8_t)(f->regs[p] & 0xFF);
        p++;
    }
    return true;
}

void fake_ina226_setup(fake_ina226_t *f)
{
    memset(f, 0, sizeof *f);
    f->regs[0x00] = 0x4127;
    f->regs[0xFE] = 0x5449;
    f->regs[0xFF] = 0x2260;
    fake_ina226_set(f, 16.4f, 5.0f);              /* batterie 4S à 4,10 V par élément */
    fake_add_i2c(0x40, ina_write, ina_read, f);
}

void fake_ina226_set(fake_ina226_t *f, float bus_v, float shunt_mv)
{
    f->regs[0x02] = (uint16_t)lroundf(bus_v / 0.00125f);
    f->regs[0x01] = (uint16_t)(int16_t)lroundf(shunt_mv / 0.0025f);
}

/* ================================================================== DS18B20 */
uint8_t fake_crc8_maxim(const uint8_t *d, int n)
{
    uint8_t crc = 0;
    for (int i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (uint8_t)((crc >> 1) ^ 0x8C) : (uint8_t)(crc >> 1);
        }
    }
    return crc;
}

static bool ow_reset(void *ctx)
{
    fake_ds18b20_t *f = ctx;
    f->state = 0;
    return f->present;
}

static void ow_write(void *ctx, uint8_t b)
{
    fake_ds18b20_t *f = ctx;
    if (f->state == 0) {
        if (b == 0xCC) f->state = 1;
    } else if (f->state == 1) {
        if (b == 0x44) {
            f->converting = true;
            f->conv_start = fake_time_get();
            f->conversions++;
            f->state = 0;
        } else if (b == 0xBE) {
            bool done = f->converting && (fake_time_get() - f->conv_start >= 750);
            int16_t raw = done ? f->raw : 0x0550;           /* 85 degC tant que rien n'est converti */
            f->scratch[0] = (uint8_t)(raw & 0xFF);
            f->scratch[1] = (uint8_t)((uint16_t)raw >> 8);
            f->scratch[2] = 0x4B; f->scratch[3] = 0x46; f->scratch[4] = 0x7F;
            f->scratch[5] = 0xFF; f->scratch[6] = 0x0C; f->scratch[7] = 0x10;
            f->scratch[8] = fake_crc8_maxim(f->scratch, 8);
            if (f->corrupt_crc) f->scratch[8] ^= 0x5A;
            f->idx = 0;
            f->state = 2;
        }
    }
}

static uint8_t ow_read(void *ctx)
{
    fake_ds18b20_t *f = ctx;
    return (f->state == 2 && f->idx < 9) ? f->scratch[f->idx++] : 0xFF;
}

void fake_ds18b20_setup(fake_ds18b20_t *f, float temp_c)
{
    memset(f, 0, sizeof *f);
    f->present = true;
    f->raw = (int16_t)lroundf(temp_c * 16.0f);
    fake_ow_t ow = { ow_reset, ow_write, ow_read, f };
    fake_set_ow(&ow);
}

/* ================================================================== IMU LSM6DSOX */
static void imu_write_hook(fake_regmap_t *m, uint8_t reg, uint8_t val)
{
    if (reg == 0x12 && (val & 0x01)) {
        m->regs[0x12] = (uint8_t)(val & ~0x01);       /* SW_RESET se remet seul à 0 */
    }
}

void fake_imu_setup(fake_regmap_t *m, uint8_t addr)
{
    fake_regmap_init(m, false);
    m->regs[0x0F] = 0x6C;                              /* WHO_AM_I */
    m->regs[0x12] = 0x04;
    m->regs[0x1E] = 0x03;                              /* donnée accel + gyro prête */
    m->on_write_reg = imu_write_hook;
    fake_add_i2c(addr, fake_regmap_write, fake_regmap_read, m);
}

void fake_imu_set(fake_regmap_t *m, const float acc_g[3], const float gyr_dps[3])
{
    static const double acc_mg[4]   = { 0.061, 0.488, 0.122, 0.244 };   /* fiche : FS_XL = 00,01,10,11 */
    static const double gyr_mdps[4] = { 8.75, 17.5, 35.0, 70.0 };       /* fiche : FS_G  = 00,01,10,11 */
    double sa = acc_mg[(m->regs[0x10] >> 2) & 3] / 1000.0;
    double sg = gyr_mdps[(m->regs[0x11] >> 2) & 3] / 1000.0;
    for (int i = 0; i < 3; i++) {
        int16_t g = (int16_t)lround(gyr_dps[i] / sg);
        int16_t a = (int16_t)lround(acc_g[i] / sa);
        m->regs[0x22 + 2 * i] = (uint8_t)((uint16_t)g & 0xFF);
        m->regs[0x23 + 2 * i] = (uint8_t)((uint16_t)g >> 8);
        m->regs[0x28 + 2 * i] = (uint8_t)((uint16_t)a & 0xFF);
        m->regs[0x29 + 2 * i] = (uint8_t)((uint16_t)a >> 8);
    }
}

/* ================================================================== magnétomètre LIS3MDL */
void fake_mag_setup(fake_regmap_t *m, uint8_t addr)
{
    fake_regmap_init(m, true);
    m->regs[0x0F] = 0x3D;                              /* WHO_AM_I */
    m->regs[0x27] = 0x08;                              /* ZYXDA */
    fake_add_i2c(addr, fake_regmap_write, fake_regmap_read, m);
}

void fake_mag_set(fake_regmap_t *m, const float gauss[3])
{
    static const double lsb_per_gauss[4] = { 6842.0, 3421.0, 2281.0, 1711.0 };
    double s = lsb_per_gauss[(m->regs[0x21] >> 5) & 3];
    for (int i = 0; i < 3; i++) {
        int16_t r = (int16_t)lround(gauss[i] * s);
        m->regs[0x28 + 2 * i] = (uint8_t)((uint16_t)r & 0xFF);
        m->regs[0x29 + 2 * i] = (uint8_t)((uint16_t)r >> 8);
    }
}

/* ================================================================== sous-marin complet */
void fake_sub_setup(fake_sub_t *s)
{
    memset(s, 0, sizeof *s);
    fake_ms5837_setup(&s->baro, true);
    fake_imu_setup(&s->imu, 0x6A);
    fake_mag_setup(&s->mag, 0x1C);
    fake_ina226_setup(&s->ina);
    fake_ds18b20_setup(&s->temp, 21.5f);
    /* IMU : à plat, immobile (repère sous-marin Z vers le bas : lecture Z = -1 g) */
    s->imu.regs[0x10] = 0x40; s->imu.regs[0x11] = 0x44;   /* config par défaut avant init (FS 2g / 500dps après init) */
}
