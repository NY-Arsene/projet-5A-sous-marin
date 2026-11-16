/**
 * @file materiel_faux.c
 * @brief Simulation matérielle (Mock I2C, 1-Wire, ADC, Timers) pour exécuter banc_run_all() sur PC/CI.
 */
#include "materiel.h"
#include <string.h>

static uint32_t sim_time_ms = 0;
static uint16_t sim_adc_leak = 120; /* Valeur a sec (< 75% du seuil) */
static bool     sim_pins[PORT_PIN_COUNT] = {true, true, true, true};
static uint8_t  last_i2c_cmd = 0;

void port_init(void) {
    sim_time_ms = 0;
}

/* ---- Temps ---- */
uint32_t port_millis(void) { return sim_time_ms; }
void port_delay_ms(uint32_t ms) { sim_time_ms += ms; }

/* ---- Calcul CRC4 pour la PROM du MS5837 ---- */
static uint8_t ms5837_crc4(uint16_t n_prom[]) {
    uint16_t n_rem = 0;
    n_prom[0] = (n_prom[0] & 0x0FFF);
    n_prom[7] = 0;
    for (uint8_t cnt = 0; cnt < 16; cnt++) {
        if (cnt % 2 == 1) n_rem ^= (uint16_t)((n_prom[cnt >> 1]) & 0x00FF);
        else              n_rem ^= (uint16_t)(n_prom[cnt >> 1] >> 8);
        for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) n_rem = (n_rem << 1) ^ 0x3000;
            else                n_rem = (n_rem << 1);
        }
    }
    return (uint8_t)((n_rem >> 12) & 0x000F);
}

/* ---- Simulation des transactions I2C ---- */
bool port_i2c_probe(uint8_t addr7) {
    /* On declare presents les 4 capteurs attendus par bench_i2c_scan() */
    return (addr7 == 0x6A || addr7 == 0x1C || addr7 == 0x40 || addr7 == 0x76);
}

bool port_i2c_write(uint8_t addr7, const uint8_t *tx, uint16_t len) {
    if (!port_i2c_probe(addr7)) return false;
    if (tx && len > 0) last_i2c_cmd = tx[0];
    return true;
}

bool port_i2c_read(uint8_t addr7, uint8_t *rx, uint16_t len) {
    if (!port_i2c_probe(addr7) || !rx) return false;
    memset(rx, 0, len);

    /* MS5837 (0x76) : lecture PROM (0xA0..0xAE) ou lecture ADC 24 bits (0x00) */
    if (addr7 == 0x76) {
        static uint16_t prom[8] = {0x0000, 46372, 43981, 29059, 27842, 31553, 28165, 0};
        prom[0] = (uint16_t)((ms5837_crc4(prom) << 12) | 0x0010);

        if (last_i2c_cmd >= 0xA0 && last_i2c_cmd <= 0xAE && len >= 2) {
            uint8_t idx = (last_i2c_cmd - 0xA0) >> 1;
            rx[0] = (uint8_t)(prom[idx] >> 8);
            rx[1] = (uint8_t)(prom[idx] & 0xFF);
        } else if (last_i2c_cmd == 0x00 && len >= 3) {
            /* Valeurs ADC brutes correspondant a ~1013 mbar et ~20 C */
            static uint32_t d1 = 6465444; /* Pression */
            static uint32_t d2 = 8077636; /* Temperature */
            uint32_t val = (d1 ^ d2); /* Alternance simple si besoin, ici fixe plausible */
            (void)val;
            /* On renvoie D1 ou D2 selon la derniere conversion demandee */
            static bool next_is_temp = false;
            uint32_t adc = next_is_temp ? d2 : d1;
            next_is_temp = !next_is_temp;
            rx[0] = (uint8_t)(adc >> 16);
            rx[1] = (uint8_t)(adc >> 8);
            rx[2] = (uint8_t)(adc);
        }
    }
    return true;
}

bool port_i2c_write_read(uint8_t addr7, const uint8_t *tx, uint16_t txlen, uint8_t *rx, uint16_t rxlen) {
    if (!port_i2c_probe(addr7) || !tx || txlen == 0 || !rx) return false;
    uint8_t reg = tx[0];
    memset(rx, 0, rxlen);

    /* LSM6DSOX (0x6A) */
    if (addr7 == 0x6A) {
        if (reg == 0x0F && rxlen >= 1) rx[0] = 0x6C; /* WHO_AM_I */
        else if (reg == 0x1E && rxlen >= 1) rx[0] = 0x03; /* STATUS_REG: XLDA + GDA prets */
        else if (reg == 0x22 && rxlen >= 12) {
            /* Gyro = 0 dps, Accel Z = +1g (~16393 LSB a +-2g) */
            int16_t az = 16393;
            rx[10] = (uint8_t)(az & 0xFF);
            rx[11] = (uint8_t)((az >> 8) & 0xFF);
        }
    }
    /* LIS3MDL (0x1C) */
    else if (addr7 == 0x1C) {
        uint8_t r = reg & 0x7F; /* Masque auto-increment bit 7 */
        if (r == 0x0F && rxlen >= 1) rx[0] = 0x3D; /* WHO_AM_I */
        else if (r == 0x27 && rxlen >= 1) rx[0] = 0x08; /* STATUS_REG: ZYXDA pret */
        else if (r == 0x28 && rxlen >= 6) {
            /* Champ magnetique terrestre ~0.45 Gauss (~3079 LSB a +-4G) */
            int16_t mx = 2200, mz = 2150;
            rx[0] = (uint8_t)(mx & 0xFF); rx[1] = (uint8_t)(mx >> 8);
            rx[4] = (uint8_t)(mz & 0xFF); rx[5] = (uint8_t)(mz >> 8);
        }
    }
    /* INA226 (0x40) */
    else if (addr7 == 0x40 && rxlen >= 2) {
        if (reg == 0xFE) { rx[0] = 0x54; rx[1] = 0x49; }      /* Manufacturer ID "TI" */
        else if (reg == 0xFF) { rx[0] = 0x22; rx[1] = 0x60; } /* Die ID 0x2260 */
        else if (reg == 0x02) {
            /* Bus voltage: 12.0 V / 1.25 mV = 9600 (0x2580) */
            rx[0] = 0x25; rx[1] = 0x80;
        } else if (reg == 0x01) {
            /* Shunt voltage: 1.0 A * 0.010 Ohm = 10 mV / 2.5 uV = 4000 (0x0FA0) */
            rx[0] = 0x0F; rx[1] = 0xA0;
        }
    }
    return true;
}

/* ---- Entrées ---- */
uint16_t port_adc_read(port_adc_t ch) {
    return (ch == PORT_ADC_LEAK) ? sim_adc_leak : 0;
}
bool port_gpio_read(port_pin_t pin) {
    return (pin < PORT_PIN_COUNT) ? sim_pins[pin] : false;
}

/* ---- 1-Wire DS18B20 (Temperature = 22.5 C -> 0x0168, CRC = 0x5B) ---- */
static uint8_t ow_idx = 0;
static const uint8_t ds18b20_scratchpad[9] = {0x68, 0x01, 0x4B, 0x46, 0x7F, 0xFF, 0x0C, 0x10, 0x5B};

bool port_ow_reset(void) { ow_idx = 0; return true; }
void port_ow_write_byte(uint8_t b) { if (b == 0xBE) ow_idx = 0; }
uint8_t port_ow_read_byte(void) {
    if (ow_idx < 9) return ds18b20_scratchpad[ow_idx++];
    return 0xFF;
}

/* ---- Console, Moteurs & Liaison série ---- */
int port_console_getc(uint32_t timeout_ms) { (void)timeout_ms; return -1; }
void port_motor_enable(port_motor_t m, bool on) { (void)m; (void)on; }
void port_motor_dir(port_motor_t m, bool fill) { (void)m; (void)fill; }
void port_motor_step(port_motor_t m) { (void)m; }
uint16_t port_link_read(uint8_t *buf, uint16_t max) { (void)buf; (void)max; return 0; }
uint16_t port_link_write(const uint8_t *buf, uint16_t len) { (void)buf; return len; }