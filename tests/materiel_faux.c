#include "materiel.h"
#include <string.h>

static uint32_t sim_time_ms = 0;
static uint16_t sim_adc_leak = 120;
static bool     sim_pins[PORT_PIN_COUNT] = {true, true, true, true};
static uint8_t  last_i2c_cmd = 0;
static bool     ms5837_is_temp = false;

void port_init(void) {
    sim_time_ms = 0;
    ms5837_is_temp = false;
}
uint32_t port_millis(void) { return sim_time_ms; }
void port_delay_ms(uint32_t ms) { sim_time_ms += ms; }

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

bool port_i2c_probe(uint8_t addr7) {
    return (addr7 == 0x6A || addr7 == 0x1C || addr7 == 0x40 || addr7 == 0x76);
}

bool port_i2c_write(uint8_t addr7, const uint8_t *tx, uint16_t len) {
    if (!port_i2c_probe(addr7)) return false;
    if (tx && len > 0) {
        last_i2c_cmd = tx[0];
        if (addr7 == 0x76) {
            /* 0x40..0x48 = conversion D1 (Pression), 0x50..0x58 = conversion D2 (Temperature) */
            if ((tx[0] & 0xF0) == 0x40) ms5837_is_temp = false;
            else if ((tx[0] & 0xF0) == 0x50) ms5837_is_temp = true;
        }
    }
    return true;
}

bool port_i2c_read(uint8_t addr7, uint8_t *rx, uint16_t len) {
    if (!port_i2c_probe(addr7) || !rx) return false;
    memset(rx, 0, len);

    if (addr7 == 0x76) {
        static uint16_t prom[8] = {0x0000, 46372, 43981, 29059, 27842, 31553, 28165, 0};
        prom[0] = (uint16_t)((ms5837_crc4(prom) << 12) | 0x0010);

        if (last_i2c_cmd >= 0xA0 && last_i2c_cmd <= 0xAE && len >= 2) {
            uint8_t idx = (last_i2c_cmd - 0xA0) >> 1;
            rx[0] = (uint8_t)(prom[idx] >> 8);
            rx[1] = (uint8_t)(prom[idx] & 0xFF);
        } else if (last_i2c_cmd == 0x00 && len >= 3) {
            uint32_t adc = ms5837_is_temp ? 8077636 : 6465444;
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

    if (addr7 == 0x6A) {
        if (reg == 0x0F && rxlen >= 1) rx[0] = 0x6C;
        else if (reg == 0x1E && rxlen >= 1) rx[0] = 0x03;
        else if (reg == 0x22 && rxlen >= 12) {
            /* OUTX_L_G (0..5) puis OUTX_L_A (6..11) -> Z accel est aux octets 10 et 11 */
            int16_t az = 16393; /* +1.0 g */
            rx[10] = (uint8_t)(az & 0xFF);
            rx[11] = (uint8_t)((az >> 8) & 0xFF);
        }
    } else if (addr7 == 0x1C) {
        uint8_t r = reg & 0x7F;
        if (r == 0x0F && rxlen >= 1) rx[0] = 0x3D;
        else if (r == 0x27 && rxlen >= 1) rx[0] = 0x08;
        else if (r == 0x28 && rxlen >= 6) {
            int16_t mx = 2200, mz = 2150;
            rx[0] = (uint8_t)(mx & 0xFF); rx[1] = (uint8_t)(mx >> 8);
            rx[4] = (uint8_t)(mz & 0xFF); rx[5] = (uint8_t)(mz >> 8);
        }
    } else if (addr7 == 0x40 && rxlen >= 2) {
        if (reg == 0xFE) { rx[0] = 0x54; rx[1] = 0x49; }
        else if (reg == 0xFF) { rx[0] = 0x22; rx[1] = 0x60; }
        else if (reg == 0x02) { rx[0] = 0x25; rx[1] = 0x80; }
        else if (reg == 0x01) { rx[0] = 0x0F; rx[1] = 0xA0; }
    }
    return true;
}

uint16_t port_adc_read(port_adc_t ch) { return (ch == PORT_ADC_LEAK) ? sim_adc_leak : 0; }
bool port_gpio_read(port_pin_t pin) { return (pin < PORT_PIN_COUNT) ? sim_pins[pin] : false; }

static uint8_t ow_idx = 0;
static const uint8_t ds18b20_scratchpad[9] = {0x68, 0x01, 0x4B, 0x46, 0x7F, 0xFF, 0x0C, 0x10, 0x5B};
bool port_ow_reset(void) { ow_idx = 0; return true; }
void port_ow_write_byte(uint8_t b) { if (b == 0xBE) ow_idx = 0; }
uint8_t port_ow_read_byte(void) { return (ow_idx < 9) ? ds18b20_scratchpad[ow_idx++] : 0xFF; }

int port_console_getc(uint32_t timeout_ms) { (void)timeout_ms; return -1; }
void port_motor_enable(port_motor_t m, bool on) { (void)m; (void)on; }
void port_motor_dir(port_motor_t m, bool fill) { (void)m; (void)fill; }
void port_motor_step(port_motor_t m) { (void)m; }
uint16_t port_link_read(uint8_t *buf, uint16_t max) { (void)buf; (void)max; return 0; }
uint16_t port_link_write(const uint8_t *buf, uint16_t len) { (void)buf; return len; }