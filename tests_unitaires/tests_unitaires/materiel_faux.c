#include "materiel_faux.h"
#include <string.h>

#define MAX_DEV 8

typedef struct {
    bool          used;
    uint8_t       addr;
    fake_write_fn w;
    fake_read_fn  r;
    void         *ctx;
} dev_t_;

static dev_t_   s_dev[MAX_DEV];
static bool     s_fail_all;
static uint32_t s_now;
static fake_ow_t s_ow;
static char     s_console[256];
static size_t   s_console_pos;

uint16_t fake_adc_value[PORT_ADC_COUNT];
bool     fake_gpio_level[PORT_PIN_COUNT];
uint32_t fake_i2c_transactions;

int32_t  fake_motor_phys[PORT_MOTOR_COUNT];
int32_t  fake_motor_phys_min[PORT_MOTOR_COUNT];
int32_t  fake_motor_phys_max[PORT_MOTOR_COUNT];
bool     fake_motor_enabled[PORT_MOTOR_COUNT];
bool     fake_motor_dir_fill[PORT_MOTOR_COUNT];
uint32_t fake_motor_pulses[PORT_MOTOR_COUNT];
uint32_t fake_motor_pulses_disabled[PORT_MOTOR_COUNT];

#define LINK_BUF 4096
static uint8_t  s_link_rx[LINK_BUF];
static uint16_t s_link_rx_len, s_link_rx_pos;
static uint8_t  s_link_tx[LINK_BUF];
static uint16_t s_link_tx_len, s_link_tx_cap;

void fake_reset(void)
{
    memset(s_dev, 0, sizeof s_dev);
    memset(&s_ow, 0, sizeof s_ow);
    memset(fake_adc_value, 0, sizeof fake_adc_value);
    for (int i = 0; i < PORT_PIN_COUNT; i++) {
        fake_gpio_level[i] = true;               /* fins de course au repos : pull-up => niveau haut */
    }
    s_fail_all = false;
    s_now = 1000;
    fake_i2c_transactions = 0;
    s_console[0] = '\0';
    s_console_pos = 0;
    for (int i = 0; i < PORT_MOTOR_COUNT; i++) {
        fake_motor_phys[i] = 0;
        fake_motor_phys_min[i] = -1000000000;     /* pas de butée mécanique tant que le test n'en demande pas */
        fake_motor_phys_max[i] = 1000000000;
        fake_motor_enabled[i] = false;
        fake_motor_dir_fill[i] = true;
        fake_motor_pulses[i] = 0;
        fake_motor_pulses_disabled[i] = 0;
    }
    s_link_rx_len = s_link_rx_pos = 0;
    s_link_tx_len = 0;
    s_link_tx_cap = LINK_BUF;
}

void fake_add_i2c(uint8_t addr, fake_write_fn w, fake_read_fn r, void *ctx)
{
    for (int i = 0; i < MAX_DEV; i++) {
        if (!s_dev[i].used) {
            s_dev[i] = (dev_t_){ true, addr, w, r, ctx };
            return;
        }
    }
}

void fake_remove_i2c(uint8_t addr)
{
    for (int i = 0; i < MAX_DEV; i++) {
        if (s_dev[i].used && s_dev[i].addr == addr) {
            s_dev[i].used = false;
        }
    }
}

void fake_i2c_set_fail(bool all_nack) { s_fail_all = all_nack; }
void fake_set_ow(const fake_ow_t *ow) { s_ow = *ow; }
void fake_time_set(uint32_t ms)       { s_now = ms; }
void fake_time_advance(uint32_t ms)   { s_now += ms; }
uint32_t fake_time_get(void)          { return s_now; }

void fake_console_feed(const char *s)
{
    strncpy(s_console, s, sizeof s_console - 1);
    s_console[sizeof s_console - 1] = '\0';
    s_console_pos = 0;
}

static dev_t_ *find(uint8_t addr)
{
    if (s_fail_all) {
        return NULL;
    }
    for (int i = 0; i < MAX_DEV; i++) {
        if (s_dev[i].used && s_dev[i].addr == addr) {
            return &s_dev[i];
        }
    }
    return NULL;
}

bool port_i2c_write(uint8_t addr7, const uint8_t *tx, uint16_t len)
{
    fake_i2c_transactions++;
    dev_t_ *d = find(addr7);
    return d && d->w && d->w(d->ctx, tx, len);
}

bool port_i2c_read(uint8_t addr7, uint8_t *rx, uint16_t len)
{
    fake_i2c_transactions++;
    dev_t_ *d = find(addr7);
    return d && d->r && d->r(d->ctx, rx, len);
}

bool port_i2c_write_read(uint8_t addr7, const uint8_t *tx, uint16_t txlen, uint8_t *rx, uint16_t rxlen)
{
    fake_i2c_transactions++;
    dev_t_ *d = find(addr7);
    return d && d->w && d->r && d->w(d->ctx, tx, txlen) && d->r(d->ctx, rx, rxlen);
}

bool port_i2c_probe(uint8_t addr7) { return find(addr7) != NULL; }

void     port_init(void)           { }
uint32_t port_millis(void)         { return s_now; }
void     port_delay_ms(uint32_t ms){ s_now += ms; }

uint16_t port_adc_read(port_adc_t ch) { return (ch < PORT_ADC_COUNT) ? fake_adc_value[ch] : 0; }
bool     port_gpio_read(port_pin_t p) { return (p < PORT_PIN_COUNT) ? fake_gpio_level[p] : false; }

bool port_ow_reset(void)                { return s_ow.reset ? s_ow.reset(s_ow.ctx) : false; }
void port_ow_write_byte(uint8_t b)      { if (s_ow.write_byte) s_ow.write_byte(s_ow.ctx, b); }
uint8_t port_ow_read_byte(void)         { return s_ow.read_byte ? s_ow.read_byte(s_ow.ctx) : 0xFF; }

int port_console_getc(uint32_t timeout_ms)
{
    (void)timeout_ms;
    if (s_console[s_console_pos] == '\0') {
        return -1;
    }
    return (unsigned char)s_console[s_console_pos++];
}

/* ------------------------------------------------------------------ moteurs pas à pas */
void port_motor_enable(port_motor_t m, bool on)
{
    if ((unsigned)m < (unsigned)PORT_MOTOR_COUNT) {
        fake_motor_enabled[m] = on;
    }
}

void port_motor_dir(port_motor_t m, bool fill)
{
    if ((unsigned)m < (unsigned)PORT_MOTOR_COUNT) {
        fake_motor_dir_fill[m] = fill;
    }
}

void port_motor_step(port_motor_t m)
{
    if ((unsigned)m >= (unsigned)PORT_MOTOR_COUNT) {
        return;
    }
    fake_motor_pulses[m]++;
    if (!fake_motor_enabled[m]) {
        fake_motor_pulses_disabled[m]++;
        return;
    }
    int32_t next = fake_motor_phys[m] + (fake_motor_dir_fill[m] ? 1 : -1);
    if (next >= fake_motor_phys_min[m] && next <= fake_motor_phys_max[m]) {
        fake_motor_phys[m] = next;            /* contre la butée mécanique, le moteur "décroche" : le pas est perdu */
    }
}

/* ------------------------------------------------------------------ liaison série */
void fake_link_feed(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len && s_link_rx_len < LINK_BUF; i++) {
        s_link_rx[s_link_rx_len++] = data[i];
    }
}

uint16_t port_link_read(uint8_t *buf, uint16_t max)
{
    uint16_t n = 0;
    while (n < max && s_link_rx_pos < s_link_rx_len) {
        buf[n++] = s_link_rx[s_link_rx_pos++];
    }
    if (s_link_rx_pos == s_link_rx_len) {
        s_link_rx_pos = s_link_rx_len = 0;
    }
    return n;
}

uint16_t port_link_write(const uint8_t *buf, uint16_t len)
{
    uint16_t n = 0;
    while (n < len && s_link_tx_len < s_link_tx_cap) {
        s_link_tx[s_link_tx_len++] = buf[n++];
    }
    return n;
}

uint16_t fake_link_tx_take(uint8_t *out, uint16_t max)
{
    uint16_t n = (s_link_tx_len < max) ? s_link_tx_len : max;
    for (uint16_t i = 0; i < n; i++) {
        out[i] = s_link_tx[i];
    }
    for (uint16_t i = n; i < s_link_tx_len; i++) {
        s_link_tx[i - n] = s_link_tx[i];
    }
    s_link_tx_len = (uint16_t)(s_link_tx_len - n);
    return n;
}

void fake_link_set_tx_capacity(uint16_t n) { s_link_tx_cap = (n < LINK_BUF) ? n : LINK_BUF; }
