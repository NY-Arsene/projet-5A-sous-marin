#include "faux_registres.h"
#include <string.h>

void fake_regmap_init(fake_regmap_t *m, bool autoinc_needs_msb)
{
    memset(m, 0, sizeof *m);
    m->autoinc_needs_msb = autoinc_needs_msb;
}

bool fake_regmap_write(void *ctx, const uint8_t *tx, uint16_t len)
{
    fake_regmap_t *m = ctx;
    if (len == 0) {
        return true;
    }
    uint8_t reg = tx[0];
    m->last_msb = (reg & 0x80) != 0;
    if (m->autoinc_needs_msb) {
        reg &= 0x7F;
    }
    m->ptr = reg;
    for (uint16_t i = 1; i < len; i++) {
        m->regs[m->ptr] = tx[i];
        m->writes++;
        if (m->on_write_reg) {
            m->on_write_reg(m, m->ptr, tx[i]);
        }
        if (!m->autoinc_needs_msb || m->last_msb) {
            m->ptr++;
        }
    }
    return true;
}

bool fake_regmap_read(void *ctx, uint8_t *rx, uint16_t len)
{
    fake_regmap_t *m = ctx;
    for (uint16_t i = 0; i < len; i++) {
        if (m->on_read_reg) {
            m->on_read_reg(m, m->ptr);
        }
        rx[i] = m->regs[m->ptr];
        m->reads++;
        if (!m->autoinc_needs_msb || m->last_msb) {
            m->ptr++;
        }
    }
    return true;
}
