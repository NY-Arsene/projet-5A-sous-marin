/** @file faux_registres.h  Faux périphérique I2C à registres 8 bits (LSM6DSOX, LIS3MDL). */
#ifndef FAUX_REGISTRES_H
#define FAUX_REGISTRES_H

#include <stdint.h>
#include <stdbool.h>

typedef struct fake_regmap {
    uint8_t  regs[256];
    uint8_t  ptr;
    bool     autoinc_needs_msb;   /**< LIS3MDL : sans le bit 7 de l'adresse, pas d'incrément automatique */
    bool     last_msb;
    void   (*on_write_reg)(struct fake_regmap *m, uint8_t reg, uint8_t val);
    void   (*on_read_reg)(struct fake_regmap *m, uint8_t reg);   /**< appelé avant chaque octet lu */
    uint32_t reads, writes;
    void    *user;
} fake_regmap_t;

void fake_regmap_init(fake_regmap_t *m, bool autoinc_needs_msb);
bool fake_regmap_write(void *ctx, const uint8_t *tx, uint16_t len);
bool fake_regmap_read(void *ctx, uint8_t *rx, uint16_t len);

#endif
