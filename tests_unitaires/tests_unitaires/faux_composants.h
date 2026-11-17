/**
 * @file faux_composants.h
 * @brief Modèles simulés des capteurs (MS5837, LSM6DSOX, LIS3MDL, INA226, DS18B20) pour les tests.
 *
 * Ces modèles sont écrits à partir des fiches techniques, INDÉPENDAMMENT des drivers :
 * si un driver se trompe (mauvaise sensibilité, bit d'auto-incrément oublié...), le test échoue.
 */
#ifndef FAUX_COMPOSANTS_H
#define FAUX_COMPOSANTS_H

#include <stdint.h>
#include <stdbool.h>
#include "faux_registres.h"

/* ---------------------------------------------------------------- MS5837 (modèle 02BA par défaut) */
typedef struct {
    uint16_t prom[7];
    uint32_t d1, d2;
    uint8_t  mode;            /* 0 aucun, 1 PROM, 2 ADC */
    uint8_t  prom_idx;
    uint8_t  conv_kind;       /* 0 aucune, 1 pression, 2 température */
    uint32_t ready_at;
    uint32_t resets;
    uint32_t nack_next;       /* nb de transactions à refuser */
    uint32_t nack_skip;       /* nb de transactions acceptées avant de commencer à refuser */
    bool     dead_adc;        /* la conversion ne se termine jamais (ADC = 0) */
    uint32_t conv_d1_count, conv_d2_count;
    float    target_p;        /* pression visée (mbar) : D1 est recalculé quand la température change */
} fake_ms5837_t;

void fake_ms5837_setup(fake_ms5837_t *f, bool is_02ba);          /* PROM valide + ajoute l'appareil à 0x76 */
void fake_ms5837_set_pressure(fake_ms5837_t *f, float mbar);     /* à 20 degC */
void fake_ms5837_set_temp(fake_ms5837_t *f, float temp_c);

/* ---------------------------------------------------------------- INA226 (registres 16 bits) */
typedef struct {
    uint16_t regs[256];
    uint8_t  ptr;
    uint16_t last_config;
} fake_ina226_t;

void fake_ina226_setup(fake_ina226_t *f);                        /* ID valides, ajoute à 0x40 */
void fake_ina226_set(fake_ina226_t *f, float bus_v, float shunt_mv);

/* ---------------------------------------------------------------- DS18B20 */
typedef struct {
    bool     present;
    int16_t  raw;
    uint32_t conv_start;
    bool     converting;
    uint8_t  state;           /* 0 attend ROM cmd, 1 attend fonction, 2 lecture scratchpad */
    uint8_t  scratch[9];
    uint8_t  idx;
    bool     corrupt_crc;
    uint32_t conversions;
} fake_ds18b20_t;

void fake_ds18b20_setup(fake_ds18b20_t *f, float temp_c);        /* branche le faux bus 1-Wire */
uint8_t fake_crc8_maxim(const uint8_t *d, int n);                /* implémentation indépendante du driver */

/* ---------------------------------------------------------------- IMU / magnétomètre */
void fake_imu_setup(fake_regmap_t *m, uint8_t addr);
void fake_imu_set(fake_regmap_t *m, const float acc_g[3], const float gyr_dps[3]);   /* utilise la pleine échelle configurée */
void fake_mag_setup(fake_regmap_t *m, uint8_t addr);
void fake_mag_set(fake_regmap_t *m, const float gauss[3]);

/* ---------------------------------------------------------------- sous-marin complet */
typedef struct {
    fake_ms5837_t  baro;
    fake_regmap_t  imu;
    fake_regmap_t  mag;
    fake_ina226_t  ina;
    fake_ds18b20_t temp;
} fake_sub_t;

/** Sous-marin sain : à plat, immobile, 1013 mbar, batterie 4S à 16,4 V, 0,5 A, tout répond. */
void fake_sub_setup(fake_sub_t *s);

#endif
