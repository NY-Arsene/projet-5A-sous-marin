/**
 * @file ina226.h
 * @brief Driver du moniteur de puissance INA226 (tension batterie + courant via shunt), I2C 0x40.
 *
 * Le courant est calculé en logiciel à partir de la tension du shunt (registre 0x01, 2,5 uV/LSB) :
 * pas de registre de calibration à régler. Plage mesurable : +-81,92 mV / R_shunt
 * (avec 10 mohm : +-8,19 A ; au-delà le champ `saturated` passe à true).
 */
#ifndef INA226_H
#define INA226_H

#include <stdint.h>
#include <stdbool.h>
#include "types_capteurs.h"

#define INA226_I2C_ADDR 0x40

typedef enum {
    INA226_AVG_1 = 0, INA226_AVG_4, INA226_AVG_16, INA226_AVG_64,
    INA226_AVG_128, INA226_AVG_256, INA226_AVG_512, INA226_AVG_1024
} ina226_avg_t;

typedef struct {
    uint8_t addr;
    float   shunt_ohm;
} ina226_t;

typedef struct {
    float bus_v;
    float shunt_mv;
    float current_a;      /**< positif = décharge (courant entrant côté charge) */
    float power_w;
    bool  saturated;      /**< convertisseur en butée : courant réel supérieur */
} ina226_data_t;

sens_status_t ina226_init(ina226_t *s, uint8_t addr, float shunt_ohm, ina226_avg_t avg);
sens_status_t ina226_read(ina226_t *s, ina226_data_t *out);

#endif
