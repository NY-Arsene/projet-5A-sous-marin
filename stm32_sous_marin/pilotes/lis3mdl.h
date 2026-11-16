/** @file lis3mdl.h  Driver du magnétomètre LIS3MDL, I2C 0x1C / 0x1E. */
#ifndef LIS3MDL_H
#define LIS3MDL_H

#include <stdint.h>
#include "types_capteurs.h"

#define LIS3MDL_I2C_ADDR_LOW   0x1C
#define LIS3MDL_I2C_ADDR_HIGH  0x1E
#define LIS3MDL_WHO_AM_I_VALUE 0x3D

typedef enum { LIS3_FS_4G = 0, LIS3_FS_8G = 1, LIS3_FS_12G = 2, LIS3_FS_16G = 3 } lis3mdl_fs_t;   /* gauss */
typedef enum { LIS3_ODR_10HZ = 4, LIS3_ODR_20HZ = 5, LIS3_ODR_40HZ = 6, LIS3_ODR_80HZ = 7 } lis3mdl_odr_t;

typedef struct {
    uint8_t addr;
    float   gauss_per_lsb;
} lis3mdl_t;

typedef struct {
    int16_t raw[3];
    float   gauss[3];     /**< 1 gauss = 100 microtesla ; champ terrestre : 0,25 à 0,65 gauss */
} lis3mdl_data_t;

sens_status_t lis3mdl_init(lis3mdl_t *s, uint8_t addr, lis3mdl_fs_t fs, lis3mdl_odr_t odr);
/** SENS_ERR_NOT_READY si pas de nouvelle donnée. */
sens_status_t lis3mdl_read(lis3mdl_t *s, lis3mdl_data_t *out);

#endif
