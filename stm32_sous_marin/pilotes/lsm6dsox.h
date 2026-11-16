/**
 * @file lsm6dsox.h
 * @brief Driver de la centrale inertielle LSM6DSOX (accéléromètre + gyroscope), I2C 0x6A / 0x6B.
 */
#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include <stdint.h>
#include "types_capteurs.h"

#define LSM6DSOX_I2C_ADDR_LOW   0x6A
#define LSM6DSOX_I2C_ADDR_HIGH  0x6B
#define LSM6DSOX_WHO_AM_I_VALUE 0x6C

/* valeurs = codes des registres CTRL1_XL / CTRL2_G */
typedef enum { LSM6_XL_2G = 0, LSM6_XL_16G = 1, LSM6_XL_4G = 2, LSM6_XL_8G = 3 } lsm6dsox_acc_fs_t;
typedef enum { LSM6_G_250DPS = 0, LSM6_G_500DPS = 1, LSM6_G_1000DPS = 2, LSM6_G_2000DPS = 3 } lsm6dsox_gyr_fs_t;
typedef enum {
    LSM6_ODR_12_5HZ = 1, LSM6_ODR_26HZ = 2, LSM6_ODR_52HZ = 3, LSM6_ODR_104HZ = 4,
    LSM6_ODR_208HZ = 5, LSM6_ODR_416HZ = 6, LSM6_ODR_833HZ = 7
} lsm6dsox_odr_t;

typedef struct {
    uint8_t addr;
    float   acc_g_per_lsb;
    float   gyr_dps_per_lsb;
    float   gyro_bias_dps[3];    /**< biais soustrait à chaque lecture (voir calibrate_gyro) */
} lsm6dsox_t;

typedef struct {
    int16_t acc_raw[3];
    int16_t gyr_raw[3];
    float   acc_g[3];            /**< en g                                    */
    float   gyr_dps[3];          /**< en deg/s, biais retiré                  */
} lsm6dsox_data_t;

sens_status_t lsm6dsox_init(lsm6dsox_t *s, uint8_t addr, lsm6dsox_acc_fs_t acc_fs,
                            lsm6dsox_gyr_fs_t gyr_fs, lsm6dsox_odr_t odr);
/** SENS_ERR_NOT_READY si aucune nouvelle donnée (normal). */
sens_status_t lsm6dsox_read(lsm6dsox_t *s, lsm6dsox_data_t *out);
/**
 * Mesure le biais du gyroscope : le sous-marin doit être IMMOBILE.
 * @return SENS_ERR_RANGE si la dispersion trahit un mouvement (biais inchangé), TIMEOUT si pas de données.
 */
sens_status_t lsm6dsox_calibrate_gyro(lsm6dsox_t *s, uint16_t n_samples, uint32_t timeout_ms);

#endif
