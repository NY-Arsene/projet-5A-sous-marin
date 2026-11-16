/**
 * @file gestionnaire_capteurs.h
 * @brief Gestionnaire de capteurs du sous-marin : initialisation, ordonnancement, santé, sécurités.
 *
 * Usage dans le firmware :
 *   sensor_manager_t sm;  sensors_config_t cfg;
 *   sensors_default_config(&cfg);
 *   uint32_t failed = sensors_init(&sm, &cfg, HAL_GetTick());
 *   sensors_calibrate_gyro(&sm, 200);   // sous-marin immobile
 *   sensors_tare_depth(&sm, 100);       // capteur à la surface
 *   for (;;) {
 *       sensors_poll(&sm, HAL_GetTick());          // le plus souvent possible (>= 200 Hz)
 *       const sensor_data_t *d = sensors_data(&sm);
 *       if (d->failsafe_mask) { ...remontée d'urgence... }
 *   }
 */
#ifndef GESTIONNAIRE_CAPTEURS_H
#define GESTIONNAIRE_CAPTEURS_H

#include <stdint.h>
#include <stdbool.h>
#include "types_capteurs.h"
#include "materiel.h"
#include "ms5837.h"
#include "lsm6dsox.h"
#include "lis3mdl.h"
#include "ina226.h"
#include "ds18b20.h"
#include "detecteur_fuite.h"
#include "anti_rebond.h"
#include "profondeur.h"
#include "assiette.h"
#include "calibration_magneto.h"
#include "batterie.h"

typedef enum {
    SENSOR_BARO = 0, SENSOR_IMU, SENSOR_MAG, SENSOR_POWER, SENSOR_TPROBE, SENSOR_COUNT
} sensor_id_t;

/* Bits de sécurité (failsafe_mask) : à brancher sur le mode de sécurité de la fiche (remontée automatique) */
#define FS_LEAK        (1u << 0)   /**< eau détectée dans la coque (mémorisé)          */
#define FS_BATT_WARN   (1u << 1)   /**< batterie faible                                */
#define FS_BATT_CRIT   (1u << 2)   /**< batterie critique : remonter immédiatement     */
#define FS_DEPTH_LOST  (1u << 3)   /**< capteur de pression muet : profondeur inconnue */
#define FS_IMU_LOST    (1u << 4)   /**< centrale inertielle muette : assiette inconnue */
#define FS_OVER_DEPTH  (1u << 5)   /**< profondeur maximale autorisée dépassée         */

#define SENSORS_INIT_LEAK_FAILED (1u << SENSOR_COUNT)   /**< bit supplémentaire du retour de sensors_init() */

typedef struct {
    ms5837_model_t baro_model;
    ms5837_osr_t   baro_osr;
    float          rho_kg_m3;
    uint8_t        imu_addr, mag_addr, ina_addr;
    float          shunt_ohm;
    int8_t         imu_axis_map[3];      /**< axes capteur -> repère sous-marin (voir attitude_remap_axes) */
    int8_t         mag_axis_map[3];
    uint8_t        batt_cells;
    float          v_warn_cell, v_crit_cell;
    uint16_t       leak_thr_on, leak_thr_off;
    bool           leak_active_high;
    bool           limit_active_low;     /**< fin de course câblé contre la masse avec pull-up */
    float          max_depth_m;
    uint32_t       timeout_ms[SENSOR_COUNT];   /**< sans mesure valide plus longtemps => capteur "perdu" */
    uint32_t       imu_period_ms, mag_period_ms, power_period_ms, tprobe_period_ms, digital_period_ms;
    uint32_t       retry_period_ms;      /**< délai entre deux tentatives de ré-initialisation d'un capteur absent */
} sensors_config_t;

typedef struct {
    float        depth_m;            /**< profondeur filtrée (+ vers le bas)   */
    float        depth_rate_m_s;
    float        pressure_mbar;
    float        water_temp_c;       /**< température mesurée par le MS5837     */
    float        hull_temp_c;        /**< sonde DS18B20 (intérieur de la coque) */
    float        roll_deg, pitch_deg, yaw_deg;
    float        acc_g[3], gyro_dps[3], mag_gauss[3];   /**< repère sous-marin    */
    float        vbat_v, ibat_a, pbat_w, soc_pct;
    batt_level_t batt_level;
    bool         leak;
    bool         limit[4];           /**< ordre de port_pin_t : AV haut, AV bas, AR haut, AR bas */
    bool         depth_ref_valid;    /**< false tant que sensors_tare_depth() n'a pas abouti */
    uint32_t     healthy_mask;       /**< bit (1<<sensor_id_t) = capteur sain          */
    uint32_t     failsafe_mask;      /**< FS_*                                         */
    uint32_t     timestamp_ms;
} sensor_data_t;

typedef struct {
    sensors_config_t cfg;
    ms5837_t   baro;
    lsm6dsox_t imu;
    lis3mdl_t  mag;
    ina226_t   power;
    ds18b20_t  tprobe;
    leak_t     leak;
    debounce_t limit[4];
    depth_t    depth;
    attitude_t att;
    mag_cal_t  mag_cal;
    battery_t  batt;
    bool       present[SENSOR_COUNT];
    bool       ever_ok[SENSOR_COUNT];
    uint32_t   last_ok_ms[SENSOR_COUNT];
    uint32_t   last_progress_ms[SENSOR_COUNT];   /**< dernier succès ou dernière (ré)initialisation : sert de chien de garde */
    uint32_t   last_run_ms[SENSOR_COUNT];
    uint32_t   last_digital_ms;
    uint32_t   last_retry_ms;
    uint32_t   last_att_ms;
    bool       mag_fresh;
    float      mag_body[3];
    sensor_data_t data;
} sensor_manager_t;

void     sensors_default_config(sensors_config_t *cfg);
/** @return masque des capteurs dont l'initialisation a échoué (bit SENSOR_x ; SENSORS_INIT_LEAK_FAILED pour la fuite) */
uint32_t sensors_init(sensor_manager_t *m, const sensors_config_t *cfg, uint32_t now_ms);
void     sensors_poll(sensor_manager_t *m, uint32_t now_ms);
const sensor_data_t *sensors_data(const sensor_manager_t *m);

sens_status_t sensors_calibrate_gyro(sensor_manager_t *m, uint16_t n_samples);
void          sensors_tare_depth(sensor_manager_t *m, uint16_t n_samples);
void          sensors_set_mag_cal(sensor_manager_t *m, const mag_cal_t *cal);
void          sensors_clear_leak(sensor_manager_t *m);

#endif
