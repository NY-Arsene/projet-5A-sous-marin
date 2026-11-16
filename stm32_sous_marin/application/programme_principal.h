/**
 * @file programme_principal.h
 * @brief Application complète du sous-marin côté STM32 : assemble capteurs, ballasts, régulation, sécurité, liaison.
 *
 * Intégration dans le main.c généré par CubeMX (zones USER CODE) :
 *
 *   #include "programme_principal.h"
 *   ...
 *   // USER CODE BEGIN 2
 *   app_init(&g_app, HAL_GetTick());          // bloque ~2 s (calibration du gyroscope) : sous-marin immobile !
 *   HAL_TIM_Base_Start_IT(&htim6);            // TIM6 réglé à 20 kHz
 *   // USER CODE END 2
 *   while (1) {
 *       // USER CODE BEGIN 3
 *       app_step(&g_app, HAL_GetTick());
 *   }
 *   ...
 *   void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 *   {
 *       if (htim->Instance == TIM6) { app_tick(&g_app); }
 *   }
 */
#ifndef PROGRAMME_PRINCIPAL_H
#define PROGRAMME_PRINCIPAL_H

#include <stdint.h>
#include <stdbool.h>
#include "gestionnaire_capteurs.h"
#include "ballast.h"
#include "regulation_profondeur.h"
#include "securite.h"
#include "liaison.h"

#define APP_CONTROL_PERIOD_MS    10u      /**< régulation à 100 Hz        */
#define APP_TELEMETRY_PERIOD_MS  100u     /**< télémétrie à 10 Hz         */
#define APP_TICK_HZ              20000u   /**< fréquence de app_tick()    */

enum { BALLAST_FRONT = 0, BALLAST_AFT = 1 };

typedef struct {
    sensor_manager_t sm;
    sensors_config_t scfg;
    ballast_t        ballast[2];
    depth_ctrl_t     dc;
    safety_t         safety;
    link_rx_t        rx;

    float      manual_ml[2];          /**< volumes demandés en mode MANUAL */
    sub_mode_t prev_mode;
    uint32_t   last_ctrl_ms, last_tlm_ms;
    bool       clear_pending, tare_pending;
    uint32_t   sensors_init_failed;   /**< masque renvoyé par sensors_init() */
    bool       init_ok;               /**< false si une des initialisations a échoué (config invalide) */

    /* compteurs de diagnostic (envoyés plus tard en télémétrie si besoin) */
    uint32_t   cmd_frames, tlm_sent, tx_overflow;
} app_t;

extern app_t g_app;

/** Initialise tout. Bloquant (~2 s) : le sous-marin doit être immobile et à la surface. */
void app_init(app_t *a, uint32_t now_ms);

/** Boucle principale : à appeler aussi souvent que possible. Ne bloque jamais. */
void app_step(app_t *a, uint32_t now_ms);

/** Interruption timer à APP_TICK_HZ : fait avancer les moteurs. Très courte. */
void app_tick(app_t *a);

#endif
