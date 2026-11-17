/**
 * @file materiel_faux.h
 * @brief Implémentation "PC" de materiel.h : faux bus I2C, temps simulé, ADC/GPIO/1-Wire pilotables.
 *
 * Permet de tester les drivers SANS matériel et d'injecter des pannes (NACK, capteur débranché...).
 */
#ifndef MATERIEL_FAUX_H
#define MATERIEL_FAUX_H

#include "materiel.h"

typedef bool (*fake_write_fn)(void *ctx, const uint8_t *tx, uint16_t len);
typedef bool (*fake_read_fn)(void *ctx, uint8_t *rx, uint16_t len);

typedef struct {
    bool    (*reset)(void *ctx);
    void    (*write_byte)(void *ctx, uint8_t b);
    uint8_t (*read_byte)(void *ctx);
    void    *ctx;
} fake_ow_t;

void fake_reset(void);                                   /**< remet tout à zéro avant chaque test */
void fake_add_i2c(uint8_t addr, fake_write_fn w, fake_read_fn r, void *ctx);
void fake_remove_i2c(uint8_t addr);                      /**< simule un capteur débranché */
void fake_i2c_set_fail(bool all_nack);                   /**< simule un bus bloqué : tout est NACK */
void fake_set_ow(const fake_ow_t *ow);

void     fake_time_set(uint32_t ms);
void     fake_time_advance(uint32_t ms);
uint32_t fake_time_get(void);

extern uint16_t fake_adc_value[PORT_ADC_COUNT];
extern bool     fake_gpio_level[PORT_PIN_COUNT];
extern uint32_t fake_i2c_transactions;                   /**< nombre de transactions depuis fake_reset */

void fake_console_feed(const char *s);                   /**< caractères que port_console_getc() renverra */

/* ---- Moteurs pas à pas simulés : le faux driver compte les impulsions et modélise les butées mécaniques ---- */
extern int32_t  fake_motor_phys[PORT_MOTOR_COUNT];       /**< position physique du piston (pas) */
extern int32_t  fake_motor_phys_min[PORT_MOTOR_COUNT];   /**< butée mécanique basse : les impulsions au-delà sont perdues */
extern int32_t  fake_motor_phys_max[PORT_MOTOR_COUNT];   /**< butée mécanique haute */
extern bool     fake_motor_enabled[PORT_MOTOR_COUNT];
extern bool     fake_motor_dir_fill[PORT_MOTOR_COUNT];
extern uint32_t fake_motor_pulses[PORT_MOTOR_COUNT];     /**< impulsions STEP reçues depuis fake_reset */
extern uint32_t fake_motor_pulses_disabled[PORT_MOTOR_COUNT]; /**< impulsions reçues alors que le driver était coupé (anomalie) */

/* ---- Liaison série Raspberry Pi simulée ---- */
void     fake_link_feed(const uint8_t *data, uint16_t len);       /**< octets que port_link_read() renverra */
uint16_t fake_link_tx_take(uint8_t *out, uint16_t max);           /**< octets émis par le firmware depuis le dernier appel */
void     fake_link_set_tx_capacity(uint16_t n);                   /**< simule une file d'émission presque pleine */

#endif
