/**
 * @file materiel.h
 * @brief Couche d'abstraction matérielle (HAL minimale).
 *
 * Tout le code capteurs (drivers + lib + app) n'appelle QUE ces fonctions.
 *  - Sur la cible : materiel/materiel_stm32.c (HAL STM32).
 *  - Sur PC (tests unitaires) : tests/materiel_faux.c (faux bus I2C, temps simulé).
 * C'est ce qui permet de tester les drivers sans matériel.
 */
#ifndef MATERIEL_H
#define MATERIEL_H

#include <stdint.h>
#include <stdbool.h>

typedef enum { PORT_ADC_LEAK = 0, PORT_ADC_COUNT } port_adc_t;

typedef enum {
    PORT_PIN_LIMIT_FWD_TOP = 0,   /**< fin de course ballast avant, haut */
    PORT_PIN_LIMIT_FWD_BOTTOM,    /**< fin de course ballast avant, bas  */
    PORT_PIN_LIMIT_AFT_TOP,       /**< fin de course ballast arrière, haut */
    PORT_PIN_LIMIT_AFT_BOTTOM,    /**< fin de course ballast arrière, bas  */
    PORT_PIN_COUNT
} port_pin_t;

/* ---- I2C (adresses 7 bits) : renvoient true si la transaction est acquittée ---- */
bool port_i2c_write(uint8_t addr7, const uint8_t *tx, uint16_t len);
bool port_i2c_read(uint8_t addr7, uint8_t *rx, uint16_t len);
/** Écriture de txlen octets (adresse registre) puis lecture avec repeated-start. */
bool port_i2c_write_read(uint8_t addr7, const uint8_t *tx, uint16_t txlen, uint8_t *rx, uint16_t rxlen);
/** true si un périphérique répond à cette adresse (scan I2C du banc de test). */
bool port_i2c_probe(uint8_t addr7);

/** Initialisations dépendantes de la cible (sur la Nucleo : démarre la réception de la liaison série). Sans effet sur PC. */
void port_init(void);

/* ---- Temps ---- */
uint32_t port_millis(void);
void     port_delay_ms(uint32_t ms);

/* ---- Entrées ---- */
uint16_t port_adc_read(port_adc_t ch);   /**< valeur brute 12 bits (0..4095) */
bool     port_gpio_read(port_pin_t pin); /**< true = niveau haut             */

/* ---- 1-Wire (DS18B20 unique sur le bus) ---- */
bool    port_ow_reset(void);             /**< true si impulsion de présence */
void    port_ow_write_byte(uint8_t b);
uint8_t port_ow_read_byte(void);

/* ---- Console (banc de test) : -1 si aucun caractère reçu dans le délai ---- */
int port_console_getc(uint32_t timeout_ms);

/* ---- Moteurs pas à pas des ballasts (interface STEP / DIR / EN : A4988, DRV8825, TMC2209...) ---- */
typedef enum { PORT_MOTOR_FWD = 0, PORT_MOTOR_AFT, PORT_MOTOR_COUNT } port_motor_t;

/** true = driver alimenté (broche EN active). Appelable depuis le programme principal. */
void port_motor_enable(port_motor_t m, bool on);
/** Sens du mouvement : true = le piston avance dans le sens "le ballast se remplit" (volume qui augmente).
 *  L'inversion éventuelle du câblage se règle dans materiel_stm32.c, pas dans le code applicatif. */
void port_motor_dir(port_motor_t m, bool fill);
/** Une impulsion sur STEP = un micropas. Appelée depuis l'interruption du timer : doit être très courte. */
void port_motor_step(port_motor_t m);

/* ---- Liaison série avec la Raspberry Pi (non bloquante) ---- */
/** Copie dans buf (max octets) ce qui est arrivé depuis le dernier appel ; renvoie le nombre d'octets copiés. */
uint16_t port_link_read(uint8_t *buf, uint16_t max);
/** Met en file d'émission ; renvoie le nombre d'octets acceptés (moins que len si la file est pleine). */
uint16_t port_link_write(const uint8_t *buf, uint16_t len);

#endif /* MATERIEL_H */
