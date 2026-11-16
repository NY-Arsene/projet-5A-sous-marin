/**
 * @file banc_test.h
 * @brief Banc de test des capteurs : à exécuter sur la cible (Nucleo) via la console série.
 *
 * Chaque test affiche des mesures puis un verdict PASS / FAIL selon des critères chiffrés, pour
 * valider un capteur branché AVANT de l'intégrer au sous-marin (et à chaque changement de câblage).
 * Les fonctions renvoient le nombre de critères en échec (0 = tout est bon).
 *
 * Dans main.c :   bench_menu();     // boucle interactive sur la console (115200 bauds)
 */
#ifndef BANC_TEST_H
#define BANC_TEST_H

#include <stdint.h>

int  bench_i2c_scan(void);           /**< présence des 4 composants I2C attendus           */
int  bench_baro(void);               /**< MS5837 : PROM/CRC, pression, bruit, température    */
int  bench_imu(void);                /**< LSM6DSOX : WHO_AM_I, 1 g, bruit gyro, biais        */
int  bench_mag(void);                /**< LIS3MDL : WHO_AM_I, norme du champ, bruit          */
int  bench_power(void);              /**< INA226 : tension batterie, courant                 */
int  bench_tprobe(void);             /**< DS18B20 : présence, CRC, plage                     */
int  bench_leak(void);               /**< détecteur de fuite à sec : pas de fausse alarme    */
int  bench_manager(uint32_t duration_ms); /**< intégration : tous les capteurs ensemble      */
int  bench_run_all(void);            /**< tous les tests non interactifs ; renvoie le nb d'échecs */

/* --- Tests interactifs (manipulation requise) --- */
int  bench_leak_wet_test(uint32_t timeout_ms);       /**< mouiller les électrodes            */
void bench_limits(uint32_t duration_ms);             /**< appuyer sur les fins de course     */
void bench_attitude_live(uint32_t duration_ms);      /**< incliner le module : vérifier les signes */
int  bench_mag_calibration(uint32_t duration_ms);    /**< tourner le module dans tous les sens */
void bench_depth_demo(void);                         /**< tare puis immersion de 20 cm       */

void bench_menu(void);
#endif
