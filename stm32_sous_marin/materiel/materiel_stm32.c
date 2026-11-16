/**
 * @file materiel_stm32.c
 * @brief Implémentation de materiel.h pour la NUCLEO-F446RE (STM32 HAL).
 *
 * Prérequis dans STM32CubeMX / CubeIDE :
 *  - I2C1 (PB8 = SCL, PB9 = SDA sur la Nucleo), 100 kHz au début puis 400 kHz si le câblage le permet,
 *    avec résistances de rappel (les modules Adafruit en ont déjà : ne pas en ajouter trop).
 *  - ADC1 sur UNE voie (détecteur de fuite), 12 bits, conversion unique (single conversion).
 *  - USART2 (liaison ST-Link virtuelle) en 115200 bauds pour la console du banc de test.
 *  - Une broche GPIO en "Output Open-Drain" avec résistance externe 4,7 kohm vers 3,3 V pour le 1-Wire.
 *  - 4 entrées GPIO avec pull-up interne pour les fins de course.
 *  - Étiquettes (User Label) à donner dans CubeMX : OW, LIMIT_FWD_TOP, LIMIT_FWD_BOTTOM,
 *    LIMIT_AFT_TOP, LIMIT_AFT_BOTTOM  (=> macros OW_Pin / OW_GPIO_Port, etc. dans main.h).
 *  - printf avec des float : ajouter  -u _printf_float  aux options de l'éditeur de liens.
 *
 * Ajouts pour l'application complète (ballasts + liaison Raspberry Pi) :
 *  - 6 sorties GPIO push-pull, vitesse "High" ; niveau initial BAS pour STEP et DIR, niveau initial HAUT pour EN
 *    (driver coupé dès la mise sous tension, avant même app_init) ; étiquettes :
 *        STEP_FWD, DIR_FWD, EN_FWD   (driver du ballast avant)
 *        STEP_AFT, DIR_AFT, EN_AFT   (driver du ballast arrière)
 *    (EN : actif à l'état bas sur A4988 / DRV8825 / TMC2209 ; voir MOTOR_EN_ACTIVE_LEVEL plus bas.)
 *  - TIM6 : fréquence de mise à jour 20 kHz (APB1 timer clock / (PSC+1) / (ARR+1)), interruption activée,
 *    priorité PLUS HAUTE que celle de l'USART. Le callback HAL_TIM_PeriodElapsedCallback appelle app_tick() (voir programme_principal.h).
 *  - USART1 (PA9 = TX, PA10 = RX) en 115200 bauds 8N1, interruption globale activée : liaison avec la Raspberry Pi
 *    (TX de l'un sur RX de l'autre, masse commune, niveaux 3,3 V des deux côtés).
 */
#include "materiel.h"
#include "main.h"        /* HAL + étiquettes des broches générées par CubeMX */
#include <stdio.h>

extern I2C_HandleTypeDef  hi2c1;
extern ADC_HandleTypeDef  hadc1;
extern UART_HandleTypeDef huart2;

#define I2C_TIMEOUT_MS      5u
#define I2C_FAIL_RECOVER    5u      /* nb d'échecs consécutifs avant de réinitialiser le périphérique I2C */

static uint8_t s_i2c_fail_count;

static bool i2c_result(HAL_StatusTypeDef st)
{
    if (st == HAL_OK) {
        s_i2c_fail_count = 0;
        return true;
    }
    /* Un bus bloqué (SDA tenu à 0 par un esclave) se débloque en réinitialisant le périphérique. */
    if (++s_i2c_fail_count >= I2C_FAIL_RECOVER) {
        s_i2c_fail_count = 0;
        HAL_I2C_DeInit(&hi2c1);
        HAL_I2C_Init(&hi2c1);
    }
    return false;
}

bool port_i2c_write(uint8_t addr7, const uint8_t *tx, uint16_t len)
{
    return i2c_result(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr7 << 1), (uint8_t *)tx, len, I2C_TIMEOUT_MS));
}

bool port_i2c_read(uint8_t addr7, uint8_t *rx, uint16_t len)
{
    return i2c_result(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(addr7 << 1), rx, len, I2C_TIMEOUT_MS));
}

bool port_i2c_write_read(uint8_t addr7, const uint8_t *tx, uint16_t txlen, uint8_t *rx, uint16_t rxlen)
{
    if (txlen != 1) {                 /* tous nos capteurs utilisent une adresse registre sur 8 bits */
        return false;
    }
    return i2c_result(HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr7 << 1), tx[0], I2C_MEMADD_SIZE_8BIT,
                                       rx, rxlen, I2C_TIMEOUT_MS));
}

bool port_i2c_probe(uint8_t addr7)
{
    return HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1), 1, I2C_TIMEOUT_MS) == HAL_OK;
}

uint32_t port_millis(void)            { return HAL_GetTick(); }
void     port_delay_ms(uint32_t ms)   { HAL_Delay(ms); }

uint16_t port_adc_read(port_adc_t ch)
{
    (void)ch;                          /* une seule voie configurée dans CubeMX */
    uint16_t v = 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK) {
        v = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return v;
}

bool port_gpio_read(port_pin_t pin)
{
    switch (pin) {
    case PORT_PIN_LIMIT_FWD_TOP:    return HAL_GPIO_ReadPin(LIMIT_FWD_TOP_GPIO_Port,    LIMIT_FWD_TOP_Pin)    == GPIO_PIN_SET;
    case PORT_PIN_LIMIT_FWD_BOTTOM: return HAL_GPIO_ReadPin(LIMIT_FWD_BOTTOM_GPIO_Port, LIMIT_FWD_BOTTOM_Pin) == GPIO_PIN_SET;
    case PORT_PIN_LIMIT_AFT_TOP:    return HAL_GPIO_ReadPin(LIMIT_AFT_TOP_GPIO_Port,    LIMIT_AFT_TOP_Pin)    == GPIO_PIN_SET;
    case PORT_PIN_LIMIT_AFT_BOTTOM: return HAL_GPIO_ReadPin(LIMIT_AFT_BOTTOM_GPIO_Port, LIMIT_AFT_BOTTOM_Pin) == GPIO_PIN_SET;
    default:                        return false;
    }
}

/* ------------------------------------------------------------------ 1-Wire (bit-banging) */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000u);
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) { }
}

static inline void ow_low(void)     { HAL_GPIO_WritePin(OW_GPIO_Port, OW_Pin, GPIO_PIN_RESET); }
static inline void ow_release(void) { HAL_GPIO_WritePin(OW_GPIO_Port, OW_Pin, GPIO_PIN_SET); }   /* open-drain : Hi-Z */
static inline bool ow_level(void)   { return HAL_GPIO_ReadPin(OW_GPIO_Port, OW_Pin) == GPIO_PIN_SET; }

bool port_ow_reset(void)
{
    static bool dwt_ready = false;
    if (!dwt_ready) {
        dwt_init();
        dwt_ready = true;
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ow_low();
    delay_us(480);
    ow_release();
    delay_us(70);
    bool presence = !ow_level();       /* la sonde tire la ligne à 0 pour signaler sa présence */
    delay_us(410);
    __set_PRIMASK(primask);
    return presence;
}

static void ow_write_bit(bool b)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();                   /* gabarits temporels de l'ordre de la microseconde */
    ow_low();
    if (b) {
        delay_us(6);
        ow_release();
        delay_us(64);
    } else {
        delay_us(60);
        ow_release();
        delay_us(10);
    }
    __set_PRIMASK(primask);
}

static bool ow_read_bit(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ow_low();
    delay_us(6);
    ow_release();
    delay_us(9);
    bool b = ow_level();
    delay_us(55);
    __set_PRIMASK(primask);
    return b;
}

void port_ow_write_byte(uint8_t v)
{
    for (int i = 0; i < 8; i++) {
        ow_write_bit((v >> i) & 1u);   /* bit de poids faible en premier */
    }
}

uint8_t port_ow_read_byte(void)
{
    uint8_t v = 0;
    for (int i = 0; i < 8; i++) {
        if (ow_read_bit()) {
            v |= (uint8_t)(1u << i);
        }
    }
    return v;
}

/* ------------------------------------------------------------------ console */
int port_console_getc(uint32_t timeout_ms)
{
    uint8_t c;
    return (HAL_UART_Receive(&huart2, &c, 1, timeout_ms) == HAL_OK) ? (int)c : -1;
}

/** Redirige printf vers l'USART2 (newlib). */
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, 100);
    return len;
}

/* ================================================================== moteurs pas à pas */
/* Niveau logique qui ACTIVE le driver (broche EN). A4988 / DRV8825 / TMC2209 : actif à l'état bas. */
#define MOTOR_EN_ACTIVE_LEVEL      GPIO_PIN_RESET
/* Niveau de la broche DIR qui correspond à "le ballast se remplit". Inverser ici si le moteur tourne à l'envers. */
#define MOTOR_FWD_DIR_FILL_LEVEL   GPIO_PIN_SET
#define MOTOR_AFT_DIR_FILL_LEVEL   GPIO_PIN_SET
/* Durée de l'impulsion STEP en boucles vides. Il faut >= 2 us pour un DRV8825 : à régler à l'oscilloscope
 * (en -O0, une boucle vide dure ~ 6 à 8 cycles : à 84 MHz, 60 boucles ~ 5 us ; à 180 MHz ~ 2,5 us). */
#define STEP_PULSE_LOOPS           60u

void port_motor_enable(port_motor_t m, bool on)
{
    GPIO_PinState level = on ? MOTOR_EN_ACTIVE_LEVEL
                             : (MOTOR_EN_ACTIVE_LEVEL == GPIO_PIN_RESET ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (m == PORT_MOTOR_FWD)      HAL_GPIO_WritePin(EN_FWD_GPIO_Port, EN_FWD_Pin, level);
    else if (m == PORT_MOTOR_AFT) HAL_GPIO_WritePin(EN_AFT_GPIO_Port, EN_AFT_Pin, level);
}

void port_motor_dir(port_motor_t m, bool fill)
{
    if (m == PORT_MOTOR_FWD) {
        GPIO_PinState inv = (MOTOR_FWD_DIR_FILL_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin(DIR_FWD_GPIO_Port, DIR_FWD_Pin, fill ? MOTOR_FWD_DIR_FILL_LEVEL : inv);
    } else if (m == PORT_MOTOR_AFT) {
        GPIO_PinState inv = (MOTOR_AFT_DIR_FILL_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin(DIR_AFT_GPIO_Port, DIR_AFT_Pin, fill ? MOTOR_AFT_DIR_FILL_LEVEL : inv);
    }
}

/** Appelée dans l'interruption TIM6. Écriture directe dans BSRR (atomique, une seule instruction) plutôt que HAL_GPIO_WritePin. */
void port_motor_step(port_motor_t m)
{
    GPIO_TypeDef *port;
    uint16_t pin;
    if (m == PORT_MOTOR_FWD)      { port = STEP_FWD_GPIO_Port; pin = STEP_FWD_Pin; }
    else if (m == PORT_MOTOR_AFT) { port = STEP_AFT_GPIO_Port; pin = STEP_AFT_Pin; }
    else                          { return; }
    port->BSRR = pin;                                   /* front montant */
    for (volatile uint32_t i = 0; i < STEP_PULSE_LOOPS; i++) { }
    port->BSRR = (uint32_t)pin << 16;                   /* front descendant */
}

/* ================================================================== liaison série Raspberry Pi (USART1, interruptions) */
extern UART_HandleTypeDef huart1;

#define LINK_RX_SIZE 256u                               /* puissances de 2 : masque au lieu d'un modulo */
#define LINK_TX_SIZE 512u

static uint8_t           s_rx_ring[LINK_RX_SIZE];
static volatile uint16_t s_rx_head, s_rx_tail;          /* head : écrit par l'interruption ; tail : lu par le programme */
static uint8_t           s_rx_byte;
static uint8_t           s_tx_ring[LINK_TX_SIZE];
static volatile uint16_t s_tx_head, s_tx_tail;          /* head : écrit par le programme ; tail : lu par l'interruption */
static volatile bool     s_tx_busy;
static uint8_t           s_tx_byte;

void port_init(void)
{
    s_rx_head = s_rx_tail = s_tx_head = s_tx_tail = 0;
    s_tx_busy = false;
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);        /* démarre la réception d'un octet à la fois */
}

uint16_t port_link_read(uint8_t *buf, uint16_t max)
{
    uint16_t n = 0;
    while (n < max && s_rx_tail != s_rx_head) {
        buf[n++] = s_rx_ring[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1u) & (LINK_RX_SIZE - 1u));
    }
    return n;
}

static void tx_start_next(void)                         /* appelée avec les interruptions masquées ou depuis l'interruption */
{
    if (s_tx_tail != s_tx_head) {
        s_tx_byte = s_tx_ring[s_tx_tail];
        s_tx_tail = (uint16_t)((s_tx_tail + 1u) & (LINK_TX_SIZE - 1u));
        s_tx_busy = true;
        HAL_UART_Transmit_IT(&huart1, &s_tx_byte, 1);
    } else {
        s_tx_busy = false;
    }
}

uint16_t port_link_write(const uint8_t *buf, uint16_t len)
{
    uint16_t n = 0;
    while (n < len) {
        uint16_t next = (uint16_t)((s_tx_head + 1u) & (LINK_TX_SIZE - 1u));
        if (next == s_tx_tail) {
            break;                                      /* file pleine : on rend le nombre d'octets acceptés */
        }
        s_tx_ring[s_tx_head] = buf[n++];
        s_tx_head = next;
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!s_tx_busy) {
        tx_start_next();
    }
    __set_PRIMASK(primask);
    return n;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint16_t next = (uint16_t)((s_rx_head + 1u) & (LINK_RX_SIZE - 1u));
        if (next != s_rx_tail) {                        /* si l'anneau est plein, l'octet est perdu (le protocole se re-synchronise) */
            s_rx_ring[s_rx_head] = s_rx_byte;
            s_rx_head = next;
        }
        HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        tx_start_next();
    }
}

/** Après une erreur (débordement, bruit...), HAL arrête la réception : on la relance, sinon la liaison reste muette. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(huart, &s_rx_byte, 1);
    }
}
