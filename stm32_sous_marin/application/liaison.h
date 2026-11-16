/**
 * @file liaison.h
 * @brief Protocole de la liaison série STM32 <-> Raspberry Pi : trames avec CRC, commandes, télémétrie.
 *
 * Trame (octets) :   [0xA5] [LEN] [TYPE] [PAYLOAD x LEN] [CRC_HI] [CRC_LO]
 *   - LEN = nombre d'octets de PAYLOAD (0 à LINK_MAX_PAYLOAD) ;
 *   - CRC-16/CCITT-FALSE calculé sur LEN, TYPE et PAYLOAD, envoyé octet de poids fort d'abord ;
 *   - tous les nombres du payload sont en petit-boutiste (little-endian) ; les flottants sont des float IEEE-754 32 bits.
 *
 * Le récepteur re-synchronise tout seul : après un octet perdu ou altéré, il retrouve le début de la trame
 * suivante sans intervention. Aucune allocation dynamique, aucune récursion.
 *
 * Raspberry Pi -> STM32 (commandes)                       payload
 *   0x01 HEARTBEAT      "je suis là" (toutes les ~500 ms)   -
 *   0x02 SET_MODE       0 IDLE, 1 MANUAL, 2 DEPTH_HOLD      u8
 *   0x03 SET_DEPTH      consigne de profondeur (m)          f32
 *   0x04 SET_BALLAST    volumes manuels (mL)                f32 avant, f32 arrière
 *   0x05 CLEAR_FAULT    acquitter une urgence               -
 *   0x06 TARE_DEPTH     mesurer la pression de surface      -   (en IDLE seulement)
 *   0x07 EMERGENCY      arrêt d'urgence : remonter          -
 *   0x08 SET_PID        0 profondeur / 1 tangage            u8, f32 kp, f32 ki, f32 kd   (en IDLE seulement)
 * STM32 -> Raspberry Pi
 *   0x81 TELEMETRY      état complet toutes les 100 ms      voir telemetry_t
 *   0x82 ACK            réponse à une commande              u8 type de la commande, u8 résultat (ACK_*)
 */
#ifndef LIAISON_H
#define LIAISON_H

#include <stdint.h>
#include <stdbool.h>

#define LINK_SOF          0xA5u
#define LINK_MAX_PAYLOAD  64u
#define LINK_OVERHEAD     5u                                   /* SOF + LEN + TYPE + CRC (2) */
#define LINK_MAX_FRAME    (LINK_MAX_PAYLOAD + LINK_OVERHEAD)

enum {
    MSG_HEARTBEAT   = 0x01,
    MSG_SET_MODE    = 0x02,
    MSG_SET_DEPTH   = 0x03,
    MSG_SET_BALLAST = 0x04,
    MSG_CLEAR_FAULT = 0x05,
    MSG_TARE_DEPTH  = 0x06,
    MSG_EMERGENCY   = 0x07,
    MSG_SET_PID     = 0x08,
    MSG_TELEMETRY   = 0x81,
    MSG_ACK         = 0x82
};

enum { ACK_OK = 0, ACK_REJECTED = 1, ACK_INVALID = 2, ACK_UNKNOWN = 3 };

/** Appelée pour chaque trame dont le CRC est correct. */
typedef void (*link_frame_cb)(void *ctx, uint8_t type, const uint8_t *payload, uint8_t len);

typedef struct {
    uint8_t       buf[LINK_MAX_FRAME];
    uint8_t       n;
    link_frame_cb cb;
    void         *ctx;
    uint32_t      frames_ok;
    uint32_t      crc_errors;      /**< trames complètes rejetées pour CRC faux */
    uint32_t      dropped_bytes;   /**< octets écartés pendant la re-synchronisation */
} link_rx_t;

void link_rx_init(link_rx_t *rx, link_frame_cb cb, void *ctx);

/** Donne des octets reçus au récepteur ; appelle cb pour chaque trame valide trouvée. */
void link_rx_feed(link_rx_t *rx, const uint8_t *data, uint16_t len);

/** Construit une trame. @return longueur écrite, 0 si out_size est trop petit ou len > LINK_MAX_PAYLOAD. */
uint16_t link_encode(uint8_t *out, uint16_t out_size, uint8_t type, const uint8_t *payload, uint8_t len);

/* ---- lecture / écriture de nombres en petit-boutiste ---- */
void     link_put_u16(uint8_t *p, uint16_t v);
void     link_put_u32(uint8_t *p, uint32_t v);
void     link_put_f32(uint8_t *p, float v);
uint16_t link_get_u16(const uint8_t *p);
uint32_t link_get_u32(const uint8_t *p);
float    link_get_f32(const uint8_t *p);

/* ---- télémétrie ---- */
typedef struct {
    uint32_t t_ms;
    uint8_t  mode;               /**< sub_mode_t                                             */
    uint8_t  ballast_state;      /**< bits 0-3 : état ballast avant ; bits 4-7 : arrière     */
    uint16_t reasons;            /**< causes d'urgence mémorisées (FS_* | SAFE_*)            */
    uint16_t failsafe_mask;      /**< sécurités des capteurs actuellement actives            */
    uint8_t  healthy_mask;       /**< capteurs sains (bit = sensor_id_t)                     */
    float    depth_m, depth_setpoint_m;
    float    roll_deg, pitch_deg, yaw_deg;
    float    vbat_v, ibat_a;
    float    water_temp_c;
    float    vol_front_ml, vol_aft_ml;
} telemetry_t;

#define TELEMETRY_PAYLOAD_LEN  51u

/** @return longueur du payload (TELEMETRY_PAYLOAD_LEN) */
uint8_t telemetry_encode(uint8_t *payload, const telemetry_t *t);
/** @return false si la longueur ne correspond pas */
bool    telemetry_decode(const uint8_t *payload, uint8_t len, telemetry_t *t);

#endif
