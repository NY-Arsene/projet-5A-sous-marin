/**
 * @file detecteur_fuite.h
 * @brief Détecteur de fuite à électrodes lu sur ADC : hystérésis + confirmation + mémorisation.
 *
 * Une fuite détectée reste MÉMORISÉE (latch) jusqu'à leak_clear() : on ne doit jamais
 * "oublier" qu'il y a eu de l'eau dans la coque.
 */
#ifndef DETECTEUR_FUITE_H
#define DETECTEUR_FUITE_H

#include <stdbool.h>
#include <stdint.h>
#include "materiel.h"
#include "types_capteurs.h"

typedef struct {
    port_adc_t ch;
    uint16_t   thr_on;       /**< seuil de détection d'eau (brut ADC)           */
    uint16_t   thr_off;      /**< seuil de retour à sec (hystérésis)            */
    uint8_t    confirm_n;    /**< lectures "mouillé" consécutives avant alarme   */
    uint8_t    cnt;
    bool       active_high;  /**< true : mouillé => tension ADC plus haute       */
    bool       wet_now;
    bool       leak;         /**< alarme mémorisée                               */
    uint16_t   last_raw;
} leak_t;

/** active_high : thr_on > thr_off. Sinon (actif bas) : thr_on < thr_off. Sinon SENS_ERR_PARAM. */
sens_status_t leak_init(leak_t *l, port_adc_t ch, uint16_t thr_on, uint16_t thr_off,
                        uint8_t confirm_n, bool active_high);
/** Traitement pur d'une valeur brute (testable). Renvoie l'état d'alarme mémorisé. */
bool leak_process(leak_t *l, uint16_t adc_raw);
/** Lit l'ADC puis traite. */
bool leak_poll(leak_t *l);
void leak_clear(leak_t *l);

#endif
