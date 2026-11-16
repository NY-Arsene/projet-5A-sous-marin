/** @file anti_rebond.h  Anti-rebond d'une entrée logique (fins de course) avec détection de front. */
#ifndef ANTI_REBOND_H
#define ANTI_REBOND_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool     stable;       /**< état filtré                                 */
    bool     last_raw;
    uint32_t t_change;     /**< date du dernier changement de la brute      */
    uint16_t debounce_ms;
    int8_t   edge;         /**< +1 front montant, -1 descendant, 0 : sur le dernier appel */
} debounce_t;

void debounce_init(debounce_t *d, bool initial, uint16_t debounce_ms);
/** Renvoie l'état stable. d->edge indique si l'état stable vient de changer. */
bool debounce_update(debounce_t *d, bool raw, uint32_t now_ms);

#endif
