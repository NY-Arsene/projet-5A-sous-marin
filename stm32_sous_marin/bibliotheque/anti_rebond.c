#include "anti_rebond.h"

void debounce_init(debounce_t *d, bool initial, uint16_t debounce_ms)
{
    d->stable = initial;
    d->last_raw = initial;
    d->t_change = 0;
    d->debounce_ms = debounce_ms;
    d->edge = 0;
}

bool debounce_update(debounce_t *d, bool raw, uint32_t now_ms)
{
    d->edge = 0;
    if (raw != d->last_raw) {
        d->last_raw = raw;
        d->t_change = now_ms;
    } else if (raw != d->stable && (uint32_t)(now_ms - d->t_change) >= d->debounce_ms) {
        d->stable = raw;
        d->edge = raw ? 1 : -1;
    }
    return d->stable;
}
