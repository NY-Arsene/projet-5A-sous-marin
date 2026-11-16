#include "detecteur_fuite.h"

sens_status_t leak_init(leak_t *l, port_adc_t ch, uint16_t thr_on, uint16_t thr_off,
                        uint8_t confirm_n, bool active_high)
{
    if (l == 0 || confirm_n == 0) {
        return SENS_ERR_PARAM;
    }
    if (active_high ? (thr_on <= thr_off) : (thr_on >= thr_off)) {
        return SENS_ERR_PARAM;
    }
    l->ch = ch;
    l->thr_on = thr_on;
    l->thr_off = thr_off;
    l->confirm_n = confirm_n;
    l->cnt = 0;
    l->active_high = active_high;
    l->wet_now = false;
    l->leak = false;
    l->last_raw = 0;
    return SENS_OK;
}

bool leak_process(leak_t *l, uint16_t adc_raw)
{
    l->last_raw = adc_raw;
    if (l->active_high) {
        if (adc_raw >= l->thr_on)       l->wet_now = true;
        else if (adc_raw <= l->thr_off) l->wet_now = false;
    } else {
        if (adc_raw <= l->thr_on)       l->wet_now = true;
        else if (adc_raw >= l->thr_off) l->wet_now = false;
    }
    if (l->wet_now) {
        if (l->cnt < 255) l->cnt++;
    } else {
        l->cnt = 0;
    }
    if (l->cnt >= l->confirm_n) {
        l->leak = true;
    }
    return l->leak;
}

bool leak_poll(leak_t *l)
{
    return leak_process(l, port_adc_read(l->ch));
}

void leak_clear(leak_t *l)
{
    l->leak = false;
    l->cnt = 0;
    l->wet_now = false;
}
