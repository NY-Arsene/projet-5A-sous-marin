#include "batterie.h"
#include <math.h>

typedef struct { float v; float soc; } soc_pt_t;

static const soc_pt_t k_table[] = {
    {3.00f,   0.0f}, {3.30f,   3.0f}, {3.50f,  10.0f}, {3.60f,  20.0f}, {3.70f,  35.0f},
    {3.80f,  50.0f}, {3.90f,  62.0f}, {4.00f,  75.0f}, {4.10f,  88.0f}, {4.20f, 100.0f},
};
#define K_TABLE_N (sizeof k_table / sizeof k_table[0])

float battery_soc_from_cell_v(float v)
{
    if (!isfinite(v) || v <= k_table[0].v) {
        return 0.0f;
    }
    if (v >= k_table[K_TABLE_N - 1].v) {
        return 100.0f;
    }
    for (unsigned i = 1; i < K_TABLE_N; i++) {
        if (v <= k_table[i].v) {
            float t = (v - k_table[i - 1].v) / (k_table[i].v - k_table[i - 1].v);
            return k_table[i - 1].soc + t * (k_table[i].soc - k_table[i - 1].soc);
        }
    }
    return 100.0f;
}

void battery_init(battery_t *b, uint8_t cells, float v_warn_cell, float v_crit_cell)
{
    b->cells = (cells == 0) ? 1 : cells;
    b->v_warn_cell = v_warn_cell;
    b->v_crit_cell = v_crit_cell;
    b->hyst_cell = 0.10f;
    b->confirm_n = 5;
    b->cnt = 0;
    ema_init(&b->ema_v, 0.2f);
    b->v_filt = 0.0f;
    b->soc_pct = 0.0f;
    b->level = BATT_OK;
}

batt_level_t battery_update(battery_t *b, float v_bus)
{
    if (!isfinite(v_bus)) {
        return b->level;
    }
    b->v_filt = ema_update(&b->ema_v, v_bus);
    float vc = b->v_filt / (float)b->cells;
    b->soc_pct = battery_soc_from_cell_v(vc);

    batt_level_t cand = b->level;
    switch (b->level) {
    case BATT_OK:
        if (vc < b->v_crit_cell)      cand = BATT_CRITICAL;
        else if (vc < b->v_warn_cell) cand = BATT_WARNING;
        break;
    case BATT_WARNING:
        if (vc < b->v_crit_cell)                    cand = BATT_CRITICAL;
        else if (vc > b->v_warn_cell + b->hyst_cell) cand = BATT_OK;
        break;
    case BATT_CRITICAL:
        if (vc > b->v_crit_cell + b->hyst_cell) {
            cand = (vc > b->v_warn_cell + b->hyst_cell) ? BATT_OK : BATT_WARNING;
        }
        break;
    }

    if (cand == b->level) {
        b->cnt = 0;
    } else if (++b->cnt >= b->confirm_n) {
        b->level = cand;
        b->cnt = 0;
    }
    return b->level;
}
