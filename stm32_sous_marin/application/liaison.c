#include "liaison.h"
#include "crc16.h"
#include <string.h>

void link_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

void link_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)(v >> 24);
}

void link_put_f32(uint8_t *p, float v)
{
    uint32_t u;
    memcpy(&u, &v, sizeof u);                  /* pas de conversion de pointeur : conforme au C strict */
    link_put_u32(p, u);
}

uint16_t link_get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

uint32_t link_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

float link_get_f32(const uint8_t *p)
{
    uint32_t u = link_get_u32(p);
    float v;
    memcpy(&v, &u, sizeof v);
    return v;
}

uint16_t link_encode(uint8_t *out, uint16_t out_size, uint8_t type, const uint8_t *payload, uint8_t len)
{
    if (len > LINK_MAX_PAYLOAD || out_size < (uint16_t)(len + LINK_OVERHEAD) || (len > 0 && payload == NULL)) {
        return 0;
    }
    out[0] = (uint8_t)LINK_SOF;
    out[1] = len;
    out[2] = type;
    if (len > 0) {
        memcpy(&out[3], payload, len);
    }
    uint16_t crc = crc16_ccitt(&out[1], (size_t)len + 2u);
    out[3 + len] = (uint8_t)(crc >> 8);
    out[4 + len] = (uint8_t)(crc & 0xFFu);
    return (uint16_t)(len + LINK_OVERHEAD);
}

void link_rx_init(link_rx_t *rx, link_frame_cb cb, void *ctx)
{
    memset(rx, 0, sizeof *rx);
    rx->cb = cb;
    rx->ctx = ctx;
}

static void drop_first(link_rx_t *rx, uint8_t count)
{
    memmove(rx->buf, rx->buf + count, (size_t)(rx->n - count));
    rx->n = (uint8_t)(rx->n - count);
    rx->dropped_bytes += count;
}

/** Cherche des trames complètes au début du tampon ; re-synchronise sur le prochain 0xA5 en cas d'erreur. */
static void parse(link_rx_t *rx)
{
    while (rx->n > 0) {
        if (rx->buf[0] != LINK_SOF) {
            drop_first(rx, 1);
            continue;
        }
        if (rx->n < 2) {
            return;
        }
        uint8_t len = rx->buf[1];
        if (len > LINK_MAX_PAYLOAD) {          /* longueur impossible : ce 0xA5 n'était pas un début de trame */
            drop_first(rx, 1);
            continue;
        }
        uint8_t total = (uint8_t)(len + LINK_OVERHEAD);
        if (rx->n < total) {
            return;                            /* trame incomplète : on attend la suite */
        }
        uint16_t expect = crc16_ccitt(&rx->buf[1], (size_t)len + 2u);
        uint16_t got = (uint16_t)(((uint16_t)rx->buf[3 + len] << 8) | rx->buf[4 + len]);
        if (expect != got) {
            rx->crc_errors++;
            drop_first(rx, 1);                 /* on repart juste après ce 0xA5 : une vraie trame peut être cachée dedans */
            continue;
        }
        uint8_t type = rx->buf[2];
        uint8_t payload[LINK_MAX_PAYLOAD];
        memcpy(payload, &rx->buf[3], len);
        memmove(rx->buf, rx->buf + total, (size_t)(rx->n - total));   /* on libère le tampon AVANT le rappel */
        rx->n = (uint8_t)(rx->n - total);
        rx->frames_ok++;
        if (rx->cb != NULL) {
            rx->cb(rx->ctx, type, payload, len);
        }
    }
}

void link_rx_feed(link_rx_t *rx, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (rx->n >= LINK_MAX_FRAME) {         /* ne devrait pas arriver (parse libère le tampon), mais on ne déborde jamais */
            drop_first(rx, 1);
        }
        rx->buf[rx->n++] = data[i];
        parse(rx);
    }
}

uint8_t telemetry_encode(uint8_t *p, const telemetry_t *t)
{
    link_put_u32(p + 0, t->t_ms);
    p[4] = t->mode;
    p[5] = t->ballast_state;
    link_put_u16(p + 6, t->reasons);
    link_put_u16(p + 8, t->failsafe_mask);
    p[10] = t->healthy_mask;
    const float f[10] = { t->depth_m, t->depth_setpoint_m, t->roll_deg, t->pitch_deg, t->yaw_deg,
                          t->vbat_v, t->ibat_a, t->water_temp_c, t->vol_front_ml, t->vol_aft_ml };
    for (int i = 0; i < 10; i++) {
        link_put_f32(p + 11 + 4 * i, f[i]);
    }
    return (uint8_t)TELEMETRY_PAYLOAD_LEN;
}

bool telemetry_decode(const uint8_t *p, uint8_t len, telemetry_t *t)
{
    if (len != TELEMETRY_PAYLOAD_LEN) {
        return false;
    }
    t->t_ms = link_get_u32(p + 0);
    t->mode = p[4];
    t->ballast_state = p[5];
    t->reasons = link_get_u16(p + 6);
    t->failsafe_mask = link_get_u16(p + 8);
    t->healthy_mask = p[10];
    float *dst[10] = { &t->depth_m, &t->depth_setpoint_m, &t->roll_deg, &t->pitch_deg, &t->yaw_deg,
                       &t->vbat_v, &t->ibat_a, &t->water_temp_c, &t->vol_front_ml, &t->vol_aft_ml };
    for (int i = 0; i < 10; i++) {
        *dst[i] = link_get_f32(p + 11 + 4 * i);
    }
    return true;
}
