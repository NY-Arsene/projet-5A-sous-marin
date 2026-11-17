/* Tests du protocole série : CRC, encodage, réception avec re-synchronisation, télémétrie. */
#include "mini_tests.h"
#include "crc16.h"
#include "liaison.h"
#include <string.h>

typedef struct { int count; uint8_t type[600]; uint8_t len[600]; uint8_t data[600][LINK_MAX_PAYLOAD]; } sink_t;
static sink_t sink;

static void on_frame(void *ctx, uint8_t type, const uint8_t *p, uint8_t len)
{
    sink_t *s = ctx;
    if (s->count < 600) {
        s->type[s->count] = type;
        s->len[s->count] = len;
        memcpy(s->data[s->count], p, len);
    }
    s->count++;
}

static link_rx_t rx;
static void rx_setup(void)
{
    memset(&sink, 0, sizeof sink);
    link_rx_init(&rx, on_frame, &sink);
}

static uint16_t make(uint8_t *out, uint8_t type, const uint8_t *pl, uint8_t len)
{
    uint16_t n = link_encode(out, LINK_MAX_FRAME, type, pl, len);
    MU_ASSERT(n == len + LINK_OVERHEAD);
    return n;
}

static void test_crc_known_vector(void)
{
    MU_ASSERT_INT(0x29B1, crc16_ccitt((const uint8_t *)"123456789", 9));   /* vecteur officiel CRC-16/CCITT-FALSE */
    MU_ASSERT_INT(0xFFFF, crc16_ccitt((const uint8_t *)"", 0));
}

static void test_encode_matches_independent_python_reference(void)
{
    uint8_t f[LINK_MAX_FRAME];
    /* valeurs calculées par un script Python distinct (crc_hqx) */
    uint16_t n = make(f, MSG_HEARTBEAT, NULL, 0);
    const uint8_t hb[] = { 0xA5, 0x00, 0x01, 0x0D, 0x2E };
    MU_ASSERT_INT(5, n); MU_ASSERT(memcmp(f, hb, 5) == 0);

    const uint8_t m2[] = { 0x02 };
    n = make(f, MSG_SET_MODE, m2, 1);
    const uint8_t ref_mode[] = { 0xA5, 0x01, 0x02, 0x02, 0xBD, 0x8C };
    MU_ASSERT_INT(6, n); MU_ASSERT(memcmp(f, ref_mode, 6) == 0);

    uint8_t pl[4]; link_put_f32(pl, 1.5f);
    n = make(f, MSG_SET_DEPTH, pl, 4);
    const uint8_t ref_depth[] = { 0xA5, 0x04, 0x03, 0x00, 0x00, 0xC0, 0x3F, 0x37, 0x8B };
    MU_ASSERT_INT(9, n); MU_ASSERT(memcmp(f, ref_depth, 9) == 0);
}

static void test_encode_rejects_bad_arguments(void)
{
    uint8_t f[LINK_MAX_FRAME], pl[LINK_MAX_PAYLOAD + 1] = {0};
    MU_ASSERT_INT(0, link_encode(f, 4, MSG_HEARTBEAT, NULL, 0));                  /* tampon trop petit */
    MU_ASSERT_INT(0, link_encode(f, sizeof f, MSG_SET_DEPTH, NULL, 4));           /* payload manquant */
    MU_ASSERT_INT(0, link_encode(f, sizeof f, MSG_SET_DEPTH, pl, LINK_MAX_PAYLOAD + 1));
    MU_ASSERT_INT(LINK_MAX_FRAME, link_encode(f, sizeof f, MSG_SET_DEPTH, pl, LINK_MAX_PAYLOAD));   /* taille maximale OK */
    MU_ASSERT_INT(0, link_encode(f, LINK_MAX_FRAME - 1, MSG_SET_DEPTH, pl, LINK_MAX_PAYLOAD));   /* un octet de moins : refus */
}

static void test_single_frame_roundtrip(void)
{
    rx_setup();
    uint8_t f[LINK_MAX_FRAME], pl[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    uint16_t n = make(f, MSG_SET_BALLAST, pl, 8);
    link_rx_feed(&rx, f, n);
    MU_ASSERT_INT(1, sink.count);
    MU_ASSERT_INT(MSG_SET_BALLAST, sink.type[0]);
    MU_ASSERT_INT(8, sink.len[0]);
    MU_ASSERT(memcmp(sink.data[0], pl, 8) == 0);
    MU_ASSERT_INT(1, rx.frames_ok);
    MU_ASSERT_INT(0, rx.n);                       /* tampon vidé */
}

static void test_frame_delivered_byte_by_byte(void)
{
    rx_setup();
    uint8_t f[LINK_MAX_FRAME], pl[4]; link_put_f32(pl, 2.25f);
    uint16_t n = make(f, MSG_SET_DEPTH, pl, 4);
    for (uint16_t i = 0; i < n; i++) {
        MU_ASSERT_INT(0, sink.count);             /* pas de trame tant qu'elle n'est pas complète */
        link_rx_feed(&rx, &f[i], 1);
    }
    MU_ASSERT_INT(1, sink.count);
    MU_ASSERT_NEAR(2.25, link_get_f32(sink.data[0]), 1e-9);
}

static void test_back_to_back_frames_in_one_chunk(void)
{
    rx_setup();
    uint8_t buf[200], tmp[LINK_MAX_FRAME]; uint16_t pos = 0;
    uint8_t mode = 1;
    pos += make(tmp, MSG_HEARTBEAT, NULL, 0);          memcpy(buf, tmp, pos);
    uint16_t n = make(tmp, MSG_SET_MODE, &mode, 1);    memcpy(buf + pos, tmp, n); pos += n;
    n = make(tmp, MSG_CLEAR_FAULT, NULL, 0);           memcpy(buf + pos, tmp, n); pos += n;
    link_rx_feed(&rx, buf, pos);
    MU_ASSERT_INT(3, sink.count);
    MU_ASSERT_INT(MSG_HEARTBEAT, sink.type[0]);
    MU_ASSERT_INT(MSG_SET_MODE, sink.type[1]);
    MU_ASSERT_INT(MSG_CLEAR_FAULT, sink.type[2]);
}

static void test_garbage_before_frame_is_skipped(void)
{
    rx_setup();
    const uint8_t junk[] = { 0x00, 0xFF, 0x13, 0x37, 0x80 };
    uint8_t f[LINK_MAX_FRAME];
    uint16_t n = make(f, MSG_HEARTBEAT, NULL, 0);
    link_rx_feed(&rx, junk, sizeof junk);
    link_rx_feed(&rx, f, n);
    MU_ASSERT_INT(1, sink.count);
    MU_ASSERT_INT(5, rx.dropped_bytes);
}

static void test_corrupted_frame_is_dropped_and_next_one_accepted(void)
{
    rx_setup();
    uint8_t a[LINK_MAX_FRAME], b[LINK_MAX_FRAME], pl[4]; link_put_f32(pl, 1.0f);
    uint16_t na = make(a, MSG_SET_DEPTH, pl, 4);
    uint16_t nb = make(b, MSG_HEARTBEAT, NULL, 0);
    a[4] ^= 0x01;                                    /* un bit altéré dans le payload */
    link_rx_feed(&rx, a, na);
    link_rx_feed(&rx, b, nb);
    MU_ASSERT_INT(1, sink.count);                    /* seule la trame saine passe */
    MU_ASSERT_INT(MSG_HEARTBEAT, sink.type[0]);
    MU_ASSERT(rx.crc_errors >= 1);
}

static void test_every_single_bit_flip_is_detected(void)
{
    uint8_t f[LINK_MAX_FRAME], pl[6] = { 9, 8, 7, 6, 5, 4 };
    uint16_t n = make(f, MSG_SET_BALLAST, pl, 6);
    for (uint16_t byte = 0; byte < n; byte++) {
        for (int bit = 0; bit < 8; bit++) {
            rx_setup();
            uint8_t g[LINK_MAX_FRAME];
            memcpy(g, f, n);
            g[byte] ^= (uint8_t)(1u << bit);
            link_rx_feed(&rx, g, n);
            /* aucune trame altérée ne doit être acceptée (sauf si l'altération laisse une trame cohérente : impossible ici) */
            MU_ASSERT_INT(0, sink.count);
        }
    }
}

static void test_impossible_length_resyncs_immediately(void)
{
    rx_setup();
    uint8_t f[LINK_MAX_FRAME];
    uint16_t n = make(f, MSG_HEARTBEAT, NULL, 0);
    const uint8_t fake_start[] = { 0xA5, 0xFF };     /* "longueur 255" : impossible */
    link_rx_feed(&rx, fake_start, 2);
    link_rx_feed(&rx, f, n);
    MU_ASSERT_INT(1, sink.count);                    /* on n'attend pas 260 octets avant de revenir sur la bonne voie */

    /* Longueurs juste au-dessus du maximum (65) et intermédiaires : le récepteur ne doit jamais attendre ces octets. */
    const uint8_t bad_len[] = { LINK_MAX_PAYLOAD + 1, 100, 200, 251 };
    for (unsigned i = 0; i < sizeof bad_len; i++) {
        rx_setup();
        const uint8_t start[] = { 0xA5, bad_len[i] };
        link_rx_feed(&rx, start, 2);
        link_rx_feed(&rx, f, n);
        MU_ASSERT_INT(1, sink.count);
    }
}

static void test_valid_frame_hidden_inside_a_bad_one_is_found(void)
{
    rx_setup();
    uint8_t inner[LINK_MAX_FRAME];
    uint16_t ni = make(inner, MSG_CLEAR_FAULT, NULL, 0);
    /* un faux début de trame (A5, longueur 8) dont le "payload" contient une vraie trame complète */
    uint8_t buf[64]; uint16_t pos = 0;
    buf[pos++] = 0xA5; buf[pos++] = 8; buf[pos++] = MSG_SET_BALLAST;
    memcpy(buf + pos, inner, ni); pos += ni;
    while (pos < 3 + 8 + 2) buf[pos++] = 0x55;       /* complète la fausse trame : son CRC est faux */
    link_rx_feed(&rx, buf, pos);
    MU_ASSERT_INT(1, sink.count);
    MU_ASSERT_INT(MSG_CLEAR_FAULT, sink.type[0]);
}

static void test_truncated_frame_followed_by_real_frame(void)
{
    rx_setup();
    uint8_t a[LINK_MAX_FRAME], b[LINK_MAX_FRAME], pl[8] = {0};
    uint16_t na = make(a, MSG_SET_BALLAST, pl, 8);
    uint16_t nb = make(b, MSG_HEARTBEAT, NULL, 0);
    link_rx_feed(&rx, a, na - 4);                    /* la première trame est coupée (câble débranché un instant) */
    link_rx_feed(&rx, b, nb);
    link_rx_feed(&rx, b, nb);                        /* la suivante, elle, arrive complète */
    MU_ASSERT(sink.count >= 1);
    MU_ASSERT_INT(MSG_HEARTBEAT, sink.type[sink.count - 1]);
}

static void test_noise_and_frames_fuzz_deterministic(void)
{
    rx_setup();
    uint32_t seed = 12345;
#define RND() (seed = seed * 1664525u + 1013904223u, (uint8_t)(seed >> 24))
    int sent = 0;
    for (int k = 0; k < 500; k++) {
        int noise = RND() % 30;
        for (int i = 0; i < noise; i++) {
            link_rx_feed(&rx, (uint8_t[]){ RND() }, 1);
        }
        uint8_t pl[LINK_MAX_PAYLOAD], f[LINK_MAX_FRAME];
        uint8_t len = (uint8_t)(RND() % 20);
        for (uint8_t i = 0; i < len; i++) pl[i] = RND();
        uint16_t n = link_encode(f, sizeof f, (uint8_t)(1 + RND() % 8), pl, len);
        link_rx_feed(&rx, f, n);
        sent++;
    }
    uint8_t flush[LINK_MAX_FRAME] = {0};
    link_rx_feed(&rx, flush, sizeof flush);          /* octets neutres : laissent le récepteur finir toute attente */
    MU_ASSERT_INT(sent, sink.count);                 /* toutes les vraies trames retrouvées, aucune fausse */
    MU_ASSERT_INT(sink.count, rx.frames_ok);
}

static void test_pure_noise_never_crashes_or_invents_frames(void)
{
    rx_setup();
    uint32_t seed = 99;
    for (int i = 0; i < 300000; i++) {
        uint8_t b = RND();
        if ((i % 7) == 0) b = 0xA5;                  /* beaucoup de faux débuts de trame */
        link_rx_feed(&rx, &b, 1);
    }
    MU_ASSERT(rx.n <= LINK_MAX_FRAME);
    MU_ASSERT(sink.count <= 2);                      /* un CRC 16 bits laisse passer ~1 trame sur 65536 : très peu */
}

static void test_scalar_encoding_is_little_endian_and_lossless(void)
{
    uint8_t b[4];
    link_put_u32(b, 0x01020304u);
    MU_ASSERT_INT(0x04, b[0]); MU_ASSERT_INT(0x01, b[3]);
    MU_ASSERT_INT(0x01020304u, link_get_u32(b));
    link_put_u16(b, 0xBEEF);
    MU_ASSERT_INT(0xEF, b[0]); MU_ASSERT_INT(0xBE, b[1]);
    MU_ASSERT_INT(0xBEEF, link_get_u16(b));
    link_put_f32(b, 1.5f);
    MU_ASSERT_INT(0x00, b[0]); MU_ASSERT_INT(0xC0, b[2]); MU_ASSERT_INT(0x3F, b[3]);   /* 1,5 = 0x3FC00000 */
    MU_ASSERT_NEAR(1.5, link_get_f32(b), 0);
    link_put_f32(b, -0.0f);
    MU_ASSERT(signbit(link_get_f32(b)));
    link_put_f32(b, INFINITY);
    MU_ASSERT(isinf(link_get_f32(b)));
    link_put_f32(b, NAN);
    MU_ASSERT(isnan(link_get_f32(b)));
}

static void test_telemetry_roundtrip_and_layout(void)
{
    telemetry_t t = { .t_ms = 0x01020304u, .mode = 3, .ballast_state = 0x21, .reasons = 0x0405, .failsafe_mask = 0x0021,
                      .healthy_mask = 0x1F, .depth_m = 1.5f, .depth_setpoint_m = 2.0f, .roll_deg = -3.5f,
                      .pitch_deg = 4.25f, .yaw_deg = 271.5f, .vbat_v = 12.34f, .ibat_a = 0.75f, .water_temp_c = 18.5f,
                      .vol_front_ml = 55.5f, .vol_aft_ml = 44.25f };
    uint8_t p[LINK_MAX_PAYLOAD];
    MU_ASSERT_INT(TELEMETRY_PAYLOAD_LEN, telemetry_encode(p, &t));
    MU_ASSERT_INT(0x04, p[0]); MU_ASSERT_INT(0x01, p[3]);          /* t_ms en petit-boutiste */
    MU_ASSERT_INT(3, p[4]);
    MU_ASSERT_INT(0x00, p[11]); MU_ASSERT_INT(0xC0, p[13]); MU_ASSERT_INT(0x3F, p[14]);   /* depth = 1,5 à l'offset 11 */
    telemetry_t u;
    MU_ASSERT(telemetry_decode(p, TELEMETRY_PAYLOAD_LEN, &u));
    MU_ASSERT(
        u.t_ms == t.t_ms && u.mode == t.mode && u.ballast_state == t.ballast_state && u.reasons == t.reasons &&
        u.failsafe_mask == t.failsafe_mask && u.healthy_mask == t.healthy_mask && u.depth_m == t.depth_m &&
        u.depth_setpoint_m == t.depth_setpoint_m && u.roll_deg == t.roll_deg && u.pitch_deg == t.pitch_deg &&
        u.yaw_deg == t.yaw_deg && u.vbat_v == t.vbat_v && u.ibat_a == t.ibat_a && u.water_temp_c == t.water_temp_c &&
        u.vol_front_ml == t.vol_front_ml && u.vol_aft_ml == t.vol_aft_ml);
    MU_ASSERT(!telemetry_decode(p, TELEMETRY_PAYLOAD_LEN - 1, &u));
    MU_ASSERT(!telemetry_decode(p, TELEMETRY_PAYLOAD_LEN + 1, &u));
}

static void test_telemetry_fits_in_one_frame_and_crosses_the_link(void)
{
    rx_setup();
    telemetry_t t = { .t_ms = 777, .mode = 1, .depth_m = 0.42f };
    uint8_t p[LINK_MAX_PAYLOAD], f[LINK_MAX_FRAME];
    uint8_t len = telemetry_encode(p, &t);
    uint16_t n = link_encode(f, sizeof f, MSG_TELEMETRY, p, len);
    MU_ASSERT(n > 0 && n <= LINK_MAX_FRAME);
    link_rx_feed(&rx, f, n);
    MU_ASSERT_INT(1, sink.count);
    telemetry_t u;
    MU_ASSERT(telemetry_decode(sink.data[0], sink.len[0], &u));
    MU_ASSERT_INT(777, u.t_ms);
    MU_ASSERT_NEAR(0.42, u.depth_m, 1e-6);
}

int main(void)
{
    MU_RUN(test_crc_known_vector);
    MU_RUN(test_encode_matches_independent_python_reference);
    MU_RUN(test_encode_rejects_bad_arguments);
    MU_RUN(test_single_frame_roundtrip);
    MU_RUN(test_frame_delivered_byte_by_byte);
    MU_RUN(test_back_to_back_frames_in_one_chunk);
    MU_RUN(test_garbage_before_frame_is_skipped);
    MU_RUN(test_corrupted_frame_is_dropped_and_next_one_accepted);
    MU_RUN(test_every_single_bit_flip_is_detected);
    MU_RUN(test_impossible_length_resyncs_immediately);
    MU_RUN(test_valid_frame_hidden_inside_a_bad_one_is_found);
    MU_RUN(test_truncated_frame_followed_by_real_frame);
    MU_RUN(test_noise_and_frames_fuzz_deterministic);
    MU_RUN(test_pure_noise_never_crashes_or_invents_frames);
    MU_RUN(test_scalar_encoding_is_little_endian_and_lossless);
    MU_RUN(test_telemetry_roundtrip_and_layout);
    MU_RUN(test_telemetry_fits_in_one_frame_and_crosses_the_link);
    return MU_SUMMARY();
}
