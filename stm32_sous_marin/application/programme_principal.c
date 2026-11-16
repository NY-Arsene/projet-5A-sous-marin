#include "programme_principal.h"
#include <math.h>
#include <string.h>

app_t g_app;

#define MAX_LINK_READS_PER_CYCLE 8u
#define MAX_GAIN                 1000.0f
#define TARE_MAX_DEPTH_M         0.5f
#define NO_SETPOINT              (-1.0f)       /* valeur de depth_setpoint_m en télémétrie quand on ne régule pas */

/* ------------------------------------------------------------------ entrées de la machine d'états */
static safety_inputs_t make_inputs(const app_t *a, uint32_t now_ms)
{
    const sensor_data_t *d = sensors_data(&a->sm);
    ballast_state_t sf = ballast_state(&a->ballast[BALLAST_FRONT]);
    ballast_state_t sa = ballast_state(&a->ballast[BALLAST_AFT]);
    safety_inputs_t in;
    in.failsafe_mask   = d->failsafe_mask;
    in.depth_ref_valid = d->depth_ref_valid && !depth_is_taring(&a->sm.depth);
    in.depth_m         = (d->healthy_mask & (1u << SENSOR_BARO)) ? d->depth_m : NAN;
    in.ballasts_ready  = (sf == BALLAST_READY) && (sa == BALLAST_READY);
    in.ballast_fault   = (sf == BALLAST_FAULT) || (sa == BALLAST_FAULT);
    in.homing_done     = sf != BALLAST_HOMING && sf != BALLAST_UNHOMED && sa != BALLAST_HOMING && sa != BALLAST_UNHOMED;
    in.now_ms          = now_ms;
    return in;
}

/* ------------------------------------------------------------------ envoi de messages */
static void send_frame(app_t *a, uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[LINK_MAX_FRAME];
    uint16_t n = link_encode(frame, sizeof frame, type, payload, len);
    if (n == 0 || port_link_write(frame, n) != n) {
        a->tx_overflow++;              /* file d'émission pleine : le récepteur se re-synchronisera sur la trame suivante */
    }
}

static void send_ack(app_t *a, uint8_t cmd_type, uint8_t result)
{
    uint8_t p[2] = { cmd_type, result };
    send_frame(a, MSG_ACK, p, 2);
}

static void send_telemetry(app_t *a, uint32_t now_ms)
{
    const sensor_data_t *d = sensors_data(&a->sm);
    sub_mode_t mode = safety_mode(&a->safety);
    telemetry_t t;
    memset(&t, 0, sizeof t);
    t.t_ms = now_ms;
    t.mode = (uint8_t)mode;
    t.ballast_state = (uint8_t)((unsigned)ballast_state(&a->ballast[BALLAST_FRONT]) |
                                ((unsigned)ballast_state(&a->ballast[BALLAST_AFT]) << 4));
    t.reasons = (uint16_t)(safety_reasons(&a->safety) & 0xFFFFu);
    t.failsafe_mask = (uint16_t)(d->failsafe_mask & 0xFFFFu);
    t.healthy_mask = (uint8_t)(d->healthy_mask & 0xFFu);
    t.depth_m = d->depth_m;
    t.depth_setpoint_m = (mode == MODE_DEPTH_HOLD) ? a->dc.setpoint_eff_m : NO_SETPOINT;
    t.roll_deg = d->roll_deg;
    t.pitch_deg = d->pitch_deg;
    t.yaw_deg = d->yaw_deg;
    t.vbat_v = d->vbat_v;
    t.ibat_a = d->ibat_a;
    t.water_temp_c = d->water_temp_c;
    t.vol_front_ml = ballast_volume_ml(&a->ballast[BALLAST_FRONT]);
    t.vol_aft_ml = ballast_volume_ml(&a->ballast[BALLAST_AFT]);
    uint8_t payload[LINK_MAX_PAYLOAD];
    uint8_t len = telemetry_encode(payload, &t);
    send_frame(a, MSG_TELEMETRY, payload, len);
    a->tlm_sent++;
}

/* ------------------------------------------------------------------ commandes reçues */
static uint8_t ack_from_request(safe_req_t r)
{
    return (r == SAFE_REQ_OK) ? ACK_OK : (r == SAFE_REQ_REJECTED) ? ACK_REJECTED : ACK_INVALID;
}

static uint8_t handle_command(app_t *a, uint8_t type, const uint8_t *p, uint8_t len, uint32_t now_ms)
{
    safety_inputs_t in = make_inputs(a, now_ms);
    sub_mode_t mode = safety_mode(&a->safety);

    switch (type) {
    case MSG_SET_MODE:
        if (len != 1) return ACK_INVALID;
        return ack_from_request(safety_request_mode(&a->safety, p[0], &in));

    case MSG_SET_DEPTH:
        if (len != 4) return ACK_INVALID;
        if (mode != MODE_DEPTH_HOLD) return ACK_REJECTED;
        return depth_ctrl_set_depth(&a->dc, link_get_f32(p)) ? ACK_OK : ACK_INVALID;

    case MSG_SET_BALLAST: {
        if (len != 8) return ACK_INVALID;
        float f = link_get_f32(p), r = link_get_f32(p + 4);
        float max = a->ballast[BALLAST_FRONT].cfg.max_volume_ml;
        if (!isfinite(f) || !isfinite(r) || f < 0.0f || r < 0.0f || f > max || r > max) return ACK_INVALID;
        if (mode != MODE_MANUAL) return ACK_REJECTED;
        a->manual_ml[BALLAST_FRONT] = f;
        a->manual_ml[BALLAST_AFT] = r;
        return ACK_OK;
    }

    case MSG_CLEAR_FAULT:
        a->clear_pending = true;
        return ACK_OK;

    case MSG_TARE_DEPTH:
        if (mode != MODE_IDLE || !(fabsf(in.depth_m) < TARE_MAX_DEPTH_M)) return ACK_REJECTED;   /* NaN : refusé aussi */
        a->tare_pending = true;
        return ACK_OK;

    case MSG_EMERGENCY:
        safety_emergency_stop(&a->safety);
        return ACK_OK;

    case MSG_SET_PID: {
        if (len != 13) return ACK_INVALID;
        float kp = link_get_f32(p + 1), ki = link_get_f32(p + 5), kd = link_get_f32(p + 9);
        if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) || kp > MAX_GAIN || ki > MAX_GAIN || kd > MAX_GAIN) return ACK_INVALID;
        if (mode != MODE_IDLE) return ACK_REJECTED;
        return depth_ctrl_set_gains(&a->dc, p[0], kp, ki, kd) ? ACK_OK : ACK_INVALID;
    }

    default:
        return ACK_UNKNOWN;
    }
}

static void on_frame(void *ctx, uint8_t type, const uint8_t *payload, uint8_t len)
{
    app_t *a = ctx;
    uint32_t now = port_millis();
    a->cmd_frames++;
    safety_link_activity(&a->safety, now);          /* toute trame valide prouve que la liaison vit */
    if (type == MSG_HEARTBEAT) {
        return;                                     /* pas d'acquittement : on n'encombre pas la liaison */
    }
    send_ack(a, type, handle_command(a, type, payload, len, now));
}

/* ------------------------------------------------------------------ initialisation */
void app_init(app_t *a, uint32_t now_ms)
{
    memset(a, 0, sizeof *a);
    port_init();
    sensors_default_config(&a->scfg);
    a->sensors_init_failed = sensors_init(&a->sm, &a->scfg, now_ms);
    (void)sensors_calibrate_gyro(&a->sm, 200);      /* échec toléré : le biais reste nul et FS_IMU_LOST le signalera si l'IMU est absente */
    sensors_tare_depth(&a->sm, 100);

    ballast_cfg_t bc;
    ballast_default_config(&bc);
    bool ok = ballast_init(&a->ballast[BALLAST_FRONT], PORT_MOTOR_FWD, PORT_PIN_LIMIT_FWD_TOP, PORT_PIN_LIMIT_FWD_BOTTOM, &bc) &&
              ballast_init(&a->ballast[BALLAST_AFT],   PORT_MOTOR_AFT, PORT_PIN_LIMIT_AFT_TOP, PORT_PIN_LIMIT_AFT_BOTTOM, &bc);

    depth_ctrl_cfg_t dcfg;
    depth_ctrl_default_config(&dcfg);
    dcfg.max_volume_ml = bc.max_volume_ml;
    ok = ok && depth_ctrl_init(&a->dc, &dcfg);
    a->init_ok = ok;

    safety_cfg_t scfg;
    safety_default_config(&scfg);
    safety_init(&a->safety, &scfg, now_ms);
    link_rx_init(&a->rx, on_frame, a);

    a->prev_mode = MODE_INIT;
    a->last_ctrl_ms = now_ms;
    a->last_tlm_ms = now_ms;
    if (ok) {
        ballast_home_start(&a->ballast[BALLAST_FRONT], now_ms);
        ballast_home_start(&a->ballast[BALLAST_AFT], now_ms);
    }
}

/* ------------------------------------------------------------------ cycle de régulation (100 Hz) */
static void on_mode_enter(app_t *a, sub_mode_t mode)
{
    const sensor_data_t *d = sensors_data(&a->sm);
    float vf = ballast_volume_ml(&a->ballast[BALLAST_FRONT]);
    float va = ballast_volume_ml(&a->ballast[BALLAST_AFT]);
    if (mode == MODE_DEPTH_HOLD) {
        depth_ctrl_start(&a->dc, d->depth_m, vf, va);       /* reprend la main sans à-coup, en tenant la profondeur actuelle */
    } else if (mode == MODE_MANUAL) {
        a->manual_ml[BALLAST_FRONT] = vf;                   /* on garde les ballasts où ils sont */
        a->manual_ml[BALLAST_AFT] = va;
    }
}

static void control_cycle(app_t *a, uint32_t now_ms)
{
    float dt = (float)(uint32_t)(now_ms - a->last_ctrl_ms) / 1000.0f;
    a->last_ctrl_ms = now_ms;

    /* 1. liaison : lecture bornée (une rafale d'octets ne peut pas monopoliser la boucle) */
    uint8_t buf[64];
    for (uint32_t i = 0; i < MAX_LINK_READS_PER_CYCLE; i++) {
        uint16_t n = port_link_read(buf, sizeof buf);
        if (n == 0) break;
        link_rx_feed(&a->rx, buf, n);
    }

    /* 2. actions différées demandées par les commandes */
    if (a->tare_pending) {
        a->tare_pending = false;
        sensors_tare_depth(&a->sm, 50);
    }
    if (a->clear_pending) {
        a->clear_pending = false;
        sensors_clear_leak(&a->sm);
        sensors_poll(&a->sm, now_ms);                       /* rafraîchit failsafe_mask avant d'évaluer l'acquittement */
        safety_clear_fault(&a->safety);
    }

    /* 3. ballasts, puis sécurité */
    const sensor_data_t *d = sensors_data(&a->sm);
    ballast_update(&a->ballast[BALLAST_FRONT], d->limit, now_ms);
    ballast_update(&a->ballast[BALLAST_AFT], d->limit, now_ms);
    safety_inputs_t in = make_inputs(a, now_ms);
    safety_update(&a->safety, &in);

    /* 4. action selon le mode */
    sub_mode_t mode = safety_mode(&a->safety);
    if (mode != a->prev_mode) {
        on_mode_enter(a, mode);
        a->prev_mode = mode;
    }
    switch (mode) {
    case MODE_IDLE:
        (void)ballast_set_volume_ml(&a->ballast[BALLAST_FRONT], 0.0f);
        (void)ballast_set_volume_ml(&a->ballast[BALLAST_AFT], 0.0f);
        break;
    case MODE_MANUAL:
        (void)ballast_set_volume_ml(&a->ballast[BALLAST_FRONT], a->manual_ml[BALLAST_FRONT]);
        (void)ballast_set_volume_ml(&a->ballast[BALLAST_AFT], a->manual_ml[BALLAST_AFT]);
        break;
    case MODE_DEPTH_HOLD: {
        bool pitch_ok = (d->healthy_mask & (1u << SENSOR_IMU)) && !(d->failsafe_mask & FS_IMU_LOST);
        depth_ctrl_update(&a->dc, d->depth_m, d->depth_rate_m_s, d->pitch_deg, d->gyro_dps[1], pitch_ok, dt);
        (void)ballast_set_volume_ml(&a->ballast[BALLAST_FRONT], a->dc.front_ml);
        (void)ballast_set_volume_ml(&a->ballast[BALLAST_AFT], a->dc.aft_ml);
        break;
    }
    case MODE_EMERGENCY:
        ballast_emergency_empty(&a->ballast[BALLAST_FRONT], now_ms);
        ballast_emergency_empty(&a->ballast[BALLAST_AFT], now_ms);
        break;
    case MODE_INIT:
    default:
        break;
    }
}

void app_step(app_t *a, uint32_t now_ms)
{
    sensors_poll(&a->sm, now_ms);
    if ((uint32_t)(now_ms - a->last_ctrl_ms) >= APP_CONTROL_PERIOD_MS) {
        control_cycle(a, now_ms);
    }
    if ((uint32_t)(now_ms - a->last_tlm_ms) >= APP_TELEMETRY_PERIOD_MS) {
        a->last_tlm_ms = now_ms;
        send_telemetry(a, now_ms);
    }
}

void app_tick(app_t *a)
{
    ballast_tick(&a->ballast[BALLAST_FRONT]);
    ballast_tick(&a->ballast[BALLAST_AFT]);
}
