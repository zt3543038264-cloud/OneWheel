#include "HW_Drivers/Motor/W_Motor.h"
#include "Common/board_config.h"
#if QCAR_DRIVER_TEXT_DUTY
#include "IfxStm.h"
#endif
#include <string.h>
#if VEHICLE_MOTOR_OUTPUT_ALLOWED && !QCAR_DRIVER_TEXT_DUTY
#error "Ordinary dual output requires SAFE capability2 text driver"
#endif

#define W_MOTOR_FRAME_HEAD       0xA5u
#define W_MOTOR_SET_DUTY_CMD     0x01u
#define W_MOTOR_GET_SPEED_CMD    0x02u
#define W_MOTOR_FRAME_SIZE       7u
#define W_MOTOR_TX_FIFO_DEPTH    16u
#define WM_LINE_SIZE            72u
#define WM_STATUS_AGE_MS         60u
#define WM_STAGE_TIMEOUT_MS     60u
#define WM_COMMAND_PERIOD_MS    20u
#define WM_TX_ZERO              1u
#define WM_TX_SAFE              2u
#define WM_TX_STOP              3u
#define WM_RX_QUEUE_SIZE        256u
#define WM_RX_DRAIN_BUDGET      32u
#define WM_RX_PARSE_BUDGET      128u

#pragma section all "cpu0_dsram"
/* RX ISR is the sole head writer; control Tick is the sole tail writer.
 * All cells are published before head using dsync; only a short pop is locked. */
static volatile uint8 wm_rx_queue[WM_RX_QUEUE_SIZE];
static volatile uint16 wm_rx_head, wm_rx_tail;
static volatile uint8 wm_rx_fault;
/* Startup telemetry is drained without building a queue before control exists. */
static volatile uint8 wm_rx_started;
static uint8 wm_rx_resync;
#if QCAR_DRIVER_TEXT_DUTY
static volatile uint64 wm_rx_arrival[WM_RX_QUEUE_SIZE];
static uint64 wm_received_at;
#endif
static uint8 w_motor_rx_buffer[W_MOTOR_FRAME_SIZE], w_motor_rx_length;
static volatile int16 w_motor_speed_1, w_motor_speed_2;
static volatile uint32 w_motor_rx_frames;
static uint32 w_motor_seen_frames;
static volatile uint16 w_motor_link_age_ms, w_motor_request_age_ms;
static volatile uint8 w_motor_brake_locked;
#if QCAR_DRIVER_TEXT_DUTY
/* Fixed storage, no heap. A complete line already in the UART/current buffer
 * must finish before emergency DISARM; unsent pending commands are discarded.
 * Stop cannot retract bytes already queued in the UART FIFO or on the wire. */
static uint8 wm_tx_current[WM_LINE_SIZE], wm_tx_pending[WM_LINE_SIZE];
static volatile uint8 wm_tx_len, wm_tx_pos, wm_tx_kind, wm_pending_len, wm_pending_kind;
static volatile uint8 wm_query_pending, wm_status_pending, wm_status_query_ms;
static uint8 wm_rx_line[WM_LINE_SIZE], wm_rx_used, wm_rx_overflow, wm_rx_cr;
static volatile uint8 wm_clock_ok, wm_identity, wm_driver_state, wm_driver_reason;
static volatile uint8 wm_bench_state, wm_bench_reason;
static volatile uint8 wm_ordinary, wm_ordinary_failed, wm_cap2_seen, wm_open_sent;
#define WM_WAIT_OPEN 5u
static int16 wm_left_duty;
static uint64 wm_input_at;
static volatile uint16 wm_duration_ms;
static volatile int16 wm_right_duty;
static volatile uint32 wm_ticks_per_ms, wm_driver_session, wm_driver_seq;
static volatile uint32 wm_session, wm_consumed_session, wm_sent_seq, wm_ack_seq;
static volatile uint32 wm_status_count, wm_stage_status_count;
static volatile uint64 wm_status_at, wm_stage_at, wm_pulse_at, wm_command_at, wm_ack_at;
#endif
#pragma section all restore

#if QCAR_DRIVER_TEXT_DUTY
static uint64 wm_now(void)
{
    uint32 hi, lo, again;
    do {
        hi = IfxStm_getOffset32Timer(&MODULE_STM0);
        lo = IfxStm_getLower(&MODULE_STM0);
        again = IfxStm_getOffset32Timer(&MODULE_STM0);
    } while (hi != again);
    return ((uint64)hi << 32) | lo;
}
static uint8 wm_expired(uint64 now, uint64 since, uint16 ms)
{
    return (uint8)(now - since >= (uint64)wm_ticks_per_ms * ms);
}
static uint8 wm_busy(void)
{
    return (uint8)((wm_bench_state >= W_BENCH_WAIT_ZERO && wm_bench_state <= W_BENCH_PULSE) || (wm_ordinary && wm_bench_state==WM_WAIT_OPEN));
}
static void wm_tx_enable(uint8 enabled)
{
    volatile Ifx_SRC_SRCR *src = IfxAsclin_getSrcPointerTx(uart3_handle.asclin);
    IfxAsclin_enableTxFifoFillLevelFlag(uart3_handle.asclin, (boolean)enabled);
    if (enabled) IfxSrc_enable(src); else IfxSrc_disable(src);
}
static uint8 wm_append_u32(uint8 *out, uint32 value)
{
    uint8 digits[10], n = 0, j = 0;
    do { digits[n++] = (uint8)('0' + value % 10u); value /= 10u; } while (value);
    while (n) out[j++] = digits[--n];
    return j;
}
static uint8 wm_append_signed(uint8 *out, int16 value)
{
    uint8 n = 0;
    if (value < 0) { out[n++] = '-'; value = (int16)-value; }
    return (uint8)(n + wm_append_u32(out + n, (uint32)value));
}
static void wm_queue_stop_locked(uint8 reason)
{
    static const uint8 stop[] = "SAFE-DISARM,1\r\nSET-DUTY,0,0\r\n";
    if (wm_busy()) wm_consumed_session = wm_session;
    /* Also consume a displayed grant on explicit Stop, not on an idle zero. */
    if (wm_identity && wm_driver_session) wm_consumed_session = wm_driver_session;
    wm_bench_state = W_BENCH_LATCHED;
    wm_bench_reason = reason;
    wm_right_duty = wm_left_duty = 0;
    if (wm_ordinary) wm_ordinary_failed = 1;
    wm_identity = 0;
    memcpy(wm_tx_pending, stop, sizeof(stop) - 1u);
    wm_pending_len = (uint8)(sizeof(stop) - 1u);
    wm_pending_kind = WM_TX_STOP;
    wm_query_pending = wm_status_pending = 0;
}
static void wm_watchdog_locked(uint64 now)
{
    if (wm_rx_fault) {
        if (wm_busy()) wm_queue_stop_locked(W_BENCH_REASON_LINK);
        return;
    }
    if (!wm_busy()) return;
    if (!wm_clock_ok) { wm_queue_stop_locked(W_BENCH_REASON_INVALID); return; }
    if (wm_ordinary && wm_expired(now, wm_input_at, WM_STATUS_AGE_MS)) { wm_queue_stop_locked(W_BENCH_REASON_TIMEOUT); return; }
    if (!wm_ordinary && wm_bench_state == W_BENCH_PULSE && wm_expired(now, wm_pulse_at, wm_duration_ms)) {
        wm_queue_stop_locked(W_BENCH_REASON_COMPLETE); return;
    }
    if (wm_ordinary && wm_bench_state==WM_WAIT_OPEN) {
        if(wm_expired(now,wm_stage_at,WM_STAGE_TIMEOUT_MS)) wm_queue_stop_locked(W_BENCH_REASON_TIMEOUT);
        return;
    }
    if (!wm_identity || wm_expired(now, wm_status_at, WM_STATUS_AGE_MS)) {
        wm_queue_stop_locked(W_BENCH_REASON_LINK); return;
    }
    if (wm_driver_session != wm_session) {
        wm_queue_stop_locked(W_BENCH_REASON_SESSION); return;
    }
    if (wm_driver_reason || wm_driver_state == 0u || wm_driver_state == 4u) {
        wm_queue_stop_locked(W_BENCH_REASON_DRIVER); return;
    }
    if (wm_bench_state != W_BENCH_PULSE && wm_expired(now, wm_stage_at, WM_STAGE_TIMEOUT_MS)) {
        wm_queue_stop_locked(W_BENCH_REASON_TIMEOUT); return;
    }
    if (wm_bench_state == W_BENCH_PULSE && wm_expired(now, wm_ack_at, WM_STATUS_AGE_MS))
        wm_queue_stop_locked(W_BENCH_REASON_TIMEOUT);
}
static void wm_tx_pump_locked(void)
{
    uint8 sent;
    wm_watchdog_locked(wm_now());
    for (sent = 0; sent < W_MOTOR_TX_FIFO_DEPTH; ++sent) {
        if ((uint32)IfxAsclin_getTxFifoFillLevel(uart3_handle.asclin) >= W_MOTOR_TX_FIFO_DEPTH) break;
        if (wm_tx_pos >= wm_tx_len) {
            wm_tx_pos = wm_tx_len = wm_tx_kind = 0;
            if (wm_pending_kind >= WM_TX_SAFE) {
                memcpy(wm_tx_current, wm_tx_pending, wm_pending_len);
                wm_tx_len = wm_pending_len; wm_tx_kind = wm_pending_kind; wm_pending_kind = 0;
            } else if (wm_status_pending) {
                memcpy(wm_tx_current, "SAFE-STATUS,1\r\n", 15u); wm_tx_len = 15u; wm_status_pending = 0;
            } else if (wm_pending_kind) {
                memcpy(wm_tx_current, wm_tx_pending, wm_pending_len);
                wm_tx_len = wm_pending_len; wm_tx_kind = wm_pending_kind; wm_pending_kind = 0;
            } else if (wm_query_pending) {
                memcpy(wm_tx_current, "GET-SPEED\r\n", 11u); wm_tx_len = 11u; wm_query_pending = 0;
            } else break;
        }
        IfxAsclin_writeTxData(uart3_handle.asclin, wm_tx_current[wm_tx_pos++]);
    }
    wm_tx_enable((uint8)(wm_tx_pos < wm_tx_len || wm_pending_kind || wm_status_pending || wm_query_pending));
}
static uint8 wm_safe_command_locked(const char *prefix, uint8 duty)
{
    uint8 n;
    if (wm_pending_kind >= WM_TX_SAFE) return 0;
    n = (uint8)strlen(prefix);
    memcpy(wm_tx_pending, prefix, n);
    n += wm_append_u32(wm_tx_pending + n, wm_session);
    if (duty) {
        wm_tx_pending[n++] = ',';
        n += wm_append_u32(wm_tx_pending + n, wm_sent_seq);
        wm_tx_pending[n++] = ',';
        n += wm_append_signed(wm_tx_pending+n,wm_left_duty);
        wm_tx_pending[n++] = ',';
        n += wm_append_signed(wm_tx_pending + n, wm_right_duty);
    }
    wm_tx_pending[n++] = '\r'; wm_tx_pending[n++] = '\n';
    wm_pending_len = n; wm_pending_kind = WM_TX_SAFE;
    return 1;
}
/* Existing callers, including normal Test/Run/Jog, can only enqueue zero. */
static void wm_text_command(uint8 command, int16 left, int16 right)
{
    uint32 irq = interrupt_global_disable();
    (void)left; (void)right;
    if (command == W_MOTOR_SET_DUTY_CMD) {
        if (wm_busy()) wm_queue_stop_locked(W_BENCH_REASON_STOP);
        if (wm_pending_kind < WM_TX_SAFE) {
            memcpy(wm_tx_pending, "SET-DUTY,0,0\r\n", 14u);
            wm_pending_len = 14u; wm_pending_kind = WM_TX_ZERO;
        }
    } else if (command == W_MOTOR_GET_SPEED_CMD) wm_query_pending = 1;
    wm_tx_pump_locked();
    interrupt_global_enable(irq);
}
/* No strtol overflow/whitespace/sign acceptance: every field is bounded and
 * delimited, and the caller must consume the entire line. */
static uint8 wm_parse_u32(const uint8 **cursor, uint32 maximum, uint32 *value)
{
    const uint8 *p = *cursor;
    uint32 v = 0, d;
    if (*p < '0' || *p > '9') return 0;
    do {
        d = (uint32)(*p - '0');
        if (v > maximum / 10u || (v == maximum / 10u && d > maximum % 10u)) return 0;
        v = v * 10u + d; ++p;
    } while (*p >= '0' && *p <= '9');
    *cursor = p; *value = v; return 1;
}
static uint8 wm_parse_rpm(const uint8 **p, int16 *out)
{
    uint8 negative = (uint8)(**p == '-');
    uint32 value;
    if (negative) ++*p;
    if (!wm_parse_u32(p, negative ? 32768u : 32767u, &value)) return 0;
    *out = (int16)(negative ? -(int32)value : (int32)value); return 1;
}
static void wm_status_line(const uint8 *p)
{
    uint32 values[6], i;
    const uint32 limits[6] = {1u, 4u, 0xffffffffu, 0xffffffffu, 10u, 2u};
    uint64 now;
    for (i = 0; i < 6u; ++i) {
        if (!wm_parse_u32(&p, limits[i], &values[i])) {
            goto invalid;
        }
        if (i < 5u) { if (*p++ != ',') goto invalid; }
    }
    if (*p) goto invalid;
    if (values[0] != 1u) { goto invalid; }
    if (values[5] != 2u) { goto invalid; }
    if ((values[1] >= 1u && values[1] <= 3u) && (!values[2] || values[4])) {
        goto invalid;
    }
    if ((values[1] == 1u || values[1] == 2u) && values[3]) {
        goto invalid;
    }
    now = wm_now();
    wm_cap2_seen = 1; wm_identity = 1; wm_driver_state = (uint8)values[1]; wm_driver_session = values[2];
    wm_driver_seq = values[3]; wm_driver_reason = (uint8)values[4];
    wm_status_at = wm_received_at; ++wm_status_count;
    if (wm_ordinary && wm_bench_state==WM_WAIT_OPEN) {
        /* Status only records state. Sensor-checked Service sends ZERO/ARM. */
        return;
    }
    if (wm_busy()) {
        if (wm_driver_session != wm_session) { wm_queue_stop_locked(W_BENCH_REASON_SESSION); return; }
        if ((wm_bench_state == W_BENCH_WAIT_ZERO && wm_driver_state > 2u) ||
            (wm_bench_state == W_BENCH_WAIT_ARM && wm_driver_state == 1u) ||
            (wm_bench_state == W_BENCH_PULSE && wm_driver_state != 3u) ||
            wm_driver_seq > wm_sent_seq || wm_driver_seq < wm_ack_seq) {
            wm_queue_stop_locked(W_BENCH_REASON_PROTOCOL); return;
        }
        if (wm_driver_seq > wm_ack_seq) { wm_ack_seq = wm_driver_seq; wm_ack_at = now; }
        wm_watchdog_locked(now);
    }
    return;
invalid:
    wm_cap2_seen = 0; wm_identity = 0;
    if (wm_busy()) wm_queue_stop_locked(W_BENCH_REASON_PROTOCOL);
}
static void wm_text_rx(uint8 byte)
{
    const uint8 *p;
    int16 a, b;
    if (byte == '\n') {
        if (!wm_rx_overflow) {
            wm_rx_line[wm_rx_used] = 0;
            if (wm_rx_used >= 12u && !memcmp(wm_rx_line, "SAFE-STATUS,", 12u))
                wm_status_line(wm_rx_line + 12u);
            else {
                p = wm_rx_line;
                if (wm_parse_rpm(&p, &a) && *p++ == ',' && wm_parse_rpm(&p, &b) && !*p) {
                    w_motor_speed_1 = (int16)(a * MOTOR_DIR_A);
                    w_motor_speed_2 = (int16)(b * MOTOR_DIR_B); ++w_motor_rx_frames;
                }
            }
        } else {
            wm_identity = 0;
            if (wm_busy()) wm_queue_stop_locked(W_BENCH_REASON_PROTOCOL);
        }
        wm_rx_used = wm_rx_overflow = wm_rx_cr = 0; return;
    }
    if (byte == '\r' && !wm_rx_cr) { wm_rx_cr = 1; return; }
    if (wm_rx_overflow) return;
    if (wm_rx_cr || byte < 32u || byte > 126u || wm_rx_used >= sizeof(wm_rx_line) - 1u) {
        wm_rx_overflow = 1; return;
    }
    wm_rx_line[wm_rx_used++] = byte;
}
#endif

static void W_Motor_SendFrame(uint8 command)
{
#if QCAR_DRIVER_TEXT_DUTY
    wm_text_command(command, 0, 0);
#else
    uint8 frame[7] = {W_MOTOR_FRAME_HEAD, 0, 0, 0, 0, 0, 0}, i;
    uint32 irq = interrupt_global_disable();
    frame[1] = command; frame[6] = (uint8)(frame[0] + command);
    if ((uint32)IfxAsclin_getTxFifoFillLevel(uart3_handle.asclin) <= W_MOTOR_TX_FIFO_DEPTH - W_MOTOR_FRAME_SIZE)
        for (i = 0; i < W_MOTOR_FRAME_SIZE; ++i) IfxAsclin_writeTxData(uart3_handle.asclin, frame[i]);
    interrupt_global_enable(irq);
#endif
}
void W_Motor_Init(void)
{
    wm_rx_head = wm_rx_tail = 0;
    wm_rx_fault = wm_rx_resync = 0;
    wm_rx_started = 0;
#if QCAR_DRIVER_TEXT_DUTY
    wm_received_at = 0;
#endif
    w_motor_rx_length = 0; w_motor_speed_1 = w_motor_speed_2 = 0;
    w_motor_rx_frames = w_motor_seen_frames = 0;
    w_motor_link_age_ms = W_MOTOR_LINK_TIMEOUT_MS; w_motor_request_age_ms = 0;
    w_motor_brake_locked = 1;
    uart_init(W_MOTOR_UART, W_MOTOR_UART_BAUDRATE, W_MOTOR_UART_TX_PIN, W_MOTOR_UART_RX_PIN);
#if QCAR_DRIVER_TEXT_DUTY
    {
        float hz = IfxStm_getFrequency(&MODULE_STM0);
        wm_clock_ok = (uint8)(hz >= 1000.0f && hz <= 1000000000.0f);
        wm_ticks_per_ms = wm_clock_ok ? (uint32)(hz / 1000.0f) : 0u;
    }
    wm_tx_len = wm_tx_pos = wm_tx_kind = wm_pending_len = wm_pending_kind = 0;
    wm_query_pending = wm_status_pending = wm_status_query_ms = 0;
    wm_rx_used = wm_rx_overflow = wm_rx_cr = 0;
    wm_identity = wm_driver_state = wm_driver_reason = 0;
    wm_bench_state = W_BENCH_IDLE; wm_bench_reason = W_BENCH_REASON_NONE;
    wm_ordinary = wm_ordinary_failed = wm_cap2_seen = wm_open_sent = 0; wm_left_duty=0; wm_input_at=0;
    wm_duration_ms = 0; wm_right_duty = 0;
    wm_driver_session = wm_driver_seq = wm_session = wm_consumed_session = 0;
    wm_sent_seq = wm_ack_seq = wm_status_count = wm_stage_status_count = 0;
    wm_status_at = wm_stage_at = wm_pulse_at = wm_command_at = wm_ack_at = 0;
    wm_tx_enable(0);
#endif
    uart_rx_interrupt(W_MOTOR_UART, 1);
    W_Motor_Stop();
    W_Motor_SendFrame(W_MOTOR_GET_SPEED_CMD);
}
void W_Motor_TxHandler(void)
{
#if QCAR_DRIVER_TEXT_DUTY
    uint32 irq = interrupt_global_disable();
    IfxAsclin_clearTxFifoFillLevelFlag(uart3_handle.asclin);
    wm_tx_pump_locked(); interrupt_global_enable(irq);
#endif
}
void W_Motor_SetDuty(int32 left, int32 right)
{
#if QCAR_DRIVER_TEXT_DUTY
    uint32 irq=interrupt_global_disable();
    wm_watchdog_locked(wm_now());
    if (wm_ordinary && !wm_ordinary_failed && !w_motor_brake_locked && wm_busy()) {
        wm_input_at=wm_now();
        left=func_limit_ab(left,-W_MOTOR_DUTY_MAX,W_MOTOR_DUTY_MAX);
        right=func_limit_ab(right,-W_MOTOR_DUTY_MAX,W_MOTOR_DUTY_MAX);
        wm_left_duty=(int16)(left*MOTOR_DIR_A); wm_right_duty=(int16)(right*MOTOR_DIR_B);
        W_Motor_BenchService();
    } else if (!wm_busy()) W_Motor_SendFrame(W_MOTOR_SET_DUTY_CMD);
    else wm_queue_stop_locked(W_BENCH_REASON_STOP);
    interrupt_global_enable(irq);
#else
    (void)left;(void)right;W_Motor_SendFrame(W_MOTOR_SET_DUTY_CMD);
#endif
}
void W_Motor_KeepZero(void) { W_Motor_SetDuty(0,0); }
void W_Motor_Stop(void)
{
    w_motor_brake_locked=1;
#if QCAR_DRIVER_TEXT_DUTY
    uint32 irq=interrupt_global_disable();
    wm_queue_stop_locked(W_BENCH_REASON_STOP);wm_ordinary=0;wm_ordinary_failed=0;
    wm_tx_pump_locked();interrupt_global_enable(irq);
#else
    W_Motor_SendFrame(W_MOTOR_SET_DUTY_CMD);
#endif
}
void W_Motor_Release(void) {
#if QCAR_DRIVER_TEXT_DUTY
    if(wm_ordinary && !wm_ordinary_failed && wm_busy()) w_motor_brake_locked=0;
#endif
}
uint8 W_Motor_OrdinaryStart(void) {
#if QCAR_DRIVER_TEXT_DUTY
    uint32 irq=interrupt_global_disable(); uint64 now=wm_now();
    if(wm_rx_fault || wm_rx_resync || wm_busy() || !wm_clock_ok || !wm_cap2_seen || wm_expired(now,wm_status_at,WM_STATUS_AGE_MS)) {interrupt_global_enable(irq);return 0;}
    wm_ordinary=1;wm_ordinary_failed=0;wm_open_sent=0;
    wm_left_duty=wm_right_duty=0;wm_input_at=wm_stage_at=now;
    wm_sent_seq=wm_ack_seq=0;wm_stage_status_count=wm_status_count;
    wm_bench_state=WM_WAIT_OPEN;wm_bench_reason=W_BENCH_REASON_NONE;
    w_motor_brake_locked=1;
    interrupt_global_enable(irq);return 1;
#else
    return 0;
#endif
}
uint8 W_Motor_OrdinaryPending(void) {
#if QCAR_DRIVER_TEXT_DUTY
 return (uint8)(wm_ordinary && wm_busy() && wm_bench_state!=W_BENCH_PULSE);
#else
 return 0;
#endif
}
uint8 W_Motor_OrdinaryFailed(void) {
#if QCAR_DRIVER_TEXT_DUTY
 return wm_ordinary_failed;
#else
 return 0;
#endif
}
void W_Motor_OrdinaryService(void) {
#if QCAR_DRIVER_TEXT_DUTY
 uint32 irq=interrupt_global_disable();
 wm_watchdog_locked(wm_now());
 if(wm_ordinary && !wm_ordinary_failed){wm_input_at=wm_now();W_Motor_BenchService();}
 interrupt_global_enable(irq);
#endif
}
void W_Motor_BenchAbort(uint8 reason)
{
#if QCAR_DRIVER_TEXT_DUTY
    uint32 irq = interrupt_global_disable();
    wm_queue_stop_locked(reason); wm_tx_pump_locked(); interrupt_global_enable(irq);
#else
    (void)reason; W_Motor_SendFrame(W_MOTOR_SET_DUTY_CMD);
#endif
}
uint8 W_Motor_BenchBusy(void)
{
#if QCAR_DRIVER_TEXT_DUTY
    uint8 busy; uint32 irq = interrupt_global_disable();
    busy = (uint8)(wm_busy() && !wm_ordinary); interrupt_global_enable(irq); return busy;
#else
    return 0;
#endif
}
uint8 W_Motor_BenchRequestB(int16 raw_duty, uint16 duration_ms, uint32 expected_session)
{
#if QCAR_DRIVER_TEXT_DUTY && !C_BENCH_ONLY_PROFILE
    uint32 irq = interrupt_global_disable();
    uint64 now = wm_now();
    if (raw_duty == 0 || raw_duty < -W_BENCH_MAX_DUTY || raw_duty > W_BENCH_MAX_DUTY ||
        !duration_ms || duration_ms > W_BENCH_MAX_MS || wm_rx_fault || wm_rx_resync || wm_busy() || !wm_clock_ok ||
        !expected_session || expected_session != wm_driver_session ||
        !wm_identity || wm_expired(now, wm_status_at, WM_STATUS_AGE_MS) ||
        wm_driver_state != 1u || wm_driver_reason || !wm_driver_session || wm_driver_seq ||
        wm_driver_session == wm_consumed_session || wm_pending_kind >= WM_TX_SAFE ||
        (wm_tx_kind == WM_TX_STOP && (wm_tx_pos < wm_tx_len || IfxAsclin_getTxFifoFillLevel(uart3_handle.asclin)))) {
        interrupt_global_enable(irq); return 0;
    }
    wm_session = wm_consumed_session = wm_driver_session;
    wm_ordinary=wm_ordinary_failed=0;wm_left_duty=0;
    wm_right_duty = raw_duty; wm_duration_ms = duration_ms;
    wm_sent_seq = wm_ack_seq = 0; wm_stage_at = now; wm_pulse_at = 0;
    wm_stage_status_count = wm_status_count;
    wm_bench_state = W_BENCH_WAIT_ZERO; wm_bench_reason = W_BENCH_REASON_NONE;
    wm_safe_command_locked("SAFE-ZERO,1,", 0);
    wm_tx_pump_locked(); interrupt_global_enable(irq); return 1;
#else
    (void)raw_duty; (void)duration_ms; (void)expected_session; return 0;
#endif
}
void W_Motor_BenchService(void)
{
#if QCAR_DRIVER_TEXT_DUTY
    uint32 irq = interrupt_global_disable();
    uint64 now = wm_now();
    wm_watchdog_locked(now);
    if(wm_ordinary && wm_bench_state==WM_WAIT_OPEN) {
        if(!wm_open_sent && !wm_pending_kind && wm_tx_pos>=wm_tx_len && !IfxAsclin_getTxFifoFillLevel(uart3_handle.asclin)) {
            memcpy(wm_tx_pending,"SAFE-OPEN,1\r\n",13u);wm_pending_len=13u;wm_pending_kind=WM_TX_SAFE;
            wm_open_sent=1;wm_stage_status_count=wm_status_count;
        } else if(wm_open_sent && wm_status_count!=wm_stage_status_count && wm_driver_state==1u && !wm_driver_reason && wm_driver_session && !wm_driver_seq && wm_pending_kind<WM_TX_SAFE) {
            wm_session=wm_consumed_session=wm_driver_session;
            wm_stage_at=now;wm_stage_status_count=wm_status_count;wm_bench_state=W_BENCH_WAIT_ZERO;
            wm_safe_command_locked("SAFE-ZERO,1,",0);
        }
    }
    else if (wm_bench_state == W_BENCH_WAIT_ZERO && wm_driver_state == 2u &&
        wm_status_count != wm_stage_status_count && wm_driver_seq == 0u && wm_pending_kind < WM_TX_SAFE) {
        wm_bench_state = W_BENCH_WAIT_ARM; wm_stage_at = now;
        wm_stage_status_count = wm_status_count;
        wm_safe_command_locked("SAFE-ARM,1,", 0);
    } else if (wm_bench_state == W_BENCH_WAIT_ARM && wm_driver_state == 3u &&
        wm_status_count != wm_stage_status_count && wm_driver_seq == 0u && wm_pending_kind < WM_TX_SAFE) {
        wm_bench_state = W_BENCH_PULSE; wm_pulse_at = wm_command_at = wm_ack_at = now;
        wm_sent_seq = 1u; wm_safe_command_locked("SAFE-DUTY,1,", 1);
    } else if (wm_bench_state == W_BENCH_PULSE && wm_expired(now, wm_command_at, WM_COMMAND_PERIOD_MS) &&
               wm_pending_kind < WM_TX_SAFE) {
        if (wm_sent_seq == 0xffffffffu) wm_queue_stop_locked(W_BENCH_REASON_PROTOCOL);
        else { ++wm_sent_seq; wm_command_at = now; wm_safe_command_locked("SAFE-DUTY,1,", 1); }
    }
    wm_tx_pump_locked(); interrupt_global_enable(irq);
#endif
}
void W_Motor_BenchReadStatus(w_motor_bench_status_t *out)
{
    uint32 irq;
    if (!out) return;
    irq = interrupt_global_disable(); memset(out, 0, sizeof(*out));
#if QCAR_DRIVER_TEXT_DUTY
    {
        uint64 now = wm_now(), elapsed = 0;
        out->host_state = wm_bench_state; out->driver_state = wm_driver_state;
        out->busy = wm_busy(); out->reason = wm_bench_reason; out->driver_reason = wm_driver_reason;
        out->session = wm_driver_session; out->last_seq = wm_driver_seq;
        out->right_duty = wm_bench_state == W_BENCH_PULSE ? wm_right_duty : 0;
        out->ready = (uint8)(!wm_rx_fault && !wm_rx_resync && !out->busy && wm_clock_ok && wm_identity && wm_driver_state == 1u &&
            !wm_driver_reason && wm_driver_session && wm_driver_session != wm_consumed_session &&
            !wm_expired(now, wm_status_at, WM_STATUS_AGE_MS));
        if (wm_bench_state == W_BENCH_PULSE && wm_ticks_per_ms) elapsed = (now - wm_pulse_at) / wm_ticks_per_ms;
        out->elapsed_ms = (uint16)(elapsed > W_BENCH_MAX_MS ? W_BENCH_MAX_MS : elapsed);
    }
#endif
    interrupt_global_enable(irq);
}
/* Error ISR calls this before the vendor handler clears hardware flags.
 * Events with no received byte must also invalidate a partial frame. */
void W_Motor_RxErrorHandler(void)
{
    if (IfxAsclin_getRxFifoOverflowFlagStatus(uart3_handle.asclin) ||
        IfxAsclin_getFrameErrorFlagStatus(uart3_handle.asclin) ||
        IfxAsclin_getParityErrorFlagStatus(uart3_handle.asclin)) wm_rx_fault = 1;
}

/* High-priority ISR: bounded transport ONLY. Never parse, format, send,
 * touch protocol state, arm a motor or call control code from here. */
void W_Motor_RxHandler(void)
{
    uint8 byte, budget = WM_RX_DRAIN_BUDGET;
    uint32 irq = interrupt_global_disable();
#if QCAR_DRIVER_TEXT_DUTY
    uint64 received_at = wm_now();
#endif
    /* Do not clear hardware flags; existing error ISR still owns that. */
    W_Motor_RxErrorHandler();
    while (budget-- && uart_query_byte(W_MOTOR_UART, &byte)) {
        uint16 head, next;
        if (!wm_rx_started) {
            continue;
        }
        head = wm_rx_head;
        next = (uint16)((head + 1u) & (WM_RX_QUEUE_SIZE-1u));
        if (wm_rx_fault || next == wm_rx_tail) {
            wm_rx_fault = 1; continue;
        }
        wm_rx_queue[head] = byte;
#if QCAR_DRIVER_TEXT_DUTY
        wm_rx_arrival[head] = received_at;
#endif
        __dsync(); wm_rx_head = next;

    }
    interrupt_global_enable(irq);
}

/* Sole consumer: called from 1ms control ISR, never from foreground or RX ISR.
 * Higher-priority RX only changes transport fields, not these protocol fields. */
static void wm_rx_consume(void)
{
    uint16 budget = WM_RX_PARSE_BUDGET;
    while (budget--) {
        uint8 byte, checksum, index;
        uint32 irq = interrupt_global_disable();
#if QCAR_DRIVER_TEXT_DUTY
        uint64 arrived;
#endif
        if (wm_rx_fault) {
            /* Discard all queued bytes around a loss, then discard through LF.
             * No partial pre-loss frame may be joined to post-loss data. */
            wm_rx_tail = wm_rx_head; wm_rx_fault = 0; wm_rx_resync = 1;
            w_motor_rx_length = 0;
#if QCAR_DRIVER_TEXT_DUTY
            wm_rx_used = wm_rx_overflow = wm_rx_cr = 0;
            wm_cap2_seen = wm_identity = 0;
            if (wm_busy()) wm_queue_stop_locked(W_BENCH_REASON_LINK);
#endif
        }
        if (wm_rx_tail == wm_rx_head) { interrupt_global_enable(irq); break; }
        __dsync(); byte = wm_rx_queue[wm_rx_tail];
#if QCAR_DRIVER_TEXT_DUTY
        arrived = wm_rx_arrival[wm_rx_tail];
#endif
        wm_rx_tail = (uint16)((wm_rx_tail + 1u) & (WM_RX_QUEUE_SIZE-1u));
        interrupt_global_enable(irq);
#if QCAR_DRIVER_TEXT_DUTY
        /* Buffered data must not resurrect a stale link after a CPU stall. */
        if (!wm_clock_ok || wm_expired(wm_now(), arrived, WM_STATUS_AGE_MS)) {
            wm_rx_resync = 1; w_motor_rx_length = 0;
            wm_rx_used = wm_rx_overflow = wm_rx_cr = 0;
            wm_cap2_seen = wm_identity = 0;
            if (wm_busy()) wm_queue_stop_locked(W_BENCH_REASON_LINK);
            continue;
        }
        wm_received_at = arrived;
#endif
        if (wm_rx_resync) {
            if (byte == '\n') wm_rx_resync = 0;
            continue;
        }
        if (!w_motor_rx_length && byte != W_MOTOR_FRAME_HEAD) {
#if QCAR_DRIVER_TEXT_DUTY
            wm_text_rx(byte);
#endif
            continue;
        }
#if QCAR_DRIVER_TEXT_DUTY
        if (!w_motor_rx_length) wm_rx_used = wm_rx_overflow = wm_rx_cr = 0;
#endif
        w_motor_rx_buffer[w_motor_rx_length++] = byte;
        if (w_motor_rx_length < W_MOTOR_FRAME_SIZE) continue;
        w_motor_rx_length = 0; checksum = 0;
        for (index = 0; index < W_MOTOR_FRAME_SIZE - 1u; ++index) checksum = (uint8)(checksum + w_motor_rx_buffer[index]);
        if (checksum != w_motor_rx_buffer[6] || w_motor_rx_buffer[1] != W_MOTOR_GET_SPEED_CMD) continue;
        w_motor_speed_1 = (int16)((int16)(((uint16)w_motor_rx_buffer[2] << 8) | w_motor_rx_buffer[3]) * MOTOR_DIR_A);
        w_motor_speed_2 = (int16)((int16)(((uint16)w_motor_rx_buffer[4] << 8) | w_motor_rx_buffer[5]) * MOTOR_DIR_B);
        ++w_motor_rx_frames;
    }
#if QCAR_DRIVER_TEXT_DUTY
    { uint32 irq = interrupt_global_disable(); wm_tx_pump_locked(); interrupt_global_enable(irq); }
#endif
}
void W_Motor_Tick1ms(void)
{
    uint32 frames;
    if (!wm_rx_started) {
        uint32 irq = interrupt_global_disable();
        /* First control tick follows initialization/calibration. The early STOP
         * and TX remain unchanged. Never accept a startup frame fragment. */
        wm_rx_tail = wm_rx_head;
        w_motor_rx_length = 0;
#if QCAR_DRIVER_TEXT_DUTY
        wm_rx_used = wm_rx_overflow = wm_rx_cr = 0;
        wm_cap2_seen = wm_identity = 0;
        wm_rx_resync = 1;
#endif
        /* Preserve any real error event and library flags collected at boot. */
        __dsync(); wm_rx_started = 1;
        interrupt_global_enable(irq);
    }
    wm_rx_consume();
    frames = w_motor_rx_frames;
    if (frames != w_motor_seen_frames) { w_motor_seen_frames = frames; w_motor_link_age_ms = 0; }
    else if (w_motor_link_age_ms < W_MOTOR_LINK_TIMEOUT_MS) ++w_motor_link_age_ms;
    if (w_motor_request_age_ms < W_MOTOR_SPEED_REQUEST_MS) ++w_motor_request_age_ms;
    else { w_motor_request_age_ms = 0; W_Motor_SendFrame(W_MOTOR_GET_SPEED_CMD); }
#if QCAR_DRIVER_TEXT_DUTY
    {
        uint32 irq = interrupt_global_disable();
        /* Observation and stop only: this function never advances an arm. */
        wm_watchdog_locked(wm_now());
        if (++wm_status_query_ms >= WM_COMMAND_PERIOD_MS) { wm_status_query_ms = 0; wm_status_pending = 1; }
        wm_tx_pump_locked(); interrupt_global_enable(irq);
    }
#endif
}
int16 W_Motor_GetSpeed1(void) { return w_motor_speed_1; }
int16 W_Motor_GetSpeed2(void) { return w_motor_speed_2; }
uint8 W_Motor_LinkLost(void) {
#if QCAR_DRIVER_TEXT_DUTY
 uint32 irq=interrupt_global_disable();
 uint8 lost=(uint8)(wm_rx_fault || wm_rx_resync || wm_ordinary_failed || (!wm_ordinary && (!wm_cap2_seen || wm_expired(wm_now(),wm_status_at,WM_STATUS_AGE_MS))) || (wm_ordinary && wm_bench_state!=WM_WAIT_OPEN && (!wm_identity || wm_expired(wm_now(),wm_status_at,WM_STATUS_AGE_MS))));
 interrupt_global_enable(irq);return lost;
#else
 return 1;
#endif
}
