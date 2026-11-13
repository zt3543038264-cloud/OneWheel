/* Included once by control.c, so ADS project files do not need a new C unit.
 * Only CPU0 foreground/ISR callers; public state transitions mask interrupts.
 * Runtime deadlines cannot remove PWM while the CPU is hung or power is lost.
 * Encoder IDs prove scheduled samples only, not wiring/calibration/rotation. */
#if CONTROL_C_BENCH_RAW_LIMIT != C_BENCH_RAW_DUTY_MAX
#error "C bench controller and final PWM driver raw bounds must agree"
#endif
#pragma section all "cpu0_dsram"
static volatile control_c_bench_diag_t s_c_bench;
static uint64 s_c_started, s_c_heartbeat, s_c_imu_seen_at, s_c_encoder_seen_at;
static uint64 s_c_pi_at;
static uint32 s_c_imu_seq, s_c_encoder_seq, s_c_pi_seq;
static uint8 s_c_imu_seen, s_c_encoder_seen;
static float s_c_integral;
static float s_c_roll_zero, s_c_pitch_zero;
#pragma section all restore

static uint8 c_bench_powered(uint8 mode)
{ return (uint8)(mode == C_BENCH_POWERED_RATE_POS || mode == C_BENCH_POWERED_RATE_NEG); }
static uint8 c_bench_signal(uint8 mode)
{ return (uint8)(mode == C_BENCH_SIGNAL_RAW || mode == C_BENCH_SIGNAL_RATE || c_bench_powered(mode)); }
static uint8 c_bench_rate(uint8 mode)
{ return (uint8)(mode == C_BENCH_DRY_RATE || mode == C_BENCH_SIGNAL_RATE || c_bench_powered(mode)); }
static uint32 c_bench_age_ms(uint64 now, uint64 since)
{
    uint64 age;
    if (!s_control_ticks_per_ms) return 0xffffffffu;
    age = (now - since) / s_control_ticks_per_ms;
    return age > 0xffffffffu ? 0xffffffffu : (uint32)age;
}
static uint8 c_bench_expired(uint64 now, uint64 since, uint32 ms)
{ return (uint8)(now - since >= (uint64)s_control_ticks_per_ms * ms); }

/* Observe publications in the actual control loop, never from status/keepalive.
 * A repeated sequence cannot refresh data age. A reset-to-zero encoder ID must
 * first produce a new complete window before another start is accepted. */
static void c_bench_observe(void)
{
    uint64 now = control_stm_ticks();
    uint32 seq = imu_get_sample_sequence();
    if (seq && (!s_c_imu_seen || seq != s_c_imu_seq)) {
        s_c_imu_seq = seq; s_c_imu_seen_at = now; s_c_imu_seen = 1;
    }
    seq = Y_Motor_GetEncoderSampleId20ms();
    if (!seq) s_c_encoder_seen = 0;
    else if (!s_c_encoder_seen || seq != s_c_encoder_seq) {
        s_c_encoder_seq = seq; s_c_encoder_seen_at = now; s_c_encoder_seen = 1;
    }
}
static void c_bench_snapshot(uint64 now)
{
    /* 2 is a distinct protocol profile, never an isolated-signal grant. */
    s_c_bench.signal_allowed = C_BENCH_POWERED_RATE_PROFILE ? 2u : C_BENCH_SIGNAL_OUTPUT_ALLOWED;
    s_c_bench.imu_age_ms = s_c_imu_seen ? c_bench_age_ms(now, s_c_imu_seen_at) : 0xffffffffu;
    s_c_bench.encoder20ms = Y_Motor_GetSpeed20ms();
    /* A fault/stop ACK must remain parseable when a sensor is nonfinite.
     * Zero is only a placeholder in fault diagnostics, never valid feedback. */
    s_c_bench.angle_deg = ctrl_is_finite(att.pitch) ? att.pitch : 0.0f;
    s_c_bench.rate_dps = pitch_coordinates_valid() ? pitch_euler_rate() : 0.0f;
    if (!ctrl_is_finite(s_c_bench.rate_dps)) s_c_bench.rate_dps = 0.0f;
    if (s_c_bench.active) {
        s_c_bench.elapsed_ms = c_bench_age_ms(now, s_c_started);
        s_c_bench.heartbeat_age_ms = c_bench_age_ms(now, s_c_heartbeat);
    }
}
static void c_bench_finish(control_c_bench_reason_t reason)
{
    c_bench_snapshot(control_stm_ticks());
    s_c_bench.active = 0;
    s_c_bench.state = (reason == C_BENCH_REASON_MANUAL || reason == C_BENCH_REASON_EXPIRED)
        ? C_BENCH_STOPPED : C_BENCH_FAULT;
    s_c_bench.reason = reason;
    s_c_bench.calculated = 0.0f;
    s_c_bench.applied = 0;
    s_c_integral = 0.0f; s_c_pi_seq = 0; s_c_pi_at = 0;
    s_pitch_rate_cache_valid = 0;
    control_zero_sink();
}
static uint8 c_bench_conflict(void)
{
    return (uint8)(W_Motor_BenchBusy() || s_preview.active || s_test_running ||
        s_jog_target != MOTOR_JOG_NONE || start_flag != START_STOP ||
        s_remote_active || s_run_active || s_path_active || path_nav_busy());
}
static control_c_bench_reason_t c_bench_guard(uint64 now, uint8 starting)
{
    int32 encoder = Y_Motor_GetSpeed20ms();
    control_test_status_t sensor;
    if (!s_control_ticks_per_ms || !s_control_timing.valid ||
        !s_control_timing.period_us || s_control_timing.period_us >= CONTROL_BENCH_MAX_GAP_US ||
        s_control_timing.work_us >= CONTROL_BENCH_MAX_WORK_US ||
        control_ticks_us(now - s_control_last_ticks) >=
            (starting ? CONTROL_BENCH_MAX_GAP_US : CONTROL_BENCH_MAX_WORK_US))
        return C_BENCH_REASON_TIMING;
    if (!starting) {
        if (c_bench_expired(now, s_c_started, s_c_bench.duration_ms)) return C_BENCH_REASON_EXPIRED;
        if (c_bench_expired(now, s_c_heartbeat, CONTROL_C_BENCH_LEASE_MS)) return C_BENCH_REASON_LEASE;
    }
    if (c_bench_conflict()) return C_BENCH_REASON_BUSY;
    sensor = control_sensor_reason();
    s_c_bench.sensor_reason = sensor;
    if (sensor != CTRL_TEST_STATUS_OK || !s_c_imu_seen ||
        c_bench_expired(now, s_c_imu_seen_at, CONTROL_C_BENCH_IMU_MAX_AGE_MS))
        return C_BENCH_REASON_SENSOR;
    /* Fixed limits cannot be relaxed by editing old menu protection settings. */
    if (fabsf(att.roll) >= CONTROL_C_BENCH_TILT_LIMIT_DEG ||
        fabsf(att.pitch) >= CONTROL_C_BENCH_TILT_LIMIT_DEG ||
        fabsf(att.roll - g_roll_zero) >= CONTROL_C_BENCH_TILT_LIMIT_DEG ||
        fabsf(att.pitch - g_pitch_zero) >= CONTROL_C_BENCH_TILT_LIMIT_DEG ||
        fabsf(att.pitch_rate) >= CONTROL_C_BENCH_RATE_LIMIT_DPS ||
        fabsf(att.roll_rate) >= CONTROL_C_BENCH_RATE_LIMIT_DPS ||
        fabsf(imu.gyro_z) >= CONTROL_C_BENCH_RATE_LIMIT_DPS ||
        fabsf(pitch_euler_rate()) >= CONTROL_C_BENCH_RATE_LIMIT_DPS ||
        (!starting && (g_roll_zero != s_c_roll_zero || g_pitch_zero != s_c_pitch_zero)))
        return C_BENCH_REASON_SENSOR;
    if (!s_c_encoder_seen || !Y_Motor_GetEncoderSampleId20ms() ||
        c_bench_expired(now, s_c_encoder_seen_at, CONTROL_C_BENCH_ENCODER_MAX_AGE_MS) ||
        encoder > (starting ? CONTROL_C_BENCH_START_ENCODER_LIMIT : CONTROL_C_BENCH_ENCODER_LIMIT) ||
        encoder < -(starting ? CONTROL_C_BENCH_START_ENCODER_LIMIT : CONTROL_C_BENCH_ENCODER_LIMIT))
        return C_BENCH_REASON_ENCODER;
    return C_BENCH_REASON_OK;
}

uint32 control_c_bench_start(control_c_bench_mode_t mode, float target,
    float kp, float ki, uint16 duration_ms, uint16 limit, uint32 expected_session)
{
    uint32 mask = interrupt_global_disable();
    uint64 now = control_stm_ticks();
    control_c_bench_reason_t reason = C_BENCH_REASON_OK;
    uint32 result = 0;
    if (s_c_bench.active || c_bench_conflict()) reason = C_BENCH_REASON_BUSY;
    else if (expected_session != s_c_bench.session || s_c_bench.session == 0xffffffffu)
        reason = C_BENCH_REASON_SESSION;
    else if ((int)mode < (int)C_BENCH_DRY_RAW || mode > C_BENCH_POWERED_RATE_NEG ||
        !ctrl_is_finite(target) || !ctrl_is_finite(kp) || !ctrl_is_finite(ki) ||
        duration_ms == 0 || duration_ms > CONTROL_C_BENCH_MAX_MS ||
        limit == 0 || limit > CONTROL_C_BENCH_RAW_LIMIT ||
        kp < 0.0f || kp > CONTROL_C_BENCH_KP_MAX || ki < 0.0f || ki > CONTROL_C_BENCH_KI_MAX ||
        (c_bench_rate((uint8)mode) ? fabsf(target) > CONTROL_C_BENCH_TARGET_DPS :
            (fabsf(target) > (float)limit || kp != 0.0f || ki != 0.0f)))
        reason = C_BENCH_REASON_INVALID;
    else if (c_bench_powered((uint8)mode) &&
        (target != 0.0f || ki != 0.0f || kp <= 0.0f || kp > CONTROL_C_POWER_KP_MAX ||
         duration_ms > CONTROL_C_POWER_MAX_MS || limit > CONTROL_C_POWER_RAW_LIMIT))
        reason = C_BENCH_REASON_INVALID;
    else if ((c_bench_powered((uint8)mode) && !C_BENCH_POWERED_RATE_PROFILE) ||
        (C_BENCH_POWERED_RATE_PROFILE &&
         (mode == C_BENCH_SIGNAL_RAW || mode == C_BENCH_SIGNAL_RATE)))
        reason = C_BENCH_REASON_LOCKED;
    else if (c_bench_powered((uint8)mode) &&
        (!pitch_coordinates_valid() || !ctrl_is_finite(pitch_euler_rate()) ||
         fabsf(pitch_euler_rate()) >= CONTROL_C_POWER_START_RATE_DPS))
        reason = C_BENCH_REASON_SENSOR;
    else if (c_bench_signal((uint8)mode) && !C_BENCH_SIGNAL_OUTPUT_ALLOWED)
        reason = C_BENCH_REASON_LOCKED;
    else reason = c_bench_guard(now, 1);
    if (reason == C_BENCH_REASON_OK) {
        control_zero_sink(); /* A/B explicitly disarmed, C initially zero. */
        cascade_reset(); s_pitch_rate_cache_valid = 0;
        s_c_integral = 0.0f;
        s_c_roll_zero = g_roll_zero; s_c_pitch_zero = g_pitch_zero;
        s_c_pi_at = s_c_started = s_c_heartbeat = control_stm_ticks();
        s_c_pi_seq = s_c_imu_seq;
        s_c_bench.mode = (uint8)mode; s_c_bench.state = C_BENCH_RUNNING;
        s_c_bench.target = target; s_c_bench.kp = kp; s_c_bench.ki = ki;
        s_c_bench.duration_ms = duration_ms; s_c_bench.limit = limit;
        s_c_bench.elapsed_ms = s_c_bench.heartbeat_age_ms = s_c_bench.heartbeat_sequence = 0;
        s_c_bench.calculated = 0.0f; s_c_bench.applied = 0;
        s_c_bench.reason = C_BENCH_REASON_OK;
        result = ++s_c_bench.session;
        s_c_bench.active = 1;
        c_bench_snapshot(s_c_started);
    } else if (!s_c_bench.active) {
        s_c_bench.reason = reason; /* Active starts are immutable, even on bad retries. */
    }
    interrupt_global_enable(mask);
    return result;
}

uint8 control_c_bench_heartbeat(uint32 session, uint32 sequence)
{
    uint8 ok = 0;
    uint32 mask = interrupt_global_disable();
    uint64 now = control_stm_ticks();
    if (s_c_bench.active) {
        /* A late packet must never renew an expired lease or resurrect output.
         * Foreground freshness uses time since last loop, not ISR work time. */
        control_c_bench_reason_t reason = C_BENCH_REASON_OK;
        if (c_bench_expired(now, s_c_started, s_c_bench.duration_ms)) reason = C_BENCH_REASON_EXPIRED;
        else if (c_bench_expired(now, s_c_heartbeat, CONTROL_C_BENCH_LEASE_MS)) reason = C_BENCH_REASON_LEASE;
        else if (control_ticks_us(now - s_control_last_ticks) >= CONTROL_BENCH_MAX_GAP_US)
            reason = C_BENCH_REASON_TIMING;
        if (reason != C_BENCH_REASON_OK) c_bench_finish(reason);
        else if (session == s_c_bench.session && sequence && sequence > s_c_bench.heartbeat_sequence) {
            s_c_bench.heartbeat_sequence = sequence; s_c_heartbeat = now; ok = 1;
        }
    }
    interrupt_global_enable(mask);
    return ok;
}
void control_c_bench_stop(void)
{
    uint32 mask = interrupt_global_disable();
    if (s_c_bench.active) c_bench_finish(C_BENCH_REASON_MANUAL);
    else {
        /* STOP is an idempotent physical zero request, even if no session is
         * active. Keep the latched reason/state for post-fault diagnostics. */
        Y_Motor_Stop();
        s_c_bench.applied = 0;
        g_motor_c = 0;
    }
    interrupt_global_enable(mask);
}
uint8 control_c_bench_running(void) { return s_c_bench.active; }
void control_c_bench_status(control_c_bench_diag_t *out)
{
    uint32 mask;
    if (!out) return;
    mask = interrupt_global_disable();
    c_bench_snapshot(control_stm_ticks());
    *out = s_c_bench;
    interrupt_global_enable(mask);
}

static void control_c_bench_run(void)
{
    uint64 now = control_stm_ticks();
    control_c_bench_reason_t reason = c_bench_guard(now, 0);
    float requested;
    if (reason != C_BENCH_REASON_OK) { c_bench_finish(reason); return; }
    if (c_bench_signal(s_c_bench.mode) && !C_BENCH_SIGNAL_OUTPUT_ALLOWED) {
        c_bench_finish(C_BENCH_REASON_LOCKED); return;
    }
    requested = s_c_bench.calculated;
    if (!c_bench_rate(s_c_bench.mode)) requested = s_c_bench.target;
    else if (s_c_pi_seq != s_c_imu_seq) {
        float dt = (float)(now - s_c_pi_at) / ((float)s_control_ticks_per_ms * 1000.0f);
        float error = s_c_bench.target - pitch_euler_rate();
        if (s_c_bench.mode == C_BENCH_POWERED_RATE_NEG) error = -error;
        float step = CONTROL_C_BENCH_SLEW_PER_S * dt;
        float next_i = s_c_integral + s_c_bench.ki * error * dt;
        float raw = s_c_bench.kp * error + next_i;
        float lo = fmaxf(-(float)s_c_bench.limit, requested - step);
        float hi = fminf((float)s_c_bench.limit, requested + step);
        if (!ctrl_is_finite(dt) || dt <= 0.0f || dt >= 0.006f || !ctrl_is_finite(raw)) {
            c_bench_finish(C_BENCH_REASON_OUTPUT); return;
        }
        /* Conditional integration for both PWM saturation and slew limiting;
         * actual zero during direction deadtime also freezes signal integral. */
        if ((raw > hi && error > 0.0f) || (raw < lo && error < 0.0f) ||
            (c_bench_signal(s_c_bench.mode) && s_c_bench.applied != (int32)requested))
            next_i = s_c_integral;
        s_c_integral = constrain_float(next_i, -(float)s_c_bench.limit, (float)s_c_bench.limit);
        requested = constrain_float(s_c_bench.kp * error + s_c_integral, lo, hi);
        s_c_pi_seq = s_c_imu_seq; s_c_pi_at = now;
    }
    if (!ctrl_is_finite(requested) || fabsf(requested) > (float)s_c_bench.limit) {
        c_bench_finish(C_BENCH_REASON_OUTPUT); return;
    }
    W_Motor_KeepZero();
    /* Recheck real time after calculations/driver transport, BEFORE a C write. */
    now = control_stm_ticks();
    reason = c_bench_guard(now, 0);
    if (reason != C_BENCH_REASON_OK) { c_bench_finish(reason); return; }
    s_c_bench.calculated = requested;
    if (c_bench_signal(s_c_bench.mode))
        s_c_bench.applied = Y_Motor_BenchSetRawDuty((int32)requested);
    else { Y_Motor_Stop(); s_c_bench.applied = 0; }
    if (s_c_bench.applied > (int32)s_c_bench.limit || s_c_bench.applied < -(int32)s_c_bench.limit ||
        (s_c_bench.applied && s_c_bench.applied != (int32)requested)) {
        c_bench_finish(C_BENCH_REASON_OUTPUT); return;
    }
    start_flag = START_STOP; g_target_distance = 0;
    g_pwm_roll = g_pwm_pitch = g_pwm_yaw = 0.0f;
    g_motor_a = g_motor_b = 0; g_motor_c = (int16)s_c_bench.applied;
    c_bench_snapshot(now);
}
