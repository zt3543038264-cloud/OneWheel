/* CPU0 only. The volatile phase hands ownership of SPI/estimator state
 * between foreground and the 1 ms ISR; never initialize from the ISR. */
#if defined(__TASKING__)
#pragma section farbss "cpu1_dsram"
#pragma section fardata "cpu1_dsram"
#endif
static volatile control_imu_recovery_t s_imu_recovery = CTRL_IMU_IDLE;
static uint32 s_imu_phase_ms;
#if defined(__TASKING__)
#pragma section fardata restore
#pragma section farbss restore
#endif

control_imu_recovery_t control_imu_recovery_state(void) { return s_imu_recovery; }

uint8 control_imu_recovery_busy(void)
{
    return (uint8)(s_imu_recovery >= CTRL_IMU_INIT &&
                   s_imu_recovery <= CTRL_IMU_CONVERGING);
}

uint8 control_imu_recovery_needed(void)
{
    /* An ordinary startup convergence wait or tilt/BLDC protection is not
     * an IMU restart request. Coordinate changes still require SAVE/REBOOT. */
    return (uint8)(!g_imu_ok || imu_link_lost() ||
        imu_calib_state() != IMU_CALIB_OK || attitude_diverged());
}

static void control_imu_recovery_fail(control_test_status_t reason)
{
    g_imu_ok = 0;
    s_test_status = reason;
    s_imu_recovery = CTRL_IMU_FAILED;
}

uint8 control_imu_recovery_request(void)
{
    uint32 irq = interrupt_global_disable();
    if (control_imu_recovery_busy() || !control_imu_recovery_needed() ||
        imu_forward_pending() || start_flag != START_STOP || s_test_running ||
        s_jog_target != MOTOR_JOG_NONE || path_nav_busy()) {
        interrupt_global_enable(irq);
        return 0;
    }
    control_stop();
    g_imu_ok = 0;                   /* ISR relinquishes sensor ownership. */
    s_pitch_rate_cache_valid = 0;
    path_nav_invalidate_heading(); /* Reinitialization resets continuous yaw. */
    s_imu_phase_ms = g_control_uptime_ms;
    s_imu_recovery = CTRL_IMU_INIT;
    interrupt_global_enable(irq);
    return 1;
}

/* Called exactly once per interrupt before the ordinary IMU update. */
static void control_imu_recovery_tick(void)
{
    if (s_imu_recovery != CTRL_IMU_CALIBRATING) return;
    imu_calibrate_tick();
    if (!imu_calibrate_busy()) {
        if (imu_link_lost()) control_imu_recovery_fail(CTRL_TEST_STATUS_IMU_LOST);
        else if (imu_calib_state() != IMU_CALIB_OK)
            control_imu_recovery_fail(CTRL_TEST_STATUS_IMU_CALIB);
        else s_imu_recovery = CTRL_IMU_ALIGN;
    }
}

void control_imu_recovery_poll(void)
{
    switch (s_imu_recovery) {
    case CTRL_IMU_INIT:
        /* Only the hardware settle delay blocks (~150 ms); interrupts,
         * zero-output heartbeats, keys and byte transport continue. */
        if (imu_init() != 0) {
            control_imu_recovery_fail(CTRL_TEST_STATUS_IMU_FAIL);
            break;
        }
        imu_calibrate_begin();
        s_imu_phase_ms = g_control_uptime_ms;
        s_imu_recovery = CTRL_IMU_CALIBRATING;
        break;
    case CTRL_IMU_CALIBRATING:
        if ((uint32)(g_control_uptime_ms-s_imu_phase_ms) > 20000u)
            control_imu_recovery_fail(CTRL_TEST_STATUS_IMU_CALIB);
        break;
    case CTRL_IMU_ALIGN:
        /* The ISR still has g_imu_ok=0 and cannot touch estimator state. */
        attitude_init();
        if (attitude_diverged()) {
            control_imu_recovery_fail(CTRL_TEST_STATUS_ATT_DIVERGED);
            break;
        }
        s_imu_phase_ms = g_control_uptime_ms;
        s_imu_recovery = CTRL_IMU_CONVERGING;
        g_imu_ok = 1;               /* Resume real 1 kHz samples, motors gated. */
        break;
    case CTRL_IMU_CONVERGING:
        if (imu_link_lost()) control_imu_recovery_fail(CTRL_TEST_STATUS_IMU_LOST);
        else if (attitude_diverged() ||
                 (uint32)(g_control_uptime_ms-s_imu_phase_ms) > 5000u)
            control_imu_recovery_fail(CTRL_TEST_STATUS_ATT_DIVERGED);
        else if (attitude_converged()) {
            g_yaw_target = imu_get_angle_yaw();
            s_test_status = CTRL_TEST_STATUS_OK;
            s_imu_recovery = CTRL_IMU_READY;
            /* No start call here. STOP and the flywheel brake stay latched. */
        }
        break;
    default: break;
    }
}
