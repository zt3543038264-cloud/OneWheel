/* Local attended, raw C PWM. No timed/remote renewal and no automatic restart.
 * Back/Stop/fault ends it. This is not a current/thermal limiter or hardware watchdog. */
int control_manual_raw_running(void) { return s_manual_raw_dir; }
void control_manual_raw_stop(void) { control_stop(); }
uint8 control_manual_raw_start(int direction)
{
    uint32 irq=interrupt_global_disable();
    control_test_status_t reason=CTRL_TEST_STATUS_OK;
    if(!VEHICLE_MOTOR_OUTPUT_ALLOWED) reason=CTRL_TEST_STATUS_ADAPT_LOCKED;
    else if((direction!=1 && direction!=-1) || JOG_DUTY_DRIVE<0 || JOG_DUTY_DRIVE>Y_MOTOR_MANUAL_RAW_LIMIT) reason=CTRL_TEST_STATUS_INVALID;
    else if(start_flag!=START_STOP || s_manual_raw_dir || control_c_bench_running() ||
            s_preview.active || s_test_running || s_jog_target!=MOTOR_JOG_NONE ||
            W_Motor_BenchBusy() || W_Motor_OrdinaryPending() || path_nav_busy()) reason=CTRL_TEST_STATUS_BUSY;
    else if(!control_clock_ready() || !s_control_timing.valid ||
            s_control_timing.period_us>=CONTROL_BENCH_MAX_GAP_US ||
            s_control_timing.work_us>=CONTROL_BENCH_MAX_WORK_US) reason=CTRL_TEST_STATUS_SAFETY;
    else reason=control_sensor_reason();
    if(reason!=CTRL_TEST_STATUS_OK){s_test_status=reason;interrupt_global_enable(irq);return 0;}
    control_zero_sink();cascade_reset();
    s_manual_raw_dir=direction;start_flag=START_DRIVE_ONLY;s_test_status=CTRL_TEST_STATUS_OK;
    interrupt_global_enable(irq);return 1;
}
static void control_manual_raw_run(void)
{
    int32 amplitude=JOG_DUTY_DRIVE,command,applied;
    control_test_status_t reason=control_sensor_reason();
    uint64 now=control_stm_ticks();
    if(start_flag!=START_DRIVE_ONLY || !VEHICLE_MOTOR_OUTPUT_ALLOWED ||
       (s_manual_raw_dir!=1 && s_manual_raw_dir!=-1) ||
       amplitude<0 || amplitude>Y_MOTOR_MANUAL_RAW_LIMIT) reason=CTRL_TEST_STATUS_INVALID;
    if(!s_control_timing.valid || s_control_timing.period_us>=CONTROL_BENCH_MAX_GAP_US ||
       !s_c_imu_seen || c_bench_expired(now,s_c_imu_seen_at,CONTROL_C_BENCH_IMU_MAX_AGE_MS)) reason=CTRL_TEST_STATUS_SAFETY;
    if(reason!=CTRL_TEST_STATUS_OK){control_stop();s_test_status=reason;return;}
    W_Motor_Stop(); /* no flywheel actuation in this mode */
    command=amplitude*s_manual_raw_dir;
    applied=Y_Motor_ManualRawDuty(command);
    g_pwm_roll=g_pwm_yaw=0;g_motor_a=g_motor_b=0;
    g_pwm_pitch=(float)command;g_motor_c=(int16)applied;
}
