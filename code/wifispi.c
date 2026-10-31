/** 移植自 MCARpreparation/project/code/wifispi.c。
 * TC264 网络驱动、JustFloat 打包及可订阅遥测。
 * 收发均为同步操作，只在主循环且开中断时调用。
 */
#include "wifispi.h"
#include "imu.h"
#if QCAR_WIFI_ENABLED
#include "IfxStm.h"
#include "zf_common_interrupt.h"
#include "zf_device_wifi_spi.h"
#include "zf_driver_delay.h"
#include "balance_control.h"
#include "small_driver_uart_control.h"
#include "wheel_encoder.h"
#include "wheel_hold.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#endif
#include <string.h>

volatile int imu_wifi_status;
volatile uint32_t imu_wifi_tx_packets, imu_wifi_init_attempts;
volatile int imu_wifi_last_error;
volatile int imu_wifi_module_result = -1;
volatile uint32_t wifi_telemetry_channel_count, wifi_telemetry_period_ms;
volatile uint32_t wifi_telemetry_commands, wifi_telemetry_command_errors;
volatile uint32_t wifi_telemetry_stream_enabled;

/* 编译期确认 float32；协议不依赖主机字节序。 */
typedef char float_must_be_32_bits[(sizeof(float)==4)?1:-1];
static void pack_float32_le(uint8_t *out,float value)
{
    uint32_t bits;
    unsigned j;
    memcpy(&bits,&value,4);
    for (j=0;j<4;++j) out[j]=(uint8_t)(bits>>(8*j));
}
size_t vofa_pack(uint8_t *out,size_t out_capacity,const float *channels,size_t count)
{
    size_t i,frame_bytes;
    if (out==NULL || channels==NULL || count==0u || count>VOFA_MAX_CHANNELS) return 0u;
    frame_bytes=VOFA_FRAME_BYTES(count);
    if (out_capacity<frame_bytes) return 0u;
    for (i=0u;i<count;++i) pack_float32_le(&out[4u*i],channels[i]);
    out[4u*count]=0x00u;
    out[4u*count+1u]=0x00u;
    out[4u*count+2u]=0x80u;
    out[4u*count+3u]=0x7fu;
    return frame_bytes;
}
#if QCAR_WIFI_ENABLED
static int wifi_ready;
static int g_announce_pending;
static void (*g_progress_hook)(void);
static int g_init_cancelled;
static uint32 g_last_wifi_tx_ticks, g_last_wifi_rx_ticks, g_ticks_per_ms;
static uint32 g_time_ticks;
static float g_telemetry_ms;

/* CPU0 owns the transport and snapshots its ISR-produced data. CPU1 never
 * accesses this module when QCAR_WIFI_ENABLED is set. */
#define TELEMETRY_VARIABLES(X) \
    X(roll_deg, imu_roll_deg) X(pitch_deg, imu_pitch_deg) X(yaw_deg, imu_yaw_deg) \
    X(accel_x_g, imu_accel_g[0]) X(accel_y_g, imu_accel_g[1]) X(accel_z_g, imu_accel_g[2]) \
    X(gyro_x_dps, imu_gyro_dps[0]) X(gyro_y_dps, imu_gyro_dps[1]) X(gyro_z_dps, imu_gyro_dps[2]) \
    X(imu_status, imu_attitude_status) X(imu_cal_percent, imu_calibration_percent) \
    X(gz_bias_dps, imu_gyro_bias_z_dps) X(imu_dt_ms, imu_sample_dt_ms) \
    X(imu_solve_us, imu_solve_us) X(imu_solve_max_us, imu_solve_max_us) \
    X(imu_accel_used, imu_accel_used) X(ekf_nis, imu_ekf_nis) \
    X(ekf_bias_x_dps, imu_ekf_bias_xy_dps[0]) X(ekf_bias_y_dps, imu_ekf_bias_xy_dps[1]) \
    X(imu_seq, imu_sample_sequence) X(imu_method, imu_attitude_method) \
    X(accel_norm_g, imu_accel_norm_g) X(gyro_z_integral_deg, imu_gyro_z_integral_deg) \
    X(telemetry_ms, g_telemetry_ms) X(rate_seq, roll_rate_test_sequence) \
    X(rate_target_dps, pitch_balance_target_rate) X(rate_actual_dps, roll_rate_test_actual) \
    X(rate_error_dps, roll_rate_test_error) X(rate_pwm, roll_rate_test_pwm) \
    X(rate_state, roll_rate_test_state) X(rate_enabled, roll_rate_test_enabled) \
    X(rate_kp, ground_rate_test_enabled?ground_rate_test_kp:roll_rate_test_kp) X(rate_sign, ground_rate_test_enabled?ground_rate_test_sign:roll_rate_test_sign) \
    X(rate_pwm_limit, ground_rate_test_enabled?ground_rate_test_limit:roll_rate_test_limit) \
    X(motor_left_pwm, QCAR_FLYWHEEL_LEFT_SIGN * costrained_flywheel_speed_left) \
    X(motor_right_pwm, QCAR_FLYWHEEL_RIGHT_SIGN * -costrained_flywheel_speed_right) \
    X(flywheel_left_speed_raw, motor_value.receive_left_speed_data) \
    X(flywheel_right_speed_raw, motor_value.receive_right_speed_data) \
    X(driver_speed_frames, small_driver_speed_frames) \
    X(driver_feedback_age_ms, small_driver_feedback_age_ms) \
    X(driver_feedback_fresh, small_driver_feedback_fresh) \
    X(balance_error_deg, pitch_balance_error) X(balance_kp, rate_only_test?0.0f:(ground_rate_test_enabled?ground_angle_kp:pitch_balance_kp)) \
    X(balance_target_deg, pitch_balance_target_angle) X(balance_rate_limit_dps, ground_rate_test_enabled?ground_angle_rate_limit:pitch_balance_rate_limit) \
    X(rate_axis, dual_balance_enabled?2:ground_rate_test_enabled) X(motor_ground_pwm, costrained_wheel_speed) \
    X(balance_actual_deg, pitch_balance_actual_angle) \
    X(balance_rate_saturated, roll_rate_test_state==2 && fabsf(pitch_balance_target_rate)>=(ground_rate_test_enabled?ground_angle_rate_limit:pitch_balance_rate_limit)) \
    X(balance_pwm_saturated, roll_rate_test_state==2 && (ground_rate_test_enabled?ground_rate_test_limit:roll_rate_test_limit)>0 && fabsf(roll_rate_test_pwm)>=(ground_rate_test_enabled?ground_rate_test_limit:roll_rate_test_limit)) \
    X(dual_enabled, dual_balance_enabled) \
    X(dual_roll_actual_deg, dual_angle[0]) \
    X(dual_roll_target_deg, dual_target[0]) \
    X(dual_roll_error_deg, dual_angle_error[0]) \
    X(dual_roll_target_rate_dps, dual_target_rate[0]) \
    X(dual_roll_rate_dps, dual_rate[0]) \
    X(dual_roll_rate_error_dps, dual_rate_error[0]) \
    X(dual_roll_pwm, dual_pwm[0]) \
    X(dual_pitch_actual_deg, dual_angle[1]) \
    X(dual_pitch_target_deg, dual_target[1]) \
    X(dual_pitch_error_deg, dual_angle_error[1]) \
    X(dual_pitch_target_rate_dps, dual_target_rate[1]) \
    X(dual_pitch_rate_dps, dual_rate[1]) \
    X(dual_pitch_rate_error_dps, dual_rate_error[1]) \
    X(dual_pitch_pwm, dual_pwm[1]) \
    X(fly_angle_kp, pitch_balance_kp) \
    X(fly_rate_kp, roll_rate_test_kp) \
    X(fly_rate_ki, roll_rate_test_ki) \
    X(fly_vel_kp, flywheel_vel_kp) \
    X(fly_vel_out, flywheel_vel_angle_out) \
    X(fly_sign, roll_rate_test_sign) \
    X(fly_rate_limit_dps, pitch_balance_rate_limit) \
    X(fly_pwm_limit, roll_rate_test_limit) \
    X(ground_angle_kp, ground_angle_kp) \
    X(ground_rate_kp, ground_rate_test_kp) \
    X(ground_sign, ground_rate_test_sign) \
    X(ground_rate_limit_dps, ground_angle_rate_limit) \
    X(ground_pwm_limit, ground_rate_test_limit) \
    X(dual_roll_rate_sat, roll_rate_test_state==2 && fabsf(dual_target_rate[0])>=pitch_balance_rate_limit) \
    X(dual_roll_pwm_sat, roll_rate_test_state==2 && roll_rate_test_limit>0 && fabsf(dual_pwm[0])>=roll_rate_test_limit) \
    X(dual_pitch_rate_sat, roll_rate_test_state==2 && fabsf(dual_target_rate[1])>=ground_angle_rate_limit) \
    X(dual_pitch_pwm_sat, roll_rate_test_state==2 && ground_rate_test_limit>0 && fabsf(dual_pwm[1])>=ground_rate_test_limit) \
    X(rate_only, rate_only_test) X(startup_scale, control_startup_scale) X(startup_ramp_ms, control_startup_ramp_ms) \
    X(fault_reason, control_fault_reason) X(fault_imu_status, control_fault_imu_status) \
    X(fault_imu_seq, control_fault_seq) X(fault_imu_dt_ms, control_fault_dt_ms) \
    X(imu_spi_timeouts, imu_spi_timeouts) X(imu_not_ready, imu_not_ready_samples) X(imu_status_reg, imu_last_status_reg) \
    X(imu_bad_bursts, imu_bad_bursts) \
    X(pwm_debug_enabled, pwm_debug_enabled) \
    X(debug_fly_left_pwm, pwm_debug_fly_left) X(debug_fly_right_pwm, pwm_debug_fly_right) \
    X(debug_ground_pwm, pwm_debug_ground) \
    X(wheel_encoder_raw, wheel_encoder_raw) X(wheel_encoder_delta, wheel_encoder_delta) \
    X(wheel_speed_cps, wheel_speed_cps) X(wheel_position_count, wheel_position_count) \
    X(wheel_encoder_seq, wheel_encoder_seq) X(wheel_encoder_dt_ms, wheel_encoder_dt_ms) \
    X(wheel_encoder_valid, wheel_encoder_valid) X(wheel_position_valid, wheel_position_valid) \
    X(wheel_encoder_errors, wheel_encoder_errors) X(wheel_encoder_sign, WHEEL_ENCODER_SIGN) \
    X(control_mode, balance_control_mode) X(wheel_hold_state, wheel_hold_state) \
    X(wheel_target_cps, wheel_target_cps) X(wheel_speed_error, wheel_speed_error) \
    X(wheel_tilt_offset, wheel_tilt_offset) X(wheel_speed_integral, wheel_speed_integral) \
    X(wheel_feedback_age_ms, wheel_feedback_age_ms) X(wheel_feedback_valid, wheel_feedback_valid) \
    X(wheel_speed_kp, wheel_speed_kp) X(wheel_speed_ki, wheel_speed_ki) \
    X(wheel_tilt_limit, wheel_tilt_limit) X(wheel_i_limit, wheel_i_limit) \
    X(wheel_ref_count, wheel_ref_count) X(wheel_pos_error, wheel_pos_error) \
    X(wheel_pos_kp, wheel_pos_kp) X(wheel_speed_limit, wheel_speed_limit) \
    X(step_state, pwm_step_state) X(step_ms, pwm_step_ms) X(step_stage, pwm_step_stage) \
    X(imu_quality, imu_quality) X(imu_reject_reason, imu_reject_reason) \
    X(imu_reject_ms, imu_reject_ms) X(imu_accept_ratio, imu_accept_ratio) \
    X(imu_reject_count, imu_reject_count) X(imu_accel_vib_g, imu_accel_vibration_g) \
    X(imu_gyro_step_dps, imu_gyro_step_dps) X(imu_r_scale, imu_r_scale) \
    X(imu_bias_learning, imu_bias_learning) X(imu_bias_slew_dps2, imu_bias_slew_dps2) \
    X(imu_gyro_clips, imu_gyro_clips) X(imu_accel_clips, imu_accel_clips) \
    X(imu_recovery_pct, imu_recovery_percent) X(imu_recoveries, imu_recovery_count) \
    X(imu_pre_gx_dps, imu_pre_gyro_dps[0]) X(imu_pre_gy_dps, imu_pre_gyro_dps[1]) X(imu_pre_gz_dps, imu_pre_gyro_dps[2]) \
    X(imu_pre_ax_g, imu_pre_accel_g[0]) X(imu_pre_ay_g, imu_pre_accel_g[1]) X(imu_pre_az_g, imu_pre_accel_g[2]) \
    X(imu_pre_norm_g, imu_pre_norm_g) X(imu_reacquiring, imu_reacquiring) \
    X(ground_deadzone_pos_pwm, ground_deadzone_pos_pwm) \
    X(ground_deadzone_neg_pwm, ground_deadzone_neg_pwm) \
    X(fly_angle_kd, pitch_balance_kd) X(fly_vel_kd, flywheel_vel_kd) \
    X(fly_angle_d_out, fly_angle_d_out) X(fly_vel_d_out, fly_vel_d_out)

#define VARIABLE_ID(name, expr) TV_##name,
enum { TELEMETRY_VARIABLES(VARIABLE_ID) TV_COUNT };
#undef VARIABLE_ID
#define VARIABLE_NAME(name, expr) #name,
static const char *const g_variable_names[]={TELEMETRY_VARIABLES(VARIABLE_NAME)};
#undef VARIABLE_NAME
typedef char variable_ids_fit_byte[(TV_COUNT<=256)?1:-1];
static uint8_t g_channels[VOFA_MAX_CHANNELS];
static char g_command[1024];
static size_t g_command_length;
static int g_command_discard;
static int g_slider_receiving;

/* A slider names one public parameter, never an address. Add mappings here
 * when another application variable should be editable from the host. */
typedef struct {
    const char *name;
    volatile float *value;
    float minimum, maximum;
} slider_parameter_t;
static const slider_parameter_t g_slider_parameters[] = {
    /* Tuning ranges match controller and Flash validation in config.h. */
    {"rate_kp", &ground_rate_test_kp, 0.0f, 500.0f},
    {"rate_pwm_limit", &ground_rate_test_limit, 0.0f, BALANCE_PWM_LIMIT_MAX},
    {"rate_sign", &ground_rate_test_sign, -1.0f, 1.0f},
    /* 模式 4 外环：角度P 与目标角速度限幅，只在 rate_only=0 时参与。 */
    {"ground_angle_kp", &ground_angle_kp, 0.0f, 100.0f},
    {"ground_deadzone_pos_pwm", &ground_deadzone_pos_pwm, 0.0f, BALANCE_PWM_LIMIT_MAX},
    {"ground_deadzone_neg_pwm", &ground_deadzone_neg_pwm, 0.0f, BALANCE_PWM_LIMIT_MAX},
    /* 模式 5 横向（飞轮，代码里是 pitch/imu_pitch_deg，屏幕标 Roll）。 */
    {"fly_angle_kp", &pitch_balance_kp, 0.0f, 1000.0f},
    {"fly_angle_kd", &pitch_balance_kd, 0.0f, 1000.0f},
    {"fly_rate_limit_dps", &pitch_balance_rate_limit, 0.1f, BALANCE_RATE_LIMIT_MAX_DPS},
    {"fly_rate_kp", &roll_rate_test_kp, 0.0f, 1000.0f},
    {"fly_rate_ki", &roll_rate_test_ki, 0.0f, 1000.0f},
    {"fly_vel_kp", &flywheel_vel_kp, -1000.0f, 1000.0f},
    {"fly_vel_kd", &flywheel_vel_kd, 0.0f, 1000.0f},
    {"fly_pwm_limit", &roll_rate_test_limit, 0.0f, BALANCE_PWM_LIMIT_MAX},
    {"fly_sign", &roll_rate_test_sign, -1.0f, 1.0f},
    /* 停车时单独开环驱动左右无刷（排查飞轮通道用，0 = 不驱动）。 */
    {"fly_test_left", &small_driver_test_left, -4000.0f, 4000.0f},
    {"fly_test_right", &small_driver_test_right, -4000.0f, 4000.0f},
    /* Mode8 raw commands, reset by Start/Stop; not saved to Flash. */
    {"debug_fly_left_pwm", &pwm_debug_fly_left, -DEBUG_PWM_LIMIT, DEBUG_PWM_LIMIT},
    {"debug_fly_right_pwm", &pwm_debug_fly_right, -DEBUG_PWM_LIMIT, DEBUG_PWM_LIMIT},
    {"debug_ground_pwm", &pwm_debug_ground, -DEBUG_PWM_LIMIT, DEBUG_PWM_LIMIT},
    {"ground_rate_limit_dps", &ground_angle_rate_limit, 0.1f, BALANCE_RATE_LIMIT_MAX_DPS},
    {"balance_angle_x", &mono_params.balance_angle_x, -20.0f, 20.0f},
    {"balance_angle_y", &mono_params.balance_angle_y, -20.0f, 20.0f},
    /* Modes 2/10; RAM tuning, not part of the existing upright Flash record. */
    {"wheel_speed_kp", &wheel_speed_kp, 0.0f, WHEEL_SPEED_GAIN_MAX},
    {"wheel_speed_ki", &wheel_speed_ki, 0.0f, WHEEL_SPEED_GAIN_MAX},
    {"wheel_tilt_limit", &wheel_tilt_limit, 0.0f, WHEEL_SPEED_TILT_MAX_DEG},
    {"wheel_i_limit", &wheel_i_limit, 0.0f, WHEEL_SPEED_TILT_MAX_DEG},
    {"wheel_pos_kp", &wheel_pos_kp, 0.0f, WHEEL_POSITION_GAIN_MAX},
    {"wheel_speed_limit", &wheel_speed_limit, 0.0f, WHEEL_POSITION_SPEED_MAX_CPS}
};

static void telemetry_reset(void)
{
    g_channels[0]=TV_roll_deg; g_channels[1]=TV_pitch_deg; g_channels[2]=TV_yaw_deg;
    wifi_telemetry_channel_count=3;
    wifi_telemetry_period_ms=QCAR_WIFI_PERIOD_MS;
    wifi_telemetry_stream_enabled=1;
    wifi_telemetry_commands=wifi_telemetry_command_errors=0;
    g_command_length=0; g_command_discard=0;
    g_slider_receiving=0;
    g_announce_pending=0;
    g_telemetry_ms=0.0f;
}

static void send_reply(const char *text)
{
    if (wifi_spi_send_buffer((const uint8 *)text,(uint32)strlen(text))!=0 ||
        wifi_spi_udp_send_now()!=0) imu_wifi_status=-4;
}
static void command_error(const char *reason)
{
    char reply[96];
    ++wifi_telemetry_command_errors;
    (void)snprintf(reply,sizeof(reply),"MCAR ERR %s\n",reason);
    send_reply(reply);
}
static char *trim(char *text)
{
    char *end;
    while (*text==' ' || *text=='\t') ++text;
    end=text+strlen(text);
    while (end>text && (end[-1]==' ' || end[-1]=='\t')) --end;
    *end='\0';
    return text;
}
static char *slider_token(char *text)
{
    size_t length;
    text=trim(text); length=strlen(text);
    if (length>=2 && text[0]=='"' && text[length-1]=='"') {
        text[length-1]='\0'; ++text;
    }
    if (*text=='\0' || strchr(text,'"')!=NULL) return NULL;
    return text;
}
/* Decimal number, optionally signed and with exponent; not a general JSON
 * parser. Strings/escapes, extra fields, NaN/Inf and hex numbers are rejected. */
static int slider_number(const char *text, float *out)
{
    const char *p=text;
    char *end;
    if (*p=='-' || *p=='+') ++p;
    if (*p<'0' || *p>'9') return 0;
    while (*p>='0' && *p<='9') ++p;
    if (*p=='.') {
        ++p;
        if (*p<'0' || *p>'9') return 0;
        while (*p>='0' && *p<='9') ++p;
    }
    if (*p=='e' || *p=='E') {
        ++p;
        if (*p=='+' || *p=='-') ++p;
        if (*p<'0' || *p>'9') return 0;
        while (*p>='0' && *p<='9') ++p;
    }
    if (*p!='\0') return 0;
    *out=strtof(text,&end);
    return *end=='\0' && isfinite(*out);
}
static void process_slider(char *command)
{
    char *kind, *name, *number, *comma;
    size_t length=strlen(command), i;
    float value;
    const slider_parameter_t *parameter=NULL;
    uint32 primask;
    char reply[96];
    if (length<2 || command[length-1]!=']') goto bad_packet;
    command[length-1]='\0'; kind=command+1;
    comma=strchr(kind,',');
    if (comma==NULL) goto bad_packet;
    *comma='\0'; name=comma+1;
    comma=strchr(name,',');
    if (comma==NULL) goto bad_packet;
    *comma='\0'; number=trim(comma+1);
    kind=slider_token(kind); name=slider_token(name);
    if (kind==NULL || name==NULL || !slider_number(number,&value)) goto bad_packet;
    if (strcmp(kind,"slider")!=0) { command_error("unknown control type"); return; }
    if (strcmp(name,"debug_step_run")==0) {
        int accepted=0;
        if (value!=0.0f && value!=1.0f) { command_error("step expects 0 or 1"); return; }
        primask=interrupt_global_disable();
        accepted=pwm_step_command((int)value);
        interrupt_global_enable(primask);
        if (!accepted) { command_error("step requires Mode8 Start, full ramp and zero commands"); return; }
        (void)snprintf(reply,sizeof(reply),"MCAR SLIDER debug_step_run %.9g\n",(double)value);
        send_reply(reply);
        return;
    }
    if (pwm_step_state==1 && (strcmp(name,"debug_fly_left_pwm")==0 ||
        strcmp(name,"debug_fly_right_pwm")==0 || strcmp(name,"debug_ground_pwm")==0)) {
        command_error("step running; cancel with debug_step_run=0 first"); return;
    }
    for (i=0;i<sizeof(g_slider_parameters)/sizeof(g_slider_parameters[0]);++i) {
        if (strcmp(name,g_slider_parameters[i].name)==0) { parameter=&g_slider_parameters[i]; break; }
    }
    if (parameter==NULL) { command_error("unknown parameter"); return; }
    if (value<parameter->minimum || value>parameter->maximum) {
        command_error("parameter out of range"); return;
    }
    primask=interrupt_global_disable();
    *parameter->value=value;
    value=*parameter->value;
    interrupt_global_enable(primask);
    (void)snprintf(reply,sizeof(reply),"MCAR SLIDER %s %.9g\n",name,(double)value);
    send_reply(reply);
    return;
bad_packet:
    command_error("expected [slider,parameter,number]");
}
static void reply_names(int all)
{
    /* Maximum names + delimiters are checked at append time. One UDP datagram. */
    static char reply[4096]; /* Main-loop only; keep metadata off the ISR-shared stack. */
    size_t used=0, i, count=all?TV_COUNT:wifi_telemetry_channel_count;
    int n=snprintf(reply,sizeof(reply),all?"MCAR LIST ":"MCAR SUB ");
    used=(size_t)n;
    for (i=0;i<count;++i) {
        n=snprintf(reply+used,sizeof(reply)-used,"%s%s",i?",":"",
                   g_variable_names[all?i:g_channels[i]]);
        if (n<0 || (size_t)n>=sizeof(reply)-used-1) {
            command_error("reply too long"); return;
        }
        used+=(size_t)n;
    }
    reply[used++]='\n'; reply[used]='\0';
    send_reply(reply);
}
static void process_command(char *line)
{
    char *command=trim(line), *arg, *end;
    uint8_t selected[VOFA_MAX_CHANNELS];
    size_t count=0, id;
    unsigned long number;
    char reply[80];
    ++wifi_telemetry_commands;
    if (*command=='[') { process_slider(command); return; }
    if (strncmp(command,"DTRACE ",7)==0) {
        unsigned index;
        char *op=command+7;
        if(strcmp(op,"CANCEL")==0)index=65533u;
        else {
            if(!pwm_debug_enabled || roll_rate_test_state!=2 || pwm_step_state==1 ||
               pwm_debug_fly_left!=0 || pwm_debug_fly_right!=0 || pwm_debug_ground!=0 ||
               costrained_flywheel_speed_left || costrained_flywheel_speed_right || costrained_wheel_speed) {
                command_error("DTRACE requires started zero-output Mode8");return;
            }
            if(strcmp(op,"ARM")==0)index=65534u;
            else if(strcmp(op,"META")==0)index=65535u;
            else {
                int event=strncmp(op,"EVENT ",6)==0;
                if(!event && strncmp(op,"ROW ",4)!=0) {command_error("DTRACE ARM/CANCEL/META/EVENT n/ROW n");return;}
                op+=event?6:4;
                if(*op<'0' || *op>'9') {command_error("DTRACE index");return;}
                errno=0;number=strtoul(op,&end,10);
                if(errno || *end || number>=(event?16u:8192u)) {command_error("DTRACE index");return;}
                index=(unsigned)number+(event?60000u:0u);
            }
        }
        if(!small_driver_trace_request(index)) {command_error("DTRACE pending reply or request");return;}
        send_reply("MCAR DTRACE QUEUED\n");return;
    }
    if (strcmp(command,"LIST?")==0) { reply_names(1); return; }
    if (strcmp(command,"GET?")==0) {
        reply_names(0);
        (void)snprintf(reply,sizeof(reply),"MCAR RATE %lu\n",(unsigned long)wifi_telemetry_period_ms);
        send_reply(reply);
        (void)snprintf(reply,sizeof(reply),"MCAR STREAM %lu\n",(unsigned long)wifi_telemetry_stream_enabled);
        send_reply(reply); return;
    }
    arg=strpbrk(command," \t");
    if (arg==NULL) { command_error("expected SUB/RATE/STREAM/LIST?/GET?"); return; }
    *arg++='\0'; arg=trim(arg);
    if (strcmp(command,"SUB")==0) {
        /* Validate the entire list before replacing anything. Empty fields,
         * duplicate names, unknown names and >VOFA_MAX_CHANNELS channels leave the old list intact. */
        while (1) {
            char *comma=strchr(arg,',');
            size_t j;
            if (comma!=NULL) *comma='\0';
            arg=trim(arg);
            if (*arg=='\0' || count==VOFA_MAX_CHANNELS) { command_error("invalid channel count"); return; }
            for (id=0;id<TV_COUNT;++id) if (strcmp(arg,g_variable_names[id])==0) break;
            if (id==TV_COUNT) { command_error("unknown variable"); return; }
            for (j=0;j<count;++j) if (selected[j]==id) { command_error("duplicate variable"); return; }
            selected[count++]=(uint8_t)id;
            if (comma==NULL) break;
            arg=comma+1;
        }
        memcpy(g_channels,selected,count);
        wifi_telemetry_channel_count=(uint32_t)count;
        reply_names(0); return;
    }
    if (*arg<'0' || *arg>'9') { command_error("expected unsigned integer"); return; }
    errno=0;
    number=strtoul(arg,&end,10);
    if (errno!=0 || *end!='\0') { command_error("invalid integer"); return; }
    if (strcmp(command,"RATE")==0 && number>=2 && number<=1000) {
        wifi_telemetry_period_ms=(uint32_t)number;
        (void)snprintf(reply,sizeof(reply),"MCAR RATE %lu\n",number);
    } else if (strcmp(command,"STREAM")==0 && number<=1) {
        wifi_telemetry_stream_enabled=(uint32_t)number;
        (void)snprintf(reply,sizeof(reply),"MCAR STREAM %lu\n",number);
    } else { command_error("RATE 2..1000 or STREAM 0/1"); return; }
    send_reply(reply);
}
static void receive_commands(void)
{
    uint8_t bytes[256];
    uint32 i, count=wifi_spi_read_buffer(bytes,sizeof(bytes));
    for (i=0;i<count;++i) {
        uint8_t ch=bytes[i];
        if (ch=='[' && !g_slider_receiving && !g_command_discard) {
            g_command_length=0; g_slider_receiving=1;
        }
        if (ch=='\n' || (g_slider_receiving && ch==']')) {
            if (ch==']' && !g_command_discard) {
                if (g_command_length==sizeof(g_command)-1) g_command_discard=1;
                else g_command[g_command_length++]=(char)ch;
            }
            if (g_command_discard) {
                ++wifi_telemetry_commands;
                command_error("invalid or oversized line");
            }
            else if (g_command_length!=0) {
                g_command[g_command_length]='\0'; process_command(g_command);
            }
            g_command_length=0; g_command_discard=0; g_slider_receiving=0;
        } else if (ch=='\r') {
            /* Accept both LF and CRLF from the existing host text sender. */
        } else if (g_command_discard) {
            /* Recover only at the next newline. */
        } else if ((ch<32 && ch!='\t') || ch>126 || g_command_length==sizeof(g_command)-1) {
            g_command_discard=1;
        } else g_command[g_command_length++]=(char)ch;
    }
}
static size_t capture_channels(float channels[VOFA_MAX_CHANNELS])
{
    size_t i, count=wifi_telemetry_channel_count;
    uint32 primask=interrupt_global_disable();
    for (i=0;i<count;++i) {
        switch (g_channels[i]) {
#define VARIABLE_VALUE(name, expr) case TV_##name: channels[i]=(float)(expr); break;
            TELEMETRY_VARIABLES(VARIABLE_VALUE)
#undef VARIABLE_VALUE
            default: channels[i]=0.0f; break;
        }
    }
    interrupt_global_enable(primask);
    return count;
}
static int wifi_connect_stage(int stage)
{
    imu_wifi_status=stage;
    if (g_progress_hook) g_progress_hook();
    return !g_init_cancelled;
}
static int imu_wifi_init_once(void)
{
    int result;
    memset(wifi_spi_version,0,sizeof(wifi_spi_version));
    memset(wifi_spi_ip_addr_port,0,sizeof(wifi_spi_ip_addr_port));
    if (!wifi_connect_stage(2)) return -6;
    imu_wifi_module_result=-1;
    result=wifi_spi_init(0,0);
    imu_wifi_module_result=result;
    if (g_init_cancelled) return -6;
    if (result!=0 || strncmp(wifi_spi_version,"V2",2)!=0) return -1;
    if (!wifi_connect_stage(3)) return -6;
    result=wifi_spi_wifi_connect(QCAR_WIFI_SSID,QCAR_WIFI_PASSWORD);
    if (g_init_cancelled) return -6;
    if (result!=0) return -2;
    if (!wifi_connect_stage(4)) return -6;
    result=wifi_spi_socket_connect("UDP",QCAR_WIFI_TARGET_IP,
                                  QCAR_WIFI_TARGET_PORT,QCAR_WIFI_LOCAL_PORT);
    if (g_init_cancelled) return -6;
    return result!=0 ? -3 : 1;
}
#endif
void wifispi_set_progress_hook(void (*hook)(void))
{
#if QCAR_WIFI_ENABLED
    g_progress_hook=hook;
#else
    (void)hook;
#endif
}
void wifispi_cancel_init(void)
{
#if QCAR_WIFI_ENABLED
    g_init_cancelled=1;
    wifi_spi_cancel_wait();
#endif
}
int wifispi_init(void)
{
#if QCAR_WIFI_ENABLED
    uint32_t attempt;
    int result;
    wifi_ready=0; g_init_cancelled=0;
    wifi_spi_set_wait_callback(g_progress_hook);
    imu_wifi_status=0; imu_wifi_init_attempts=0; imu_wifi_last_error=0;
    imu_wifi_module_result=-1;
    /* 保留原初始化统计语义：启用模式重新初始化不清发送累计数。 */
    system_delay_ms(QCAR_WIFI_STARTUP_DELAY_MS);
    for (attempt=0;attempt<QCAR_WIFI_INIT_ATTEMPTS;++attempt) {
        if (attempt!=0) system_delay_ms(QCAR_WIFI_RETRY_DELAY_MS);
        ++imu_wifi_init_attempts;
        result=imu_wifi_init_once();
        if (result==1) { wifi_ready=1; imu_wifi_status=1; wifi_spi_set_wait_callback(NULL); return 1; }
        imu_wifi_last_error=result; imu_wifi_status=result;
        if (g_progress_hook) g_progress_hook();
        if (g_init_cancelled) break;
    }
    wifi_spi_set_wait_callback(NULL);
    return 0;
#else
    /* 禁用时绝不初始化 SPI/GPIO，防止覆盖电机引脚复用。 */
    imu_wifi_status=-5; imu_wifi_init_attempts=0;
    imu_wifi_last_error=0; imu_wifi_tx_packets=0;
    return 0;
#endif
}
int wifispi_send_floats(const float *channels,size_t count)
{
#if QCAR_WIFI_ENABLED
    uint8_t packet[VOFA_MAX_FRAME_BYTES];
    size_t packet_bytes;
    if (!wifi_ready) return 0;
    packet_bytes=vofa_pack(packet,sizeof(packet),channels,count);
    if (packet_bytes==0u) return 0;
    /* send_buffer 返回未发送字节数；send_now 结束本 UDP 数据报。 */
    if (wifi_spi_send_buffer(packet,(uint32_t)packet_bytes)!=0 ||
        wifi_spi_udp_send_now()!=0) { imu_wifi_status=-4; return 0; }
    imu_wifi_status=1; ++imu_wifi_tx_packets;
    return 1;
#else
    (void)channels; (void)count;
    return 0;
#endif
}
int wifispi_send_imu(const float angles_deg[3])
{
    return wifispi_send_floats(angles_deg,3u);
}
void wifispi_telemetry_init(void)
{
#if QCAR_WIFI_ENABLED
    telemetry_reset();
#endif
    (void)wifispi_init();
#if QCAR_WIFI_ENABLED
    g_ticks_per_ms=(uint32)(IfxStm_getFrequency(&MODULE_STM0)/1000.0f);
    g_last_wifi_tx_ticks=g_last_wifi_rx_ticks=g_time_ticks=IfxStm_getLower(&MODULE_STM0);
#endif
}
void wifispi_telemetry_service(void)
{
#if QCAR_WIFI_ENABLED
    uint32 now;
    float channels[VOFA_MAX_CHANNELS];
    size_t count;
    if (!wifi_ready) return;
    if (g_announce_pending) { g_announce_pending=0; wifispi_telemetry_announce(); }
    now=IfxStm_getLower(&MODULE_STM0);
    g_telemetry_ms+=(float)(uint32)(now-g_time_ticks)/(float)g_ticks_per_ms;
    g_time_ticks=now;
    /* Poll independently of streaming so STREAM 0 and slow RATE cannot lock
     * the host out. Limit SPI polling to 100 Hz in the main loop. */
    if ((uint32)(now-g_last_wifi_rx_ticks)>=g_ticks_per_ms*10u) {
        g_last_wifi_rx_ticks=now; receive_commands();
    }
    {
        static char line[1600];
        int n=small_driver_trace_take(line+11,sizeof(line)-12u);
        if(n>0) {
            memcpy(line,"MCAR TRACE ",11);line[11+n]='\n';line[12+n]='\0';
            send_reply(line);
        }
    }
    now=IfxStm_getLower(&MODULE_STM0);
    if (!wifi_telemetry_stream_enabled ||
        (uint32)(now-g_last_wifi_tx_ticks)<g_ticks_per_ms*wifi_telemetry_period_ms) return;
    g_last_wifi_tx_ticks=now;
    count=capture_channels(channels);
    (void)wifispi_send_floats(channels,count);
#endif
}

void wifispi_telemetry_angle_profile(void)
{
#if QCAR_WIFI_ENABLED
    static const uint8_t channels[]={TV_telemetry_ms,TV_rate_seq,TV_imu_seq,TV_imu_status,TV_rate_state,TV_rate_enabled,TV_balance_target_deg,TV_balance_actual_deg,TV_balance_error_deg,TV_rate_target_dps,TV_rate_actual_dps,TV_rate_error_dps,TV_rate_pwm,TV_motor_left_pwm,TV_motor_right_pwm,TV_balance_kp,TV_rate_kp,TV_rate_sign,TV_balance_rate_limit_dps,TV_rate_pwm_limit,TV_balance_rate_saturated,TV_balance_pwm_saturated,TV_flywheel_left_speed_raw,TV_flywheel_right_speed_raw,TV_driver_speed_frames,TV_imu_dt_ms,TV_roll_deg,TV_pitch_deg};
    memcpy(g_channels,channels,sizeof(channels));
    wifi_telemetry_channel_count=sizeof(channels)/sizeof(channels[0]);
    wifi_telemetry_period_ms=APP_TELEMETRY_PROFILE_MS;
    wifi_telemetry_stream_enabled=1;
    g_announce_pending=1;
#endif
}

void wifispi_telemetry_startup_profile(void)
{
#if QCAR_WIFI_ENABLED
    static const uint8_t channels[]={TV_telemetry_ms,TV_rate_seq,TV_imu_seq,TV_imu_status,TV_rate_state,TV_rate_enabled,TV_balance_target_deg,TV_balance_actual_deg,TV_balance_error_deg,TV_rate_target_dps,TV_rate_actual_dps,TV_rate_error_dps,TV_rate_pwm,TV_motor_left_pwm,TV_motor_right_pwm,TV_balance_kp,TV_rate_kp,TV_rate_sign,TV_balance_rate_limit_dps,TV_rate_pwm_limit,TV_balance_rate_saturated,TV_balance_pwm_saturated,TV_imu_dt_ms,TV_rate_axis,TV_rate_only,TV_startup_scale,TV_startup_ramp_ms,TV_fault_reason,TV_fault_imu_status,TV_fault_imu_seq,TV_fault_imu_dt_ms,TV_imu_spi_timeouts,TV_imu_not_ready,TV_imu_status_reg,TV_flywheel_left_speed_raw,TV_flywheel_right_speed_raw,TV_driver_speed_frames,TV_driver_feedback_age_ms,TV_driver_feedback_fresh};
    memcpy(g_channels,channels,sizeof(channels));
    wifi_telemetry_channel_count=sizeof(channels);wifi_telemetry_period_ms=APP_TELEMETRY_PROFILE_MS;
    wifi_telemetry_stream_enabled=1;g_announce_pending=1;
#endif
}

void wifispi_telemetry_dual_profile(void)
{
#if QCAR_WIFI_ENABLED
    static const uint8_t channels[]={TV_telemetry_ms,TV_rate_seq,TV_imu_seq,TV_imu_status,TV_rate_state,TV_dual_enabled,TV_dual_roll_actual_deg,TV_dual_roll_target_deg,TV_dual_roll_error_deg,TV_dual_roll_target_rate_dps,TV_dual_roll_rate_dps,TV_dual_roll_rate_error_dps,TV_dual_roll_pwm,TV_dual_pitch_actual_deg,TV_dual_pitch_target_deg,TV_dual_pitch_error_deg,TV_dual_pitch_target_rate_dps,TV_dual_pitch_rate_dps,TV_dual_pitch_rate_error_dps,TV_dual_pitch_pwm,TV_fly_angle_kp,TV_fly_rate_kp,TV_fly_sign,TV_fly_rate_limit_dps,TV_fly_pwm_limit,TV_ground_angle_kp,TV_ground_rate_kp,TV_ground_sign,TV_ground_rate_limit_dps,TV_ground_pwm_limit,TV_motor_left_pwm,TV_motor_right_pwm,TV_motor_ground_pwm,TV_imu_dt_ms,TV_fault_reason,TV_fault_imu_status,TV_fault_imu_dt_ms,TV_imu_spi_timeouts,TV_imu_not_ready,TV_startup_scale,TV_flywheel_left_speed_raw,TV_flywheel_right_speed_raw,TV_driver_speed_frames,TV_driver_feedback_age_ms,TV_driver_feedback_fresh};
    memcpy(g_channels,channels,sizeof(channels));
    wifi_telemetry_channel_count=sizeof(channels)/sizeof(channels[0]);wifi_telemetry_period_ms=APP_TELEMETRY_PROFILE_MS;
    wifi_telemetry_stream_enabled=1;g_announce_pending=1;
#endif
}

void wifispi_telemetry_roll_profile(void)
{
#if QCAR_WIFI_ENABLED
    static const uint8_t channels[]={TV_dual_roll_actual_deg,TV_dual_roll_target_deg,
        TV_dual_roll_target_rate_dps,TV_dual_roll_rate_dps,TV_dual_roll_pwm,
        TV_motor_left_pwm,TV_motor_right_pwm};
    memcpy(g_channels,channels,sizeof(channels));
    wifi_telemetry_channel_count=sizeof(channels)/sizeof(channels[0]);
    wifi_telemetry_period_ms=APP_TELEMETRY_PROFILE_MS;
    wifi_telemetry_stream_enabled=1;g_announce_pending=1;
#endif
}

void wifispi_telemetry_speed_profile(void)
{
#if QCAR_WIFI_ENABLED
    static const uint8_t channels[]={TV_control_mode,TV_telemetry_ms,TV_imu_status,TV_rate_state,TV_fault_reason,TV_startup_scale,TV_rate_seq,TV_wheel_hold_state,TV_wheel_speed_cps,TV_wheel_target_cps,TV_wheel_speed_error,TV_wheel_tilt_offset,TV_wheel_speed_integral,TV_wheel_feedback_age_ms,TV_wheel_feedback_valid,TV_wheel_encoder_seq,TV_wheel_encoder_dt_ms,TV_wheel_encoder_valid,TV_wheel_position_count,TV_wheel_speed_kp,TV_wheel_speed_ki,TV_wheel_tilt_limit,TV_wheel_i_limit,TV_dual_pitch_actual_deg,TV_dual_pitch_target_deg,TV_dual_pitch_target_rate_dps,TV_dual_pitch_rate_dps,TV_dual_pitch_pwm,TV_dual_roll_actual_deg,TV_dual_roll_target_deg,TV_dual_roll_target_rate_dps,TV_dual_roll_rate_dps,TV_dual_roll_pwm,TV_motor_left_pwm,TV_motor_right_pwm,TV_motor_ground_pwm,TV_flywheel_left_speed_raw,TV_flywheel_right_speed_raw,TV_driver_feedback_fresh,TV_ground_angle_kp,TV_ground_rate_kp,TV_ground_rate_limit_dps,TV_ground_pwm_limit};
    memcpy(g_channels,channels,sizeof(channels));
    wifi_telemetry_channel_count=sizeof(channels)/sizeof(channels[0]);
    wifi_telemetry_period_ms=APP_TELEMETRY_PROFILE_MS;
    wifi_telemetry_stream_enabled=1;g_announce_pending=1;
#endif
}

void wifispi_telemetry_hold_profile(void)
{
#if QCAR_WIFI_ENABLED
    static const uint8_t channels[]={TV_control_mode,TV_telemetry_ms,TV_imu_status,TV_rate_state,TV_fault_reason,TV_startup_scale,TV_rate_seq,TV_wheel_hold_state,TV_wheel_speed_cps,TV_wheel_target_cps,TV_wheel_speed_error,TV_wheel_tilt_offset,TV_wheel_speed_integral,TV_wheel_feedback_age_ms,TV_wheel_feedback_valid,TV_wheel_encoder_seq,TV_wheel_encoder_dt_ms,TV_wheel_encoder_valid,TV_wheel_position_count,TV_wheel_speed_kp,TV_wheel_speed_ki,TV_wheel_tilt_limit,TV_wheel_i_limit,TV_dual_pitch_actual_deg,TV_dual_pitch_target_deg,TV_dual_pitch_target_rate_dps,TV_dual_pitch_rate_dps,TV_dual_pitch_pwm,TV_dual_roll_actual_deg,TV_dual_roll_target_deg,TV_dual_roll_target_rate_dps,TV_dual_roll_rate_dps,TV_dual_roll_pwm,TV_motor_left_pwm,TV_motor_right_pwm,TV_motor_ground_pwm,TV_flywheel_left_speed_raw,TV_flywheel_right_speed_raw,TV_driver_feedback_fresh,TV_ground_angle_kp,TV_ground_rate_kp,TV_ground_rate_limit_dps,TV_ground_pwm_limit,TV_wheel_ref_count,TV_wheel_pos_error,TV_wheel_pos_kp,TV_wheel_speed_limit,TV_wheel_position_valid};
    memcpy(g_channels,channels,sizeof(channels));
    wifi_telemetry_channel_count=sizeof(channels)/sizeof(channels[0]);
    wifi_telemetry_period_ms=APP_TELEMETRY_PROFILE_MS;
    wifi_telemetry_stream_enabled=1;g_announce_pending=1;
#endif
}

void wifispi_telemetry_announce(void)
{
#if QCAR_WIFI_ENABLED
    char reply[80];
    if (!wifi_ready) return;
    reply_names(0);
    (void)snprintf(reply,sizeof(reply),"MCAR RATE %lu\n",(unsigned long)wifi_telemetry_period_ms);
    send_reply(reply);
    (void)snprintf(reply,sizeof(reply),"MCAR STREAM %lu\n",(unsigned long)wifi_telemetry_stream_enabled);
    send_reply(reply);
#endif
}

void wifispi_telemetry_stop(void)
{
#if QCAR_WIFI_ENABLED
    int was_ready=wifi_ready;
    wifi_ready=0;
    wifi_telemetry_stream_enabled=0;
    if (was_ready && wifi_spi_socket_disconnect()!=0) { imu_wifi_status=-4; return; }
#endif
    imu_wifi_status=0;
}
int wifispi_is_ready(void)
{
#if QCAR_WIFI_ENABLED
    return wifi_ready;
#else
    return 0;
#endif
}
