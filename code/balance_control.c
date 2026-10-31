/* Upright controller, extracted from upstream pid.c (GPL-3.0-or-later).
 * Modes 3..7 retain gains/signs; IMU preprocessing owns all feedback filtering.
 * Mode 8 remains raw PWM.
 */
#include "balance_control.h"
#include "pid.h"
#include "wheel_hold.h"

uint8_t shacheflag;
int16 costrained_flywheel_speed_left, costrained_flywheel_speed_right, costrained_wheel_speed;
float cascade_angle_out,cascade_angle_out_p,cascade_angle_vel_out,cascade_angle_vel_out_p;
Mono_params mono_params;
// 函数简介：将输入值限制在指定的上下限范围内。
// 参数说明：v 为待限制的输入值；lo 为下限；hi 为上限。
// 返回值：返回限制后的数值。
float constrain_float(float v,float lo,float hi) { return v<lo?lo:v>hi?hi:v; }
/* One private PID instance per axis and loop; no shared error history. */
static pid_controller_t angle_pid[2],rate_pid[2];
typedef struct { float last_error, last_derivative, out; } mode10_inc_pid_t;
static mode10_inc_pid_t mode10_roll_rate_pid;
typedef struct { float speed, derivative, elapsed; uint32 sequence; uint8 initialized; } mode10_speed_d_t;
static mode10_speed_d_t mode10_speed_d;
static void mode10_derivatives_reset(void)
{
    mode10_speed_d=(mode10_speed_d_t){0};
    fly_angle_d_out=fly_vel_d_out=0.0f;
}
// 函数简介：获取控制器使用的固定采样周期，单位为秒。
// 参数说明：无。
// 返回值：返回控制周期，单位为秒。
static float controller_dt_s(void) {return (float)APP_IMU_PERIOD_US*0.000001f;}
// 函数简介：复位两个控制轴的角度环、角速度环和飞轮滤波器。
// 参数说明：无。
// 返回值：无。
static void balance_pid_reset(void)
{
    unsigned i;for(i=0;i<2;++i){pid_reset(&angle_pid[i]);pid_reset(&rate_pid[i]);}
    mode10_roll_rate_pid.last_error=mode10_roll_rate_pid.last_derivative=mode10_roll_rate_pid.out=0.0f;
    mode10_derivatives_reset();
}
// 函数简介：执行指定控制轴的姿态角环 PID，生成目标角速度。
// 参数说明：axis 为控制轴索引；target 为目标角度；angle 为当前角度；
//           kp 为角度环比例参数；rate_limit 为目标角速度限幅。
// 返回值：返回角度环输出的目标角速度。
static float balance_angle_pid(unsigned axis,float target,float angle,float kp,float rate_limit)
{
    pid_controller_t *p=&angle_pid[axis];
    p->kp=kp;p->ki=axis?GROUND_ANGLE_KI:FLY_ANGLE_KI;p->kd=axis?GROUND_ANGLE_KD:FLY_ANGLE_KD;
    p->integral_limit=BALANCE_ANGLE_I_LIMIT_DPS;
    p->output_min=-rate_limit;p->output_max=rate_limit;
#if BALANCE_ANGLE_PID_INCREMENTAL
    return pid_incremental_update(p,target,angle,controller_dt_s());
#else
    return pid_position_update(p,target,angle,controller_dt_s());
#endif
}
// 函数简介：执行指定控制轴的角速度环 PID，生成带方向的 PWM 输出。
// 参数说明：axis 为控制轴索引；target 为目标角速度；rate 为当前角速度；
//           kp 为角速度环比例参数；pwm_limit 为 PWM 限幅；
//           sign 为输出方向符号，只允许为 1 或 -1。
// 返回值：返回带方向且经过限幅的 PWM 输出。
static float balance_rate_pid(unsigned axis,float target,float rate,float kp,float pwm_limit,float sign,float angle_ff)
{
    float output;
    pid_controller_t *p=&rate_pid[axis];
    p->kp=kp;p->ki=axis?GROUND_RATE_KI:FLY_RATE_KI;p->kd=axis?GROUND_RATE_KD:FLY_RATE_KD;
    p->integral_limit=BALANCE_RATE_I_LIMIT_PWM;
    p->output_min=-pwm_limit;p->output_max=pwm_limit;
#if BALANCE_RATE_PID_INCREMENTAL
    output=pid_incremental_update(p,target,rate,controller_dt_s());
#else
    output=pid_position_update(p,target,rate,controller_dt_s());
#endif
    /* Same signed actuator path for PID and angle compensation; the combined
     * command must obey the startup-scaled PWM limit, including with Kp=0. */
    return sign*constrain_float(output+angle_ff,-pwm_limit,pwm_limit);
}

static float mode10_flywheel_velocity_pid(float vel_kp,float vel_kd,float left_speed,float right_speed,uint32 sequence)
{
    float real_speed=(left_speed-right_speed)*0.005f;
    float err=0.0f-real_speed;
    float output;
    if (!isfinite(vel_kp) || !isfinite(vel_kd) || vel_kd<0 || !isfinite(err)) {
        mode10_derivatives_reset();return 0.0f;
    }
    mode10_speed_d.elapsed+=controller_dt_s();
    if (!mode10_speed_d.initialized) {
        mode10_speed_d.speed=real_speed;mode10_speed_d.sequence=sequence;
        mode10_speed_d.derivative=mode10_speed_d.elapsed=0;
        mode10_speed_d.initialized=sequence!=0;
    } else if (sequence!=mode10_speed_d.sequence) {
        float dt=mode10_speed_d.elapsed;
        if (dt>0 && dt<=(float)QCAR_DRIVER_FEEDBACK_TIMEOUT_MS*.001f) {
            float derivative=-(real_speed-mode10_speed_d.speed)/dt;
            float alpha=dt/(FLY_VEL_D_LPF_TAU_S+dt);
            mode10_speed_d.derivative+=alpha*(derivative-mode10_speed_d.derivative);
        } else mode10_speed_d.derivative=0;
        mode10_speed_d.speed=real_speed;mode10_speed_d.sequence=sequence;
        mode10_speed_d.elapsed=0;
    }
    if (mode10_speed_d.elapsed>(float)QCAR_DRIVER_FEEDBACK_TIMEOUT_MS*.001f)
        mode10_speed_d.derivative=0;
    fly_vel_d_out=vel_kd*mode10_speed_d.derivative;
    output=vel_kp*err+fly_vel_d_out;
    if (!isfinite(output)) {mode10_derivatives_reset();return 0.0f;}
    return constrain_float(output,-FLY_VEL_ANGLE_LIMIT_DEG,FLY_VEL_ANGLE_LIMIT_DEG);
}

static float mode10_roll_rate_incremental_pid(float target,float rate,float kp,float ki,float pwm_limit,float sign)
{
    float err=target-rate,deriv,delta;
    if (!isfinite(err) || !isfinite(kp) || !isfinite(ki) || !isfinite(pwm_limit) || pwm_limit<0.0f) {
        mode10_roll_rate_pid.last_error=mode10_roll_rate_pid.last_derivative=mode10_roll_rate_pid.out=0.0f;
        return 0.0f;
    }
    deriv=err-mode10_roll_rate_pid.last_error;
    delta=kp*deriv+ki*err+FLY_RATE_KD*(deriv-mode10_roll_rate_pid.last_derivative);
    mode10_roll_rate_pid.last_error=err;
    mode10_roll_rate_pid.last_derivative=deriv;
    mode10_roll_rate_pid.out+=delta;
    if (!isfinite(mode10_roll_rate_pid.out)) {
        mode10_roll_rate_pid.last_error=mode10_roll_rate_pid.last_derivative=mode10_roll_rate_pid.out=0.0f;
        return 0.0f;
    }
    mode10_roll_rate_pid.out=constrain_float(mode10_roll_rate_pid.out,-pwm_limit,pwm_limit);
    return sign*mode10_roll_rate_pid.out;
}

/* States: 0 off, 1 calibration, 2 running;
 * -1 IMU failure/stale, -2 tilt, -3 invalid parameter. Faults latch until Start. */
volatile uint8 roll_rate_test_enabled;
volatile uint8 ground_rate_test_enabled;
volatile uint8 dual_balance_enabled;
volatile uint8 balance_control_mode;
volatile uint8 rate_only_test;
volatile uint8 pwm_debug_enabled;
volatile float pwm_debug_fly_left, pwm_debug_fly_right, pwm_debug_ground;
volatile int32 pwm_step_state;
volatile uint32 pwm_step_ms, pwm_step_stage;
static uint32 pwm_step_next_ms;

/* Called with interrupts disabled by the WiFi parser. No remote Start. */
int pwm_step_command(int run)
{
    if (!run) {
        pwm_step_state=-1;
        pwm_debug_fly_left=pwm_debug_fly_right=pwm_debug_ground=0.0f;
        return 1;
    }
    if (!pwm_debug_enabled || !roll_rate_test_enabled || roll_rate_test_state!=2 ||
        !shacheflag || control_startup_scale<1.0f ||
        !isfinite(DEBUG_PWM_LIMIT) || DEBUG_PWM_LIMIT<600.0f || DEBUG_PWM_LIMIT>4000.0f)
        return 0;
    if (pwm_step_state==1) return 1; /* Duplicate UDP trigger never restarts. */
    if (pwm_debug_fly_left!=0 || pwm_debug_fly_right!=0 || pwm_debug_ground!=0)
        return 0;
    pwm_step_ms=pwm_step_stage=pwm_step_next_ms=0;
    pwm_step_state=1;
    return 1;
}

static void pwm_step_update(void)
{
    uint32 t;
    if (pwm_step_state!=1) return;
    if (!pwm_debug_enabled || roll_rate_test_state!=2 || !shacheflag ||
        control_startup_scale<1.0f) {
        pwm_step_state=-1;
        pwm_debug_fly_left=pwm_debug_fly_right=pwm_debug_ground=0.0f;
        return;
    }
    t=pwm_step_next_ms;
    pwm_step_ms=t;
    pwm_debug_fly_right=pwm_debug_ground=0.0f;
    if (t<1000u) { pwm_step_stage=1; pwm_debug_fly_left=0; }
    else if (t<2500u) { pwm_step_stage=2; pwm_debug_fly_left=300; }
    else if (t<4000u) { pwm_step_stage=3; pwm_debug_fly_left=600; }
    else if (t<5500u) { pwm_step_stage=4; pwm_debug_fly_left=300; }
    else { pwm_step_stage=5; pwm_debug_fly_left=0; }
    if (t>=7500u) pwm_step_state=2;
    else pwm_step_next_ms+=APP_CONTROL_TICK_US*APP_CONTROL_DIVIDER/1000u;
}
volatile int32 control_fault_reason,control_fault_imu_status;
volatile uint32 control_fault_seq;
volatile float control_fault_dt_ms,control_startup_scale;
uint32 control_startup_ramp_ms=BALANCE_STARTUP_RAMP_MS;
static uint32 control_startup_elapsed_ms;

/* Index 0: displayed Roll/flywheel; index 1: displayed Pitch/ground wheel. */
volatile float dual_angle[2],dual_target[2],dual_angle_error[2];
volatile float dual_rate[2],dual_target_rate[2],dual_rate_error[2],dual_pwm[2];

/* Compiled defaults live in config.h; valid upright Flash values may override. */
float ground_rate_test_kp=GROUND_RATE_KP, ground_rate_test_limit=GROUND_PWM_LIMIT, ground_rate_test_sign=GROUND_OUTPUT_SIGN;
float ground_angle_kp=GROUND_ANGLE_KP, ground_angle_rate_limit=GROUND_RATE_LIMIT_DPS;
/* Static-friction feed-forward for the geared travel wheel (see config.h). */
float ground_deadzone_pos_pwm=GROUND_DEADZONE_POS_PWM;
float ground_deadzone_neg_pwm=GROUND_DEADZONE_NEG_PWM;
uint8 ground_deadzone_enable=1;
volatile uint32 roll_rate_test_sequence;
volatile int32 roll_rate_test_state;
volatile float roll_rate_test_actual, roll_rate_test_error, roll_rate_test_pwm;
volatile float pitch_balance_target_rate, pitch_balance_error;
volatile float pitch_balance_actual_angle, pitch_balance_target_angle;
float pitch_balance_kp = FLY_ANGLE_KP;
float flywheel_vel_kp = FLY_VEL_KP;
float pitch_balance_kd = FLY_ANGLE_KD, flywheel_vel_kd = FLY_VEL_KD;
volatile float fly_angle_d_out, fly_vel_d_out;
/* Flywheel speed feedback copied from the driver UART by the control ISR (user/isr.c). */
volatile int16 flywheel_speed_feedback_left, flywheel_speed_feedback_right;
volatile uint32 flywheel_speed_feedback_sequence;
volatile float flywheel_vel_angle_out;
float pitch_balance_rate_limit = FLY_RATE_LIMIT_DPS;
float roll_rate_test_kp = FLY_RATE_KP;
float roll_rate_test_ki = FLY_RATE_KI;
float roll_rate_test_limit = FLY_PWM_LIMIT;
float roll_rate_test_sign = FLY_OUTPUT_SIGN;
static uint8 rate_divider;
static uint32 rate_stale_ms, rate_sequence;
// 函数简介：停止飞轮角速度测试并清除测试状态和输出。
// 参数说明：无。
// 返回值：无。
void roll_rate_test_stop(void)
{
    balance_pid_reset();
    flywheel_vel_angle_out=0.0f;
    wheel_hold_enabled=0;wheel_hold_reset();balance_control_mode=0;
    roll_rate_test_enabled = 0;
    ground_rate_test_enabled = 0;
    dual_balance_enabled=0;rate_only_test=0;
    pwm_debug_enabled=0;
    pwm_step_state=0;pwm_step_ms=pwm_step_stage=pwm_step_next_ms=0;
    pwm_debug_fly_left=pwm_debug_fly_right=pwm_debug_ground=0.0f;
    control_fault_reason=control_fault_imu_status=0;control_fault_seq=0;
    control_fault_dt_ms=control_startup_scale=0;control_startup_elapsed_ms=0;
    for (unsigned i=0;i<2;i++) {
        dual_angle[i]=dual_target[i]=dual_angle_error[i]=0;
        dual_rate[i]=dual_target_rate[i]=dual_rate_error[i]=dual_pwm[i]=0;
    }
    roll_rate_test_state = 0;
    roll_rate_test_actual = roll_rate_test_error = roll_rate_test_pwm = 0.0f;
    pitch_balance_target_rate = pitch_balance_error = 0.0f;
    pitch_balance_actual_angle = pitch_balance_target_angle = 0.0f;
    rate_divider = 0;
    rate_stale_ms = 0;
}

// 函数简介：启动飞轮角速度测试。
// 参数说明：无。
// 返回值：无。
void roll_rate_test_arm(void)
{
    roll_rate_test_stop();
    rate_sequence = imu_sample_sequence;
    /* 角度低通以当前姿态为初值，避免起跑瞬间的大误差。 */
    roll_rate_test_state = 1;
    roll_rate_test_enabled = 1;
    balance_control_mode=3;
}

// 函数简介：周期性更新飞轮角速度测试控制量并驱动电机。
// 参数说明：无。
// 返回值：无。
// 函数简介：启动地轮角速度测试模式。
// 参数说明：无。
// 返回值：无。
void ground_rate_test_arm(void)
{
    roll_rate_test_arm();
    ground_rate_test_enabled=1;
    balance_control_mode=4;
}

// 函数简介：启动仅角速度反馈的飞轮测试模式。
// 参数说明：无。
// 返回值：无。
void roll_rate_only_arm(void) {roll_rate_test_arm();rate_only_test=1;balance_control_mode=6;}
// 函数简介：启动仅角速度反馈的地轮测试模式。
// 参数说明：无。
// 返回值：无。
void ground_rate_only_arm(void) {ground_rate_test_arm();rate_only_test=1;balance_control_mode=7;}
// 函数简介：锁存控制故障并记录故障发生时的控制状态。
// 参数说明：state 为故障状态码；reason 为故障原因码。
// 返回值：无。
static void control_latch_fault(int32 state,int32 reason)
{
    uint32 irq=interrupt_global_disable();
    roll_rate_test_state=state;control_fault_reason=reason;
    control_fault_imu_status=imu_attitude_status;control_fault_seq=imu_sample_sequence;
    control_fault_dt_ms=imu_sample_dt_ms;
    interrupt_global_enable(irq);
}
// 函数简介：推进启动缓升计时并计算当前 PWM 输出缩放系数。
// 参数说明：无。
// 返回值：无。
static void control_startup_advance(void)
{
    if(control_startup_elapsed_ms<5000u)control_startup_elapsed_ms+=APP_IMU_PERIOD_US/1000u;
    control_startup_scale=control_startup_ramp_ms==0?1.0f:
        constrain_float((float)control_startup_elapsed_ms/(float)control_startup_ramp_ms,0.0f,1.0f);
}

// 函数简介：启动飞轮和地轮双轴平衡控制模式。
// 参数说明：无。
// 返回值：无。
void dual_balance_arm(void)
{
    ground_rate_test_arm();
    dual_balance_enabled=1;
    balance_control_mode=5;
}

void mode10_balance_arm(void)
{
    /* Reuse the ground position/speed cascade and capture a new session origin. */
    speed_balance_arm();
    balance_control_mode=10;
}

void speed_balance_arm(void)
{
    dual_balance_arm();
    (void)wheel_encoder_rebase_position();
    wheel_hold_enabled=1;balance_control_mode=2;
}

/* Bench mode: direct signed PWM, no attitude controller or IMU interlock.
 * Arm/Stop zero all three commands. Only the control ISR owns active outputs. */
// 函数简介：启动三路 PWM 直接调试模式。
// 参数说明：无。
// 返回值：无。
void pwm_debug_arm(void)
{
    roll_rate_test_stop();
    pwm_debug_enabled=1;
    roll_rate_test_enabled=1;
    roll_rate_test_state=2;
    balance_control_mode=8;
}

// 函数简介：读取三路调试 PWM，限幅后直接驱动飞轮和地轮。
// 参数说明：无。
// 返回值：无。
static void pwm_debug_update(void)
{
    float left,right,ground;
    pwm_step_update();
    int output[3]={0,0,0};
    uint32 irq=interrupt_global_disable();
    left=pwm_debug_fly_left;right=pwm_debug_fly_right;ground=pwm_debug_ground;
    interrupt_global_enable(irq);
    if (roll_rate_test_state==2) {
        if (!shacheflag) roll_rate_test_state=0;
        else if (!isfinite(left) || !isfinite(right) || !isfinite(ground) ||
                 !isfinite(DEBUG_PWM_LIMIT) || DEBUG_PWM_LIMIT<0.0f || DEBUG_PWM_LIMIT>4000.0f ||
                 fabsf(left)>DEBUG_PWM_LIMIT || fabsf(right)>DEBUG_PWM_LIMIT ||
                 fabsf(ground)>DEBUG_PWM_LIMIT || control_startup_ramp_ms>5000u)
            control_latch_fault(-3,6);
        else {
            control_startup_advance();
            float limit=DEBUG_PWM_LIMIT*control_startup_scale;
            output[0]=(int)roundf(constrain_float(left,-limit,limit));
            output[1]=(int)roundf(constrain_float(right,-limit,limit));
            output[2]=(int)roundf(constrain_float(ground,-limit,limit));
        }
    }
    for(unsigned i=0;i<2;i++) {
        dual_target[i]=dual_angle_error[i]=0.0f;
        dual_target_rate[i]=dual_rate_error[i]=0.0f;
    }
    /* Compatibility channels: Roll PWM is LEFT only in Mode8, Pitch is ground.
     * motor_left_pwm/motor_right_pwm expose the actual independent commands. */
    dual_pwm[0]=(float)output[0];dual_pwm[1]=(float)output[2];
    /* Optional measurements remain visible; they never gate raw PWM. */
    dual_angle[0]=imu_pitch_deg;dual_angle[1]=imu_roll_deg;
    dual_rate[0]=attitude_state.gyro.y;dual_rate[1]=attitude_state.gyro.x;
    roll_rate_test_pwm=(float)output[2];
    cascade_angle_out=cascade_angle_out_p=0.0f;
    cascade_angle_vel_out=(float)output[0];cascade_angle_vel_out_p=(float)output[2];
    costrained_flywheel_speed_left=(int16)output[0];
    /* Preserve the legacy internal right-sign convention used by telemetry. */
    costrained_flywheel_speed_right=(int16)-output[1];
    costrained_wheel_speed=(int16)output[2];
    small_driver_set_duty((int16)output[0],(int16)output[1]);
    motor_m_ctrl(output[2]);
}

typedef struct {
    float angle,rate,target,angle_kp,rate_limit,rate_kp,rate_ki,pwm_limit,sign;
} balance_axis_snapshot;
// 函数简介：检查单个平衡轴的目标、PID 参数和输出限幅是否有效。
// 参数说明：a 为待检查的平衡轴参数快照指针。
// 返回值：参数有效返回 1，否则返回 0。
static int dual_axis_parameters_valid(const balance_axis_snapshot *a)
{
    return isfinite(a->target) && fabsf(a->target)<=20 &&
        isfinite(a->angle_kp) && a->angle_kp>=0 &&
        isfinite(a->rate_kp) && a->rate_kp>=0 &&
        isfinite(a->rate_ki) && a->rate_ki>=0 &&
        isfinite(a->rate_limit) && a->rate_limit>0 && a->rate_limit<=BALANCE_RATE_LIMIT_MAX_DPS &&
        isfinite(a->pwm_limit) && a->pwm_limit>=0 && a->pwm_limit<=BALANCE_PWM_LIMIT_MAX &&
        (a->sign==1 || a->sign==-1);
}
/* Static-friction feed-forward: the geared travel wheel needs roughly
 * direction-dependent duty before it breaks away, so a nonzero closed-loop
 * command is offset in its final signed direction. The offset follows the startup ramp
 * (no kick when Start finishes ramping) and stays inside the same limit the
 * PID already used. Mode8/menu bench paths never call this. */
static int ground_deadzone_shape(int pwm, float limit)
{
    float magnitude, offset, cap, compensation;
    if (!ground_deadzone_enable || pwm == 0) return pwm;
    compensation=pwm>0?ground_deadzone_pos_pwm:ground_deadzone_neg_pwm;
    if (!isfinite(compensation) || compensation <= 0.0f || compensation>BALANCE_PWM_LIMIT_MAX) return pwm;
    if (!isfinite(limit) || limit <= 0.0f) return pwm;
    offset = compensation * control_startup_scale;
    cap = limit * control_startup_scale;
    magnitude = (float)(pwm > 0 ? pwm : -pwm) + offset;
    magnitude = constrain_float(magnitude, 0.0f, cap);
    return (int)(pwm > 0 ? magnitude : -magnitude);
}

// 函数简介：执行飞轮和地轮双轴串级平衡控制并输出电机 PWM。
// 参数说明：无。
// 返回值：无。
static void dual_balance_update(void)
{
    balance_axis_snapshot a[2];
    float vel_kp,vel_kd,angle_kd,fly_left_speed,fly_right_speed;
    uint32 seq,irq,fly_sequence; int32 status;
    int output[2]={0,0},ground_output;
    irq=interrupt_global_disable();
    a[0].angle=imu_pitch_deg; a[0].rate=attitude_state.gyro.y;
    a[0].target=mono_params.balance_angle_y;
    a[0].angle_kp=pitch_balance_kp; a[0].rate_limit=pitch_balance_rate_limit;
    a[0].rate_kp=roll_rate_test_kp; a[0].rate_ki=roll_rate_test_ki; a[0].pwm_limit=roll_rate_test_limit; a[0].sign=roll_rate_test_sign;
    a[1].angle=imu_roll_deg; a[1].rate=attitude_state.gyro.x;
    a[1].target=mono_params.balance_angle_x;
    a[1].angle_kp=ground_angle_kp; a[1].rate_limit=ground_angle_rate_limit;
    a[1].rate_kp=ground_rate_test_kp; a[1].rate_ki=GROUND_RATE_KI; a[1].pwm_limit=ground_rate_test_limit; a[1].sign=ground_rate_test_sign;
    seq=imu_sample_sequence; status=imu_attitude_status;
    vel_kp=flywheel_vel_kp;
    vel_kd=flywheel_vel_kd;angle_kd=pitch_balance_kd;
    fly_sequence=flywheel_speed_feedback_sequence;
    fly_left_speed=(float)flywheel_speed_feedback_left;
    fly_right_speed=(float)flywheel_speed_feedback_right;
    interrupt_global_enable(irq);
    for(unsigned i=0;i<2;i++) {
        dual_angle[i]=a[i].angle;dual_rate[i]=a[i].rate;dual_target[i]=a[i].target;
        dual_angle_error[i]=a[i].target-a[i].angle;
        dual_target_rate[i]=dual_rate_error[i]=dual_pwm[i]=0;
    }
    if (roll_rate_test_state==1 || roll_rate_test_state==2) {
        if (!shacheflag) roll_rate_test_state=0;
        else if (status<0) control_latch_fault(-1,1);
        else if (status==IMU_ATTITUDE_CALIBRATING) {
            roll_rate_test_state=1;rate_stale_ms=0;rate_sequence=seq;
        } else if (!seq || !isfinite(imu_roll_deg) || !isfinite(imu_pitch_deg) || !isfinite(a[0].angle) || !isfinite(a[1].angle) ||
                   !isfinite(a[0].rate) || !isfinite(a[1].rate)) control_latch_fault(-1,seq?3:2);
        else if (fabsf(imu_pitch_deg)>BALANCE_TILT_LIMIT_DEG || fabsf(imu_roll_deg)>BALANCE_TILT_LIMIT_DEG) control_latch_fault(-2,5);
        else if (!dual_axis_parameters_valid(&a[0]) || !dual_axis_parameters_valid(&a[1]) ||
                 !isfinite(FLY_ANGLE_FF_KP) || FLY_ANGLE_FF_KP<0.0f || FLY_ANGLE_FF_KP>1000.0f ||
                 !isfinite(vel_kp) ||
                 (balance_control_mode==10u && (!isfinite(vel_kd) || vel_kd<0 ||
                    !isfinite(angle_kd) || angle_kd<0)) ||
                 control_startup_ramp_ms>5000u)
            control_latch_fault(-3,6);
        else {
            if (seq==rate_sequence) rate_stale_ms+=APP_IMU_PERIOD_US/1000u;
            else {rate_stale_ms=0;rate_sequence=seq;}
            if (rate_stale_ms>=BALANCE_IMU_STALE_MS) control_latch_fault(-1,4);
            else {
                int ready=1;
                if (wheel_hold_enabled) {
                    wheel_encoder_sample_t encoder;
                    int result;
                    wheel_encoder_snapshot(&encoder);
                    result=wheel_hold_update(&encoder,control_startup_scale<1.0f ||
                        a[1].angle_kp<=0.0f || a[1].rate_kp<=0.0f ||
                        fabsf((float)costrained_wheel_speed)>=a[1].pwm_limit*control_startup_scale-0.5f);
                    if (result<0) {control_latch_fault(result==-9?-3:-4,-result);ready=0;}
                    else if (!result) {roll_rate_test_state=1;ready=0;}
                    else {
                        a[1].target+=wheel_tilt_offset;
                        if (!isfinite(a[1].target) || fabsf(a[1].target)>20.0f) {
                            control_latch_fault(-3,9);ready=0;
                        }
                        dual_target[1]=a[1].target;
                        dual_angle_error[1]=a[1].target-a[1].angle;
                    }
                }
                if (ready) {roll_rate_test_state=2;control_startup_advance();}
                for(unsigned i=0;ready && i<2;i++) {
                    if (i==0u) {
                        dual_rate[i]=a[i].rate;
                    }
                    if (i==0u && balance_control_mode==10u) {
                        flywheel_vel_angle_out=mode10_flywheel_velocity_pid(vel_kp,vel_kd,fly_left_speed,fly_right_speed,fly_sequence);
                        float roll_target=constrain_float(a[0].target+flywheel_vel_angle_out,-20.0f,20.0f);
                        dual_target[0]=roll_target;
                        dual_angle_error[0]=roll_target-a[0].angle;
                        fly_angle_d_out=-angle_kd*a[0].rate;
                        dual_target_rate[0]=constrain_float(a[0].angle_kp*dual_angle_error[0]+fly_angle_d_out,-a[0].rate_limit,a[0].rate_limit);
                        dual_rate_error[0]=dual_target_rate[0]-a[0].rate;
                        output[0]=(int)roundf(mode10_roll_rate_incremental_pid(dual_target_rate[0],a[0].rate,a[0].rate_kp,a[0].rate_ki,a[0].pwm_limit*control_startup_scale,a[0].sign));
                        dual_pwm[0]=(float)output[0];
                        continue;
                    }
                    dual_target_rate[i]=balance_angle_pid(i,a[i].target,a[i].angle,a[i].angle_kp,a[i].rate_limit);
                    dual_rate_error[i]=dual_target_rate[i]-a[i].rate;
                    output[i]=(int)roundf(balance_rate_pid(i,dual_target_rate[i],a[i].rate,a[i].rate_kp,a[i].pwm_limit*control_startup_scale,a[i].sign,
                        i==0u && a[i].angle_kp>0.0f ? FLY_ANGLE_FF_KP*dual_angle_error[i] : 0.0f));
                    dual_pwm[i]=(float)output[i];
                }
            }
        }
    }
    ground_output=ground_deadzone_shape(output[1], a[1].pwm_limit);
    if (wheel_hold_enabled && roll_rate_test_state!=2) wheel_hold_suspend();
    /* Generic test channels continue to describe the ground/Pitch axis. */
    pitch_balance_actual_angle=dual_angle[1];pitch_balance_target_angle=dual_target[1];
    pitch_balance_error=dual_angle_error[1];pitch_balance_target_rate=dual_target_rate[1];
    roll_rate_test_actual=dual_rate[1];roll_rate_test_error=dual_rate_error[1];roll_rate_test_pwm=dual_pwm[1];
    cascade_angle_out=dual_target_rate[0];cascade_angle_out_p=dual_target_rate[1];
    cascade_angle_vel_out=(float)output[0];cascade_angle_vel_out_p=(float)output[1];
    costrained_flywheel_speed_left=costrained_flywheel_speed_right=(int16)output[0];
    costrained_wheel_speed=(int16)ground_output;
    small_driver_set_duty((int16)output[0],(int16)(-output[0]));
    motor_m_ctrl(ground_output);
}

// 函数简介：周期性执行单轴角度/角速度控制、测试模式和电机输出。
// 参数说明：无。
// 返回值：无。
static void roll_rate_test_update(void)
{
    float rate, roll, pitch, kp, limit, direction;
    float angle_kp, rate_limit, balance_angle;
    uint32 sequence, irq;
    int32 status;
    int output = 0, ground_output = 0;
    if (++rate_divider < APP_CONTROL_DIVIDER) return;
    rate_divider = 0;
    ++roll_rate_test_sequence;
    if (pwm_debug_enabled) { pwm_debug_update(); return; }
    if (dual_balance_enabled) { dual_balance_update(); return; }
    irq = interrupt_global_disable();
    rate = ground_rate_test_enabled ? attitude_state.gyro.x : attitude_state.gyro.y;
    roll = imu_roll_deg;
    pitch = imu_pitch_deg;
    sequence = imu_sample_sequence;
    status = imu_attitude_status;
    kp = ground_rate_test_enabled ? ground_rate_test_kp : roll_rate_test_kp;
    limit = ground_rate_test_enabled ? ground_rate_test_limit : roll_rate_test_limit;
    direction = ground_rate_test_enabled ? ground_rate_test_sign : roll_rate_test_sign;
    angle_kp = rate_only_test ? 0.0f : (ground_rate_test_enabled ? ground_angle_kp : pitch_balance_kp);
    rate_limit = ground_rate_test_enabled ? ground_angle_rate_limit : pitch_balance_rate_limit;
    balance_angle = rate_only_test ? 0.0f : (ground_rate_test_enabled ? mono_params.balance_angle_x : mono_params.balance_angle_y);
    interrupt_global_enable(irq);
    roll_rate_test_actual = rate;
    pitch_balance_actual_angle = ground_rate_test_enabled ? roll : pitch;
    pitch_balance_target_angle = balance_angle;
    pitch_balance_target_rate = pitch_balance_error = 0.0f;
    roll_rate_test_error = -rate;
    if (roll_rate_test_state == 1 || roll_rate_test_state == 2) {
        if (!shacheflag) roll_rate_test_state = 0;
        else if (status < 0) control_latch_fault(-1,1);
        else if (status == IMU_ATTITUDE_CALIBRATING) {
            /* Calibration never drives either flywheel. */
            roll_rate_test_state = 1;
            rate_stale_ms = 0;
            rate_sequence = sequence;
        } else if (!sequence || !isfinite(rate) || !isfinite(roll) || !isfinite(pitch))
            control_latch_fault(-1,sequence?3:2);
        /* Display labels are swapped: internal pitch is the tested Roll. */
        else if (fabsf(ground_rate_test_enabled ? roll : pitch) > BALANCE_TILT_LIMIT_DEG)
            control_latch_fault(-2,5);
        else if (!isfinite(kp) || kp < 0.0f ||
                 !isfinite(limit) || limit < 0.0f || limit > BALANCE_PWM_LIMIT_MAX ||
                 (direction != 1.0f && direction != -1.0f) ||
                 !isfinite(angle_kp) || angle_kp < 0.0f ||
                 !isfinite(rate_limit) || rate_limit <= 0.0f || rate_limit > BALANCE_RATE_LIMIT_MAX_DPS ||
                 !isfinite(balance_angle) || fabsf(balance_angle) > 20.0f ||
                 (!ground_rate_test_enabled && !rate_only_test &&
                  (!isfinite(FLY_ANGLE_FF_KP) || FLY_ANGLE_FF_KP<0.0f || FLY_ANGLE_FF_KP>1000.0f)) ||
                 control_startup_ramp_ms>5000u)
            control_latch_fault(-3,6);
        else {
            if (sequence == rate_sequence) rate_stale_ms += APP_IMU_PERIOD_US/1000u;
            else { rate_stale_ms = 0; rate_sequence = sequence; }
            if (rate_stale_ms >= BALANCE_IMU_STALE_MS) control_latch_fault(-1,4);
            else {
                roll_rate_test_state = 2;
                control_startup_advance();
                if (!ground_rate_test_enabled) {
                    roll_rate_test_actual=rate;
                }
                /* 索引与双轴路径一致：0=pitch(飞轮)，1=roll(地轮)。 */
                pitch_balance_error = balance_angle - (ground_rate_test_enabled ? roll : pitch);
                pitch_balance_target_rate = rate_only_test ? 0.0f : balance_angle_pid(ground_rate_test_enabled?1u:0u,balance_angle,balance_angle-pitch_balance_error,angle_kp,rate_limit);
                roll_rate_test_error = pitch_balance_target_rate - rate;
                output = (int)roundf(balance_rate_pid(ground_rate_test_enabled?1u:0u,pitch_balance_target_rate,rate,kp,limit*control_startup_scale,direction,
                    !ground_rate_test_enabled && !rate_only_test && angle_kp>0.0f ? FLY_ANGLE_FF_KP*pitch_balance_error : 0.0f));
            }
        }
    }
    roll_rate_test_pwm = (float)output;
    cascade_angle_out = ground_rate_test_enabled ? 0.0f : pitch_balance_target_rate;
    cascade_angle_vel_out = ground_rate_test_enabled ? 0.0f : (float)output;
    cascade_angle_vel_out_p = ground_rate_test_enabled ? (float)output : 0.0f;
    costrained_flywheel_speed_left = ground_rate_test_enabled ? 0 : (int16)output;
    costrained_flywheel_speed_right = ground_rate_test_enabled ? 0 : (int16)output;
    if (ground_rate_test_enabled) ground_output = ground_deadzone_shape(output, ground_rate_test_limit);
    costrained_wheel_speed = ground_rate_test_enabled ? (int16)ground_output : 0;
    small_driver_set_duty(costrained_flywheel_speed_left,-costrained_flywheel_speed_right);
    motor_m_ctrl(ground_output);
}

// 函数简介：以 1 ms 周期分发当前控制模式并更新电机输出。
// 参数说明：无。
// 返回值：无。
void balance_control_tick_1ms(void)
{
    if (roll_rate_test_enabled) roll_rate_test_update();
    else { small_driver_set_duty(0,0); motor_m_ctrl(0); }
}
// 函数简介：清除级联控制输出和对外暴露的电机速度命令。
// 参数说明：无。
// 返回值：无。
void clear_all_cascade_out(void)
{
    mode10_derivatives_reset();
    cascade_angle_out=cascade_angle_out_p=0;
    flywheel_vel_angle_out=0.0f;
    mode10_roll_rate_pid.last_error=mode10_roll_rate_pid.last_derivative=mode10_roll_rate_pid.out=0.0f;
    cascade_angle_vel_out=cascade_angle_vel_out_p=0;
    costrained_flywheel_speed_left=costrained_flywheel_speed_right=costrained_wheel_speed=0;
}
// 函数简介：恢复平衡控制器、测试参数和启动状态的编译默认值。
// 参数说明：无。
// 返回值：无。
void balance_control_defaults(void)
{
    roll_rate_test_stop(); clear_all_cascade_out();
    wheel_hold_defaults();
    mono_params.balance_angle_x=BALANCE_ZERO_X_DEG;
    mono_params.balance_angle_y=BALANCE_ZERO_Y_DEG;
    roll_rate_test_kp=FLY_RATE_KP;roll_rate_test_ki=FLY_RATE_KI;roll_rate_test_limit=FLY_PWM_LIMIT;roll_rate_test_sign=FLY_OUTPUT_SIGN;
    pitch_balance_kp=FLY_ANGLE_KP;pitch_balance_rate_limit=FLY_RATE_LIMIT_DPS;flywheel_vel_kp=FLY_VEL_KP;
    pitch_balance_kd=FLY_ANGLE_KD;flywheel_vel_kd=FLY_VEL_KD;
    ground_rate_test_kp=GROUND_RATE_KP;ground_rate_test_limit=GROUND_PWM_LIMIT;ground_rate_test_sign=GROUND_OUTPUT_SIGN;
    ground_angle_kp=GROUND_ANGLE_KP;ground_angle_rate_limit=GROUND_RATE_LIMIT_DPS;
    control_startup_ramp_ms=BALANCE_STARTUP_RAMP_MS;
}
