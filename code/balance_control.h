#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H
#include "zf_common_headfile.h"
extern uint8_t shacheflag;
extern Mono_params mono_params;
extern int16 costrained_flywheel_speed_left,costrained_flywheel_speed_right,costrained_wheel_speed;
extern float cascade_angle_out,cascade_angle_out_p,cascade_angle_vel_out,cascade_angle_vel_out_p;
float constrain_float(float v,float lo,float hi);
void balance_control_defaults(void);
void balance_control_tick_1ms(void);
void clear_all_cascade_out(void);
/* Flywheel Pitch/Y angle P -> rate P, configured 2ms update; angle KP=0 gives rate-only.
 * roll_rate_test_* names retained for Flash/host compatibility. */
extern volatile uint8 roll_rate_test_enabled;
extern volatile uint32 roll_rate_test_sequence;
extern volatile int32 roll_rate_test_state;
extern volatile float roll_rate_test_actual, roll_rate_test_error, roll_rate_test_pwm;
extern volatile float pitch_balance_target_rate, pitch_balance_error;
extern volatile float pitch_balance_actual_angle, pitch_balance_target_angle;
extern float pitch_balance_kp, pitch_balance_rate_limit, flywheel_vel_kp;
extern float pitch_balance_kd, flywheel_vel_kd; /* Mode10 measurement derivatives, seconds-based. */
extern volatile float fly_angle_d_out, fly_vel_d_out;
/* Mode10 flywheel speed loop: driver feedback (set by control ISR) and angle offset output. */
extern volatile int16 flywheel_speed_feedback_left, flywheel_speed_feedback_right;
extern volatile uint32 flywheel_speed_feedback_sequence;
extern volatile float flywheel_vel_angle_out;
extern float roll_rate_test_kp, roll_rate_test_ki, roll_rate_test_limit, roll_rate_test_sign;
extern volatile uint8 ground_rate_test_enabled;
extern float ground_rate_test_kp, ground_rate_test_limit, ground_rate_test_sign;
extern float ground_deadzone_pos_pwm, ground_deadzone_neg_pwm; /* nonnegative duty offsets */
extern uint8 ground_deadzone_enable;   /* 0 disables the feed-forward */
extern float ground_angle_kp, ground_angle_rate_limit;
extern volatile uint8 dual_balance_enabled;
extern volatile uint8 balance_control_mode; /* Armed mode, 0 after Stop. */
extern volatile float dual_angle[2],dual_target[2],dual_angle_error[2];
extern volatile float dual_rate[2],dual_target_rate[2],dual_rate_error[2],dual_pwm[2];
extern volatile uint8 rate_only_test;
extern volatile uint8 pwm_debug_enabled;
extern volatile float pwm_debug_fly_left, pwm_debug_fly_right, pwm_debug_ground;
void pwm_debug_arm(void);
/* Fixed left-only bench sequence. 0 idle, 1 running, 2 complete, -1 aborted. */
extern volatile int32 pwm_step_state;
extern volatile uint32 pwm_step_ms, pwm_step_stage;
int pwm_step_command(int run);
extern volatile int32 control_fault_reason,control_fault_imu_status;
extern volatile uint32 control_fault_seq;
extern volatile float control_fault_dt_ms,control_startup_scale;
extern uint32 control_startup_ramp_ms;
void roll_rate_only_arm(void);
void ground_rate_only_arm(void);
void dual_balance_arm(void);
void mode10_balance_arm(void); /* Roll incremental cascade + ground four-loop hold. */
void speed_balance_arm(void); /* Mode2: dual attitude + ground position/speed loops. */
void ground_rate_test_arm(void);
void roll_rate_test_arm(void);
void roll_rate_test_stop(void);

#endif
