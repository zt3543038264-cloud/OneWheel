"""Mode10 integration: real four-loop ground cascade and independent Roll path."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
boundary = r'''
#ifndef HOST_BOUNDARY
#define HOST_BOUNDARY
#include <stdint.h>
#include <math.h>
#include "config.h"
#include "qcar_types.h"
typedef uint8_t uint8;typedef uint32_t uint32;typedef int32_t int32;typedef int16_t int16;
#define IMU_ATTITUDE_CALIBRATING 0
extern struct host_attitude { struct {float x,y;} gyro; } attitude_state;
extern volatile float imu_roll_deg,imu_pitch_deg,imu_sample_dt_ms;
extern volatile uint32 imu_sample_sequence;
extern volatile int32 imu_attitude_status;
uint32 interrupt_global_disable(void);
void interrupt_global_enable(uint32);
void small_driver_set_duty(int16,int16);
void motor_m_ctrl(int);
#endif
'''
test = r'''
#include "balance_control.h"
#include "wheel_hold.h"
#include <assert.h>
#include <stdio.h>
struct host_attitude attitude_state;
volatile float imu_roll_deg,imu_pitch_deg,imu_sample_dt_ms=2;
volatile uint32 imu_sample_sequence;
volatile int32 imu_attitude_status;
static wheel_encoder_sample_t encoder;
static unsigned rebases;
static int left,right,ground;
uint32 interrupt_global_disable(void){return 0;}
void interrupt_global_enable(uint32 n){(void)n;}
void small_driver_set_duty(int16 l,int16 r){left=l;right=r;}
void motor_m_ctrl(int v){ground=v;}
void wheel_encoder_snapshot(wheel_encoder_sample_t *sample){*sample=encoder;}
uint8_t wheel_encoder_rebase_position(void){
    encoder=(wheel_encoder_sample_t){0};encoder.position_epoch=++rebases;
    return 1;
}
static void tick(void){
    ++imu_sample_sequence;
    for(unsigned i=0;i<APP_CONTROL_DIVIDER;++i)balance_control_tick_1ms();
}
static void publish(int32_t position,float speed){
    ++encoder.sequence;encoder.position_count=position;encoder.speed_cps=speed;
    encoder.dt_ms=20;encoder.age_ms=0;encoder.valid=encoder.position_valid=1;
}
static void arm(void){
    balance_control_defaults();ground_deadzone_enable=0;control_startup_ramp_ms=0;
    shacheflag=1;imu_attitude_status=1;imu_roll_deg=BALANCE_ZERO_X_DEG;
    imu_pitch_deg=BALANCE_ZERO_Y_DEG;attitude_state.gyro.x=attitude_state.gyro.y=0;
    ++imu_sample_sequence;mode10_balance_arm();
    ground_angle_kp=10;ground_rate_test_kp=30;ground_rate_test_sign=1;
    wheel_pos_kp=.5f;wheel_speed_limit=300;wheel_speed_kp=.001f;wheel_speed_ki=0;
    wheel_tilt_limit=1;pitch_balance_kp=10;roll_rate_test_kp=1;roll_rate_test_ki=0;
    roll_rate_test_sign=1;flywheel_vel_kp=.5f;
    flywheel_speed_feedback_left=20;flywheel_speed_feedback_right=-20;
    assert(balance_control_mode==10 && wheel_hold_enabled && dual_balance_enabled);
}
static void near(float actual,float expected){assert(fabsf(actual-expected)<.0001f);}
static void move(int32_t position){
    for(unsigned frame=0;frame<5;++frame){
        publish(position,0);
        for(unsigned i=0;i<10;++i)tick();
    }
}
int main(void){
    arm();assert(rebases==1);
    tick();assert(roll_rate_test_state==1 && !ground && !left && !right);
    publish(0,0);tick();assert(roll_rate_test_state==2 && wheel_hold_state==2);
    assert(!wheel_ref_count && !wheel_pos_error && !ground);
    assert(left==-1 && right==1); /* Roll speed P -> angle P -> incremental rate P. */
    move(512);
    assert(wheel_ref_count==0 && wheel_pos_error==-512);
    near(wheel_target_cps,-252);near(wheel_tilt_offset,.252f);
    near(dual_target[1],BALANCE_ZERO_X_DEG+.252f);
    near(dual_target_rate[1],2.52f);assert(ground==76);
    assert(left==-1 && right==1 && balance_control_mode==10);
    ground_deadzone_enable=1;ground_deadzone_pos_pwm=1800;ground_deadzone_neg_pwm=2800;
    tick();assert(dual_pwm[1]==76 && ground==1876);
    ground_deadzone_enable=0;
    move(-512);
    near(wheel_target_cps,252);near(wheel_tilt_offset,-.252f);
    near(dual_target_rate[1],-2.52f);assert(ground==-76);
    ground_deadzone_enable=1;tick();assert(dual_pwm[1]==-76 && ground==-2876);
    ground_rate_test_limit=1000;tick();assert(ground==-1000);
    ground_rate_test_limit=GROUND_PWM_LIMIT;ground_deadzone_enable=0;
    /* Encoder velocity also affects target tilt independently of position P. */
    publish(-512,100);tick();near(wheel_tilt_offset,-.152f);assert(ground==-46);
    /* Stale encoder stops both axes and latches until a new arm. */
    encoder.age_ms=WHEEL_SPEED_ENCODER_TIMEOUT_MS;tick();
    assert(roll_rate_test_state==-4 && control_fault_reason==8);
    assert(!ground && !left && !right);
    encoder.age_ms=0;tick();assert(roll_rate_test_state==-4 && !ground && !left);
    roll_rate_test_stop();tick();assert(!wheel_hold_enabled && !balance_control_mode);
    assert(!wheel_ref_count && !wheel_target_cps && !wheel_tilt_offset);
    arm();assert(rebases==2);publish(1001,0);tick();
    assert(wheel_ref_count==1001 && !wheel_pos_error && !wheel_target_cps);
    ++encoder.position_epoch;tick();
    assert(roll_rate_test_state==-4 && control_fault_reason==10 && !ground && !left);
    /* Parameter validation and invalid position use the same existing fault path. */
    arm();publish(0,0);wheel_pos_kp=NAN;tick();
    assert(roll_rate_test_state==-3 && control_fault_reason==9 && !ground && !left);
    arm();publish(0,0);encoder.position_valid=0;tick();
    assert(roll_rate_test_state==-4 && control_fault_reason==10 && !ground && !left);
    roll_rate_test_stop();dual_balance_arm();assert(!wheel_hold_enabled && balance_control_mode==5);
    speed_balance_arm();assert(wheel_hold_enabled && balance_control_mode==2);
    puts("PASS: Mode10 four-loop ground hold, independent Roll, origin reset, encoder/config faults and Stop");
}
'''
with tempfile.TemporaryDirectory(prefix='qcar-mode10-hold-') as directory:
    work = Path(directory)
    (work/'zf_common_headfile.h').write_text(boundary, encoding='utf-8')
    (work/'test.c').write_text(test, encoding='utf-8')
    exe = work/'test.exe'
    subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-I'+str(work), '-I'+str(root/'code'),
                    str(root/'code/balance_control.c'), str(root/'code/wheel_hold.c'),
                    str(root/'code/pid.c'), str(work/'test.c'), '-lm', '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=10)
