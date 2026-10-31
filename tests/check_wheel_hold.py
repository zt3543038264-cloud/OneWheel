"""Mode2 real PI + dual controller: timing, direction, gates and fault recovery."""
from pathlib import Path
import subprocess,tempfile

root=Path(__file__).resolve().parents[1]
boundary=r'''
#ifndef HOST_BOUNDARY
#define HOST_BOUNDARY
#include <stdint.h>
#include <math.h>
#include "config.h"
#include "qcar_types.h"
typedef uint8_t uint8;typedef uint32_t uint32;typedef int32_t int32;typedef int16_t int16;
#define IMU_ATTITUDE_CALIBRATING 0
extern struct host_attitude {struct {float x,y;} gyro;} attitude_state;
extern volatile float imu_roll_deg,imu_pitch_deg,imu_sample_dt_ms;
extern volatile uint32 imu_sample_sequence;
extern volatile int32 imu_attitude_status;
uint32 interrupt_global_disable(void);void interrupt_global_enable(uint32);
void small_driver_set_duty(int16,int16);void motor_m_ctrl(int);
#endif
'''
test=r'''
#include "balance_control.h"
#include "wheel_hold.h"
#include <assert.h>
#include <stdio.h>
struct host_attitude attitude_state;
volatile float imu_roll_deg,imu_pitch_deg,imu_sample_dt_ms=2;
volatile uint32 imu_sample_sequence;
volatile int32 imu_attitude_status;
static wheel_encoder_sample_t sample;
static uint32_t fake_epoch=1;
static int fly_l,fly_r,ground;
uint32 interrupt_global_disable(void){return 0;}
void interrupt_global_enable(uint32 n){(void)n;}
void small_driver_set_duty(int16 l,int16 r){fly_l=l;fly_r=r;}
void motor_m_ctrl(int p){ground=p;}
void wheel_encoder_snapshot(wheel_encoder_sample_t *p){*p=sample;}
uint8_t wheel_encoder_rebase_position(void){++fake_epoch;sample.position_epoch=fake_epoch;sample.position_count=0;return 1;}
static void close_to(float a,float b){if(fabsf(a-b)>.00001f)fprintf(stderr,"got %.9g expected %.9g\n",a,b);assert(fabsf(a-b)<.00001f);}
static void tick(void){++imu_sample_sequence;for(unsigned i=0;i<APP_CONTROL_DIVIDER;++i)balance_control_tick_1ms();}
static void fresh(float speed){++sample.sequence;sample.speed_cps=speed;sample.dt_ms=20;sample.age_ms=0;sample.valid=sample.position_valid=1;sample.position_epoch=fake_epoch;}
static void arm(int mode){
    balance_control_defaults();shacheflag=1;control_startup_ramp_ms=0;
    /* Exercise a defined inner-loop gain, independent of stored/default tuning. */
    ground_angle_kp=40;ground_rate_test_kp=30;ground_deadzone_enable=0;
    imu_roll_deg=BALANCE_ZERO_X_DEG;imu_pitch_deg=BALANCE_ZERO_Y_DEG;
    imu_attitude_status=1;attitude_state.gyro.x=attitude_state.gyro.y=0;
    fresh(0);if(mode==2)speed_balance_arm();else dual_balance_arm();fresh(0);
}
int main(void){
    wheel_hold_defaults();fresh(500);
    assert(wheel_hold_update(&sample,0)==1);close_to(wheel_tilt_offset,.5f);
    assert(wheel_hold_state==2 && wheel_feedback_valid && wheel_speed_error==-500);
    fresh(-500);assert(wheel_hold_update(&sample,0)==1);close_to(wheel_tilt_offset,-.5f);
    fresh(5000);wheel_hold_update(&sample,0);close_to(wheel_tilt_offset,1);
    wheel_tilt_limit=.1f;wheel_hold_update(&sample,0);close_to(wheel_tilt_offset,.1f);
    wheel_hold_defaults();wheel_speed_kp=0;wheel_speed_ki=.01f;fresh(10);
    wheel_hold_update(&sample,0);close_to(wheel_speed_integral,-.002f);
    for(int i=0;i<20;++i)wheel_hold_update(&sample,0);
    close_to(wheel_speed_integral,-.002f); /* No integration on repeated frames. */
    fresh(10);sample.dt_ms=21;wheel_hold_update(&sample,0);close_to(wheel_speed_integral,-.0041f);
    fresh(10);wheel_hold_update(&sample,1);close_to(wheel_speed_integral,-.0041f);
    wheel_hold_defaults();wheel_speed_kp=.1f;wheel_speed_ki=.1f;fresh(100);
    wheel_hold_update(&sample,0);close_to(wheel_tilt_offset,1);close_to(wheel_speed_integral,0);
    wheel_hold_defaults();wheel_speed_kp=0;wheel_speed_ki=.1f;
    for(int i=0;i<50;++i){fresh(100);wheel_hold_update(&sample,0);}
    close_to(wheel_speed_integral,-.5f);close_to(wheel_tilt_offset,.5f);
    wheel_i_limit=0;wheel_tilt_limit=0;fresh(100);wheel_hold_update(&sample,0);
    close_to(wheel_speed_integral,0);close_to(wheel_tilt_offset,0);
    wheel_hold_suspend();assert(!wheel_tilt_offset && !wheel_speed_integral && !wheel_feedback_valid && wheel_hold_state==1);
    wheel_hold_defaults();sample.sequence=0;sample.age_ms=20;sample.valid=0;sample.dt_ms=0;
    assert(!wheel_hold_update(&sample,0) && !wheel_tilt_offset && wheel_hold_state==1);
    sample.age_ms=60;assert(wheel_hold_update(&sample,0)==-8 && !wheel_feedback_valid);
    fresh(0);assert(wheel_hold_update(&sample,0)==-8); /* Fault latch. */
    wheel_hold_defaults();fresh(0);sample.age_ms=59;assert(wheel_hold_update(&sample,0)==1);
    sample.age_ms=60;assert(wheel_hold_update(&sample,0)==-8);
    wheel_hold_defaults();fresh(0);sample.valid=0;assert(wheel_hold_update(&sample,0)==-7);
    wheel_hold_defaults();fresh(NAN);assert(wheel_hold_update(&sample,0)==-7);
    wheel_hold_defaults();fresh(0);sample.dt_ms=0;assert(wheel_hold_update(&sample,0)==-7);
    wheel_hold_defaults();fresh(0);wheel_speed_kp=NAN;assert(wheel_hold_update(&sample,0)==-9);
    wheel_hold_defaults();fresh(0);wheel_speed_ki=-1;assert(wheel_hold_update(&sample,0)==-9);
    wheel_hold_defaults();fresh(0);wheel_tilt_limit=5.01f;assert(wheel_hold_update(&sample,0)==-9);
    wheel_hold_defaults();fresh(0);sample.sequence=UINT32_MAX;wheel_hold_update(&sample,0);
    fresh(500);assert(wheel_hold_update(&sample,0)==1);close_to(wheel_tilt_offset,.5f);
    /* Mode5 ignores the encoder and Mode2 settings completely. */
    arm(5);fresh(NAN);sample.valid=sample.position_valid=0;wheel_speed_kp=wheel_pos_kp=NAN;tick();
    assert(roll_rate_test_state==2 && !ground && !fly_l && !wheel_hold_enabled && balance_control_mode==5);
    arm(2);fresh(500);tick();
    assert(wheel_hold_enabled && balance_control_mode==2 && dual_balance_enabled && roll_rate_test_state==2);
    close_to(dual_target[1],BALANCE_ZERO_X_DEG+.5f);assert(ground==600 && !fly_l && !fly_r);
    fresh(-500);tick();close_to(dual_target[1],BALANCE_ZERO_X_DEG-.5f);assert(ground==-600);
    fresh(500);tick();sample.age_ms=60;tick();
    assert(roll_rate_test_state==-4 && control_fault_reason==8 && !ground && !fly_l && !fly_r && !wheel_tilt_offset);
    fresh(0);tick();assert(roll_rate_test_state==-4 && !ground); /* No auto-restart. */
    arm(2);tick();assert(roll_rate_test_state==2 && !ground && !wheel_speed_integral);
    sample.valid=0;tick();assert(roll_rate_test_state==-4 && control_fault_reason==7 && !ground);
    arm(2);sample.position_valid=0;tick();assert(roll_rate_test_state==-4 && control_fault_reason==10 && !ground && !fly_l);
    arm(2);tick();++sample.position_epoch;tick();assert(roll_rate_test_state==-4 && control_fault_reason==10 && !ground);
    arm(2);imu_attitude_status=0;sample.position_count=1000;tick();assert(!wheel_ref_count);
    imu_attitude_status=1;fresh(0);sample.position_count=1200;tick();assert(wheel_ref_count==1200 && !wheel_target_cps);
    for(int i=0;i<5;++i){fresh(0);sample.position_count=1300;tick();}
    close_to(wheel_target_cps,-46);close_to(dual_target[1],BALANCE_ZERO_X_DEG+.046f);assert(ground==55);
    arm(2);wheel_speed_ki=NAN;tick();assert(roll_rate_test_state==-3 && control_fault_reason==9 && !ground);
    arm(2);fresh(500);mono_params.balance_angle_x=20;tick();
    assert(roll_rate_test_state==-3 && control_fault_reason==9 && !ground);
    arm(2);fresh(500);imu_attitude_status=0;tick();
    assert(roll_rate_test_state==1 && !ground && !fly_l && !wheel_tilt_offset && !wheel_speed_integral);
    imu_attitude_status=1;tick();assert(roll_rate_test_state==2 && ground==600);
    imu_pitch_deg=31;tick();assert(roll_rate_test_state==-2 && !ground && !fly_l && !wheel_tilt_offset);
    /* Startup and downstream PWM saturation inhibit outer integral. */
    arm(2);control_startup_ramp_ms=1000;wheel_speed_kp=0;wheel_speed_ki=.01f;
    for(int i=0;i<500;++i){if(i%10==0)fresh(10);tick();}
    close_to(control_startup_scale,1);close_to(wheel_speed_integral,0);
    fresh(10);tick();assert(wheel_speed_integral<0);
    arm(2);wheel_speed_ki=.01f;ground_rate_test_limit=100;
    for(int i=0;i<30;++i){fresh(500);tick();assert(ground==100);}
    close_to(wheel_speed_integral,0);
    roll_rate_test_stop();balance_control_tick_1ms();
    assert(!wheel_hold_enabled && !wheel_tilt_offset && !wheel_speed_integral && !balance_control_mode && !ground && !fly_l);
    puts("PASS: Mode2 sign, PI actual dt/new-frame ownership, saturation/ramp antiwindup, encoder freshness/fault latch, IMU gates, restart; Mode5 isolation");
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-hold-') as directory:
    work=Path(directory)
    (work/'zf_common_headfile.h').write_text(boundary)
    (work/'test.c').write_text(test)
    exe=work/'test.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(work),'-I'+str(root/'code'),
                    str(root/'code/balance_control.c'),str(root/'code/wheel_hold.c'),str(root/'code/pid.c'),
                    str(work/'test.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=10)
