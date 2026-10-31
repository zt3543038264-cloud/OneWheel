"""Execute real upright/PID C with a fake hardware boundary (no target board)."""
from pathlib import Path
import subprocess,tempfile,os
root=Path(__file__).resolve().parents[1]
compiler=os.environ.get('HOST_CC','gcc')
with tempfile.TemporaryDirectory(prefix='onewheel-balance-') as d:
    work=Path(d)
    (work/'zf_common_headfile.h').write_text(r'''
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
''')
    (work/'test.c').write_text(r'''
#include "balance_control.h"
#include <assert.h>
#include <stdio.h>
struct host_attitude attitude_state;
volatile float imu_roll_deg,imu_pitch_deg,imu_sample_dt_ms=5;
volatile uint32 imu_sample_sequence;
volatile int32 imu_attitude_status;
static int fly_l,fly_r,ground;
uint32 interrupt_global_disable(void){return 0;}
void interrupt_global_enable(uint32 n){(void)n;}
void small_driver_set_duty(int16 l,int16 r){fly_l=l;fly_r=r;}
void motor_m_ctrl(int v){ground=v;}
static void tick(int fresh){if(fresh)++imu_sample_sequence;for(int i=0;i<APP_CONTROL_DIVIDER;++i)balance_control_tick_1ms();}
static void arm(int mode){
    balance_control_defaults();control_startup_ramp_ms=0;shacheflag=1;
    imu_attitude_status=1;imu_roll_deg=BALANCE_ZERO_X_DEG;imu_pitch_deg=BALANCE_ZERO_Y_DEG;
    attitude_state.gyro.x=attitude_state.gyro.y=0;++imu_sample_sequence;
    if(mode==3)roll_rate_test_arm();else if(mode==4)ground_rate_test_arm();
    else if(mode==5)dual_balance_arm();else if(mode==6)roll_rate_only_arm();else if(mode==10)mode10_balance_arm();else ground_rate_only_arm();
}
int main(void){
    /* Legacy expectations are computed without the static-friction feed-forward. */
    ground_deadzone_enable=0;
    /* The 1ms dispatcher must update outputs exactly once per 2ms. */
    arm(6);attitude_state.gyro.y=10;
    uint32 before_sequence=roll_rate_test_sequence;
    balance_control_tick_1ms();assert(roll_rate_test_sequence==before_sequence && !fly_l);
    balance_control_tick_1ms();assert(roll_rate_test_sequence==before_sequence+1 && fly_l==-(int)(10*FLY_RATE_KP));
    /* Rate-only axis ownership, signs, output limits and P-only equivalence. */
    arm(6);attitude_state.gyro.y=10;tick(1);assert(fly_l==-(int)(10*FLY_RATE_KP) && fly_r==(int)(10*FLY_RATE_KP) && ground==0);
    arm(7);ground_rate_test_kp=30;attitude_state.gyro.x=10;tick(1);assert(ground==-300 && fly_l==0 && fly_r==0);
    ground_rate_test_sign=-1;tick(1);assert(ground==300);
    ground_rate_test_limit=100;tick(1);assert(ground==100);
    arm(5);tick(1);assert(!ground && !fly_l && !fly_r);
    /* Compare P/P + LPF + signed flywheel angle compensation over a trajectory. */
    float filter[2]={BALANCE_ZERO_Y_DEG,BALANCE_ZERO_X_DEG};
    const float dt=(float)APP_IMU_PERIOD_US*0.000001f;
    float alpha=BALANCE_ANGLE_LPF_TAU_S<=0.0f?1.0f:(dt/(BALANCE_ANGLE_LPF_TAU_S+dt));
    float fly_rate=0.0f;
    for(int k=0;k<150;++k){
        imu_pitch_deg=BALANCE_ZERO_Y_DEG+((k%17)-8)*0.12f;
        imu_roll_deg=BALANCE_ZERO_X_DEG+((k%13)-6)*0.15f;
        attitude_state.gyro.y=((k%7)-3)*0.8f;attitude_state.gyro.x=((k%9)-4)*0.7f;
        filter[0]+=alpha*(imu_pitch_deg-filter[0]);filter[1]+=alpha*(imu_roll_deg-filter[1]);
        float tr0=FLY_ANGLE_KP*(BALANCE_ZERO_Y_DEG-filter[0]);
        float tr1=GROUND_ANGLE_KP*(BALANCE_ZERO_X_DEG-filter[1]);
        if(tr0>FLY_RATE_LIMIT_DPS)tr0=FLY_RATE_LIMIT_DPS;
        if(tr0< -FLY_RATE_LIMIT_DPS)tr0= -FLY_RATE_LIMIT_DPS;
        if(tr1>GROUND_RATE_LIMIT_DPS)tr1=GROUND_RATE_LIMIT_DPS;
        if(tr1< -GROUND_RATE_LIMIT_DPS)tr1= -GROUND_RATE_LIMIT_DPS;
        if (FLY_RATE_LPF_TAU_S<=0.0f) fly_rate=attitude_state.gyro.y;
        else fly_rate+=(attitude_state.gyro.y-fly_rate)*(dt/(FLY_RATE_LPF_TAU_S+dt));
        float uf=FLY_RATE_KP*(tr0-fly_rate);
        float ug=GROUND_RATE_KP*(tr1-attitude_state.gyro.x);
        if(uf>FLY_PWM_LIMIT)uf=FLY_PWM_LIMIT;if(uf< -FLY_PWM_LIMIT)uf= -FLY_PWM_LIMIT;
        if(pitch_balance_kp>0)uf+=FLY_ANGLE_FF_KP*(BALANCE_ZERO_Y_DEG-filter[0]);
        if(uf>FLY_PWM_LIMIT)uf=FLY_PWM_LIMIT;if(uf< -FLY_PWM_LIMIT)uf= -FLY_PWM_LIMIT;
        if(ug>GROUND_PWM_LIMIT)ug=GROUND_PWM_LIMIT;if(ug< -GROUND_PWM_LIMIT)ug= -GROUND_PWM_LIMIT;
        int f=(int)roundf(uf),g=(int)roundf(ug);
        tick(1);
        if(fly_l!=f || fly_r!=-f || ground!=g)fprintf(stderr,"k=%d actual=%d,%d expected=%d,%d\n",k,fly_l,ground,f,g);
        assert(fly_l==f && fly_r==-f && ground==g);
    }
    /* Tilt/nonfinite protection and fault latches remain unchanged. */
    arm(5);imu_roll_deg=31;tick(1);assert(roll_rate_test_state==-2 && !ground && !fly_l);
    imu_roll_deg=0;tick(1);assert(roll_rate_test_state==-2 && !ground);
    arm(5);imu_pitch_deg=NAN;tick(1);assert(roll_rate_test_state==-1 && !fly_l && !ground);
    arm(5);imu_attitude_status=-2;tick(1);assert(control_fault_reason==1 && !ground && !fly_l);
    for(int mode=3;mode<=7;mode++) {
        arm(mode);attitude_state.gyro.y=attitude_state.gyro.x=5;tick(1);
        imu_attitude_status=-5;tick(1);
        assert(roll_rate_test_state==-1 && control_fault_imu_status==-5 && !ground && !fly_l && !fly_r);
        imu_attitude_status=1;tick(1);assert(roll_rate_test_state==-1 && !ground && !fly_l);
    }
    arm(5);
    for(unsigned elapsed=APP_IMU_PERIOD_US/1000u;elapsed<BALANCE_IMU_STALE_MS;elapsed+=APP_IMU_PERIOD_US/1000u)tick(0);
    assert(roll_rate_test_state==2 && control_fault_reason==0);
    tick(0);assert(control_fault_reason==4 && !ground && !fly_l);
    /* Both rate-only and Mode10 tolerate a 30ms gap, reset the age on a new
     * sample, and still latch at the configured 50ms boundary. */
    for(unsigned test_mode=0;test_mode<2;++test_mode) {
        arm(test_mode?10:6);
        for(unsigned elapsed=0;elapsed<30;elapsed+=APP_IMU_PERIOD_US/1000u)tick(0);
        assert(roll_rate_test_state==2 && control_fault_reason==0);
        tick(1);
        for(unsigned elapsed=APP_IMU_PERIOD_US/1000u;elapsed<BALANCE_IMU_STALE_MS;elapsed+=APP_IMU_PERIOD_US/1000u)tick(0);
        assert(roll_rate_test_state==2 && control_fault_reason==0);
        tick(0);assert(roll_rate_test_state==-1 && control_fault_reason==4 && !ground && !fly_l && !fly_r);
        tick(1);assert(roll_rate_test_state==-1); /* No automatic motor restart. */
    }
    arm(5);imu_attitude_status=0;tick(1);assert(roll_rate_test_state==1 && !ground && !fly_l);
    arm(5);ground_rate_test_sign=0;tick(1);assert(roll_rate_test_state==-3 && !ground && !fly_l);
    arm(5);ground_rate_test_kp=30;attitude_state.gyro.x=4;tick(1);assert(ground!=0);shacheflag=0;tick(1);assert(!ground && !fly_l);
    arm(7);ground_rate_test_kp=30;control_startup_ramp_ms=1000;attitude_state.gyro.x=100;tick(1);assert(control_startup_scale>0 && control_startup_scale<0.01f && ground==-12);
    for(unsigned i=1;i<499;++i)tick(1);
    assert(fabsf(control_startup_scale-0.998f)<0.0001f);
    tick(1);assert(control_startup_scale==1.0f && ground==-3000);
    /* Verify requested flywheel defaults and unchanged ground limits. */
    arm(5);assert(pitch_balance_rate_limit==FLY_RATE_LIMIT_DPS && roll_rate_test_limit==8000);
    assert(roll_rate_test_kp==FLY_RATE_KP && pitch_balance_kp==FLY_ANGLE_KP && ground_rate_test_kp==GROUND_RATE_KP && mono_params.balance_angle_y==BALANCE_ZERO_Y_DEG && mono_params.balance_angle_x==BALANCE_ZERO_X_DEG && ground_rate_test_limit==6000);
    pitch_balance_rate_limit=ground_angle_rate_limit=200;
    roll_rate_test_limit=ground_rate_test_limit=8000;
    roll_rate_test_kp=ground_rate_test_kp=50;
    pitch_balance_kp=ground_angle_kp=100;
    mono_params.balance_angle_y=imu_pitch_deg+4;mono_params.balance_angle_x=imu_roll_deg+4;
    tick(1);assert(roll_rate_test_state==2 && dual_target_rate[0]==200 && dual_target_rate[1]==200);
    assert(fly_l==8000 && fly_r==-8000 && ground==8000);
    pitch_balance_rate_limit=200.1f;tick(1);assert(roll_rate_test_state==-3 && !fly_l && !ground);
    arm(5);ground_rate_test_limit=8000.1f;tick(1);assert(roll_rate_test_state==-3 && !fly_l && !ground);
    arm(3);roll_rate_test_kp=50;pitch_balance_rate_limit=200;roll_rate_test_limit=8000;
    attitude_state.gyro.y=300;tick(1);assert(roll_rate_test_state==2 && fly_l==-8000);
    roll_rate_test_limit=8000.1f;tick(1);assert(roll_rate_test_state==-3 && !fly_l);
    arm(4);ground_angle_rate_limit=200.1f;tick(1);assert(roll_rate_test_state==-3 && !ground);
    roll_rate_test_stop();balance_control_tick_1ms();assert(!ground && !fly_l && !roll_rate_test_enabled);
    /* Mode8 owns both raw outputs independently of all IMU state and PID gains. */
    pwm_debug_arm();shacheflag=1;control_startup_ramp_ms=0;
    imu_sample_sequence=0;imu_attitude_status=-1;imu_roll_deg=90;imu_pitch_deg=NAN;
    pwm_debug_fly_left=300;pwm_debug_fly_right=-700;pwm_debug_ground=-500;tick(0);
    assert(pwm_debug_enabled && roll_rate_test_state==2 && fly_l==300 && fly_r==-700 && ground==-500);
    assert(dual_pwm[0]==300 && dual_pwm[1]==-500 && control_fault_reason==0);
    assert(costrained_flywheel_speed_right==700); /* motor_right_pwm is negated internal value. */
    pwm_debug_fly_left=0;pwm_debug_fly_right=600;pwm_debug_ground=0;tick(0);
    assert(!fly_l && fly_r==600 && !ground); /* Right alone, no implicit sign inversion. */
    pwm_debug_fly_left=-300;pwm_debug_fly_right=0;tick(0);
    assert(fly_l==-300 && !fly_r && !ground); /* Left alone. */
    pwm_debug_fly_left=-4000;pwm_debug_fly_right=4000;pwm_debug_ground=4000;tick(0);
    assert(fly_l==-4000 && fly_r==4000 && ground==4000);
    pwm_debug_ground=4001;tick(0);
    assert(roll_rate_test_state==-3 && control_fault_reason==6 && !fly_l && !ground);
    pwm_debug_ground=0;tick(0);assert(roll_rate_test_state==-3 && !fly_l && !ground);
    pwm_debug_arm();assert(pwm_debug_fly_left==0 && pwm_debug_fly_right==0 && pwm_debug_ground==0);
    pwm_debug_fly_right=NAN;tick(0);assert(roll_rate_test_state==-3 && !fly_l && !fly_r && !ground);
    pwm_debug_arm();pwm_debug_fly_left=-4001;tick(0);
    assert(roll_rate_test_state==-3 && !fly_l && !fly_r && !ground);
    pwm_debug_arm();pwm_debug_fly_right=4001;tick(0);
    assert(roll_rate_test_state==-3 && !fly_l && !fly_r && !ground);
    pwm_debug_arm();control_startup_ramp_ms=1000;
    pwm_debug_fly_left=4000;pwm_debug_fly_right=-4000;pwm_debug_ground=-4000;tick(0);
    assert(fly_l==8 && fly_r==-8 && ground==-8);
    shacheflag=0;tick(0);assert(!fly_l && !ground && roll_rate_test_state==0);
    roll_rate_test_stop();assert(!pwm_debug_enabled && !pwm_debug_fly_left && !pwm_debug_fly_right && !pwm_debug_ground);
    balance_control_tick_1ms();assert(!fly_l && !ground);
    arm(5);tick(1);assert(!pwm_debug_enabled && roll_rate_test_state==2 && !ground && !fly_l);
    /* Ground static-friction feed-forward: signed offset, clamped, zero at
     * standstill, bypassed when disabled or on the Mode8 bench path. */
    ground_deadzone_enable=1;ground_deadzone_pos_pwm=1800;ground_deadzone_neg_pwm=2800;
    arm(7);ground_rate_test_kp=30;control_startup_ramp_ms=0;
    attitude_state.gyro.x=0;tick(1);assert(ground==0 && costrained_wheel_speed==0);
    attitude_state.gyro.x=10;tick(1);assert(ground==-(int)(10*ground_rate_test_kp)-2800);
    assert(costrained_wheel_speed==(int16)ground);
    ground_rate_test_sign=-1;tick(1);assert(ground==(int)(10*ground_rate_test_kp)+1800);
    ground_rate_test_limit=1000;tick(1);assert(ground==1000);
    ground_rate_test_limit=6000;ground_rate_test_sign=GROUND_OUTPUT_SIGN;
    ground_deadzone_enable=0;tick(1);assert(ground==-(int)(10*ground_rate_test_kp));
    ground_deadzone_enable=1;ground_deadzone_neg_pwm=0;tick(1);assert(ground==-(int)(10*ground_rate_test_kp));
    ground_rate_test_sign=-1;tick(1);assert(ground==(int)(10*ground_rate_test_kp)+1800);
    ground_deadzone_pos_pwm=0;tick(1);assert(ground==(int)(10*ground_rate_test_kp));
    ground_deadzone_pos_pwm=GROUND_DEADZONE_POS_PWM;ground_deadzone_neg_pwm=GROUND_DEADZONE_NEG_PWM;
    arm(7);ground_rate_test_kp=30;control_startup_ramp_ms=1000;
    attitude_state.gyro.x=.1f;tick(1);assert(ground==-(int)(3+GROUND_DEADZONE_NEG_PWM*.002f));
    ground_rate_test_sign=-1;tick(1);assert(ground==10); /* +3 PID plus 1800 * .004. */
    pwm_debug_arm();shacheflag=1;control_startup_ramp_ms=0;pwm_debug_ground=-500;tick(0);
    assert(ground==-500);
    /* Mode10: Roll incremental rate PID plus ground position/speed hold at rest. */
    ground_deadzone_enable=0;
    arm(10);assert(balance_control_mode==10 && dual_balance_enabled && roll_rate_test_ki==FLY_RATE_KI);
    pitch_balance_kp=10.0f;roll_rate_test_kp=5.0f;roll_rate_test_ki=2.0f;
    imu_pitch_deg=BALANCE_ZERO_Y_DEG-1.0f;attitude_state.gyro.y=0.0f;
    tick(1);assert(fly_l==70 && fly_r==-70);
    tick(1);assert(fly_l==90 && fly_r==-90);
    /* Mode10 speed loop (P only): real=(L-R)*0.005, roll target = zero + Kp*(0-real), angle limited. */
    arm(10);assert(flywheel_vel_kp==FLY_VEL_KP);
    pitch_balance_kp=10.0f;roll_rate_test_kp=1.0f;roll_rate_test_ki=0.0f;flywheel_vel_kp=0.5f;
    flywheel_speed_feedback_left=200;flywheel_speed_feedback_right=-200; /* real=2 -> vel_out=-1 */
    imu_pitch_deg=BALANCE_ZERO_Y_DEG;attitude_state.gyro.y=0.0f;tick(1);
    assert(fabsf(flywheel_vel_angle_out+1.0f)<1e-4f && fabsf(dual_target[0]-(BALANCE_ZERO_Y_DEG-1.0f))<1e-4f);
    assert(fly_l==-10); /* angle err -1 deg * 10 = -10 dps, rate Kp 1 */
    flywheel_speed_feedback_left=30000;flywheel_speed_feedback_right=-30000;tick(1);
    assert(fabsf(flywheel_vel_angle_out+FLY_VEL_ANGLE_LIMIT_DEG)<1e-4f);
    flywheel_speed_feedback_left=flywheel_speed_feedback_right=0;
    roll_rate_test_stop();assert(flywheel_vel_angle_out==0.0f);
    /* Angle D uses the prepared gyro, so target-angle steps create no D kick. */
    arm(10);pitch_balance_kp=0;pitch_balance_kd=.5f;
    roll_rate_test_kp=1;roll_rate_test_ki=0;attitude_state.gyro.y=4;
    tick(1);assert(fly_angle_d_out==-2 && dual_target_rate[0]==-2 && fly_l==-6);
    mono_params.balance_angle_y+=1;tick(1);
    assert(fly_angle_d_out==-2 && dual_target_rate[0]==-2);
    pitch_balance_kp=100;mono_params.balance_angle_y=imu_pitch_deg+4;
    attitude_state.gyro.y=400;tick(1);
    assert(fly_angle_d_out==-200 && dual_target_rate[0]==160);
    /* Speed D differentiates distinct feedback frames over 20ms, not the
     * 2ms controller tick. The 30ms low-pass gives -2deg for this step. */
    arm(10);pitch_balance_kp=10;roll_rate_test_kp=1;roll_rate_test_ki=0;
    flywheel_vel_kp=0;flywheel_vel_kd=.1f;
    flywheel_speed_feedback_sequence=1;
    flywheel_speed_feedback_left=flywheel_speed_feedback_right=0;
    tick(1);assert(fly_vel_d_out==0);
    for(unsigned i=0;i<9;++i)tick(1);
    flywheel_speed_feedback_left=100;flywheel_speed_feedback_right=-100;
    flywheel_speed_feedback_sequence=2;tick(1);
    assert(fabsf(fly_vel_d_out+2)<.0001f && fabsf(flywheel_vel_angle_out+2)<.0001f);
    assert(fly_l==-20);
    tick(1);assert(fabsf(fly_vel_d_out+2)<.0001f); /* Held between feedback frames. */
    for(unsigned i=0;i<60;++i)tick(1);
    assert(fly_vel_d_out==0); /* No indefinitely retained derivative after feedback stops. */
    flywheel_speed_feedback_sequence=3;flywheel_speed_feedback_left=200;
    tick(1);assert(fly_vel_d_out==0); /* First frame after a long gap seeds history. */
    roll_rate_test_stop();assert(fly_vel_d_out==0 && fly_angle_d_out==0);
    arm(10);pitch_balance_kd=NAN;tick(1);
    assert(roll_rate_test_state==-3 && control_fault_reason==6 && !fly_l && !ground);
    arm(10);flywheel_vel_kd=-1;tick(1);
    assert(roll_rate_test_state==-3 && control_fault_reason==6 && !fly_l && !ground);
    puts("balance control: 2ms dispatch, direct feedback, limits, 1s ramp, 50ms stale trip, stop and faults PASS");
    return 0;
}
''')
    exe=work/'balance.exe'
    subprocess.run([compiler,'-I'+str(work),'-I'+str(root/'code'),str(root/'code/balance_control.c'),str(root/'code/wheel_hold.c'),str(root/'tests/encoder_snapshot_stub.c'),str(root/'code/pid.c'),str(work/'test.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
