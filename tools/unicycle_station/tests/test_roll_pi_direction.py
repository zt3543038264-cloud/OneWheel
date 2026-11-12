"""Execute Roll production functions. Sign-model tests are not vehicle proof."""
import re

from test_lean_limits import ROOT, _compile_run, _function
from test_imu_forward import param_host_source


def test_incremental_pi_accumulation_saturation_reversal_and_reset(tmp_path):
    source = r'''
#include "Common/pid.c"
#include <assert.h>
#include <math.h>
static void near(float a,float b){assert(fabsf(a-b)<.001f);}
int main(void){
 pid_t p;
 pid_set(&p,2,.5f,999,100);
 near(pid_inc_pi_calc_limited(&p,1,-10,10),2.5f);
 near(pid_inc_pi_calc_limited(&p,1,-10,10),3);
 near(pid_inc_pi_calc_limited(&p,2,-10,10),6);
 assert(p.out_d==0); // even a stale legacy D must have no effect
 for(int i=0;i<10000;i++)pid_inc_pi_calc_limited(&p,100,-10,10);
 near(p.out,10);
 assert(pid_inc_pi_calc_limited(&p,-1,-10,10)<0); // no hidden windup
 pid_reset(&p);near(p.out,0);near(p.last_error,0);
 near(pid_inc_pi_calc_limited(&p,0,-10,10),0);
 p.ki=0;pid_reset(&p);
 for(int i=-4;i<=4;i++) near(pid_inc_pi_calc_limited(&p,(float)i,-10,10),2.f*i);
 // P saturation must not manufacture an integral bias or opposite full PWM.
 for(int sign=-1;sign<=1;sign+=2){
  pid_reset(&p);p.ki=0;
  near(pid_inc_pi_calc_limited(&p,100.f*sign,-10,10),10.f*sign);
  near(pid_inc_pi_calc_limited(&p,0,-10,10),0);
  p.ki=.5f;pid_reset(&p);
  for(int i=0;i<1000;i++)pid_inc_pi_calc_limited(&p,100.f*sign,-10,10);
  near(p.integrator,0);near(pid_inc_pi_calc_limited(&p,0,-10,10),0);
 }
 // No jump beyond the P-only change when Ki is disabled during tuning.
 p.ki=1;pid_reset(&p);pid_inc_pi_calc_limited(&p,1,-10,10);
 p.ki=0;near(pid_inc_pi_calc_limited(&p,1,-10,10),2);
 return 0;
}
'''
    _compile_run(tmp_path, source)


def test_roll_actual_cascade_and_test_signs_targets_limits_and_periods(tmp_path):
    control = (ROOT/'code/Controller/control.c').read_text(encoding='utf-8')
    source = r'''
#include "config.h"
#include "Common/param.h"
#include "Common/param.h"
#include "Common/pid.c"
#include "Controller/control_angle.inc"
#include <assert.h>
#include <math.h>
param_t g_param;
static pid_t r_rcy_pid,r_angle_pid,r_rate_pid,p_vel_pid,p_angle_pid,p_rate_pid,y_angle_pid,y_rate_pid;
static struct {float roll,roll_rate;} att;
static float g_roll_zero=2,s_roll_angle_target,s_rcy_fb,g_flywheel_common_rpm,g_yaw_momentum_scale,s_yaw_desat_offset,g_yaw_target,g_pwm_yaw;
static float imu_get_angle_yaw(void){return g_yaw_target;}
static int16 rpm_a,rpm_b;
static int16 W_Motor_GetSpeed1(void){return rpm_a;}
static int16 W_Motor_GetSpeed2(void){return rpm_b;}
#define TUNE_RING_RATE 0
#define TUNE_RING_ANGLE 1
#define TUNE_RING_VEL 2
static uint8 s_test_ring;
static float roll_rate_notch_apply(float x){return x;}
static void near(float a,float b){assert(fabsf(a-b)<.02f);}
'''
    for name in ('cascade_gain_refresh', 'roll_recovery_offset', 'roll_rate_ctrl',
                 'roll_target_update', 'roll_cascade_ctrl', 'roll_test_ctrl'):
        source += _function(control, name)+'\n'
    source += r'''
static void reset(void){
 pid_set(&r_rcy_pid,.0015f,0,0,R_RCY_IMAX);
 pid_set(&r_angle_pid,36,0,0,R_ANGLE_IMAX);
 pid_set(&r_rate_pid,20,0,0,R_RATE_IMAX);
 rpm_a=rpm_b=0;att.roll=g_roll_zero;att.roll_rate=0;
}
int main(void){
 g_param.r_rate_kp=20;g_param.r_rate_ki=.2f;g_param.r_rate_kd=8;
 cascade_gain_refresh();near(r_rate_pid.kd,0);near(r_rate_pid.ki,.2f);
 for(int sign=-1;sign<=1;sign+=2){
  reset();s_test_ring=TUNE_RING_RATE;att.roll_rate=10.f*sign;
  near(roll_test_ctrl(1,1),200.f*sign);
  reset();s_test_ring=TUNE_RING_ANGLE;att.roll+=sign;
  near(roll_test_ctrl(1,1),720.f*sign);
  // Differential feedback: +/-1000 and -/+1000 -> +/-2000, not zero.
  reset();rpm_a=1000*sign;rpm_b=-1000*sign;
  float balance=roll_cascade_ctrl(g_roll_zero,1,1);
  near(s_roll_angle_target,g_roll_zero+3.f*sign);
  near(s_rcy_fb,2000.f*sign);near(balance,-2160.f*sign);
  reset();s_test_ring=TUNE_RING_VEL;rpm_a=1000*sign;rpm_b=-1000*sign;
  near(roll_test_ctrl(1,1),balance);near(s_roll_angle_target,g_roll_zero+3.f*sign);
  rpm_a=rpm_b=0; // speed offset holds between 20ms updates
  roll_test_ctrl(1,0);near(s_roll_angle_target,g_roll_zero+3.f*sign);
  roll_test_ctrl(1,1);near(s_roll_angle_target,g_roll_zero);
  // Test and Balance share the +/-10deg target bound and actual telemetry.
  reset();s_test_ring=TUNE_RING_VEL;rpm_a=30000*sign;rpm_b=-30000*sign;
  roll_test_ctrl(1,1);near(s_roll_angle_target,g_roll_zero+10.f*sign);
  float target=s_roll_angle_target;
  reset();rpm_a=30000*sign;rpm_b=-30000*sign;
  roll_cascade_ctrl(g_roll_zero,1,1);near(s_roll_angle_target,target);
  // Quasi-static upright model: theta_ddot=a*theta-b*u, r_dot=-c*u.
  // Under these explicit signs, following theta_target must unload momentum.
  float theta=target-g_roll_zero,u_hold=10.f*theta;
  assert((float)sign*(-u_hold)<0);
 }
 reset();r_rate_pid.ki=.5f;s_test_ring=TUNE_RING_RATE;att.roll_rate=2;
 roll_test_ctrl(1,1);float first=r_rate_pid.out;
 roll_test_ctrl(1,0);assert(r_rate_pid.out>first);
 pid_reset(&r_rate_pid);att.roll_rate=0;near(roll_test_ctrl(1,0),0);
 return 0;
}
'''
    _compile_run(tmp_path, source)


def test_legacy_flash_disables_only_roll_d_and_preserves_other_parameters(tmp_path):
    source = param_host_source()+r'''
int main(void){
 param_init();
 assert(param_set_by_name("r_rate_kp",27));
 assert(param_set_by_name("r_rate_ki",.2f));
 assert(param_set_by_name("roll_zero_init",1.5f));
 assert(param_save());
 uint32 n=storage[1].uint32_type;
 for(uint32 i=0;i<n;i++)
  if(storage[2+2*i].uint32_type==param_name_key("r_rate_kd",1))storage[3+2*i].float_type=8;
 storage[2+2*n].uint32_type=param_crc32_words((const uint32*)storage,2+2*n);
 param_init();
 assert(g_param.r_rate_kd==0 && R_RATE_KD==0);
 assert(g_param.r_rate_kp==27 && g_param.r_rate_ki==.2f && g_param.roll_zero_init==1.5f);
 assert(!param_set_by_name("r_rate_kd",8));assert(param_set_by_name("r_rate_kd",0));
 const char *names[]={"cam_exposure"};
 assert(param_save_names(names,1));param_init();assert(g_param.r_rate_kd==0);
 assert(g_param.r_rate_kp==27 && g_param.roll_zero_init==1.5f);
 return 0;
}
'''
    _compile_run(tmp_path, source)


def test_roll_menu_ownership_follows_names(tmp_path):
    menu = (ROOT/'code/Menu/menu.c').read_text(encoding='utf-8')
    fn = _function(menu, 'group_param_edit_allowed')
    source = r'''
#include <assert.h>
#include <string.h>
#include <stdint.h>
typedef uint8_t uint8;
enum {GROUP_KIND_AXIS,GROUP_KIND_ZERO,GROUP_KIND_MOTOR,GROUP_KIND_ODOMETRY,GROUP_KIND_BEACON};
enum {TUNE_AXIS_ROLL,TUNE_AXIS_PITCH};
enum {TUNE_RING_RATE,TUNE_RING_ANGLE,TUNE_RING_VEL};
#define START_STOP 0
#define ODOM_TEST_IDLE 0
#define MOTOR_JOG_NONE 0
static int start_flag=START_STOP,g_tune_axis=TUNE_AXIS_ROLL,g_tune_ring;
static int path_nav_busy(void){return 0;}static int control_path_active(void){return 0;}
static int control_test_running(void){return 1;}static int control_jog_running(void){return 0;}
static int control_odometry_test_state(void){return ODOM_TEST_IDLE;}
typedef struct {const char *label,*name;float step;uint8 decimals;} menu_param_item_t;
typedef struct {const menu_param_item_t *items;int axis,kind;} menu_group_t;
'''
    items = re.search(r'static const menu_param_item_t s_roll_items\[\] =\s*\{.*?^\};', menu, re.M|re.S).group()
    source += items+'\n'+fn+r'''
int main(void){
 menu_group_t group={s_roll_items,TUNE_AXIS_ROLL,GROUP_KIND_AXIS};
 for(g_tune_ring=0;g_tune_ring<=2;g_tune_ring++)
  for(unsigned i=0;i<sizeof(s_roll_items)/sizeof(s_roll_items[0]);i++){
   const char *n=s_roll_items[i].name;
   assert(strcmp(n,"r_rate_kd")!=0);
   int ring=strstr(n,"r_rcy_")==n?2:strstr(n,"r_angle_")==n?1:0;
   assert(group_param_edit_allowed(&group,(uint8)i)==(ring<=g_tune_ring));
  }
 return 0;
}
'''
    _compile_run(tmp_path, source)


def test_actual_reset_clears_incremental_output_on_stop_and_restart(tmp_path):
    control = (ROOT/'code/Controller/control.c').read_text(encoding='utf-8')
    # Execute the real shared reset, and verify real entry/exit paths retain it.
    for name in ('control_stop', 'test_start', 'test_stop', 'cascade_run'):
        assert 'cascade_reset();' in _function(control, name)
    source = r'''
#include "Common/pid.c"
#include <assert.h>
static pid_t r_rcy_pid,r_angle_pid,r_rate_pid,p_vel_pid,p_angle_pid,p_rate_pid,y_angle_pid,y_rate_pid;
static float s_path_yaw_rate_target,s_path_yaw_rate_cmd,g_lean_offset,s_lean_speed_mps,s_speed_ramp;
static float s_rcy_fb,g_flywheel_common_rpm,g_yaw_momentum_scale,s_direction_offset,s_direction_last_error;
static float s_direction_yaw_rate_target,s_direction_yaw_rate_cmd,s_direction_hold_yaw;
static float s_roll_angle_target,g_roll_zero=2,s_yaw_output_applied;
static uint8 s_pitch_rate_cache_valid,s_direction_last_mode;
#define TRACK_MODE_MIDDLE 0
static unsigned notch_resets;
static void roll_notch_reset_state(void){++notch_resets;}
'''+_function(control, 'cascade_reset')+r'''
int main(void){
 pid_set(&r_rate_pid,20,.2f,0,100);
 for(int i=0;i<100;i++)pid_inc_pi_calc_limited(&r_rate_pid,5,-10000,10000);
 assert(r_rate_pid.out>0);cascade_reset();
 assert(r_rate_pid.out==0 && r_rate_pid.last_error==0 && r_rate_pid.out_i==0);
 assert(s_roll_angle_target==g_roll_zero && notch_resets==1);
 assert(pid_inc_pi_calc_limited(&r_rate_pid,0,-10000,10000)==0);
 assert(pid_inc_pi_calc_limited(&r_rate_pid,5,-10000,10000)==101);
 return 0;
}
'''
    _compile_run(tmp_path, source)
