"""No hardware. Geometry and hard output lock contracts for RA review snapshot."""
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[1]

def compile_run(tmp, source, flags=()):
    p=tmp/'test.c'; p.write_text(source)
    exe=tmp/'test'
    r=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',*flags,'-I'+str(ROOT/'code'),str(p),'-o',str(exe)],capture_output=True,text=True)
    assert r.returncode==0,r.stderr
    r=subprocess.run([str(exe)],capture_output=True,text=True,timeout=10)
    assert r.returncode==0,r.stderr+r.stdout

def test_geometry_invariants(tmp_path):
    compile_run(tmp_path,r'''
#include <assert.h>
#include "Common/vehicle_profile.h"
int main(void) {
 assert(VEHICLE_MOTOR_OUTPUT_ALLOWED == 0);
 for (int roll=-10000;roll<=10000;roll+=500) {
  assert(vehicle_roll_command((float)roll)==roll);
  for (int yaw=-10000;yaw<=10000;yaw+=500) {
   assert(vehicle_yaw_command((float)yaw)==0);
   assert(vehicle_roll_recovery_rpm((float)yaw,(float)roll)==-roll);
   assert(vehicle_yaw_momentum_rpm((float)yaw,(float)roll)==yaw);
  }
 }
 return 0;
}
''')

def test_independent_yaw_geometry_still_output_locked(tmp_path):
    compile_run(tmp_path,r'''
#include <assert.h>
#include "Common/vehicle_profile.h"
int main(void) {
 assert(VEHICLE_MOTOR_OUTPUT_ALLOWED == 0);
 assert(vehicle_yaw_command(10000)==10000);
 assert(vehicle_roll_command(-10000)==-10000);
 return 0;
}
''',['-DVEHICLE_YAW_ENABLED=1'])

def function(src,name):
    a=src.index(name+'('); a=src.index('{',a); depth=1;b=a+1
    while depth:
        depth += (src[b]=='{')-(src[b]=='}'); b+=1
    return src[a:b]

def test_all_entry_points_and_output_paths_locked():
    s=(ROOT/'code/Controller/control.c').read_text()
    for name in ['control_balance_start','control_run_start','control_remote_start','control_odometry_test_start','control_jog_start','control_test_start','control_path_start']:
        body=function(s,name)
        assert 'if (!VEHICLE_MOTOR_OUTPUT_ALLOWED)' in body,name
        assert 'CTRL_TEST_STATUS_ADAPT_LOCKED' in body,name
    assert 'vehicle_roll_recovery_rpm' in function(s,'roll_recovery_offset')
    assert 'vehicle_yaw_momentum_rpm' in function(s,'roll_recovery_offset')
    assert 'mix_a = (int32)yaw_cmd' in s and 'mix_b = (int32)vehicle_roll_command(roll_cmd)' in s
    assert 'g_motor_a = 0;\n        g_motor_b = (int16)vehicle_roll_command(output);' in s
    assert 'g_motor_a = (int16)vehicle_yaw_command(output);\n        g_motor_b = 0;' in s
    assert 'if (target == MOTOR_JOG_A && !VEHICLE_YAW_ENABLED)' in s
    assert 'if (axis == TUNE_AXIS_YAW && !VEHICLE_YAW_ENABLED)' in s
    assert '!ctrl_is_finite(g_pwm_roll)' in s
    w=(ROOT/'code/HW_Drivers/Motor/W_Motor.c').read_text()
    ordinary=function(w,'W_Motor_SetDuty')
    assert '(void)motor1_duty; (void)motor2_duty;' in ordinary
    assert 'w_motor_brake_locked = 1;' in ordinary
    assert 'W_Motor_SendFrame(W_MOTOR_SET_DUTY_CMD);' in ordinary
    assert 'wm_text_command(command, 0, 0);' in w
    assert 'if (!VEHICLE_MOTOR_OUTPUT_ALLOWED) duty = 0;' in (ROOT/'code/HW_Drivers/Motor/Y_Motor.c').read_text()
    init=function(s,'control_init')
    assert init.index('W_Motor_Init();') < init.index('param_init();') < init.index('imu_init()')
    assert init.index('Y_Motor_Init();') < init.index('param_init();')

def test_fixed_ekf_frame_cannot_flip_mechanical_zeros():
    s=(ROOT/'code/Common/param.c').read_text()
    body=function(s,'param_set_imu_forward_dir')
    assert '(void)direction;' in body
    assert 'g_param.' not in body
    assert 'value == (float)g_param.imu_forward_dir' in function(s,'param_set_by_name')

def test_no_private_wifi_dependency():
    s=(ROOT/'code/Protocols/station_wifi_config.h').read_text()
    assert '#define STATION_WIFI_ENABLED 0' in s
    assert '#define STATION_WIFI_PASSWORD ""' in s
    assert 'STATION_WIFI_ENABLED=1' not in (ROOT/'.cproject').read_text()
    assert 'excluding="code/Estimation/Attitude/r10h|' in (ROOT/'.cproject').read_text()

def test_ground_driver_cannot_output(tmp_path):
    (tmp_path/'zf_common_headfile.h').write_text(r'''#ifndef HOST_ZF
#define HOST_ZF
#include <stdint.h>
typedef uint8_t uint8; typedef uint16_t uint16; typedef uint32_t uint32;
typedef int16_t int16; typedef int32_t int32;
#define P21_4 1
#define ATOM0_CH3_P21_5 2
#define GPIO_LOW 0
#define GPIO_HIGH 1
#define GPO 1
#define GPO_PUSH_PULL 1
#define TIM2_ENCODER 1
#define TIM2_ENCODER_CH1_P33_7 1
#define TIM2_ENCODER_CH2_P33_6 1
#define func_limit_ab(x,l,h) ((x)<(l)?(l):((x)>(h)?(h):(x)))
void gpio_init(int,int,int,int); void pwm_init(int,int,int); void encoder_dir_init(int,int,int);
void encoder_clear_count(int); int encoder_get_count(int); void gpio_set_level(int,int); void pwm_set_duty(int,uint32);
uint32 interrupt_global_disable(void); void interrupt_global_enable(uint32);
#endif
''')
    compile_run(tmp_path,r'''
#include <assert.h>
#include "HW_Drivers/Motor/Y_Motor.c"
param_t g_param;
static unsigned writes;
void gpio_init(int a,int b,int c,int d){(void)a;(void)b;(void)c;(void)d;}
void pwm_init(int a,int b,int c){(void)a;(void)b;assert(c==0);}
void encoder_dir_init(int a,int b,int c){(void)a;(void)b;(void)c;}
void encoder_clear_count(int a){(void)a;} int encoder_get_count(int a){(void)a;return 0;}
void gpio_set_level(int a,int b){(void)a;(void)b;}
void pwm_set_duty(int a,uint32 duty){(void)a;assert(duty==0);++writes;}
uint32 interrupt_global_disable(void){return 0;} void interrupt_global_enable(uint32 a){(void)a;}
int main(void){
 g_param.motor_dir_c=1;g_param.enc_dir_c=1;g_param.odom_counts_per_m=1000;
 Y_Motor_Init();
 for(int i=-100000;i<=100000;i+=100)Y_Motor_SetDuty(i);
 g_param.motor_dir_c=-1;Y_Motor_SetDuty(10000);Y_Motor_Stop();
 assert(writes>2000);return 0;
}
''',['-I'+str(tmp_path),'-Wno-unknown-pragmas'])

if __name__=='__main__':
    with tempfile.TemporaryDirectory() as d:
        t=Path(d);test_geometry_invariants(t);test_independent_yaw_geometry_still_output_locked(t);test_ground_driver_cannot_output(t)
    test_all_entry_points_and_output_paths_locked();test_no_private_wifi_dependency()
    print('PASS: 5 orthogonal review groups (geometry, dormant yaw, locks, clean WiFi)')
