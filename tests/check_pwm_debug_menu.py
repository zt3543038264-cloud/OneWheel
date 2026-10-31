"""Real menu Start/Stop/Mode transitions plus real controller, mocked hardware."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'code/menu.c').read_text(encoding='utf-8')

def function(signature):
    start = source.index(signature)
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + '\n'

boundary = r'''
#ifndef HOST_BOUNDARY
#define HOST_BOUNDARY
#include <stdint.h>
#include <stdbool.h>
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

prefix = r'''
#include "balance_control.h"
#include "wheel_hold.h"
#include "menu_tree.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define CCU60_CH0 0
#define CCU60_CH1 1
struct host_attitude attitude_state;
volatile float imu_roll_deg=90,imu_pitch_deg=90,imu_sample_dt_ms;
volatile uint32 imu_sample_sequence;
volatile int32 imu_attitude_status=-1;
static uint8 wheel_ready,driver_ready,imu_ready,running,preview,redraw,run_mode=APP_DEFAULT_MODE;
static bool control_timers_ready,wheel_switch,wifi_connecting;
static bool start_switch,stop_switch,imu_switch,wifi_switch,save_switch,load_switch;
static int wheel_test_pwm;
static Menu_Item Root;
static const char *message;
static float small_driver_test_left,small_driver_test_right;
static int imu_attempts,prepares,fly_l,fly_r,ground,timers[2],ready=1,profile,imu_ok;
uint32 interrupt_global_disable(void){return 0;}
void interrupt_global_enable(uint32 n){(void)n;}
void small_driver_set_duty(int16 l,int16 r){fly_l=l;fly_r=r;}
void motor_m_ctrl(int p){ground=p;}
static void pit_disable(int ch){timers[ch]=0;}
static void pit_init(int ch,int period){(void)period;timers[ch]=1;}
static void motor_int(void){}
static uint8 ensure_imu(void){++imu_attempts;if(imu_ok)imu_ready=1;return imu_ok;}
static void small_driver_uart_init(void){}
static void imu_prepare_start(void){++prepares;}
static int wifispi_is_ready(void){return ready;}
static void wifispi_telemetry_dual_profile(void){profile=2;}
static void wifispi_telemetry_roll_profile(void){profile=5;}
static void wifispi_telemetry_startup_profile(void){profile=1;}
static void wifispi_telemetry_speed_profile(void){profile=3;}
static void wifispi_telemetry_hold_profile(void){profile=4;}
static void tick(void){for(unsigned i=0;i<APP_CONTROL_DIVIDER;++i)balance_control_tick_1ms();}
'''

main = r'''
int main(void){
    assert(run_mode==5); /* Selection only, not auto-start. */
    balance_control_defaults();Menu_Tree_Reset();Menu_Create();All_Folder_Menu_Init(&Root);
    Menu_Item *folder=Root.First_Son;int saw_debug=0;
    for(unsigned n=0;n<Root.sons;++n,folder=folder->Next_Brother){
        if(strcmp(folder->name,"PWM debug")==0){
            saw_debug=1;assert(folder->sons==3);
            assert(folder->First_Son->data==(void *)&pwm_debug_fly_left);
            assert(folder->First_Son->Next_Brother->data==(void *)&pwm_debug_fly_right);
            assert(folder->First_Son->Next_Brother->Next_Brother->data==(void *)&pwm_debug_ground);
        }
    }
    assert(saw_debug && !running && !shacheflag);
    Menu_Item mode={0};mode.data=&run_mode;
    assert(!strcmp(Root.First_Son->First_Son->name,"Mode 2..8"));
    Menu_Handle_Control_Uint8(&mode,-100);assert(run_mode==2);
    assert(!strcmp(test_mode_name(),"DUAL + POSITION hold"));
    start_car();tick();
    assert(!running && !shacheflag && !roll_rate_test_enabled && !pwm_debug_enabled);
    assert(imu_attempts==1 && !prepares && !timers[0] && !ground && !fly_l && !fly_r);
    assert(strstr(message,"IMU init failed"));imu_attempts=0;
    Menu_Handle_Control_Uint8(&mode,-1);assert(run_mode==2);
    Menu_Handle_Control_Uint8(&mode,1);assert(run_mode==3);
    Menu_Handle_Control_Uint8(&mode,2);assert(run_mode==5);
    Menu_Handle_Control_Uint8(&mode,3);assert(run_mode==8);
    pwm_debug_fly_left=900;pwm_debug_fly_right=450;pwm_debug_ground=-800;start_car();
    assert(running && shacheflag && pwm_debug_enabled && !imu_attempts && !prepares);
    assert(timers[0] && !timers[1] && profile==2 && !pwm_debug_fly_left && !pwm_debug_fly_right && !pwm_debug_ground);
    control_startup_ramp_ms=0;pwm_debug_fly_left=300;pwm_debug_fly_right=-700;pwm_debug_ground=-500;tick();
    assert(fly_l==300 && fly_r==-700 && ground==-500);
    Menu_Handle_Control_Uint8(&mode,-6);assert(run_mode==2);
    assert(!running && !shacheflag && !ground && !fly_l && !fly_r && !timers[0]);
    start_car();tick();assert(!running && !ground && !fly_l && !fly_r && imu_attempts==1);imu_attempts=0;
    Menu_Handle_Control_Uint8(&mode,100);assert(run_mode==10);
    Menu_Handle_Control_Uint8(&mode,-1);assert(run_mode==8);
    start_car();tick();assert(!ground && !fly_l && !fly_r && !pwm_debug_fly_left && !pwm_debug_fly_right && !pwm_debug_ground);
    pwm_debug_fly_left=400;pwm_debug_fly_right=800;pwm_debug_ground=600;tick();assert(ground==600);
    stop_car(); /* Immediate hardware zero, no further ISR required. */
    assert(!running && !pwm_debug_enabled && !ground && !fly_l && !fly_r && !timers[0]);
    start_car();pwm_debug_fly_left=200;pwm_debug_fly_right=900;pwm_debug_ground=300;tick();
    Menu_Handle_Control_Uint8(&mode,-3);
    assert(run_mode==5 && !running && !ground && !fly_l && !fly_r && !pwm_debug_ground && !pwm_debug_fly_right);
    start_car();assert(imu_attempts==1 && !running && !pwm_debug_enabled && !ground && !fly_l);
    imu_ok=1;run_mode=2;imu_attitude_status=1;imu_sample_sequence=10;
    imu_roll_deg=BALANCE_ZERO_X_DEG;imu_pitch_deg=BALANCE_ZERO_Y_DEG;
    start_car();tick();
    assert(running && shacheflag && wheel_hold_enabled && dual_balance_enabled && balance_control_mode==2);
    assert(profile==4 && timers[0] && timers[1] && prepares==1 && !ground && !fly_l);
    stop_car();assert(!wheel_hold_enabled && !balance_control_mode);
    Menu_Handle_Control_Uint8(&mode,8);assert(run_mode==10);
    start_car();tick();
    assert(running && shacheflag && dual_balance_enabled && wheel_hold_enabled && balance_control_mode==10 && profile==5);
    stop_car();
    puts("PASS: Mode2 IMU gate, dual+speed arm, telemetry profile, mode-change zero; Mode8 independent PWM retained");
}
'''

parts = ''.join(function(s) for s in (
    'static void ensure_wheel(void)', 'static void stop_car(void)',
    'static void start_car(void)', 'void Menu_Create(void)',
    'static const char *test_mode_name(void)',
    'static bool Menu_Handle_Control_Uint8('))
with tempfile.TemporaryDirectory(prefix='onewheel-pwm-menu-') as directory:
    work=Path(directory)
    (work/'zf_common_headfile.h').write_text(boundary,encoding='utf-8')
    (work/'test.c').write_text(prefix+parts+main,encoding='utf-8')
    exe=work/'test.exe'
    subprocess.run(['gcc','-std=c99','-I'+str(work),'-I'+str(root/'code'),
                    str(root/'code/balance_control.c'),str(root/'code/wheel_hold.c'),str(root/'tests/encoder_snapshot_stub.c'),str(root/'code/pid.c'),
                    str(root/'code/menu_tree.c'),str(work/'test.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)
