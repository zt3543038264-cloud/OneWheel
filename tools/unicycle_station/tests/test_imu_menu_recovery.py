"""Production recovery/menu transitions and real driver calibration.

Mocked peripherals do not prove on-board timing or physical recovery.
"""
import os
import re
import shutil
import subprocess
import pytest

from test_lean_limits import ROOT, _compile_run, _function


@pytest.mark.parametrize('entry,arm',[
    ('balance_action','s_balance_armed_until_ms'),
    ('remote_action','s_remote_armed_until_ms'),
    ('run_action','s_run_armed_until_ms'),
])
def test_recovery_confirmation_failures_and_fresh_key_release(tmp_path,entry,arm):
    header = (ROOT/'code/Controller/control.h').read_text(encoding='utf-8')
    enum = re.search(r'typedef enum \{\s*CTRL_IMU_IDLE.*?control_imu_recovery_t;', header, re.S).group()
    menu = (ROOT/'code/Menu/menu.c').read_text(encoding='utf-8')
    source = r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
typedef uint8_t uint8; typedef uint32_t uint32; typedef int32_t int32;
typedef int control_test_status_t;
enum {CTRL_TEST_STATUS_OK, CTRL_TEST_STATUS_IMU_LOST, CTRL_TEST_STATUS_IMU_CALIB,
      CTRL_TEST_STATUS_IMU_FAIL, CTRL_TEST_STATUS_ATT_DIVERGED};
enum {START_STOP,START_BALANCE};
#define MOTOR_JOG_NONE 0
#define IMU_CALIB_OK 1
static volatile uint8 g_imu_ok;
static uint32 g_control_uptime_ms;
static int start_flag,s_test_running,s_jog_target,s_pitch_rate_cache_valid,s_test_status;
static float g_yaw_target;
static int lost,calib,diverged,converged,pending,path_busy,init_fail,align_fail;
static int cal_ticks,cal_done=3,cal_fail,init_calls,align_calls,stop_calls,start_calls,invalidated;
static uint8 imu_link_lost(void){return lost;}
static uint8 imu_calib_state(void){return calib;}
static uint8 imu_forward_pending(void){return pending;}
static uint8 attitude_diverged(void){return diverged;}
static uint8 attitude_converged(void){return converged;}
static uint8 path_nav_busy(void){return path_busy;}
static uint32 interrupt_global_disable(void){return 0;}
static void interrupt_global_enable(uint32 x){(void)x;}
static void control_stop(void){start_flag=START_STOP;stop_calls++;}
static void path_nav_invalidate_heading(void){invalidated++;}
static uint8 imu_init(void){assert(!g_imu_ok);init_calls++;lost=0;calib=0;return init_fail;}
static void imu_calibrate_begin(void){cal_ticks=0;}
static uint8 imu_calibrate_busy(void){return cal_ticks<cal_done;}
static void imu_calibrate_tick(void){if(++cal_ticks>=cal_done)calib=cal_fail?2:IMU_CALIB_OK;}
static void attitude_init(void){assert(!g_imu_ok);align_calls++;diverged=align_fail;converged=0;}
static float imu_get_angle_yaw(void){return 12.0f;}
'''+enum+'\n'+(ROOT/'code/Controller/control_imu_recovery.inc').read_text(encoding='utf-8')+r'''
#define UI_BALANCE_CONFIRM_MS 3000u
#define UI_LIVE_PERIOD_MS 100u
#define UI_STATUS_Y 277
#define UI_YELLOW 1
#define KEY_NUMBER 4
#define KEY_LIST {0,1,2,3}
#define KEY_RELEASE_LEVEL 1
typedef int gpio_pin_enum; typedef int key_index_enum;
static uint32 s_balance_armed_until_ms,s_remote_armed_until_ms,s_run_armed_until_ms;
static uint32 s_reset_armed_until_ms,s_path_armed_until,s_last_repeat_ms[4];
static uint8 s_path_armed_action,s_enter_event_long,s_long_active[4];
static uint8 s_balance_on,s_test_on,s_run_on,s_editing,s_active_test,s_dirty;
static int physical_keys[4]={1,1,1,1},key_events[4],draws;
static char s_status[80];
static void key_clear_state(int i){key_events[i]=0;}
static int gpio_get_level(int i){return physical_keys[i];}
static void menu_status(const char *s){strcpy(s_status,s);}
static void render_page(void){draws++;}
static void ui_line(int y,const char *s,int c){(void)y;(void)s;(void)c;}
static const char *test_status_text(int s){(void)s;return "IMU FAILED";}
static int control_test_last_status(void){return s_test_status;}
static int control_test_running(void){return s_test_running;}
static int control_jog_running(void){return s_jog_target;}
static int control_balance_start(void){assert(!control_imu_recovery_busy());start_calls++;start_flag=START_BALANCE;return 1;}
static int running_kind;
static int control_remote_running(void){return start_flag==START_BALANCE && running_kind==1;}
static int control_run_running(void){return start_flag==START_BALANCE && running_kind==2;}
static int control_remote_start(void){running_kind=1;return control_balance_start();}
static int control_run_start(void){running_kind=2;return control_balance_start();}
static void control_remote_stop(void){control_stop();}
static void control_run_stop(void){control_stop();}
static int g_track_valid=1;
#define VISION_CORE_READY 1
static int vision_core_state(void){return VISION_CORE_READY;}
'''+(ROOT/'code/Menu/menu_imu_recovery.inc').read_text(encoding='utf-8')+'\n'
    source += '\n'.join(_function(menu, name) for name in ('balance_action','remote_action','run_action'))
    source += '\n#define tested_action '+entry+'\n#define tested_arm '+arm+'\n'+r'''
static void request(void){
 g_control_uptime_ms+=4000;tested_action();assert(tested_arm);
 tested_action();assert(!tested_arm);assert(start_flag==START_STOP);
 assert(control_imu_recovery_state()==CTRL_IMU_INIT);
}
static void release(void){
 assert(menu_imu_recovery_guard());
 physical_keys[2]=0;g_control_uptime_ms+=500;assert(menu_imu_recovery_guard());
 assert(!tested_arm && !key_events[2]);
 physical_keys[2]=1;g_control_uptime_ms+=151;assert(menu_imu_recovery_guard());
 assert(!menu_imu_recovery_guard());
}
int main(void){
 g_control_uptime_ms=1;g_imu_ok=1;calib=IMU_CALIB_OK;diverged=1;
 request();assert(init_calls==0 && invalidated==1 && start_calls==0);
 assert(menu_imu_recovery_guard());control_imu_recovery_poll();
 assert(init_calls==1 && !g_imu_ok && control_imu_recovery_busy());
 for(int i=0;i<3;i++){g_control_uptime_ms++;control_imu_recovery_tick();}
 assert(control_imu_recovery_state()==CTRL_IMU_ALIGN);
 assert(!g_imu_ok);control_imu_recovery_poll();assert(g_imu_ok && align_calls==1);
 assert(control_imu_recovery_busy() && !control_imu_recovery_request());
 control_imu_recovery_poll();assert(start_calls==0);
 converged=1;control_imu_recovery_poll();assert(control_imu_recovery_state()==CTRL_IMU_READY);
 assert(start_calls==0 && start_flag==START_STOP && g_yaw_target==12);
 s_run_armed_until_ms=s_path_armed_until=99;key_events[2]=1;
 release();assert(!s_run_armed_until_ms && !s_path_armed_until && !s_status[0]);
 tested_action();assert(start_calls==0);tested_action();assert(start_calls==1);
 tested_action();assert(start_flag==START_STOP);
 // Immediate init failure must also discard confirmations and held keys.
 diverged=1;init_fail=1;request();control_imu_recovery_poll();
 assert(control_imu_recovery_state()==CTRL_IMU_FAILED && !g_imu_ok);
 release();assert(!strcmp(s_status,"IMU FAILED") && start_calls==1);
 init_fail=0;cal_fail=1;request();control_imu_recovery_poll();
 for(int i=0;i<3;i++)control_imu_recovery_tick();
 assert(control_imu_recovery_state()==CTRL_IMU_FAILED && s_test_status==CTRL_TEST_STATUS_IMU_CALIB);
 release();cal_fail=0;align_fail=1;request();control_imu_recovery_poll();
 for(int i=0;i<3;i++)control_imu_recovery_tick();control_imu_recovery_poll();
 assert(control_imu_recovery_state()==CTRL_IMU_FAILED && !g_imu_ok);release();
 align_fail=0;request();control_imu_recovery_poll();
 for(int i=0;i<3;i++)control_imu_recovery_tick();control_imu_recovery_poll();
 lost=1;control_imu_recovery_poll();assert(s_test_status==CTRL_TEST_STATUS_IMU_LOST);release();
 lost=0;request();control_imu_recovery_poll();
 for(int i=0;i<3;i++)control_imu_recovery_tick();control_imu_recovery_poll();
 g_control_uptime_ms+=5001;control_imu_recovery_poll();
 assert(control_imu_recovery_state()==CTRL_IMU_FAILED && start_calls==1);release();
 pending=1;assert(!control_imu_recovery_request());pending=0;
 path_busy=1;assert(!control_imu_recovery_request());path_busy=0;
 start_flag=START_BALANCE;assert(!control_imu_recovery_request());start_flag=START_STOP;
 // Calibration deadline fails closed across uptime rollover.
 g_control_uptime_ms=0xfffffff0u;assert(control_imu_recovery_request());control_imu_recovery_poll();
 g_control_uptime_ms+=20001u;control_imu_recovery_poll();
 assert(control_imu_recovery_state()==CTRL_IMU_FAILED && !g_imu_ok && start_calls==1);
 return 0;
}
'''
    _compile_run(tmp_path, source)


def test_real_calibration_async_retry_invalid_sample_and_boot_wrapper(tmp_path):
    fixture = (ROOT/'tools/attitude_r10h/integration.c').read_text(encoding='utf-8')
    fixture = fixture[:fixture.index('static void init(void)')]
    source = fixture+r'''
static void reset(void) {
 bad_mode=moving=0;status=3;host_clock=host_clock_cost=reads=delay_ms=0;
 sensor[0]=14;sensor[1]=-28;sensor[2]=42;sensor[3]=4098;sensor[4]=sensor[5]=0;
 assert(!imu_init());
}
int main(void) {
 imu_data_t mean;
 reset();imu_calibrate_begin();unsigned delay_before=delay_ms;
 for(int i=0;i<5999;i++){assert(imu_calibrate_busy());imu_calibrate_tick();}
 assert(imu_calib_state()!=IMU_CALIB_OK && !imu_get_startup_mean(&mean));
 imu_calibrate_tick();assert(!imu_calibrate_busy() && imu_calib_state()==IMU_CALIB_OK);
 assert(reads==6000 && delay_ms==delay_before && imu_get_startup_mean(&mean));
 assert(fabsf(mean.gyro_z-14/14.3f)<.001f);
 reset();moving=1;imu_calibrate_begin();
 for(int i=0;i<6000;i++)imu_calibrate_tick();assert(imu_calibrate_busy());
 moving=0;for(int i=0;i<6000;i++)imu_calibrate_tick();
 assert(!imu_calibrate_busy() && reads==12000 && imu_calib_state()==IMU_CALIB_OK);
 reset();moving=1;imu_calibrate_begin();
 for(int i=0;i<18000;i++)imu_calibrate_tick();
 assert(!imu_calibrate_busy() && imu_calib_state()==IMU_CALIB_MOVED && !imu_get_startup_mean(&mean));
 for(int bad=1;bad<=5;bad++) {
  reset();imu_calibrate_begin();bad_mode=bad;imu_calibrate_tick();
  assert(imu_link_lost() && !imu_calibrate_busy() && !imu_get_startup_mean(&mean));
  bad_mode=0;imu_calibrate_begin();assert(!imu_calibrate_busy());
 }
 reset();imu_calibrate();assert(reads==6000 && delay_ms>=6000 && imu_calib_state()==IMU_CALIB_OK);
 return 0;
}
'''
    script = tmp_path/'calibration.c'
    script.write_text(source, encoding='utf-8')
    aggregate = tmp_path/'aggregate.c'
    aggregate.write_text('#include "'+str(ROOT/'code/HW_Drivers/IMU/imu.c').replace('\\','/')+'"\n'+
                         '#include "'+script.as_posix()+'"\n', encoding='utf-8')
    tcc = os.environ.get('ONEWHEEL_HOST_TCC')
    includes = ['-I'+str(ROOT/'tools/attitude_r10h/mocks'), '-I'+str(ROOT/'code')]
    if tcc:
        command = [tcc, '-DR10H_HOST_TEST',
            '-I'+str(ROOT/'tools/unicycle_station/tests/firmware_host/tcc'),
            *includes, '-run', str(aggregate)]
    else:
        gcc = shutil.which('gcc') or 'D:/Tools/Dev-Cpp/MinGW64/bin/gcc.exe'
        exe = tmp_path/'calibration.exe'
        build = subprocess.run([gcc, '-std=c99', '-DR10H_HOST_TEST', *includes,
            str(aggregate), '-lm', '-o', str(exe)], capture_output=True,text=True,timeout=30)
        assert build.returncode == 0, build.stdout+build.stderr
        command = [str(exe)]
    result = subprocess.run(command, capture_output=True,text=True,timeout=30)
    assert result.returncode == 0, result.stdout+result.stderr


def test_yaw_reset_preserves_recorded_route_and_invalidates_armed_revision(tmp_path):
    source = '#define PATH_HOST_TEST\n'+(ROOT/'code/Controller/Planner/path_nav.c').read_text(encoding='utf-8')+r'''
#include <assert.h>
int main(void) {
 path_point_t points[]={{0,0,0},{0,.5f,.5f}},readback;
 path_status_t before,after;
 path_nav_init();assert(path_nav_import(points,2));path_nav_saved();
 assert(path_nav_lock_heading(100));path_nav_status(&before);
 uint32_t rev=path_nav_revision();assert(!path_nav_heading_ready(0));
 path_nav_invalidate_heading();path_nav_status(&after);
 assert(!after.heading_locked && path_nav_revision()!=rev);
 assert(after.count==before.count && after.valid==before.valid && after.dirty==before.dirty);
 assert(after.length_m==before.length_m && path_nav_point(1,&readback));
 assert(!memcmp(&readback,&points[1],sizeof(readback)));
 assert(path_nav_heading_ready(0));path_nav_prepare();assert(path_nav_start(0,.2f));
 return 0;
}
'''
    _compile_run(tmp_path, source)
