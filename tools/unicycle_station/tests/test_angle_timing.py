"""Exercise real angle PID adapters and the ISR scheduler with host stubs."""
import json
import re

import pytest

from test_lean_limits import ROOT, _compile_run, _function


def test_angle_pid_physical_time_and_live_refresh(tmp_path):
    control = (ROOT/'code/Controller/control.c').read_text(encoding='utf-8')
    init = _function(control, 'control_init')
    calls = re.findall(r'^\s*angle_pid_init\([^\n]+;', init, re.M)
    assert len(calls) == 3
    source = r'''
#include "Common/board_config.h"
#include "Common/pid.c"
#include "Controller/control_angle.inc"
#include <assert.h>
#include <math.h>
param_t g_param;
static pid_t r_rcy_pid, r_angle_pid, r_rate_pid;
static pid_t p_vel_pid, p_angle_pid, p_rate_pid, y_angle_pid, y_rate_pid;
static void near(float a,float b) { assert(fabsf(a-b)<.001f); }
'''
    source += _function(control, 'cascade_gain_refresh')
    source += r'''
int main(void) {
    pid_t old, trial;
    int i;
    assert(CTRL_PERIOD_MS==1 && CTRL_DIV_ANGLE==1 && CTRL_DIV_ATT==5 && CTRL_DIV_SPEED==20);
    g_param.r_angle_kp=36; g_param.r_angle_ki=.03f; g_param.r_angle_kd=-2;
    g_param.p_angle_kp=9; g_param.p_angle_ki=-.02f; g_param.p_angle_kd=3;
    g_param.y_angle_kp=18; g_param.y_angle_ki=.01f; g_param.y_angle_kd=4;
    g_param.r_rate_kp=20; g_param.r_rate_kd=8; g_param.y_rate_kp=56;
'''
    source += '\n'.join(calls)
    source += r'''
    near(r_angle_pid.kp,36); near(r_angle_pid.ki,.006f); near(r_angle_pid.kd,-10);
    near(p_angle_pid.ki,-.004f); near(p_angle_pid.kd,15);
    near(y_angle_pid.ki,.002f); near(y_angle_pid.kd,20);
    near(r_angle_pid.imax,R_ANGLE_IMAX*5.0f);
    near(p_angle_pid.imax,P_ANGLE_IMAX*5.0f);
    near(y_angle_pid.imax,Y_ANGLE_IMAX*5.0f);
    r_angle_pid.integrator=17; r_angle_pid.last_error=3;
    g_param.r_angle_ki=-.04f; g_param.r_angle_kd=5;
    cascade_gain_refresh();
    near(r_angle_pid.ki,-.008f); near(r_angle_pid.kd,25);
    near(r_angle_pid.integrator,17); near(r_angle_pid.last_error,3);
    near(r_rate_pid.kp,20); near(r_rate_pid.kd,-8); near(y_rate_pid.kp,-56);
    /* Same constant error for one second: same integral contribution. */
    pid_set(&old,0,.03f,0,1000);
    angle_pid_init(&trial,0,.03f,0,1000);
    for(i=0;i<200;i++) pid_loc_calc(&old,2);
    for(i=0;i<1000;i++) pid_loc_calc(&trial,2);
    near(old.out_i,trial.out_i);
    /* Same integral output cap and sign after saturation. */
    pid_set(&old,0,-.2f,0,10);
    angle_pid_init(&trial,0,-.2f,0,10);
    for(i=0;i<100;i++) { pid_loc_calc(&old,2); pid_loc_calc(&trial,2); }
    near(old.out_i,trial.out_i);
    /* Same derivative of a linear ramp at common physical timestamps. */
    pid_set(&old,0,0,-3,10);
    angle_pid_init(&trial,0,0,-3,10);
    for(i=1;i<=1000;i++) {
        if(i%5==0) pid_loc_calc(&old,(float)i*.02f);
        if(i%CTRL_DIV_ANGLE==0) pid_loc_calc(&trial,(float)i*.02f);
        if(i%10==0) near(old.out_d,trial.out_d);
    }
    /* Existing P-only settings keep the same output for the same angle. */
    angle_pid_init(&trial,36,0,0,2000);
    near(pid_loc_calc(&trial,1.25f),45);
    /* A target step does not shrink with dt: its one-tick D kick doubles
       relative to the 500 Hz trial. This is expected, not a gain mismatch. */
    pid_set(&old,0,0,3*2.5f,10);
    angle_pid_init(&trial,0,0,3,10);
    pid_loc_calc(&old,0); pid_loc_calc(&trial,0);
    pid_loc_calc(&old,1); pid_loc_calc(&trial,1);
    near(trial.out_d,2*old.out_d);
    return 0;
}
'''
    _compile_run(tmp_path, source)


def test_actual_isr_publication_phase_and_independent_subrates(tmp_path):
    control = (ROOT/'code/Controller/control.c').read_text(encoding='utf-8')
    source = r'''
#include "Common/board_config.h"
#include <assert.h>
#define MOTOR_JOG_NONE 0
#define CTRL_TEST_STATUS_OK 0
#define CTRL_TEST_STATUS_SAFETY 1
static uint32 g_control_uptime_ms, g_vision_age_ms;
static uint8 g_imu_ok=1, s_angle_due, s_test_running, s_jog_target;
static int s_control_test_active, s_test_status;
static int samples, rates, core, publish, enc, path, feedback, keys, snapshots, uart, stops;
static uint32 last_core, last_publish;
static void W_Motor_Tick1ms(void) {}
static void key_scanner(void) { ++keys; }
static void Y_Motor_EncoderUpdate5ms(void) { assert(g_control_uptime_ms%5==0); ++enc; }
static void imu_update_gyro(void) { ++samples; }
static void attitude_update_rate(void) { ++rates; }
static void attitude_tick1ms(void) { ++core; last_core=g_control_uptime_ms; }
static void attitude_update(void) {
    assert(g_control_uptime_ms%CTRL_DIV_ANGLE==0 && last_core==g_control_uptime_ms);
    ++publish; last_publish=g_control_uptime_ms;
}
static void control_path_tick5ms(void) { assert(g_control_uptime_ms%5==0); ++path; }
static void motion_target_update(void) {}
static void control_vision_exchange(uint8 due) {
    assert(due==(g_control_uptime_ms%5==0)); feedback+=due;
}
static void check_angle_phase(void) {
    assert(s_angle_due==(g_control_uptime_ms%CTRL_DIV_ANGLE==0));
    if(s_angle_due) assert(last_publish==g_control_uptime_ms);
}
static void test_run(void) { check_angle_phase(); }
static void cascade_run(void) { check_angle_phase(); }
static void control_jog_run(void) {}
static void control_stop(void) { ++stops; check_angle_phase(); }
static void vofa_snapshot(void) { ++snapshots; }
static void vofa_tick1ms(void) { ++uart; }
/* This fixture tests cadence only. The complete real-controller host test
 * tests STM elapsed time, bench supervision and zero-sink behavior. */
typedef uint64_t uint64;
#define CONTROL_BENCH_MAX_GAP_US 2500u
#define CONTROL_BENCH_MAX_WORK_US 1000u
#define W_BENCH_REASON_TIMEOUT 4
static uint32 s_control_ticks_per_ms=1000;
static uint64 s_control_last_ticks;
static uint8 s_control_clock_seen;
static struct { uint32 period_us,max_period_us,work_us,max_work_us,overrun_count;uint8 valid; } s_control_timing;
static struct { uint8 active; } s_preview;
static uint64 control_stm_ticks(void) { return (uint64)g_control_uptime_ms*100; }
static uint8 control_clock_ready(void) { return 1; }
static uint32 control_ticks_us(uint64 x) { return (uint32)x; }
/* C bench behavior has separate actual-controller tests; inactive here so
 * this extraction continues to test the original publication/subrate phase. */
#define C_BENCH_REASON_TIMING 5
static void c_bench_observe(void) {}
static uint8 control_c_bench_running(void) { return 0; }
static void control_c_bench_run(void) { assert(0); }
static void c_bench_finish(int reason) { (void)reason; assert(0); }
static uint8 W_Motor_BenchBusy(void) { return 0; }
static void W_Motor_BenchAbort(int reason) { (void)reason;assert(0); }
static void control_bench_supervise(void) { assert(0); }
static void control_preview_run(void) { check_angle_phase(); }
static void control_locked_idle(void) { control_stop(); }
'''
    source += _function(control, 'control_loop')
    source += r'''
int main(void) {
    int i;
    for(i=1;i<=1000;i++) {
        /* Enter/leave modes at odd phases; angle publication stays aligned. */
        s_test_running=(uint8)(i>=101 && i<304);
        s_jog_target=(uint8)(i>=501 && i<604);
        control_loop();
    }
    assert(samples==1000 && rates==1000 && core==1000 && publish==1000);
    assert(enc==200 && path==200 && feedback==200 && keys==100);
    assert(snapshots==1000 && uart==1000 && stops==1000);
    return 0;
}
'''
    _compile_run(tmp_path, source)
    # Both real mode paths consume the ISR phase, rather than local mode time.
    for name in ('cascade_run', 'test_run'):
        body = _function(control, name)
        assert 'run_angle = s_angle_due;' in body
        assert 'run20 = ' in body and '== 0' in body
    for axis in ('roll', 'pitch', 'yaw'):
        for suffix in ('cascade_ctrl', 'test_ctrl'):
            body = _function(control, axis+'_'+suffix)
            assert 'if (run_angle)' in body


@pytest.mark.parametrize('relative', [
    'tools/unicycle_station/app/profiles/tc264_unicycle/profile.json',
    'tools/unicycle_station/examples/default.workspace.json',
])
def test_station_explains_reference_gains(relative):
    data = json.loads((ROOT/relative).read_text(encoding='utf-8'))
    profile = data.get('profile', data)
    for axis in 'rpy':
        for gain in 'pid':
            description = profile['parameters'][f'{axis}_angle_k{gain}']['description']
            assert '5 ms' in description and '1 kHz' in description
