"""Execute the actual Path yaw cascade; no vehicle dynamics are simulated here."""
from test_lean_limits import ROOT, _compile_run, _function


def test_real_path_yaw_limits_stationary_damping_and_mode_isolation(tmp_path):
    control = (ROOT/'code/Controller/control.c').read_text(encoding='utf-8')
    source = r'''
#include "config.h"
#include "Common/param.h"
#include "Common/pid.c"
#include "Controller/Planner/path_nav.h"
#include <assert.h>
#include <math.h>
static pid_t y_angle_pid, y_rate_pid;
static int s_path_active=1, moving=1;
static float speed, yaw, g_yaw_target, g_pwm_roll, s_path_yaw_rate_cmd;
static float s_path_yaw_rate_target;
static float g_yaw_momentum_scale=1;
static struct { float gyro_z; } imu;
static float Y_Motor_GetSpeedMps(void) { return speed; }
static float imu_get_angle_yaw(void) { return yaw; }
int path_nav_running(void) { return moving; }
static int ctrl_is_finite(float v) { return isfinite(v); }
''' + _function(control, 'yaw_cascade_ctrl') + r'''
int main(void) {
    int i;
    pid_set(&y_angle_pid,18,.01f,0,100);
    pid_set(&y_rate_pid,-56,0,0,100);
    g_yaw_target=20;
    for(i=0;i<1000;i++) assert(yaw_cascade_ctrl(1)==0);
    assert(y_angle_pid.integrator==0 && s_path_yaw_rate_cmd==0);
    imu.gyro_z=5; assert(yaw_cascade_ctrl(1)==280); /* Damping remains. */
    imu.gyro_z=0; speed=.2f;
    for(i=0;i<1000;i++) {
        float before=s_path_yaw_rate_cmd;
        yaw_cascade_ctrl(1);
        assert(fabsf(s_path_yaw_rate_cmd-before)<=.1801f);
        assert(fabsf(s_path_yaw_rate_cmd)<=45);
    }
    assert(fabsf(s_path_yaw_rate_cmd-45)<.001f);
    g_yaw_momentum_scale=.25f; yaw_cascade_ctrl(0);
    assert(s_path_yaw_rate_cmd<=11.25f); /* Hard new permission, even between angle ticks. */
    g_pwm_roll=FLYWHEEL_OUT_LIMIT-100; assert(fabsf(yaw_cascade_ctrl(1))<=100);
    g_pwm_roll=FLYWHEEL_OUT_LIMIT; assert(yaw_cascade_ctrl(1)==0);
    g_pwm_roll=0; speed=0; assert(yaw_cascade_ctrl(1)==0);
    assert(s_path_yaw_rate_cmd==0 && y_angle_pid.integrator==0);
    speed=.2f; g_yaw_target=-20;
    for(i=0;i<1000;i++) yaw_cascade_ctrl(1);
    assert(s_path_yaw_rate_cmd<0 && y_rate_pid.out>0);
    moving=0; speed=0; g_yaw_target=1; /* DONE keeps bounded balanced heading hold. */
    for(i=0;i<1000;i++) yaw_cascade_ctrl(1);
    assert(s_path_yaw_rate_cmd>0);
    s_path_active=0; pid_set(&y_angle_pid,18,0,0,100); pid_set(&y_rate_pid,-56,0,0,100);
    g_yaw_target=1; assert(yaw_cascade_ctrl(1)==-1008); /* Other modes unchanged. */
    return 0;
}
'''
    _compile_run(tmp_path, source)
