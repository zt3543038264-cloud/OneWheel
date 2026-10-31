"""Exercise the actual ported transport with simulated STM and WiFi datagrams."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import argparse
import os
parser=argparse.ArgumentParser()
parser.add_argument("--export-fixture")
parser.add_argument("--export-imu-fixture")
parser.add_argument("--export-angle-fixture")
parser.add_argument("--export-dual-fixture")
parser.add_argument("--export-startup-fixture")
args=parser.parse_args()
if args.export_startup_fixture: os.environ["QCAR_WIFI_STARTUP_FIXTURE"]=str(Path(args.export_startup_fixture).resolve())
if args.export_dual_fixture: os.environ["QCAR_WIFI_DUAL_FIXTURE"]=str(Path(args.export_dual_fixture).resolve())
if args.export_angle_fixture: os.environ["QCAR_WIFI_ANGLE_FIXTURE"]=str(Path(args.export_angle_fixture).resolve())
if args.export_fixture: os.environ["QCAR_WIFI_FIXTURE"]=str(Path(args.export_fixture).resolve())
if args.export_imu_fixture: os.environ["QCAR_WIFI_IMU_FIXTURE"]=str(Path(args.export_imu_fixture).resolve())

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='qcar-wifi-') as directory:
    work = Path(directory)
    shutil.copy2(root/'code/wifispi.c', work/'wifispi.c')
    (work/'zf_common_typedef.h').write_text('''
#ifndef HOST_TYPES
#define HOST_TYPES
#include <stdint.h>
#include <stdbool.h>
typedef uint8_t uint8; typedef uint16_t uint16; typedef uint32_t uint32;
typedef int16_t int16; typedef int32_t int32;
#endif
''')
    (work/'IfxStm.h').write_text('''
#include "zf_common_typedef.h"
extern int host_stm;
#define MODULE_STM0 host_stm
uint32 IfxStm_getLower(int *stm); float IfxStm_getFrequency(int *stm);
''')
    (work/'zf_common_interrupt.h').write_text('''
#include "zf_common_typedef.h"
uint32 interrupt_global_disable(void); void interrupt_global_enable(uint32);
''')
    (work/'zf_driver_delay.h').write_text('#include "zf_common_typedef.h"\nvoid system_delay_ms(uint32);\n')
    (work/'balance_control.h').write_text('''
#include "zf_common_typedef.h"
#include "qcar_types.h"
extern int16_t real_speed;
extern Mono_params mono_params;
extern float cascade_vel_out, cascade_vel_out_p, cascade_vel_out_y;
extern float cascade_angle_out, cascade_angle_out_p, cascade_angle_out_y;
extern float cascade_angle_vel_out, cascade_angle_vel_out_p, cascade_angle_vel_out_y;
''')
    (work/'small_driver_uart_control.h').write_text("""
#include "zf_common_typedef.h"
#include "qcar_types.h"
extern small_device_value_struct motor_value;
extern volatile uint32 small_driver_speed_frames,small_driver_feedback_age_ms;
extern volatile uint8 small_driver_feedback_fresh;
int small_driver_trace_request(unsigned);
int small_driver_trace_take(char *,unsigned);
extern volatile float small_driver_test_left,small_driver_test_right;
""")
    with (work/'balance_control.h').open('a') as header:
        header.write("""
extern volatile uint8 rate_only_test;
extern volatile uint8 balance_control_mode;
extern volatile uint8 pwm_debug_enabled;
extern volatile int32 pwm_step_state;
extern volatile uint32 pwm_step_ms,pwm_step_stage;
int pwm_step_command(int);
extern volatile float pwm_debug_fly_left,pwm_debug_fly_right,pwm_debug_ground;
extern volatile int32 control_fault_reason,control_fault_imu_status;
extern volatile uint32 control_fault_seq;
extern volatile float control_fault_dt_ms,control_startup_scale;
extern uint32 control_startup_ramp_ms;
extern volatile uint8 roll_rate_test_enabled, ground_rate_test_enabled, dual_balance_enabled;
extern volatile float dual_angle[2],dual_target[2],dual_angle_error[2],dual_rate[2],dual_target_rate[2],dual_rate_error[2],dual_pwm[2];
extern float ground_rate_test_kp, ground_rate_test_limit, ground_rate_test_sign;
extern float ground_angle_kp,ground_angle_rate_limit;
extern float ground_deadzone_pos_pwm,ground_deadzone_neg_pwm;
extern volatile uint32 roll_rate_test_sequence;
extern volatile int32 roll_rate_test_state;
extern volatile float roll_rate_test_actual, roll_rate_test_error, roll_rate_test_pwm;
extern volatile float pitch_balance_target_rate, pitch_balance_error;
extern volatile float pitch_balance_actual_angle, pitch_balance_target_angle;
extern float pitch_balance_kp, pitch_balance_rate_limit;
extern float roll_rate_test_kp, roll_rate_test_sign, roll_rate_test_limit;
extern float roll_rate_test_ki, flywheel_vel_kp;
extern float pitch_balance_kd, flywheel_vel_kd;
extern volatile float fly_angle_d_out,fly_vel_d_out;
extern volatile float flywheel_vel_angle_out;
extern int16 costrained_flywheel_speed_left, costrained_flywheel_speed_right, costrained_wheel_speed;
""")
    for enabled in (1, 0):
        exe = work/f'wifi-{enabled}.exe'
        subprocess.run(['gcc', '-x', 'c', '-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                        f'-DQCAR_WIFI_ENABLED={enabled}', '-I'+str(work), '-I'+str(root/'code'),
                        '-I'+str(root/'libraries/zf_device'), str(work/'wifispi.c'),
                        str(root/'tests/wifi_transport_test.c.in'), '-lm', '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True, timeout=10)
