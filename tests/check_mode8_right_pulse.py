"""Host regression for the finite right/vertical-flywheel Mode8 pulse."""
from pathlib import Path
import ast
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
tree = ast.parse((root / 'tests/check_balance_control.py').read_text(encoding='utf-8-sig'))
texts = [n.args[0].value for n in ast.walk(tree)
         if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
         and n.func.attr == 'write_text' and n.args
         and isinstance(n.args[0], ast.Constant)]
boundary = next(s for s in texts if '#ifndef HOST_BOUNDARY' in s)
fixture = next(s for s in texts if 'int main(void)' in s).split('int main(void)')[0]
test = r'''
int main(void) {
    balance_control_defaults();
    assert(!right_pulse_command(1));
    pwm_debug_arm();shacheflag=1;control_startup_ramp_ms=0;
    tick(0);assert(control_startup_scale==1);

    flywheel_speed_feedback_fresh=0;
    assert(!right_pulse_command(1));
    flywheel_speed_feedback_fresh=1;
    flywheel_speed_feedback_age_ms=RIGHT_PULSE_START_FEEDBACK_MAX_AGE_MS+1;
    assert(!right_pulse_command(1));flywheel_speed_feedback_age_ms=0;
    flywheel_speed_feedback_right=RIGHT_PULSE_START_SPEED_MAX+1;
    assert(!right_pulse_command(1));flywheel_speed_feedback_right=0;
    pwm_debug_fly_left=1;assert(!right_pulse_command(1));pwm_debug_fly_left=0;
    assert(!right_pulse_command(2));

    assert(right_pulse_command(1));
    assert(!right_pulse_command(1)); /* a duplicate cannot extend the timeout */
    for (unsigned t=0;t<RIGHT_PULSE_DURATION_MS;t+=APP_IMU_PERIOD_US/1000u) {
        tick(0);
        assert(right_pulse_state==1 && right_pulse_ms==t);
        assert(!fly_l && fly_r==(int)RIGHT_PULSE_PWM && !ground);
        assert(right_pulse_pwm==(int)RIGHT_PULSE_PWM);
    }
    tick(0);
    assert(right_pulse_state==2 && right_pulse_ms==RIGHT_PULSE_DURATION_MS);
    assert(!right_pulse_pwm && !fly_l && !fly_r && !ground);

    /* Four explicit graded pulses: no automatic level advance or arbitrary PWM. */
    const int expected[4]={150,300,450,600};
    for (int level=1;level<=4;++level) {
        assert(right_pulse_level_command(level));tick(0);
        assert(right_pulse_state==1 && right_pulse_level==(uint32)level);
        assert(right_pulse_duration_ms==RIGHT_PULSE_GRADED_DURATION_MS);
        assert(!fly_l && fly_r==expected[level-1] && !ground);
        for (unsigned t=APP_IMU_PERIOD_US/1000u;t<RIGHT_PULSE_GRADED_DURATION_MS;t+=APP_IMU_PERIOD_US/1000u) tick(0);
        tick(0);assert(right_pulse_state==2 && !fly_l && !fly_r && !ground);
    }
    assert(!right_pulse_level_command(5));

    assert(right_pulse_command(-1));tick(0);
    assert(!fly_l && fly_r==-(int)RIGHT_PULSE_PWM && !ground);
    assert(right_pulse_command(0));tick(0);
    assert(right_pulse_state==-1 && !fly_l && !fly_r && !ground);

    assert(right_pulse_command(1));tick(0);
    flywheel_speed_feedback_fresh=0;tick(0);
    assert(right_pulse_state==-1 && roll_rate_test_state==-3 && control_fault_reason==7);
    assert(!fly_l && !fly_r && !ground);

    roll_rate_test_stop();pwm_debug_arm();shacheflag=1;control_startup_ramp_ms=0;
    flywheel_speed_feedback_fresh=1;flywheel_speed_feedback_age_ms=0;tick(0);
    assert(right_pulse_level_command(-4));tick(0);
    flywheel_speed_feedback_right=RIGHT_PULSE_RUN_SPEED_MAX+1;tick(0);
    assert(right_pulse_state==-1 && roll_rate_test_state==-3 && control_fault_reason==12);
    assert(!fly_l && !fly_r && !ground);

    roll_rate_test_stop();
    assert(!right_pulse_state && !right_pulse_ms && !right_pulse_pwm);
    pwm_debug_arm();shacheflag=1;control_startup_ramp_ms=0;flywheel_speed_feedback_fresh=1;
    flywheel_speed_feedback_left=flywheel_speed_feedback_right=0;tick(0);
    assert(right_pulse_command(1));tick(0);roll_rate_test_stop();tick(0);
    assert(!fly_l && !fly_r && !ground);
    puts("PASS: right pulse gates, fixed outputs, timeout, cancel, stale feedback and Stop");
}
'''

with tempfile.TemporaryDirectory(prefix='qcar-right-pulse-') as directory:
    work = Path(directory)
    (work / 'zf_common_headfile.h').write_text(boundary, encoding='utf-8')
    (work / 'test.c').write_text(fixture + test, encoding='utf-8')
    exe = work / 'test.exe'
    subprocess.run(['gcc', '-std=c99', '-UNDEBUG', '-I' + str(work),
                    '-I' + str(root / 'code'), str(root / 'code/balance_control.c'),
                    str(root / 'code/wheel_hold.c'), str(root / 'tests/encoder_snapshot_stub.c'),
                    str(root / 'code/pid.c'), str(work / 'test.c'), '-lm', '-o', str(exe)],
                   check=True)
    subprocess.run([str(exe)], check=True, timeout=5)
