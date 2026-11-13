"""Exercise the delivered UART parser/state and real PWM gate functions with host registers.

This does not model CPU IRQ routing, GPIO electrical polarity, or target startup.
Those remain explicit hardware verification requirements.
"""
from pathlib import Path
import subprocess
import re
import os

ROOT = Path(__file__).resolve().parents[1]
CODE = ROOT / 'Seekfree_CYT2BL3_Double_Foc_Project/project/code'
MOCK = ROOT / 'tests/driver_safety'


def body(source, signature):
    start = source.index(signature)
    end = source.index('\n}', start) + 2
    return source[start:end]


def test_real_uart_parser_and_pwm_gate(tmp_path):
    source = (CODE / 'pwm_output.c').read_bytes().decode('latin1')
    helpers = source[source.index('static uint8_t safe_right_connected;'):source.index('//----------------')]
    selected = ['void motor_left_duty_set(', 'void motor_right_duty_set(',
                'void motor_left_channel_set(', 'void motor_right_channel_set(']
    gate = tmp_path / 'actual_pwm_functions.c'
    gate.write_text('#include "driver_pwm_host.h"\n#include "driver_safety.h"\n' + helpers +
                    '\n'.join(body(source, name) for name in selected))
    exe = tmp_path / 'driver_safety_test'
    command = ['gcc', '-std=c11', '-g', '-O1', '-Wall', '-Wextra', '-Werror',
               '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-no-pie',
               '-DDRIVER_SAFETY_HOST', '-I' + str(MOCK), '-I' + str(CODE),
               str(CODE / 'motor_driver_uart_control.c'), str(gate),
               str(MOCK / 'test_driver_safety.c'), '-o', str(exe)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    env = dict(os.environ, ASAN_OPTIONS="detect_leaks=0")  # ptrace host cannot run LeakSanitizer.
    result = subprocess.run([str(exe)], check=True, capture_output=True, text=True, env=env)
    assert 'mocks passed' in result.stdout


def test_startup_and_calibration_source_contract():
    """Inspect actual default wiring and blocked alternate motion entry points."""
    read = lambda name: (CODE / name).read_bytes().decode('latin1')
    config = read('driver_config.h')
    assert re.search(r'#define\s+MOTOR_DRIVER_MODE\s+FAST_FOC_MODE', config)
    assert re.search(r'#define\s+USER_CONTROL_MODE\s+\(0\)', config)
    assert '#error "Commissioning safety requires FAST_FOC and UART control"' in config
    assert re.search(r'#define\s+SENSERLESS_WORK_MUSIC\s+DRIVER_DISABLE', config)
    main = (CODE.parent / 'user/main_cm4.c').read_bytes().decode('latin1')
    assert main.index('driver_safety_init();') < main.index('debug_init();') < main.index('motor_flash_init();') < main.index('motor_control_init();')
    assert 'pit_ms_init(PIT_CH0,1)' in main
    assert 'NVIC_SetPriorityGrouping(0u)' in main
    isr = (CODE.parent / 'user/cm4_isr.c').read_bytes().decode('latin1')
    pit = body(isr, 'void pit0_ch0_isr()')
    assert 'pit_isr_flag_clear(PIT_CH0)' in pit and 'driver_safety_tick_1ms();' in pit
    pwm = read('pwm_output.c')
    assert pwm.count('NVIC_SetPriority(tcpwm_irq_cfg.intIdx, 1u)') == 2
    assert 'NVIC_SetPriority(CPUIntIdx7_IRQn,2u)' in read('motor_driver_uart_control.c')
    assert len(re.findall(r'pwm_pin_config.hsiom\s*=\s*HSIOM_SEL_GPIO', pwm)) == 12
    assert not re.search(r'pwm_pin_config.hsiom\s*=\s*MOTOR_', pwm)
    control = read('motor_control.c')
    for signature in ['void motor_zero_calibration(void)', 'uint8 motor_both_zero_calibration(void)']:
        function = body(control, signature)
        assert 'driver_safety_disarm' in function and 'motor_flash_write' not in function
        assert 'interrupt_global_disable' not in function
    assert 'motor_left.motor_duty' not in body(control, 'void driver_cmd_forthwith(void)')
    flash = read('motor_flash.c')
    assert 'motor_flash_write();' not in body(flash, 'void motor_flash_read(void)')
    assert 'flash_erase_page' not in flash and 'flash_write_page_from_buffer' not in flash
    assert 'motor_calibration_valid=0' in flash and 'motor_calibration_valid=1' in flash
    assert 'UINT32_MAX' in flash  # Stored direction -1 remains valid.
    assert 'pwm_set_duty' not in body(read('motor_voice.c'), 'void motor_voice_output(void)')
    for side in ('left', 'right'):
        assert 'driver_safety_disarm' in body(pwm, f'void motor_{side}_phase_check(')


def test_real_calibration_reader(tmp_path):
    source = (CODE / 'motor_flash.c').read_bytes().decode('latin1')
    target = tmp_path / 'calibration_test.c'
    target.write_text(r'''#include <assert.h>
#include <stdint.h>
#include <string.h>
#define SAVE_FLASH_LENGTH 7
#define MOTOR_FLASH_SECTOR 0
#define MOTOR_FLASH_PAGE 11
#define ENCODER_PRECISION 32767
#define DRIVER_SAFE_MANUAL 2
typedef uint32_t uint32;
struct {uint32_t uint32_type;} flash_union_buffer[7];
uint32_t stored[7];unsigned read_limit=7;
struct {uint32_t zero_location,pole_pairs;int32_t rotation_direction;} motor_left,motor_right;
uint8_t motor_calibration_valid;
unsigned reads,stops;
void flash_read_page_to_buffer(unsigned sector,unsigned page,unsigned length) {
  unsigned i;assert(sector==0 && page==11 && length==7);++reads;
  for(i=0;i<read_limit;i++)flash_union_buffer[i].uint32_type=stored[i];
}
void driver_safety_disarm(unsigned reason) {assert(reason==2);++stops;}
''' + body(source, 'void motor_flash_read(void)') + '\n' + body(source, 'void motor_flash_write(void)') + r'''
int main(void) {
  uint32_t good[]={123,1,7,456,UINT32_MAX,7,0x4068};unsigned i;
  for(i=0;i<7;i++)stored[i]=good[i];
  motor_flash_read();assert(motor_calibration_valid && reads==1);
  assert(motor_left.zero_location==123 && motor_right.zero_location==456);
  assert(motor_left.rotation_direction==1 && motor_right.rotation_direction==-1);
  assert(motor_left.pole_pairs==7 && motor_right.pole_pairs==7);
  for(i=0;i<7;i++) {
    uint32_t old=stored[i];
    stored[i]=(i==6)?0:((i==0 || i==3)?32767:0);
    motor_flash_read();assert(!motor_calibration_valid && !stops);
    assert(flash_union_buffer[i].uint32_type==((i==6)?0:((i==0 || i==3)?32767:0)));
    stored[i]=old;
  }
  read_limit=3;motor_flash_read();assert(!motor_calibration_valid);
  assert(flash_union_buffer[6].uint32_type==UINT32_MAX);
  motor_flash_write();assert(stops==1);
  for(i=0;i<7;i++)assert(stored[i]==good[i]);
  return 0;
}
''')
    exe = tmp_path / 'calibration_test'
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(target), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    import tempfile
    test_startup_and_calibration_source_contract()
    with tempfile.TemporaryDirectory() as directory:
        test_real_uart_parser_and_pwm_gate(Path(directory))
        test_real_calibration_reader(Path(directory))
    print('PASS: real-source parser/PWM ASan+UBSan, calibration, default startup source contract')
