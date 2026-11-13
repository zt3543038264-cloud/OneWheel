"""Type/syntax/API checks of all changed full driver C units using the real SDK.

This is native GCC syntax-only, NOT an ARM target build or linker check.
Device selectors follow cy_device_headers.h (CYT2BL3CAE) and the SDK board branch
(tviibe4m), despite the legacy SDK directory being named tviibh4m.
"""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / 'Seekfree_CYT2BL3_Double_Foc_Project'
UNITS = ('user/main_cm4.c', 'user/cm4_isr.c', 'code/motor_control.c',
         'code/pwm_output.c', 'code/motor_driver_uart_control.c',
         'code/motor_flash.c', 'code/motor_voice.c', 'code/driver_gpio.c')


def test_full_changed_units_with_bundled_sdk():
    include_paths = sorted(path for path in DRIVER.rglob('*') if path.is_dir())
    command = ['gcc', '-std=c11', '-fsyntax-only', '-D__ARM_ARCH_7EM__=1',
               '-Dtviibe4m=1', '-DCYT2BL3CAE=1', '-DCY_USE_PSVP=0',
               '-Werror=implicit-function-declaration',
               '-Werror=incompatible-pointer-types', '-Werror=int-conversion',
               # Native 64-bit GCC differs from the MCU's 32-bit pointer ABI.
               '-Wno-pointer-to-int-cast', '-Wno-int-to-pointer-cast']
    command += ['-I' + str(path) for path in include_paths]
    for unit in UNITS:
        result = subprocess.run(command + [str(DRIVER / 'project' / unit)],
                                capture_output=True, text=True, errors='replace')
        assert result.returncode == 0, unit + '\n' + result.stderr
        assert not result.stderr, unit + '\n' + result.stderr


if __name__ == '__main__':
    test_full_changed_units_with_bundled_sdk()
    print('PASS: 8 complete modified driver C units, native GCC syntax/API check with real SDK headers')
    print('NOT checked: ARM assembly/ABI/code generation, linking, memory map, reset/startup, real hardware')
