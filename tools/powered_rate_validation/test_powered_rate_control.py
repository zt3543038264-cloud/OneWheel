from pathlib import Path
import os
import subprocess
ROOT=Path(os.environ.get("ONEWHEEL_PROJECT",str(Path(__file__).resolve().parents[2])))
SUPPORT=Path(__file__).resolve().parent/"support"

def test_actual_powered_rate_controller(tmp_path):
    text=(SUPPORT/'control_host.c').read_text()
    text=text.replace('int main(void){','#include "powered_rate_cases.inc"\nint main(void){')
    text=text.replace(' c_bench_cases(); puts(', ' powered_rate_cases(); puts(')
    source=tmp_path/'control_powered_host.c';source.write_text(text)
    exe=tmp_path/'powered'
    command=['gcc','-std=c99','-O2','-Wall','-Wextra','-Wno-unknown-pragmas',
        '-Wno-unused-function','-Wno-unused-parameter','-Wno-misleading-indentation',
        '-ffunction-sections','-fdata-sections','-fsanitize=undefined','-fno-sanitize-recover=all',
        '-DHOST_C_BENCH_TEST=1','-I'+str(SUPPORT),'-I'+str(ROOT/'code'),
        str(source),'-Wl,--gc-sections','-lm','-o',str(exe)]
    result=subprocess.run(command,capture_output=True,text=True);assert result.returncode==0,result.stderr
    result=subprocess.run([str(exe)],capture_output=True,text=True);assert result.returncode==0,result.stdout+result.stderr
    assert 'PASS powered rate modes' in result.stdout

def test_powered_profile_refuses_flywheel_unlock(tmp_path):
    src=tmp_path/'profile.c';src.write_text('#include "Common/vehicle_profile.h"\nint main(void){return 0;}')
    for option in ('-DC_BENCH_ONLY_PROFILE=0','-DVEHICLE_YAW_ENABLED=1'):
        result=subprocess.run(['gcc','-I'+str(ROOT/'code'),option,str(src),'-o',str(tmp_path/'x')],capture_output=True,text=True)
        assert result.returncode != 0 and 'requires C-only' in result.stderr
