"""Compile the real driver, adapter and r10h core against bounded SPI/STM mocks.
No serial ports or hardware outputs are accessed. Requires C99 GCC.
"""
from pathlib import Path
import argparse, json, subprocess, tempfile, hashlib, sys
p=argparse.ArgumentParser();p.add_argument('--gcc',default='gcc');args=p.parse_args()
here=Path(__file__).resolve().parent;root=here.parents[1];pkg=root/'code/Estimation/Attitude/r10h'
core=list((pkg/'core').glob('*.c'))+[pkg/'src/r10h_attitude.c']
results=[]
with tempfile.TemporaryDirectory(prefix='unicycle-r10h-tests-') as temp:
    for test in ('api_contract','core_regression','strong_vibration'):
        exe=Path(temp)/(test+'.exe')
        sources=core+[here/(test+'.c')]
        if test=='integration': sources += [root/'code/HW_Drivers/IMU/imu.c',root/'code/Estimation/Attitude/attitude.c']
        cmd=[args.gcc,'-std=c99','-O2','-DR10H_HOST_TEST','-Wall','-Wextra','-I'+str(here/'mocks'),'-I'+str(root/'code'),'-I'+str(pkg/'include'),'-I'+str(pkg/'core/candidate')]+[str(x) for x in sources]+['-lm','-o',str(exe)]
        if test=='bounded_spi':
            cmd=[args.gcc,'-std=c99','-O2','-Wall','-Wextra','-I'+str(here/'mocks'),str(here/'bounded_spi.c'),'-o',str(exe)]
        subprocess.run(cmd,check=True)
        run=subprocess.run([str(exe)],capture_output=True,text=True)
        print(test+': '+run.stdout.strip());print(run.stderr.strip()) if run.stderr else None
        results.append({'test':test,'exit_code':run.returncode,'output':run.stdout.strip()})
        if run.returncode: raise SystemExit('FAILED '+test+': '+str(run.returncode))
(here/'host_results.json').write_text(json.dumps({'hardware_tested':False,'results':results},indent=2),encoding='utf-8')
print('Preserved r10h core host tests passed (no RA estimator/hardware performance claim).')

# RB integration/bus fixtures are historical; RA owns the live adapter contract.
subprocess.run([sys.executable,str(root/'tests/imu_ra/run.py')],check=True)
