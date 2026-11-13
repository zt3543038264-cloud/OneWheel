from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[2];here=Path(__file__).resolve().parent
with tempfile.TemporaryDirectory() as temp:
    exe=Path(temp)/'integration'
    subprocess.run(['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-I'+str(here/'mocks'),'-I'+str(root/'code'),'-I'+str(root/'libraries/zf_device'),str(here/'integration.c'),str(root/'code/HW_Drivers/IMU/imu.c'),str(root/'code/Estimation/Attitude/attitude.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
    for test in ('attitude_ekf_test','imu_preprocess_test'):
        exe=Path(temp)/test
        subprocess.run(['gcc','-x','c','-std=c99','-O2','-Wall','-Wextra','-Werror','-I'+str(root/'code/Estimation/Attitude/new_ekf'),str(root/'code/Estimation/Attitude/new_ekf/attitude.inc'),str(here/(test+'.c.in')),'-lm','-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
