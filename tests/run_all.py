"""Run upright regressions with GCC, or --compiler <zig.exe> for Zig cc."""
from pathlib import Path
import argparse,runpy,subprocess,sys,tempfile,json
TESTS=(
 'check_wheel_encoder.py','check_wheel_hold.py','check_wheel_position.py','check_mode10_hold.py',
 'check_driver_trace.py','check_trace_bridge.py','check_trace_client.py','check_mode8_step.py','check_flywheel_direction.py','check_driver_link.py','check_driver_protection.py','check_pid.py','check_balance_control.py','check_pwm_debug_menu.py','check_pwm_uart.py','check_storage.py','check_imu.py',
 'check_rate_filter.py','check_angle_feedback.py','check_angle_compensation.py','check_calibration.py','check_wifi.py','check_key_debounce.py',
 'check_spi_timeout.py','check_wifi_spi_driver.py','check_menu_tree.py',
 'check_menu_keys.py','check_menu_wifi_responsiveness.py','check_motor_open_loop.py',
)
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compiler',default='gcc')
    parser.add_argument('--temp-dir',type=Path)
    parser.add_argument('--report',type=Path)
    args=parser.parse_args();root=Path(__file__).resolve().parent
    if args.temp_dir:args.temp_dir.mkdir(parents=True,exist_ok=True);tempfile.tempdir=str(args.temp_dir.resolve())
    original=subprocess.run;prefix=[args.compiler]
    if Path(args.compiler).stem.lower()=='zig':prefix.append('cc')
    def run(argv,*a,**kw):
        if str(argv[0])=='gcc':argv=prefix+list(argv[1:])+['-UNDEBUG']
        return original(argv,*a,**kw)
    subprocess.run=run;results=[]
    try:
        for name in TESTS:
            print('RUN',name,flush=True);sys.argv=[str(root/name)]
            try:runpy.run_path(str(root/name),run_name='__main__');results.append({'test':name,'pass':True})
            except Exception as e:results.append({'test':name,'pass':False,'error':str(e)});print('FAIL',name,str(e),flush=True)
    finally:subprocess.run=original
    if args.report:args.report.write_text(json.dumps(results,indent=2),encoding='utf-8')
    print(f"Upright tests: {sum(r['pass'] for r in results)}/{len(results)} passed")
    return int(any(not r['pass'] for r in results))
if __name__=='__main__':raise SystemExit(main())
