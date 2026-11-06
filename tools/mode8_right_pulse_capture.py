"""Capture one fixed 300/10000, 500 ms RIGHT-flywheel Mode8 pulse.

Requires matching TC264 firmware. It never changes mode or remotely presses Start.
Without --execute it only verifies readiness and records a short baseline.
"""
from pathlib import Path
import argparse, csv, json, math, socket, struct, time

ROOT = Path(__file__).resolve().parents[1]
NAMES = ('telemetry_ms','rate_seq','rate_state','pwm_debug_enabled','startup_scale',
         'fault_reason','right_pulse_state','right_pulse_ms','right_pulse_pwm',
         'debug_fly_left_pwm','debug_fly_right_pwm','debug_ground_pwm',
         'motor_left_pwm','motor_right_pwm','motor_ground_pwm',
         'flywheel_left_speed_raw','flywheel_right_speed_raw','driver_speed_frames',
         'driver_feedback_age_ms','driver_feedback_fresh','pitch_deg','roll_deg')
TAIL = b'\x00\x00\x80\x7f'

def decode(packet):
    if len(packet) != 4*len(NAMES)+4 or not packet.endswith(TAIL): return None
    values=struct.unpack('<%df'%len(NAMES),packet[:-4])
    return dict(zip(NAMES,values)) if all(map(math.isfinite,values)) else None

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute',action='store_true',help='authorize exactly one pulse')
    ap.add_argument('--direction',type=int,choices=(-1,1),default=1)
    ap.add_argument('--wait',type=float,default=120)
    ap.add_argument('--port',type=int,default=8081)
    ap.add_argument('--module-ip')
    ap.add_argument('--module-port',type=int,default=5001)
    ap.add_argument('--output',type=Path)
    a=ap.parse_args()
    output=a.output or ROOT/'validation'/('mode8-right-pulse-'+time.strftime('%Y%m%d-%H%M%S')+'.csv')
    output.parent.mkdir(parents=True,exist_ok=True)
    rows=[];events=[];error='';peer=a.module_ip;triggered=False;completed=False
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.settimeout(.1)
    sock.bind(('0.0.0.0',a.port))
    def send(text):
        sock.sendto((text+'\n').encode('ascii'),(peer,a.module_port))
        events.append({'host_monotonic_s':time.monotonic(),'tx':text})
    def receive():
        try:
            packet,address=sock.recvfrom(8192)
            return None if peer and address[0]!=peer else (packet,address)
        except socket.timeout:return None
    def confirm(command):
        expected=('MCAR '+command+'\n').encode('ascii')
        for _ in range(3):
            send(command);end=time.monotonic()+1
            while time.monotonic()<end:
                item=receive()
                if not item:continue
                packet,_=item
                if packet==expected:return
                if packet.startswith(b'MCAR ERR '):raise RuntimeError(packet.decode().strip())
        raise RuntimeError('no acknowledgement: '+command)
    print('Listening on UDP',a.port,'; no motor command until Mode8 readiness is verified.')
    try:
        deadline=time.monotonic()+a.wait
        while not peer and time.monotonic()<deadline:
            item=receive()
            if item and (item[0].startswith(b'MCAR ') or item[0].endswith(TAIL)):peer=item[1][0]
        if not peer:raise RuntimeError('no board datagram received')
        confirm('STREAM 0');confirm('SUB '+','.join(NAMES));confirm('RATE 5');confirm('STREAM 1')
        ready_since=None;trigger_time=None;last_data=time.monotonic();done_at=None
        with output.open('w',newline='',encoding='utf-8') as handle:
            writer=csv.DictWriter(handle,fieldnames=('host_monotonic_s',)+NAMES);writer.writeheader()
            while time.monotonic()<deadline:
                now=time.monotonic()
                if triggered and now-last_data>.25:raise RuntimeError('telemetry lost >250 ms')
                item=receive()
                if not item:continue
                packet,_=item
                if packet.startswith(b'MCAR '):
                    text=packet.decode('ascii',errors='replace').strip();events.append({'host_monotonic_s':now,'rx':text})
                    if text.startswith('MCAR ERR '):raise RuntimeError(text)
                    continue
                row=decode(packet)
                if row is None:raise RuntimeError('unexpected telemetry schema or non-finite data')
                row['host_monotonic_s']=now;rows.append(row);writer.writerow(row);last_data=now
                active=(row['pwm_debug_enabled']==1 and row['rate_state']==2 and row['fault_reason']==0)
                if triggered:
                    if not active or row['right_pulse_state']==-1:raise RuntimeError('board stopped/faulted/aborted')
                    if row['driver_feedback_fresh']!=1 or row['driver_feedback_age_ms']>100:raise RuntimeError('driver feedback stale')
                    if row['motor_left_pwm']!=0 or row['motor_ground_pwm']!=0:raise RuntimeError('unexpected left/ground command')
                    if abs(row['motor_right_pwm'])>300 or abs(row['right_pulse_pwm'])>300:raise RuntimeError('right command exceeded fixed limit')
                    if row['right_pulse_state']==2:
                        if row['motor_right_pwm']!=0 or row['right_pulse_pwm']!=0:raise RuntimeError('completed without zero command')
                        completed=True;done_at=done_at or now
                        if now-done_at>=.5:break
                    elif now-trigger_time>1.5:raise RuntimeError('pulse did not finish locally')
                else:
                    zeros=all(row[n]==0 for n in ('debug_fly_left_pwm','debug_fly_right_pwm','debug_ground_pwm','motor_left_pwm','motor_right_pwm','motor_ground_pwm'))
                    ready=(active and zeros and row['startup_scale']==1 and row['right_pulse_state']!=1 and
                           row['driver_feedback_fresh']==1 and row['driver_feedback_age_ms']<50 and
                           abs(row['flywheel_left_speed_raw'])<30 and abs(row['flywheel_right_speed_raw'])<30)
                    ready_since=now if ready and ready_since is None else ready_since if ready else None
                    if ready_since is not None and now-ready_since>=.5:
                        if not a.execute:print('READY; capture-only mode, no pulse sent.');break
                        trigger_time=now;triggered=True
                        send('[slider,debug_right_pulse_run,%d]'%a.direction) # exactly once
            else:raise RuntimeError('timed out')
    except (OSError,RuntimeError,KeyboardInterrupt,ValueError) as exc:
        error=str(exc) or type(exc).__name__;print('STOP:',error)
    finally:
        if peer and triggered:
            for _ in range(3):
                try:send('[slider,debug_right_pulse_run,0]')
                except OSError:pass
        sock.close()
        report={'real_board_data':bool(rows),'frames':len(rows),'triggered':triggered,
                'completed':completed,'error':error,'csv':str(output),'events':events,
                'limits':['PWM fields are commands, not measured phase voltage/current.',
                          'Speed feedback is filtered; this test cannot directly prove motor torque.',
                          'A zero command does not guarantee immediate mechanical stop.']}
        output.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='events'},ensure_ascii=False,indent=2))
    return 0 if not error and (not a.execute or completed) else 2

if __name__=='__main__':raise SystemExit(main())
