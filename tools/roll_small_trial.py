"""Bounded Mode10 inner-Ki trial, with telemetry confirmation and rollback."""
import argparse
import json
import math
from pathlib import Path
import socket
import statistics
import struct
import time

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--ki', type=float, default=0.02)
p.add_argument('--seconds', type=float, default=8)
p.add_argument('--limit', type=float, default=400)
p.add_argument('--angle-kp', type=float, default=0)
a = p.parse_args()
if not (0 < a.ki <= 1 and 0 <= a.angle_kp <= 5 and
        0 < a.seconds <= 12 and 0 < a.limit <= 1500):
    raise SystemExit('Outside small-trial bounds')
root = Path(__file__).resolve().parents[1]
names = json.loads((root/'validation/roll-live-20261030-180750.json').read_text())['channels']
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', 8081))
s.settimeout(.15)
peer = ('192.168.0.119', 5001)
rows, events = [], []
confirmed = False
latest = None
phase = 'baseline'

def send(command):
    s.sendto((command+'\n').encode('ascii'), peer)

def receive():
    global confirmed, latest
    try:
        data, address = s.recvfrom(8192)
    except socket.timeout:
        return None
    if address[0] != peer[0]:
        return None
    if data.startswith(b'MCAR '):
        reply = data.decode('ascii').strip()
        events.append(dict(host=time.time(), reply=reply))
        if reply.startswith('MCAR SUB '):
            confirmed = reply[9:].split(',') == names
        if reply.startswith(('MCAR SLIDER ', 'MCAR ERR')):
            print(reply, flush=True)
        return reply
    if confirmed and len(data) == 4*(len(names)+1) and data[-4:] == b'\x00\x00\x80\x7f':
        values = struct.unpack('<'+str(len(names))+'f', data[:-4])
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError('Nonfinite telemetry')
        latest = dict(zip(names, values))
        latest.update(host=time.time(), phase=phase)
        rows.append(latest)
    return None

def write(name, value):
    for attempt in range(3):
        send(f'[slider,{name},{value:.9g}]')
        end = time.monotonic()+1
        while time.monotonic() < end:
            reply = receive()
            if reply and reply.startswith(f'MCAR SLIDER {name} '):
                if abs(float(reply.split()[-1])-value) < 1e-5:
                    return
            if reply and reply.startswith('MCAR ERR'):
                raise RuntimeError(reply)
    raise RuntimeError('Missing write acknowledgement: '+name)

def sample(seconds, guard=False):
    end, last_print = time.monotonic()+seconds, 0
    last_count, last_frame = len(rows), time.monotonic()
    while time.monotonic() < end:
        receive()
        if len(rows) != last_count:
            last_count, last_frame = len(rows), time.monotonic()
            if guard:
                r = latest
                if (r['control_mode'] != 10 or r['rate_state'] != 2 or
                    r['fault_reason'] != 0 or r['driver_feedback_fresh'] != 1 or
                    r['imu_status'] != 1 or abs(r['dual_roll_error_deg']) > 10 or
                    abs(r['dual_roll_rate_dps']) > 60):
                    raise RuntimeError('Trial envelope exceeded')
            if time.monotonic()-last_print >= 1:
                last_print = time.monotonic()
                print(json.dumps({k: latest[k] for k in ('phase','dual_roll_actual_deg','dual_roll_rate_dps','dual_roll_pwm','fly_rate_ki','flywheel_left_speed_raw','flywheel_right_speed_raw')}), flush=True)
        if guard and time.monotonic()-last_frame > .6:
            raise RuntimeError('Telemetry stale')

original = None
changed = False
error = ''
restored = False
try:
    send('SUB '+','.join(names))
    sample(3)
    if not confirmed or latest is None:
        raise RuntimeError('No confirmed telemetry')
    if any(latest[k] != 0 for k in ('fly_rate_kp','fly_rate_ki','fly_angle_kp','fly_vel_kp')):
        raise RuntimeError('Active manual tuning detected; no changes made')
    ready_until = time.monotonic()+30
    ready_since = None
    while time.monotonic() < ready_until:
        receive()
        ready = (time.time()-latest['host'] < .3 and
                 latest['control_mode'] == 10 and latest['rate_state'] == 2 and
                 abs(latest['dual_roll_error_deg']) < 3 and
                 abs(latest['dual_roll_rate_dps']) < 10)
        if ready:
            if ready_since is None:
                ready_since = time.monotonic()
            if time.monotonic()-ready_since > .5:
                break
        else:
            ready_since = None
    else:
        raise RuntimeError('No upright baseline within 30 seconds')
    original = dict(latest)
    if any(original[k] != 0 for k in ('fly_rate_kp','fly_rate_ki','fly_angle_kp','fly_vel_kp')):
        raise RuntimeError('Expected zero-gain baseline; no changes made')
    if original['control_mode'] != 10 or original['rate_state'] != 2 or abs(original['dual_roll_error_deg']) > 6:
        raise RuntimeError('Baseline not ready')
    changed = True
    write('fly_pwm_limit', a.limit)
    phase = 'trial'
    write('fly_rate_ki', a.ki)
    if a.angle_kp:
        write('fly_angle_kp', a.angle_kp)
    sample(a.seconds, guard=True)
except Exception as exc:
    error = str(exc)
    print('ERROR', error, flush=True)
finally:
    if changed:
        phase = 'rollback'
        try:
            write('fly_pwm_limit', 0)
            if a.angle_kp:
                write('fly_angle_kp', original['fly_angle_kp'])
            write('fly_rate_ki', original['fly_rate_ki'])
            sample(.4)
            if latest['dual_roll_pwm'] != 0:
                raise RuntimeError('Zero PWM not confirmed')
            write('fly_pwm_limit', original['fly_pwm_limit'])
            sample(.6)
            restored = (latest['fly_rate_ki'] == original['fly_rate_ki'] and
                        latest['fly_angle_kp'] == original['fly_angle_kp'] and
                        latest['fly_pwm_limit'] == original['fly_pwm_limit'] and
                        latest['dual_roll_pwm'] == 0)
        except Exception as exc:
            error += '; rollback: '+str(exc)
            # Keep requesting zero output if rollback could not be confirmed.
            for _ in range(3):
                send('[slider,fly_pwm_limit,0]')
            print('ROLLBACK NOT CONFIRMED: use physical Stop', flush=True)
    output = root/'validation'/('roll-ki-trial-'+time.strftime('%Y%m%d-%H%M%S')+'.json')
    report = dict(settings=vars(a), original=original, error=error, restored=restored, events=events, rows=rows)
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    for label in ('baseline','trial','rollback'):
        subset = [r for r in rows if r['phase'] == label]
        if subset:
            print(label, 'frames', len(subset), flush=True)
            for k in ('dual_roll_actual_deg','dual_roll_rate_dps','dual_roll_pwm','flywheel_left_speed_raw','flywheel_right_speed_raw'):
                v = [r[k] for r in subset]
                print(k, dict(min=min(v), max=max(v), rms=math.sqrt(statistics.mean(x*x for x in v)), last=v[-1]), flush=True)
    print('RESTORED', restored, 'SAVED', output, flush=True)
    s.close()
