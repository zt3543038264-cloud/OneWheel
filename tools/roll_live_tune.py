"""Apply a logged Mode10 tuning stage and observe confirmed telemetry."""
import argparse
import json
import math
from pathlib import Path
import socket
import statistics
import struct
import time

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--set', action='append', default=[])
p.add_argument('--seconds', type=float, default=10)
p.add_argument('--module-ip', default='192.168.0.119')
p.add_argument('--wait-ready',type=float,default=0,help='Wait for supported upright Mode10 before writes')
p.add_argument('--wait-stopped',type=float,default=0,help='Wait for physical Stop before preparing gains')
p.add_argument('--reset-output', action='store_true')
p.add_argument('--prepare', action='store_true', help='Set gains while stopped; never start motors')
p.add_argument('--allow-stopped', action='store_true', help='Prepare instead if the car is stopped')
a = p.parse_args()
allowed = {'fly_angle_kp': (0,150), 'fly_rate_kp': (0,100),
           'fly_rate_ki': (0,3), 'fly_vel_kp': (-2,2),
           'fly_angle_kd': (0,10), 'fly_vel_kd': (0,1)}
updates = []
for item in a.set:
    name, value = item.split('=',1)
    value = float(value)
    if name not in allowed or not math.isfinite(value) or not allowed[name][0] <= value <= allowed[name][1]:
        raise SystemExit('Invalid tuning input')
    updates.append((name,value))
root = Path(__file__).resolve().parents[1]
names = json.loads((root/'validation/roll-live-20261030-180750.json').read_text())['channels']
names += [name for name in ('fly_angle_kd','fly_vel_kd') if name not in names]
s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.bind(('0.0.0.0',8081));s.settimeout(.1)
peer = (a.module_ip,5001)
confirmed=False
latest=None
rows=[]
events=[]
phase='baseline'
last_receive=0
changed=False
error=''

def send(command):
    s.sendto((command+'\n').encode('ascii'),peer)

def receive():
    global confirmed,latest,last_receive
    try:data,address=s.recvfrom(8192)
    except socket.timeout:return None
    if address[0]!=peer[0]:return None
    if data.startswith(b'MCAR '):
        reply=data.decode('ascii').strip();events.append(dict(host=time.time(),reply=reply))
        if reply.startswith('MCAR SUB '):confirmed=reply[9:].split(',')==names
        if reply.startswith(('MCAR SLIDER ','MCAR ERR')):print(reply,flush=True)
        return reply
    if confirmed and len(data)==4*(len(names)+1) and data[-4:]==b'\x00\x00\x80\x7f':
        values=struct.unpack('<'+str(len(names))+'f',data[:-4])
        if not all(math.isfinite(x) for x in values):raise RuntimeError('Nonfinite frame')
        latest=dict(zip(names,values));latest.update(host=time.time(),phase=phase)
        rows.append(latest);last_receive=time.monotonic()
    return None

def write(name,value):
    for attempt in range(3):
        send(f'[slider,{name},{value:.9g}]')
        end=time.monotonic()+1
        while time.monotonic()<end:
            reply=receive()
            if reply and reply.startswith(f'MCAR SLIDER {name} '):
                if abs(float(reply.split()[-1])-value)<1e-5:return
            if reply and reply.startswith('MCAR ERR'):raise RuntimeError(reply)
    raise RuntimeError('Unconfirmed write '+name)

def sample(seconds,guard=False):
    end=time.monotonic()+seconds;printed=0
    while time.monotonic()<end:
        receive()
        if guard:
            if time.monotonic()-last_receive>1:raise RuntimeError('Telemetry gap >1s')
            r=latest
            if (r['control_mode']!=10 or r['rate_state']!=2 or r['fault_reason']!=0 or
                r['imu_status']!=1 or r['driver_feedback_fresh']!=1 or
                abs(r['dual_roll_actual_deg']-.5)>25 or abs(r['dual_roll_rate_dps'])>200):
                raise RuntimeError('Stage interrupted by state/response')
        if latest and time.monotonic()-printed>1:
            printed=time.monotonic()
            keys=('phase','dual_roll_actual_deg','dual_roll_target_deg','dual_roll_target_rate_dps','dual_roll_rate_dps','dual_roll_pwm','fly_rate_kp','fly_rate_ki','fly_angle_kp','fly_vel_kp','flywheel_left_speed_raw','flywheel_right_speed_raw')
            print(json.dumps({k:latest[k] for k in keys}),flush=True)

original=None
try:
    send('SUB '+','.join(names));sample(2)
    fresh_deadline=time.monotonic()+2
    while time.monotonic()-last_receive>.5 and time.monotonic()<fresh_deadline:
        receive()
    if latest is None or not confirmed or time.monotonic()-last_receive>.5:raise RuntimeError('No fresh confirmed baseline')
    if a.wait_stopped:
        phase='waiting-stop';deadline=time.monotonic()+a.wait_stopped
        while time.monotonic()<deadline:
            receive()
            if (time.monotonic()-last_receive<.3 and latest['control_mode']==0 and latest['rate_state']==0):
                a.prepare=True;break
        else:raise RuntimeError('Physical Stop not observed; no changes made')
    if a.wait_ready:
        phase='waiting';deadline=time.monotonic()+a.wait_ready;upright_since=None
        while time.monotonic()<deadline:
            receive()
            center=latest['dual_roll_target_deg']-latest['fly_vel_out']
            ready=(time.monotonic()-last_receive<.3 and latest['control_mode']==10 and
                   latest['rate_state']==2 and latest['fault_reason']==0 and latest['imu_status']==1 and
                   abs(latest['dual_roll_actual_deg']-center)<4 and abs(latest['dual_roll_rate_dps'])<15)
            if ready:
                if upright_since is None:upright_since=time.monotonic()
                if time.monotonic()-upright_since>.3:break
            else:upright_since=None
        else:raise RuntimeError('No upright running baseline; no changes made')
    original=dict(latest)
    if a.allow_stopped and latest['control_mode']==0 and latest['rate_state']==0:
        a.prepare=True
    if a.prepare:
        stopped=(latest['control_mode']==0 and latest['rate_state']==0)
        latched=(latest['control_mode']==10 and latest['rate_state']<0 and
                 latest['dual_roll_pwm']==0 and latest.get('dual_pitch_pwm',0)==0)
        if not (stopped or latched):raise RuntimeError('Prepare requires Stop or latched zero-output fault')
    elif latest['control_mode']!=10 or latest['rate_state']!=2 or latest['fault_reason']!=0:raise RuntimeError('Mode10 not running')
    phase='transition'
    if a.reset_output:
        changed=True;write('fly_pwm_limit',0)
    for name,value in updates:
        changed=True;write(name,value)
    if a.reset_output:
        sample(.4)
        if latest['dual_roll_pwm']!=0:raise RuntimeError('Reset not confirmed')
        write('fly_pwm_limit',original['fly_pwm_limit'])
    phase='stage'
    sample(min(a.seconds,1) if a.prepare else a.seconds,guard=not a.prepare)
    for name,value in updates:
        if abs(latest[name]-value)>1e-5:raise RuntimeError('Parameter changed during observation: '+name)
except Exception as exc:
    error=str(exc);print('ERROR',error,flush=True)
    if changed:
        phase='stop'
        try:
            write('fly_pwm_limit',0)
            for name in ('fly_vel_kp','fly_angle_kp','fly_rate_kp','fly_rate_ki'):write(name,0)
            sample(.5)
            if latest['dual_roll_pwm']!=0:raise RuntimeError('Zero output not confirmed')
            write('fly_pwm_limit',original['fly_pwm_limit']);sample(.5)
            print('ZERO OUTPUT CONFIRMED',latest['dual_roll_pwm']==0,flush=True)
        except Exception as stop_error:
            for _ in range(3):send('[slider,fly_pwm_limit,0]')
            error+='; stop: '+str(stop_error)
            print('STOP NOT CONFIRMED; use physical Stop',flush=True)
finally:
    output=root/'validation'/('roll-stage-'+time.strftime('%Y%m%d-%H%M%S')+'.json')
    summary={}
    stage=[r for r in rows if r['phase']=='stage']
    if stage:
        for k in ('dual_roll_actual_deg','dual_roll_error_deg','dual_roll_rate_dps','dual_roll_pwm','fly_vel_out','flywheel_left_speed_raw','flywheel_right_speed_raw'):
            v=[r[k] for r in stage];summary[k]=dict(min=min(v),max=max(v),rms=math.sqrt(statistics.mean(x*x for x in v)))
        summary['rate_limit_fraction']=statistics.mean(abs(r['dual_roll_target_rate_dps'])>=r['fly_rate_limit_dps']-.01 for r in stage)
        summary['pwm_limit_fraction']=statistics.mean(abs(r['dual_roll_pwm'])>=r['fly_pwm_limit']-.5 for r in stage)
    output.write_text(json.dumps(dict(updates=updates,original=original,latest=latest,error=error,summary=summary,events=events,rows=rows),indent=2),encoding='utf-8')
    print('SUMMARY',json.dumps(summary),flush=True)
    print('SAVED',output,'ERROR',error,flush=True)
    s.close()
