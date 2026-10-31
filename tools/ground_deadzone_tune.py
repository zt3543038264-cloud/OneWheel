"""Log and tune only the two direction-specific ground compensation values."""
import argparse
import json
from pathlib import Path
import socket
import struct
import time

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--pos',type=float)
p.add_argument('--neg',type=float)
p.add_argument('--seconds',type=float,default=12)
p.add_argument('--module-ip',default='192.168.0.119')
p.add_argument('--wait-ready',type=float,default=0)
p.add_argument('--set',action='append',default=[])
a=p.parse_args()
updates={}
for item in a.set:
    key,value=item.split('=',1);value=float(value)
    limits={'rate_kp':(0,100),'ground_angle_kp':(0,100),'wheel_pos_kp':(0,5),
            'wheel_speed_kp':(0,.1),'wheel_speed_ki':(0,.1)}
    if key not in limits or not limits[key][0]<=value<=limits[key][1]:raise SystemExit('Invalid parameter')
    updates[key]=value
for key,value in [('ground_deadzone_pos_pwm',a.pos),('ground_deadzone_neg_pwm',a.neg)]:
    if value is not None:
        if not 0<=value<=4000:raise SystemExit('Trial compensation outside 0..4000')
        updates[key]=value
names=('telemetry_ms,control_mode,rate_state,fault_reason,imu_status,startup_scale,'
       'ground_deadzone_pos_pwm,ground_deadzone_neg_pwm,dual_pitch_pwm,motor_ground_pwm,'
       'wheel_speed_cps,wheel_position_count,wheel_encoder_seq,wheel_encoder_valid,'
       'wheel_position_valid,wheel_feedback_age_ms,wheel_hold_state,dual_pitch_actual_deg,'
       'dual_pitch_target_deg,dual_pitch_rate_dps,ground_angle_kp,ground_rate_kp,'
       'ground_pwm_limit,wheel_tilt_offset,wheel_target_cps,'
       'wheel_pos_kp,wheel_speed_kp,wheel_speed_ki,wheel_speed_limit,wheel_pos_error,'
       'dual_roll_actual_deg,dual_roll_target_deg,dual_roll_rate_dps,dual_roll_pwm,'
       'fly_rate_kp,fly_rate_ki,fly_angle_kp,fly_angle_kd,fly_vel_kp,fly_vel_kd').split(',')
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('0.0.0.0',8081));s.settimeout(.1)
peer=(a.module_ip,5001)
old=None;confirmed=False;latest=None;last_frame=0
rows=[];events=[];phase='baseline';original=None;changed=False;error=''

def send(text):s.sendto((text+'\n').encode(),peer)
def receive():
    global old,confirmed,latest,last_frame
    try:data,source=s.recvfrom(8192)
    except socket.timeout:return None
    if source[0]!=peer[0]:return None
    if data.startswith(b'MCAR '):
        reply=data.decode().strip();events.append(reply)
        if reply.startswith('MCAR SUB '):
            fields=reply[9:]
            if old is None:old=fields
            confirmed=fields==','.join(names)
        if reply.startswith(('MCAR SLIDER','MCAR ERR')):print(reply,flush=True)
        return reply
    if confirmed and len(data)==4*(len(names)+1) and data[-4:]==b'\x00\x00\x80\x7f':
        latest=dict(zip(names,struct.unpack('<'+str(len(names))+'f',data[:-4])))
        latest.update(host=time.time(),phase=phase);rows.append(latest);last_frame=time.monotonic()
    return None
def write(key,value):
    for _ in range(3):
        send(f'[slider,{key},{value:.9g}]');end=time.monotonic()+1
        while time.monotonic()<end:
            reply=receive()
            if reply and reply.startswith(f'MCAR SLIDER {key} '):
                if abs(float(reply.split()[-1])-value)<1e-4:return
            if reply and reply.startswith('MCAR ERR'):raise RuntimeError(reply)
    raise RuntimeError('No acknowledgement: '+key)
def sample(seconds,guard=False):
    end=time.monotonic()+seconds;printed=0
    while time.monotonic()<end:
        receive()
        if guard:
            if time.monotonic()-last_frame>1:raise RuntimeError('Telemetry stale')
            if (latest['control_mode']!=10 or latest['rate_state']!=2 or
                latest['fault_reason']!=0 or latest['imu_status']!=1 or
                latest['wheel_encoder_valid']!=1 or latest['wheel_position_valid']!=1):
                raise RuntimeError('Control/encoder state changed')
        if latest and time.monotonic()-printed>2:
            printed=time.monotonic()
            keys=('phase','control_mode','rate_state','ground_deadzone_pos_pwm','ground_deadzone_neg_pwm',
                  'dual_pitch_pwm','motor_ground_pwm','wheel_speed_cps','dual_pitch_actual_deg')
            print(json.dumps({k:latest[k] for k in keys}),flush=True)
try:
    send('GET?');end=time.monotonic()+2
    while old is None and time.monotonic()<end:receive()
    if old is None:raise RuntimeError('No subscription metadata')
    send('SUB '+','.join(names));sample(2)
    end=time.monotonic()+2
    while (latest is None or time.monotonic()-last_frame>.5) and time.monotonic()<end:receive()
    if latest is None or not confirmed or time.monotonic()-last_frame>.5:raise RuntimeError('No fresh diagnostic frames')
    original=dict(latest)
    if a.wait_ready:
        end=time.monotonic()+a.wait_ready;stable_since=None
        while time.monotonic()<end:
            receive()
            ready=(time.monotonic()-last_frame<.3 and latest['control_mode']==10 and
                   latest['rate_state']==2 and latest['fault_reason']==0 and
                   abs(latest['dual_pitch_actual_deg']-latest['dual_pitch_target_deg'])<3 and
                   abs(latest['dual_pitch_rate_dps'])<10 and abs(latest['wheel_speed_cps'])<100)
            if ready:
                if stable_since is None:stable_since=time.monotonic()
                if time.monotonic()-stable_since>.3:break
            else:stable_since=None
        else:raise RuntimeError('No upright ground baseline; no changes made')
        original=dict(latest)
    if updates:
        if latest['control_mode']!=10 or latest['rate_state']!=2 or latest['fault_reason']!=0:
            raise RuntimeError('Mode10 not running; no changes made')
        if latest['startup_scale']<1:raise RuntimeError('Startup ramp incomplete')
        phase='transition'
        for key,value in updates.items():changed=True;write(key,value)
    phase='trial' if updates else 'observe'
    sample(a.seconds,guard=bool(updates))
    for key,value in updates.items():
        if abs(latest['ground_rate_kp' if key=='rate_kp' else key]-value)>1e-4:raise RuntimeError('Parameter changed externally: '+key)
except Exception as exc:
    error=str(exc);print('ERROR',error,flush=True)
    if changed:
        phase='restore'
        for key in updates:
            try:write(key,original['ground_rate_kp' if key=='rate_kp' else key])
            except Exception as restore_error:print('RESTORE ERROR',key,str(restore_error),flush=True)
finally:
    output=Path(__file__).resolve().parents[1]/'validation'/('ground-deadzone-'+time.strftime('%Y%m%d-%H%M%S')+'.json')
    output.write_text(json.dumps(dict(updates=updates,original=original,latest=latest,error=error,events=events,rows=rows),indent=2),encoding='utf-8')
    print('SAVED',output,'FRAMES',len(rows),flush=True)
    if old:
        send('SUB '+old)
        end=time.monotonic()+1
        while time.monotonic()<end:
            reply=receive()
            if reply=='MCAR SUB '+old:
                print('ORIGINAL SUBSCRIPTION RESTORED',flush=True);break
    s.close()
