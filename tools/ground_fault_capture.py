"""Capture fault diagnostics without changing control parameters."""
import argparse
import json
from pathlib import Path
import socket
import struct
import time

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds',type=float,default=20)
args=parser.parse_args()
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.bind(('0.0.0.0',8081));s.settimeout(.15)
peer=('192.168.0.119',5001)
old=None
s.sendto(b'GET?\n',peer)
until=time.monotonic()+5
while time.monotonic()<until:
    try:packet,source=s.recvfrom(8192)
    except socket.timeout:continue
    if packet.startswith(b'MCAR SUB '):
        peer=(source[0],5001);old=packet.decode().strip()[9:];break
if old is None:
    s.close();raise SystemExit('NO_MODULE_REPLY')
names=('telemetry_ms,control_mode,rate_state,fault_reason,fault_imu_status,fault_imu_dt_ms,'
       'imu_status,imu_seq,imu_dt_ms,imu_spi_timeouts,imu_not_ready,imu_quality,imu_reject_reason,'
       'imu_accel_vib_g,imu_gyro_step_dps,imu_gyro_clips,imu_accel_clips,wheel_hold_state,'
       'wheel_feedback_age_ms,wheel_feedback_valid,wheel_encoder_valid,wheel_position_valid,'
       'wheel_encoder_errors,wheel_encoder_dt_ms,wheel_encoder_seq,wheel_speed_cps,motor_ground_pwm,'
       'ground_angle_kp,ground_rate_kp,wheel_pos_kp,wheel_speed_kp,wheel_speed_ki,'
       'dual_pitch_actual_deg,dual_pitch_target_deg,roll_deg,pitch_deg,gyro_x_dps,gyro_y_dps,'
       'imu_solve_us,imu_solve_max_us,imu_status_reg,imu_bad_bursts,'
       'dual_pitch_pwm,dual_pitch_target_rate_dps,dual_pitch_rate_dps,dual_pitch_rate_error_dps').split(',')
rows=[];events=[];confirmed=False;last_print=0
print('MODULE',peer,'LISTENING',args.seconds,'seconds',flush=True)
try:
    s.sendto(('SUB '+','.join(names)+'\n').encode(),peer)
    until=time.monotonic()+args.seconds
    while time.monotonic()<until:
        try:packet,source=s.recvfrom(8192)
        except socket.timeout:continue
        if source[0]!=peer[0]:continue
        if packet.startswith(b'MCAR '):
            reply=packet.decode().strip();events.append(reply)
            if reply.startswith('MCAR SUB '):confirmed=reply[9:]==','.join(names)
            if 'ERR' in reply:print(reply,flush=True)
        elif confirmed and len(packet)==4*(len(names)+1) and packet[-4:]==b'\x00\x00\x80\x7f':
            row=dict(zip(names,struct.unpack('<'+str(len(names))+'f',packet[:-4])))
            row['host']=time.time();rows.append(row)
            if time.monotonic()-last_print>2:
                last_print=time.monotonic()
                keys=('control_mode','rate_state','fault_reason','fault_imu_status','fault_imu_dt_ms',
                      'imu_status','imu_dt_ms','imu_spi_timeouts','imu_quality','imu_reject_reason',
                      'wheel_hold_state','wheel_encoder_errors','motor_ground_pwm','dual_pitch_pwm',
                      'wheel_speed_cps','dual_pitch_actual_deg','dual_pitch_rate_dps','ground_angle_kp','ground_rate_kp')
                print(json.dumps({key:row[key] for key in keys}),flush=True)
    print('DIAG_FRAMES',len(rows),flush=True)
    if rows:
        print('LATEST',json.dumps(rows[-1]),flush=True)
        print('FAULT_STATES',sorted(set((r['rate_state'],r['fault_reason'],r['fault_imu_status'],r['wheel_hold_state']) for r in rows)),flush=True)
        output=root/'validation'/('ground-fault-'+time.strftime('%Y%m%d-%H%M%S')+'.json')
        output.write_text(json.dumps(dict(channels=names,rows=rows,events=events),indent=2),encoding='utf-8')
        print('SAVED',output,flush=True)
finally:
    s.sendto(('SUB '+old+'\n').encode(),peer)
    s.close();print('ORIGINAL_SUB_RESTORE_SENT',flush=True)
