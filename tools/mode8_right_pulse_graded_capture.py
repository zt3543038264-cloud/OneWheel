"""Capture one manually selected graded RIGHT-flywheel Mode8 pulse.

Levels are fixed in firmware: 1=150, 2=300, 3=450, 4=600; each is 250 ms.
This program never advances to another level automatically and never starts Mode8.
The firmware can report state=1 with zero elapsed time and zero output before
its first control update. Only this initial zero-output state is tolerated,
and a real active pulse must still be observed within 100 ms of the trigger.
"""
from pathlib import Path
import argparse, csv, json, math, socket, struct, time

ROOT=Path(__file__).resolve().parents[1]
NAMES=('telemetry_ms','rate_seq','rate_state','pwm_debug_enabled','startup_scale',
       'fault_reason','right_pulse_state','right_pulse_ms','right_pulse_pwm',
       'right_pulse_level','right_pulse_duration_ms','debug_fly_left_pwm',
       'debug_fly_right_pwm','debug_ground_pwm','motor_left_pwm','motor_right_pwm',
       'motor_ground_pwm','flywheel_left_speed_raw','flywheel_right_speed_raw',
       'driver_speed_frames','driver_feedback_age_ms','driver_feedback_fresh',
       'pitch_deg','roll_deg')
TAIL=b'\x00\x00\x80\x7f'
LEVEL_PWM={1:150,2:300,3:450,4:600}
START_TIMEOUT_S=.100
ACK_TIMEOUT_S=.350
COMPLETE_TIMEOUT_S=1.000

def decode(packet):
    if len(packet)!=4*len(NAMES)+4 or not packet.endswith(TAIL):return None
    values=struct.unpack('<%df'%len(NAMES),packet[:-4])
    return dict(zip(NAMES,values)) if all(map(math.isfinite,values)) else None

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--execute',action='store_true',help='authorize exactly one selected level')
    ap.add_argument('--level',type=int,choices=tuple(LEVEL_PWM),help='required with --execute')
    ap.add_argument('--direction',type=int,choices=(-1,1),default=1)
    ap.add_argument('--wait',type=float,default=120)
    ap.add_argument('--port',type=int,default=8081)
    ap.add_argument('--module-ip');ap.add_argument('--module-port',type=int,default=5001)
    ap.add_argument('--output',type=Path)
    a=ap.parse_args()
    if a.execute and a.level is None:ap.error('--execute requires an explicit --level 1, 2, 3 or 4')
    output=a.output or ROOT/'validation'/('mode8-right-graded-'+time.strftime('%Y%m%d-%H%M%S')+'.csv')
    output.parent.mkdir(parents=True,exist_ok=True)
    rows=[];events=[];error='';peer=a.module_ip
    trigger_sent=trigger_acked=pulse_seen=completion_seen=completed=False
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.settimeout(.1);sock.bind(('0.0.0.0',a.port))
    def send(text):
        sock.sendto((text+'\n').encode('ascii'),(peer,a.module_port))
        events.append({'host_monotonic_s':time.monotonic(),'tx':text})
    def receive():
        try:
            packet,address=sock.recvfrom(8192)
            return None if peer and address[0]!=peer else (packet,address)
        except (socket.timeout,BlockingIOError):return None
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
    def record(packet,now,writer):
        row=decode(packet)
        if row is None:return None
        row['host_monotonic_s']=now;rows.append(row);writer.writerow(row);return row
    print('Listening UDP',a.port,'; this program does not Start or change Mode8.')
    try:
        deadline=time.monotonic()+a.wait
        while not peer and time.monotonic()<deadline:
            item=receive()
            if item and (item[0].startswith(b'MCAR ') or item[0].endswith(TAIL)):peer=item[1][0]
        if not peer:raise RuntimeError('no board datagram received')
        confirm('STREAM 0');confirm('SUB '+','.join(NAMES));confirm('RATE 5');confirm('STREAM 1')
        ready_since=None;trigger_at=None;watermark_ms=-1.0
        expected_pwm=0 if a.level is None else a.direction*LEVEL_PWM[a.level]
        expected_ack=None if a.level is None else 'MCAR SLIDER debug_right_pulse_level %d'%(a.direction*a.level)
        with output.open('w',newline='',encoding='utf-8') as handle:
            writer=csv.DictWriter(handle,fieldnames=('host_monotonic_s',)+NAMES);writer.writeheader()
            while time.monotonic()<deadline:
                item=receive();now=time.monotonic()
                # Check deadlines on every receive, including a busy telemetry stream.
                # A frozen zero-output start must not be mistaken for a valid pulse.
                if trigger_sent:
                    age=now-trigger_at
                    if not trigger_acked and age>ACK_TIMEOUT_S:raise RuntimeError('trigger acknowledgement timeout')
                    if not pulse_seen and age>START_TIMEOUT_S:raise RuntimeError('active pulse not observed within 100 ms')
                    if age>COMPLETE_TIMEOUT_S:raise RuntimeError('graded pulse did not complete')
                if not item:continue
                packet,_=item
                if packet.startswith(b'MCAR '):
                    text=packet.decode('ascii',errors='replace').strip();events.append({'host_monotonic_s':now,'rx':text})
                    if text.startswith('MCAR ERR '):raise RuntimeError(text)
                    if trigger_sent and text==expected_ack:
                        trigger_acked=True
                        if completion_seen:completed=True;break
                    continue
                row=record(packet,now,writer)
                if row is None:raise RuntimeError('unexpected telemetry schema or non-finite data')
                active=(row['pwm_debug_enabled']==1 and row['rate_state']==2 and row['fault_reason']==0)
                if trigger_sent:
                    if row['telemetry_ms']<=watermark_ms:continue
                    watermark_ms=row['telemetry_ms']
                    if not active or row['right_pulse_state']==-1:raise RuntimeError('board stopped/faulted/aborted')
                    if row['driver_feedback_fresh']!=1 or row['driver_feedback_age_ms']>100:raise RuntimeError('driver feedback stale')
                    if any(row[n]!=0 for n in ('debug_fly_left_pwm','debug_ground_pwm','motor_left_pwm','motor_ground_pwm')):
                        raise RuntimeError('unexpected left/ground output')
                    if row['right_pulse_state'] in (1,2):
                        if row['right_pulse_level']!=a.level or row['right_pulse_duration_ms']!=250:
                            raise RuntimeError('board level/duration mismatch')
                    if row['right_pulse_state']==1:
                        if completion_seen:raise RuntimeError('pulse restarted after completion')
                        initial_zero=(not pulse_seen and row['right_pulse_ms']==0 and
                                      all(row[n]==0 for n in ('right_pulse_pwm','debug_fly_right_pwm','motor_right_pwm')))
                        if initial_zero:continue
                        if not 0<=row['right_pulse_ms']<250:raise RuntimeError('active pulse elapsed time mismatch')
                        if (row['right_pulse_pwm']!=expected_pwm or row['debug_fly_right_pwm']!=expected_pwm or
                            abs(row['motor_right_pwm'])!=abs(expected_pwm)):
                            raise RuntimeError('board PWM mismatch')
                        pulse_seen=True
                    elif row['right_pulse_state']==2:
                        if not pulse_seen:raise RuntimeError('completion observed without active pulse samples')
                        if row['right_pulse_ms']!=250:raise RuntimeError('completion elapsed time mismatch')
                        if any(row[n]!=0 for n in ('right_pulse_pwm','debug_fly_right_pwm','motor_right_pwm')):
                            raise RuntimeError('completion without zero output')
                        completion_seen=True
                        if trigger_acked:completed=True;break
                    elif row['right_pulse_state']!=0 or pulse_seen:
                        raise RuntimeError('unexpected pulse state')
                    elif any(row[n]!=0 for n in ('right_pulse_pwm','debug_fly_right_pwm','motor_right_pwm')):
                        raise RuntimeError('unexpected output before pulse start')
                else:
                    zeros=all(row[n]==0 for n in ('debug_fly_left_pwm','debug_fly_right_pwm','debug_ground_pwm','motor_left_pwm','motor_right_pwm','motor_ground_pwm'))
                    ready=(active and zeros and row['right_pulse_state'] in (0,2) and
                           row['driver_feedback_fresh']==1 and row['driver_feedback_age_ms']<=50 and
                           abs(row['flywheel_left_speed_raw'])<=30 and abs(row['flywheel_right_speed_raw'])<=30)
                    ready_since=now if ready and ready_since is None else ready_since if ready else None
                    if ready_since is not None and now-ready_since>=.5:
                        if not a.execute:print('READY; capture-only, no pulse sent.');break
                        # Drain queued pre-trigger telemetry and advance the board-time watermark.
                        sock.setblocking(False)
                        while True:
                            queued=receive()
                            if not queued:break
                            qpacket,_=queued;qnow=time.monotonic()
                            if qpacket.startswith(b'MCAR '):events.append({'host_monotonic_s':qnow,'rx':qpacket.decode('ascii',errors='replace').strip()})
                            else:
                                qrow=record(qpacket,qnow,writer)
                                if qrow is not None:watermark_ms=max(watermark_ms,qrow['telemetry_ms'])
                        sock.settimeout(.1)
                        watermark_ms=max(watermark_ms,row['telemetry_ms'])
                        trigger_at=time.monotonic();trigger_sent=True
                        send('[slider,debug_right_pulse_level,%d]'%(a.direction*a.level))
            else:raise RuntimeError('timed out')
    except (OSError,RuntimeError,KeyboardInterrupt,ValueError) as exc:
        error=str(exc) or type(exc).__name__;print('STOP:',error)
    finally:
        if peer and trigger_sent and not completed:
            for _ in range(3):
                try:send('[slider,debug_right_pulse_level,0]')
                except OSError:pass
        sock.close()
        report={'real_board_data':bool(rows),'frames':len(rows),'level':a.level,'direction':a.direction,
                'trigger_sent':trigger_sent,'trigger_acked':trigger_acked,'pulse_seen':pulse_seen,
                'completed':completed,'error':error,'csv':str(output),'events':events,
                'limits':['Commands are not measured voltage/current or torque.',
                          'The MCU timeout is not an independent watchdog or hardware disable.',
                          'Zero duty does not prove the driver or flywheel is mechanically stopped.']}
        output.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='events'},ensure_ascii=False,indent=2))
    return 0 if not error and (not a.execute or completed) else 2

if __name__=='__main__':raise SystemExit(main())
