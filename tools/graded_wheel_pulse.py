#!/usr/bin/env python3
"""One explicit main-wheel raw pulse, not a PID/balance trial. No retries or auto escalation."""
import argparse
import hashlib
import ipaddress
import json
from pathlib import Path
import time

_dependency = Path(__file__).with_name('powered_rate_cli.py')
if not _dependency.is_file() or hashlib.sha256(_dependency.read_bytes()).hexdigest() != 'db77ca5e30549a8a0a2128f17250056df3dffd23447d0425d2e495fc6041afff':
    raise SystemExit('Transport version mismatch; install this package tools together. No command sent.')

from powered_rate_cli import Client, UDPTransport, WireRecorder, ProtocolError

LEVELS = (200,400,600,800,1000,1200,2000)
# Fixed slots, bounded Windows wake-up jitter; never retry/catch up.
HEARTBEAT_JITTER_S = .020

def run(client, *, level, direction, trace, clock=time.monotonic, sleep=time.sleep):
    if type(level) is not int or level not in range(1,8) or type(direction) is not int or direction not in (-1,1):
        raise ValueError('Explicit level 1..7 and direction +/-1 required')
    amplitude = LEVELS[level-1]
    target = direction * amplitude
    s = client.request('cbench_status'); trace.append(('preflight',s))
    if s['profile'] != 2 or s['active'] or s['applied'] or s['session'] >= 0xffffffff:
        raise ProtocolError('Preflight requires idle profile2 and zero commanded output; no START sent')
    if abs(s['angle_deg']) >= 15 or abs(s['rate_dps']) >= 5:
        raise ProtocolError('Require Pitch <15deg and |rate|<5deg/s; firmware also checks Roll/sensors/encoder/timing; no START sent')
    session=s['session']+1
    def matches(v):
        return (v['profile']==2 and v['mode']==6 and v['session']==session and
                v['duration_ms']==1000 and v['limit']==amplitude and v['target']==target and
                v['kp']==0 and v['ki']==0 and abs(v['calculated'])<=amplitude and abs(v['applied'])<=amplitude and
                (v['calculated']==0 or v['calculated']*direction>=0) and
                (v['applied']==0 or v['applied']*direction>=0))
    primary=None
    try:
        start=clock()
        s=client.request('cbench_start',session-1,'powered_pulse',target,0,0,1000,amplitude)
        trace.append(('start',s))
        if not matches(s) or not s['active'] or s['keep_seq']!=0:
            raise ProtocolError('START mismatch; no retry')
        for heartbeat in range(1,20):
            slot=start+heartbeat*.05
            if clock()>slot+HEARTBEAT_JITTER_S:raise ProtocolError('Missed heartbeat schedule; no catch-up')
            sleep(max(0,slot-clock()))
            if clock()>slot+HEARTBEAT_JITTER_S:raise ProtocolError('Missed heartbeat schedule; no catch-up')
            s=client.request('cbench_keep',session,heartbeat);trace.append(('keep',s))
            if not matches(s) or not s['active'] or s['keep_seq']!=heartbeat:
                raise ProtocolError('KEEP mismatch or firmware stopped; no retry')
        sleep(max(0,start+1.0-clock()))
    except BaseException as exc:
        primary=exc;raise
    finally:
        try:
            s=client.request('cbench_stop');trace.append(('stop',s))
            if s['active'] or s['applied']:raise ProtocolError('STOP reports active/nonzero')
            if primary is None and (not matches(s) or s['state']!=2 or s['reason'] not in (2,3)):
                raise ProtocolError('STOP zero but trial fault/configuration mismatch')
        except BaseException as exc:
            trace.append(('stop_error',str(exc)))
            if primary is None:raise

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',required=True)
    p.add_argument('--port',type=int,default=5001)
    p.add_argument('--local-port',type=int,default=8081)
    p.add_argument('--level',type=int,choices=range(1,8),required=True)
    p.add_argument('--direction',type=int,choices=(-1,1),required=True)
    p.add_argument('--ack-powered-pulse',action='store_true',required=True)
    p.add_argument('--output',required=True)
    a=p.parse_args(argv)
    try:
        ipaddress.IPv4Address(a.host)
        if not 1<=a.port<=65535 or not 1<=a.local_port<=65535:raise ValueError('Invalid UDP port')
    except ValueError as exc:p.error(str(exc))
    trace=[];error=None;recorder=None
    with open(a.output,'x',encoding='utf-8') as report,open(a.output+'.wire.jsonl','x',encoding='utf-8') as wire:
        try:
            recorder=WireRecorder(UDPTransport(a.host,a.port,'0.0.0.0',a.local_port))
            run(Client(recorder),level=a.level,direction=a.direction,trace=trace)
        except (ProtocolError,OSError,KeyboardInterrupt) as exc:error=str(exc) or 'Interrupted'
        finally:
            if recorder is not None:recorder.close();recorder.save(wire)
        result={'completed':error is None,'error':error,'level':a.level,'direction':a.direction,
                'raw_peak':LEVELS[a.level-1],'duration_ms':1000,'ramp_up_ms':100,'plateau_ms':800,'ramp_down_ms':100,
                'keep_acks':sum(k=='keep' for k,_ in trace),
                'stop_zero_confirmed':any(k=='stop' and not v['active'] and v['applied']==0 for k,v in trace),
                'stop_error':next((v for k,v in trace if k=='stop_error'),None),
                'physical_direction_verified':False,'physical_stop_verified':False,'trace':trace}
        json.dump(result,report,indent=2);report.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k!='trace'},indent=2))
    print('No automatic retry or level increase. Software zero is not proof of mechanical stop.')
    if error:print('If STOP is unconfirmed, cut motor power independently. Keep the log.')
    return 0 if error is None else 2
if __name__=='__main__':raise SystemExit(main())
