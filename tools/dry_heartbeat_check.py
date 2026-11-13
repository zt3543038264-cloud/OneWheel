#!/usr/bin/env python3
"""Profile-2 fixed zero-output dry_raw heartbeat check. Motor power must be isolated."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

# Use the exact transport/parser delivered in the WiFi repair package.
_EXPECTED = 'c9827ec8c4f103260249bc79e937c84f4edc024581293101798d612442d0bf9b'
_dependency = Path(__file__).with_name('powered_rate_cli.py')
if not _dependency.is_file() or hashlib.sha256(_dependency.read_bytes()).hexdigest() != _EXPECTED:
    raise SystemExit('Transport version mismatch. Use powered_rate_cli.py from the 20261113 WiFi ACK package; no command sent.')
from powered_rate_cli import Client, UDPTransport, WireRecorder, ProtocolError


def check(client, trace, *, clock=time.monotonic, sleep=time.sleep):
    s = client.request('cbench_status')
    trace.append(('preflight', s))
    if s['profile'] != 2 or s['active'] or s['applied'] or s['session'] >= 0xffffffff:
        raise ProtocolError('Requires idle profile 2 and zero output; no START sent')
    if not math.isfinite(s['angle_deg']) or abs(s['angle_deg']) >= 15:
        raise ProtocolError('Pitch must be within +/-15 degrees; no START sent. Firmware also checks Roll, sensors and timing.')
    session = s['session'] + 1
    def matches(v):
        return (v['profile'] == 2 and v['mode'] == 0 and v['session'] == session
                and v['duration_ms'] == 1000 and v['limit'] == 1
                and v['target'] == 0 and v['kp'] == 0 and v['ki'] == 0
                and v['calculated'] == 0 and v['applied'] == 0)
    primary = None
    try:
        start = clock()
        s = client.request('cbench_start', session - 1, 'dry_raw', 0, 0, 0, 1000, 1)
        trace.append(('start', s))
        if not matches(s) or not s['active'] or s['keep_seq'] != 0:
            raise ProtocolError('START session/configuration/state mismatch')
        for heartbeat in range(1, 20):
            slot = start + heartbeat * .05
            if clock() > slot + .001:
                raise ProtocolError('Missed heartbeat schedule; no catch-up')
            sleep(max(0, slot - clock()))
            if clock() > slot + .001:
                raise ProtocolError('Missed heartbeat schedule; no catch-up')
            s = client.request('cbench_keep', session, heartbeat)
            trace.append(('keep', s))
            if not matches(s) or not s['active'] or s['keep_seq'] != heartbeat:
                raise ProtocolError('KEEP session/configuration/state mismatch')
        sleep(max(0, start + 1.0 - clock()))
    except BaseException as exc:
        primary = exc
        raise
    finally:
        try:
            s = client.request('cbench_stop')
            trace.append(('stop', s))
            if s['active'] or s['applied']:
                raise ProtocolError('STOP reports active/nonzero output')
            if primary is None and (not matches(s) or s['state'] != 2 or s['reason'] not in (2, 3)):
                raise ProtocolError('Stopped but run ended with fault/configuration mismatch')
        except BaseException as stop_error:
            trace.append(('stop_unconfirmed_or_invalid', str(stop_error)))
            if primary is None:
                raise
    return session


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', required=True)
    p.add_argument('--port', type=int, default=5001)
    p.add_argument('--local-port', type=int, default=8081)
    p.add_argument('--ack-power-disconnected', action='store_true', required=True)
    p.add_argument('--output', required=True)
    a = p.parse_args(argv)
    if not 1 <= a.port <= 65535 or not 1 <= a.local_port <= 65535:
        p.error('Ports must be 1..65535')
    trace, error, recorder = [], None, None
    # Reserve evidence before opening the socket. Never overwrite previous logs.
    with open(a.output, 'x', encoding='utf-8') as report, open(a.output + '.wire.jsonl', 'x', encoding='utf-8') as wire:
        try:
            recorder = WireRecorder(UDPTransport(a.host, a.port, '0.0.0.0', a.local_port))
            check(Client(recorder), trace)
        except (OSError, ValueError, ProtocolError, KeyboardInterrupt) as exc:
            error = str(exc) or 'Interrupted'
        finally:
            if recorder is not None:
                recorder.close()
                recorder.save(wire)
        result = {'completed': error is None, 'error': error, 'mode': 'dry_raw',
                  'fixed_configuration': {'profile': 2, 'target': 0, 'kp': 0, 'ki': 0, 'limit': 1, 'duration_ms': 1000},
                  'expected_keep_acks': 19, 'keep_acks': sum(k == 'keep' for k, _ in trace),
                  'start_attempted': any(k in ('start', 'stop', 'stop_unconfirmed_or_invalid') for k, _ in trace),
                  'stop_zero_confirmed': any(k == 'stop' and not v['active'] and v['applied'] == 0 for k, v in trace),
                  'stop_error': next((v for k, v in trace if k == 'stop_unconfirmed_or_invalid'), None),
                  'powered_test_validated': False, 'trace': trace}
        json.dump(result, report, indent=2)
        report.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'trace'}, indent=2))
    print('Keep motor power isolated. No automatic retry. This does not validate powered control.')
    return 0 if result['completed'] else 2

if __name__ == '__main__':
    raise SystemExit(main())
