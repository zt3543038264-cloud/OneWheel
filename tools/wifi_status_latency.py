#!/usr/bin/env python3
"""Read-only C bench status sampling. Sends NO Start/Keep/Stop or parameter commands.
This is a communication observation, not permission for a powered test.
"""
import argparse
import ipaddress
import json
import math
import statistics
import time
from powered_rate_cli import Client, ProtocolError, UDPTransport, WireRecorder


def sample(client, count, interval, *, clock=time.monotonic, sleep=time.sleep, rows=None):
    rows = [] if rows is None else rows
    for i in range(count):
        started = clock()
        try:
            state = client.request('cbench_status')
        except (ProtocolError, OSError) as exc:
            rows.append({'sample': i + 1, 'ok': False,
                         'elapsed_ms': (clock() - started) * 1000, 'error': str(exc)})
            # Stop on the first failure. No automatic retry of a failed query.
            break
        rows.append({'sample': i + 1, 'ok': True,
                     'reply_ms': (clock() - started) * 1000, 'status': state})
        if state['active'] or state['applied']:
            # Read-only tool must not silently issue STOP. Report observed state.
            rows[-1]['warning'] = 'Board reports active or nonzero output; no further sampling'
            break
        if i + 1 < count:
            sleep(interval)
    return rows


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', required=True)
    p.add_argument('--port', type=int, default=5001)
    p.add_argument('--bind', default='0.0.0.0')
    p.add_argument('--local-port', type=int, default=8081)
    p.add_argument('--count', type=int, default=20)
    p.add_argument('--interval', type=float, default=.1)
    p.add_argument('--read-timeout', type=float, default=1.0)
    p.add_argument('--output', required=True, help='New JSON report; companion .wire.jsonl is also created')
    a = p.parse_args(argv)
    try:
        ipaddress.IPv4Address(a.host)
        ipaddress.IPv4Address(a.bind)
        if not (1 <= a.port <= 65535 and 0 <= a.local_port <= 65535 and 1 <= a.count <= 100):
            raise ValueError('Invalid port or count (1..100)')
        if not math.isfinite(a.interval) or not .1 <= a.interval <= 5:
            raise ValueError('interval must be 0.1..5 seconds')
        if not math.isfinite(a.read_timeout) or not .04 <= a.read_timeout <= 1:
            raise ValueError('read-timeout must be 0.04..1 seconds')
    except ValueError as exc:
        p.error(str(exc))
    recorder = None
    rows = []
    error = None
    # Exclusive file creation before network use; preserve prior evidence.
    with open(a.output, 'x', encoding='utf-8') as report, open(a.output + '.wire.jsonl', 'x', encoding='utf-8') as wire:
        try:
            recorder = WireRecorder(UDPTransport(a.host, a.port, a.bind, a.local_port))
            sample(Client(recorder, read_timeout=a.read_timeout), a.count, a.interval, rows=rows)
        except (OSError, KeyboardInterrupt) as exc:
            error = str(exc) or 'Interrupted'
        finally:
            if recorder is not None:
                recorder.close()
                recorder.save(wire)
        times = sorted(r['reply_ms'] for r in rows if r['ok'])
        result = {'host': a.host, 'port': a.port, 'requested': a.count,
                  'successful': len(times), 'samples': rows, 'error': error,
                  'commands': ['cbench_status'], 'motion_commands_sent': 0,
                  'powered_test_validated': False}
        if times:
            result['latency_ms'] = {'min': min(times), 'median': statistics.median(times),
                                    'p95_nearest_rank': times[math.ceil(.95 * len(times)) - 1],
                                    'max': max(times), 'at_or_above_40ms': sum(t >= 40 for t in times)}
        json.dump(result, report, indent=2)
        report.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'samples'}, indent=2))
    print('Read-only observation. Does not validate motor direction, heartbeat timing or powered control.')
    return 0 if len(times) == a.count and not error and not any('warning' in r for r in rows) else 2

if __name__ == '__main__':
    raise SystemExit(main())
