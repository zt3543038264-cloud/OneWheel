#!/usr/bin/env python3
"""Explicit, no-retry UART2 commissioning client. Importing never opens a port.

Use only with motor power physically isolated until the independent cutoff/reset
qualification in COMMISSIONING.md is complete. A software STOP is not a kill.
"""
from __future__ import annotations
import argparse
import json
import secrets
import time
from typing import Callable


class ProtocolError(RuntimeError):
    pass


class Client:
    def __init__(self, port, *, clock: Callable[[], float] = time.monotonic, timeout=2.0):
        self.port, self.clock, self.timeout = port, clock, timeout
        self.seq = secrets.randbelow(60000) + 1

    def request(self, operation: str, *fields: object):
        self.seq = self.seq % 65535 + 1
        line = f"cfg:{operation},{self.seq}" + ''.join(',' + str(f) for f in fields) + '\n'
        if len(line) > 63 or any(ch in str(f) for f in fields for ch in '\r\n,'):
            raise ValueError('Invalid or oversized command')
        wire = line.encode('ascii')
        if self.port.write(wire) != len(wire):
            raise ProtocolError('Partial write; request outcome uncertain, do not retry')
        deadline = self.clock() + self.timeout
        while self.clock() < deadline:
            raw = self.port.read_until(b'\n', 256)
            if not raw or not raw.endswith(b'\n') or len(raw) >= 256:
                continue
            try:
                text = raw.decode('ascii').strip('\r\n')
            except UnicodeDecodeError:
                continue
            prefix = f'rsp:{self.seq},'
            if not text.startswith(prefix):
                continue
            values = text[len(prefix):].split(',')
            if not values or values[0] != 'ok':
                raise ProtocolError('MCU rejected request: ' + ','.join(values[1:]))
            if len(values) < 2 or values[1] != operation:
                raise ProtocolError('Mismatched response, no retry')
            return values[2:]
        raise ProtocolError(f'{operation} timed out; no automatic retry')

    def stop(self):
        # A priority command, not an acknowledgement that hardware has stopped.
        if self.port.write(b'stop\n') != 5:
            raise ProtocolError('STOP write incomplete; use independent physical cutoff')


STATUS_FIELDS = {
    'preview_status': ('version', 'active', 'axis', 'ring', 'reason', 'elapsed_ms', 'angle_deg', 'rate_dps', 'calculated_output_not_applied', 'flywheel_link_fresh'),
    'bench_status': ('version', 'host_state', 'driver_state', 'ready', 'busy', 'host_reason', 'driver_reason', 'session', 'last_sequence', 'raw_B_command', 'elapsed_ms'),
    'timing': ('version', 'valid', 'period_us', 'max_period_us', 'work_us', 'max_work_us', 'overrun_count', 'imu_sample_period_us', 'imu_solve_us', 'imu_sequence', 'sample_fresh', 'attitude_ready'),
}


def status(client):
    out = {}
    for name, fields in STATUS_FIELDS.items():
        reply = client.request(name)
        if len(reply) != len(fields) or reply[0] != '1':
            raise ProtocolError(f'Unsupported {name} schema; do not infer readiness')
        out[name] = dict(zip(fields, reply))
    return out


def pulse_b(client, pwm: int, milliseconds: int, session: int, *, acknowledged: bool):
    if not acknowledged:
        raise ValueError('Explicit --ack-isolated-bench is required')
    if not (1 <= abs(pwm) <= 500 and 1 <= milliseconds <= 150 and 1 <= session <= 0xffffffff):
        raise ValueError('Require nonzero PWM within +/-500, 1..150 ms, and a displayed nonzero session')
    # Read only; never press/renew the local grant or choose a new session for
    # the caller. A delayed old request cannot consume a later physical grant.
    reply = client.request('bench_status')
    try:
        if len(reply) != 11 or int(reply[0]) != 1:
            raise ValueError('unsupported status schema')
        host, driver, ready, busy, reason, driver_reason, actual_session, last_seq = map(int, reply[1:9])
        if host not in (0, 4) or driver != 1 or ready != 1 or busy or reason not in range(9) or driver_reason or actual_session != session or last_seq or int(reply[9]) != 0:
            raise ValueError('not a fresh matching local permission')
    except ValueError as error:
        raise ProtocolError('Bench not ready; read status and obtain a new local grant: ' + str(error)) from error
    attempted = False
    try:
        attempted = True
        return client.request('bench_start', pwm, milliseconds, 'B', session)
    except BaseException:
        if attempted:
            try:
                client.stop()
            except Exception:
                pass
        raise


def build_parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True, help='Explicit UART2 serial port, for example COM6')
    p.add_argument('--baud', type=int, default=115200)
    sub = p.add_subparsers(dest='operation', required=True)
    sub.add_parser('status', help='Read only; does not renew driver permission or start anything')
    sub.add_parser('stop', help='Send STOP once; physical stop remains unverified')
    preview = sub.add_parser('preview', help='Real controller calculation, no motor output, max10s')
    preview.add_argument('axis', choices=('roll', 'pitch'))
    preview.add_argument('ring', choices=('rate', 'angle'))
    pulse = sub.add_parser('pulse-b', help='One session-bound B request; no retry or automatic rearm')
    pulse.add_argument('--pwm', type=int, required=True)
    pulse.add_argument('--milliseconds', type=int, required=True)
    pulse.add_argument('--session', type=int, required=True)
    pulse.add_argument('--ack-isolated-bench', action='store_true',
                       help='You have read COMMISSIONING.md and physically isolated motor power for signal acceptance')
    return p


def main(argv=None, *, port_factory=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.baud <= 0:
        parser.error('baud must be positive')
    if args.operation == 'pulse-b' and (not args.ack_isolated_bench or
            not 1 <= abs(args.pwm) <= 500 or not 1 <= args.milliseconds <= 150 or
            not 1 <= args.session <= 0xffffffff):
        parser.error('pulse-b requires --ack-isolated-bench, nonzero PWM within +/-500, 1..150ms and a displayed nonzero session')
    if port_factory is None:
        import serial
        port = serial.Serial(port=None, baudrate=args.baud, timeout=.1, write_timeout=.3,
                             rtscts=False, dsrdtr=False)
        port.dtr = False
        port.rts = False
        port.port = args.port
        port.open()  # Explicit operator action only. No probing or reconnect.
    else:
        port = port_factory(args.port, args.baud)
    client = Client(port)
    try:
        port.reset_input_buffer()
        if args.operation == 'status':
            result = status(client)
        elif args.operation == 'stop':
            client.stop()
            result = {'stop': 'sent once; verify physical stop independently'}
        elif args.operation == 'preview':
            try:
                result = {'preview': client.request('preview_start', args.axis, args.ring),
                          'note': 'Calculation only. Read status separately; expires after10s.'}
            except BaseException:
                client.stop()
                raise
        else:
            result = {'pulse_b': pulse_b(client, args.pwm, args.milliseconds, args.session,
                                         acknowledged=args.ack_isolated_bench),
                      'note': 'Request accepted, NOT proof of motion or stop. Firmware terminates independently.'}
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (ProtocolError, OSError, KeyboardInterrupt) as error:
        print('ERROR:', str(error))
        return 2
    finally:
        port.close()


if __name__ == '__main__':
    raise SystemExit(main())
