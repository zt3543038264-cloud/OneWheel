#!/usr/bin/env python3
"""One explicitly acknowledged powered C-wheel rate trial. Never retries a start.
ACKs are not proof of mechanical safety or physical stop. Keep an independent
motor-power cutoff available. This tool does not contact or configure other apps.
"""
import argparse
from collections import deque
import ipaddress
import math
import secrets
import socket
import sys
import time

FIELDS = ('version active mode state reason sensor_reason session keep_seq elapsed_ms '
          'heartbeat_age_ms imu_age_ms duration_ms limit target kp ki angle_deg rate_dps '
          'encoder_20ms calculated applied profile').split()
FLOATS = {'target', 'kp', 'ki', 'angle_deg', 'rate_dps', 'calculated'}
OPERATIONS = {'cbench_status', 'cbench_start', 'cbench_keep', 'cbench_stop'}

class ProtocolError(RuntimeError):
    pass

class UDPTransport:
    def __init__(self, host, port, bind, local_port):
        self.peer = (str(ipaddress.IPv4Address(host)), port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((str(ipaddress.IPv4Address(bind)), local_port))
        except BaseException:
            self.sock.close()
            raise
    def send(self, data):
        if self.sock.sendto(data, self.peer) != len(data):
            raise ProtocolError('Partial UDP send; no retry')
    def receive(self, timeout):
        deadline = time.monotonic() + timeout
        for _ in range(64):  # Bound even a flood of unverified peers.
            if time.monotonic() >= deadline:
                break
            self.sock.settimeout(max(1e-9, deadline - time.monotonic()))
            try:
                data, peer = self.sock.recvfrom(65535)
            except socket.timeout:
                return b''
            if peer == self.peer:
                return data
        return b''
    def close(self):
        self.sock.close()

class LineFramer:
    """Bounded serial stream framing, never scan for an embedded rsp prefix.

    Retain incomplete lines across requests. An overlong line poisons the entire
    line through its newline; its trailing bytes cannot become a new response.
    """
    MAX_LINE = 511  # Includes newline, as in the firmware response envelope.
    MAX_QUEUE = 64
    def __init__(self):
        self.partial = bytearray()
        self.lines = deque()
        self.discard = False
    def feed(self, raw):
        if len(raw) > 4096:
            self.partial.clear()
            self.lines.clear()
            self.discard = True
            raise ProtocolError('Oversized UDP datagram; stream boundary uncertain')
        for byte in raw:
            if self.discard:
                if byte == 10:
                    self.discard = False
                continue
            self.partial.append(byte)
            if len(self.partial) > self.MAX_LINE:
                self.partial.clear()
                self.discard = byte != 10
                continue
            if byte == 10:
                if len(self.lines) >= self.MAX_QUEUE:
                    self.partial.clear()
                    self.lines.clear()
                    self.discard = True
                    raise ProtocolError('Response line queue budget exceeded')
                self.lines.append(bytes(self.partial))
                self.partial.clear()

def parse_status(values):
    try:
        if len(values) != len(FIELDS):
            raise ValueError('schema')
        s = {k: float(v) if k in FLOATS else int(v) for k, v in zip(FIELDS, values)}
        if not all(math.isfinite(v) for v in s.values()):
            raise ValueError('nonfinite telemetry')
        if s['version'] != 1 or s['profile'] not in (0, 1, 2) or s['mode'] not in range(6):
            raise ValueError('version/profile/mode')
        if s['active'] not in (0, 1) or s['state'] not in range(4) or s['reason'] not in range(13):
            raise ValueError('state/reason')
        if s['sensor_reason'] not in range(20):
            raise ValueError('sensor reason')
        if bool(s['active']) != (s['state'] == 1):
            raise ValueError('active/state mismatch')
        if s['active'] and s['reason'] != 1:
            raise ValueError('running reason')
        if any(not 0 <= s[k] <= 0xffffffff for k in ('session', 'keep_seq', 'elapsed_ms', 'heartbeat_age_ms', 'imu_age_ms')):
            raise ValueError('counter')
        if not (0 <= s['duration_ms'] <= 3000 and 0 <= s['limit'] <= 500):
            raise ValueError('duration/limit')
        if not -0x80000000 <= s['encoder_20ms'] <= 0x7fffffff:
            raise ValueError('encoder')
        if not s['active'] and s['applied'] != 0:
            raise ValueError('inactive output')
        if s['active'] and not (1 <= s['duration_ms'] <= 3000 and 1 <= s['limit'] <= 500):
            raise ValueError('active limits')
        if abs(s['applied']) > 500 or (s['limit'] > 0 and abs(s['applied']) > s['limit']):
            raise ValueError('output')
        if (s['profile'] == 0 or s['mode'] in (0, 1)) and s['applied']:
            raise ValueError('dry output')
        return s
    except (ValueError, OverflowError) as e:
        raise ProtocolError('Invalid status: ' + str(e)) from e

class Client:
    def __init__(self, transport, *, clock=time.monotonic, timeout=.04, sequence=None, read_timeout=1.0):
        if not math.isfinite(timeout) or not 0 < timeout <= .04:
            raise ValueError('ACK timeout must be >0 and <=40ms')
        if not math.isfinite(read_timeout) or not .04 <= read_timeout <= 1.0:
            raise ValueError('Read-only status timeout must be 0.04..1.0 seconds')
        self.read_timeout = read_timeout
        self.transport, self.clock, self.timeout = transport, clock, timeout
        self.framer = LineFramer()
        self.sequence = secrets.randbelow(58000) if sequence is None else sequence
        if type(self.sequence) is not int or not 0 <= self.sequence <= 65500:
            raise ValueError('Initial sequence must reserve STOP capacity')
    def request(self, operation, *fields):
        if operation not in OPERATIONS:
            raise ValueError('Only C bench commands allowed')
        if self.sequence >= 65535:
            raise ProtocolError('Sequence exhausted')
        pieces = [str(x) for x in fields]
        if any(not p or any(c in p for c in ',\r\n') for p in pieces):
            raise ValueError('Invalid field')
        self.sequence += 1
        data = (','.join(['cfg:' + operation, str(self.sequence)] + pieces) + '\n').encode('ascii')
        if len(data) > 191:
            raise ValueError('Oversized command')
        # Only the read-only status command gets the independent longer wait.
        # START/KEEP/STOP retain the <=40ms deadline, including send and parsing.
        wait = self.read_timeout if operation == 'cbench_status' else self.timeout
        deadline = self.clock() + wait
        self.transport.send(data)
        # Complete lines received before this command cannot acknowledge it.
        self.framer.lines.clear()
        packets = byte_count = line_count = 0
        while self.clock() < deadline:
            if not self.framer.lines:
                if packets >= 64:
                    raise ProtocolError('ACK datagram processing budget exceeded')
                raw = self.transport.receive(deadline - self.clock())
                packets += 1
                if self.clock() >= deadline or not raw:
                    break
                byte_count += len(raw)
                if byte_count > 16384:
                    self.framer.partial.clear()
                    self.framer.discard = True
                    raise ProtocolError('ACK byte processing budget exceeded')
                self.framer.feed(raw)
                continue
            if line_count >= 128:
                raise ProtocolError('ACK line processing budget exceeded')
            raw = self.framer.lines.popleft()
            line_count += 1
            try:
                # Strip only the line terminator, never arbitrary prefixes.
                parts = raw[:-1].removesuffix(b'\r').decode('ascii').split(',')
            except UnicodeDecodeError:
                continue
            if parts[0] != 'rsp:' + str(self.sequence):
                continue
            if len(parts) < 3 or parts[1:3] != ['ok', operation]:
                raise ProtocolError('Rejected or malformed matching ACK')
            status = parse_status(parts[3:])
            if self.clock() >= deadline:
                break
            if operation == 'cbench_stop' and (status['active'] or status['applied']):
                raise ProtocolError('STOP ACK reports active/nonzero output')
            return status
        if operation == 'cbench_status':
            raise ProtocolError('cbench_status read-only reply not received before deadline; no retry')
        raise ProtocolError(operation + ' ACK timeout; outcome uncertain, no retry')

def validate_run(*, ack_powered_bench, polarity, kp, duration_ms, limit):
    if ack_powered_bench is not True or type(polarity) is not int or polarity not in (-1, 1):
        raise ValueError('Explicit --ack-powered-bench and --polarity +/-1 required')
    if not math.isfinite(kp) or not 0 < kp <= 20:
        raise ValueError('kp must be finite and >0..20')
    if type(duration_ms) is not int or not 1 <= duration_ms <= 1000:
        raise ValueError('duration must be 1..1000ms')
    if type(limit) is not int or not 1 <= limit <= 200:
        raise ValueError('limit must be 1..200')

def run(client, *, ack_powered_bench, polarity, kp, duration_ms, limit,
        clock=time.monotonic, sleep=time.sleep, trace=None):
    """Buffer all telemetry. No callbacks or I/O on the active heartbeat path.

    Heartbeats are anchored every 50ms. Scheduler lateness beyond 1ms is a
    missed slot (1ms permits ordinary OS wake jitter); never catch up or restart.
    One STOP is attempted in finally even if preflight or START is uncertain.
    """
    validate_run(ack_powered_bench=ack_powered_bench, polarity=polarity, kp=kp,
                 duration_ms=duration_ms, limit=limit)
    trace = [] if trace is None else trace
    mode = 'powered_rate_pos' if polarity == 1 else 'powered_rate_neg'
    mode_id = 4 if polarity == 1 else 5
    primary = None
    started = False
    def matches(s):
        return (s['profile'] == 2 and s['mode'] == mode_id and s['duration_ms'] == duration_ms
                and s['limit'] == limit and s['target'] == 0 and s['ki'] == 0
                and math.isclose(s['kp'], kp, rel_tol=1e-6, abs_tol=1e-6))
    try:
        s = client.request('cbench_status')
        trace.append(('preflight', s))
        if s['profile'] != 2 or s['active'] or s['session'] == 0xffffffff:
            raise ProtocolError('Preflight requires idle powered profile and available session')
        session = s['session'] + 1
        start = clock()
        s = client.request('cbench_start', s['session'], mode, 0, format(kp, '.9g'), 0, duration_ms, limit)
        trace.append(('start', s))
        if s['session'] != session or s['keep_seq'] != 0 or not s['active'] or not matches(s):
            raise ProtocolError('START session/configuration mismatch')
        started = True
        end, slot, heartbeat = start + duration_ms / 1000, start + .05, 0
        while slot < end:
            now = clock()
            if now > slot + .001:
                raise ProtocolError('Missed heartbeat schedule; no catch-up')
            sleep(max(0, slot - now))
            now = clock()
            if now > slot + .001:
                raise ProtocolError('Missed heartbeat schedule; no catch-up')
            if now >= end:
                break
            heartbeat += 1
            s = client.request('cbench_keep', session, heartbeat)
            trace.append(('keep', s))
            if s['session'] != session or s['keep_seq'] != heartbeat or not s['active'] or not matches(s):
                raise ProtocolError('KEEP session/configuration/state mismatch')
            slot = start + (heartbeat + 1) * .05
        sleep(max(0, end - clock()))
    except BaseException as e:
        primary = e
        raise
    finally:
        try:
            s = client.request('cbench_stop')
            if s['active'] or s['applied'] != 0:
                raise ProtocolError('STOP not confirmed; use physical cutoff')
        except BaseException as e:
            trace.append(('stop_unconfirmed', str(e)))
            if primary is None:
                raise
        else:
            # Confirmed software zero does not mean the trial ended normally.
            trace.append(('stop', s))
            if started and (s['session'] != session or not matches(s) or
                            s['state'] != 2 or s['reason'] not in (2, 3)):
                error = ProtocolError('Software output zero confirmed, but trial ended '
                                      'with fault/session/configuration mismatch')
                trace.append(('trial_failed', str(error)))
                if primary is None:
                    raise error
    return trace

def build_parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--read-timeout', type=float, default=1.0,
                   help='Read-only status/preflight wait 0.04..1.0s (default 1.0); never retries')
    p.add_argument('--host', default='192.168.0.111')
    p.add_argument('--port', type=int, default=5001)
    p.add_argument('--bind', default='0.0.0.0')
    p.add_argument('--local-port', type=int, default=8081)
    sub = p.add_subparsers(dest='operation', required=True)
    sub.add_parser('status')
    sub.add_parser('stop')
    r = sub.add_parser('run')
    r.add_argument('--ack-powered-bench', action='store_true', required=True)
    r.add_argument('--polarity', type=int, choices=(-1, 1), required=True)
    r.add_argument('--kp', type=float, required=True)
    r.add_argument('--duration-ms', type=int, required=True)
    r.add_argument('--limit', type=int, required=True)
    return p

def main(argv=None, *, transport_factory=UDPTransport):
    p = build_parser()
    args = p.parse_args(argv)
    params = {k: getattr(args, k) for k in ('ack_powered_bench', 'polarity', 'kp', 'duration_ms', 'limit')} if args.operation == 'run' else {}
    try:
        ipaddress.IPv4Address(args.host)
        ipaddress.IPv4Address(args.bind)
        if not 1 <= args.port <= 65535 or not 0 <= args.local_port <= 65535:
            raise ValueError('Invalid UDP ports')
        if not math.isfinite(args.read_timeout) or not .04 <= args.read_timeout <= 1.0:
            raise ValueError('Read-only status timeout must be 0.04..1.0 seconds')
        if params:
            validate_run(**params)
    except ValueError as e:
        p.error(str(e))
    transport, trace, result = None, [], 0
    try:
        transport = transport_factory(args.host, args.port, args.bind, args.local_port)
        client = Client(transport, read_timeout=args.read_timeout)
        if args.operation == 'run':
            run(client, **params, trace=trace)
        else:
            s = client.request('cbench_' + args.operation)
            trace.append((args.operation, s))
            if args.operation == 'stop' and (s['active'] or s['applied']):
                raise ProtocolError('STOP not confirmed')
    except (ProtocolError, OSError, KeyboardInterrupt) as e:
        trace.append(('ERROR', str(e) or 'Interrupted'))
        result = 2
    finally:
        if transport:
            transport.close()
        for event, data in trace:
            print(event, data)
        if result:
            print('No automatic retry. If stop is uncertain, cut motor power independently.', file=sys.stderr)
    return result

if __name__ == '__main__':
    raise SystemExit(main())
