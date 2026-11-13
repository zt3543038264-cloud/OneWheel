"""Bounded existing-firmware Pitch preview. No cbench keep/start or motor commands.
Import is inert. This records sampled MCU snapshots, never reconstructs missing data.
"""
from __future__ import annotations
import ipaddress
import math
import secrets
import socket
import time

VERSION = '1.0.0'
FIRMWARE_ID = 'tc264-cfg1-task1-path1-sign2'
FIELDS = ('version', 'active', 'axis', 'ring', 'reason', 'elapsed_ms',
          'angle_deg', 'rate_dps', 'calculated_output_not_applied', 'flywheel_link_fresh')
INTEGER_FIELDS = set(FIELDS) - {'angle_deg', 'rate_dps', 'calculated_output_not_applied'}
ALLOWED = {'hello': (), 'cbench_status': (), 'preview_status': (),
           'preview_start': ('pitch', 'rate'), 'preview_stop': ()}

class ProtocolError(RuntimeError): pass
class Cancelled(ProtocolError): pass


def parse_preview(values):
    if len(values) != 10:
        raise ProtocolError('Unsupported preview_status field count')
    try:
        s = {k: int(v) if k in INTEGER_FIELDS else float(v) for k, v in zip(FIELDS, values)}
        if not all(math.isfinite(v) for v in s.values()): raise ValueError('nonfinite data')
        if s['version'] != 1 or s['active'] not in (0, 1): raise ValueError('version/active')
        if s['axis'] not in (0, 1) or s['ring'] not in (0, 1): raise ValueError('axis/ring')
        if not 0 <= s['reason'] <= 19 or not 0 <= s['elapsed_ms'] <= 10000: raise ValueError('reason/time')
        if s['flywheel_link_fresh'] not in (0, 1): raise ValueError('link state')
        if s['active'] and s['reason'] != 0: raise ValueError('active fault')
        if not s['active'] and s['calculated_output_not_applied'] != 0: raise ValueError('inactive output')
    except (ValueError, OverflowError) as exc:
        raise ProtocolError('Invalid preview status: ' + str(exc)) from exc
    return s


def validate_cbench(values):
    # Exact current v1 layout, with profile at index21 and applied at index20.
    if len(values) != 22: raise ProtocolError('Unsupported cbench_status schema')
    int_indices = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 18, 20, 21}
    try:
        n = [int(v) if i in int_indices else float(v) for i, v in enumerate(values)]
        if not all(math.isfinite(v) for v in n): raise ValueError('nonfinite')
        if n[0] != 1 or n[1] not in (0, 1) or n[2] not in range(4) or n[3] not in range(4): raise ValueError('schema/state')
        if n[4] not in range(13) or n[5] not in range(20): raise ValueError('reason')
        if any(not 0 <= n[i] <= 0xffffffff for i in range(6, 11)): raise ValueError('counter')
        if not 0 <= n[11] <= 3000 or not 0 <= n[12] <= 500: raise ValueError('bounds')
        if n[21] != 0 or n[20] != 0: raise ValueError('requires calculation-only profile and zero reported bench applied')
        if n[1]: raise ValueError('C bench is active; no preview started')
    except (ValueError, OverflowError) as exc:
        raise ProtocolError('C bench preflight failed: ' + str(exc)) from exc
    return {'profile': 0, 'bench_applied_reported': 0}


class UDPTransport:
    def __init__(self, host, port, bind, local_port):
        peer = ipaddress.IPv4Address(host)
        local = ipaddress.IPv4Address(bind)
        if peer.is_multicast or peer.is_unspecified or str(peer) == '255.255.255.255':
            raise ValueError('An explicit unicast board IPv4 is required')
        if not 1 <= port <= 65535 or not 1 <= local_port <= 65535:
            raise ValueError('UDP ports must be 1..65535')
        self.peer = (str(peer), port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            self.socket.bind((str(local), local_port))
        except BaseException:
            self.socket.close()
            raise
    def send(self, wire):
        if self.socket.sendto(wire, self.peer) != len(wire):
            raise ProtocolError('Partial send; outcome uncertain')
    def receive(self, timeout):
        self.socket.settimeout(timeout)
        try:
            raw, peer = self.socket.recvfrom(4096)
        except socket.timeout:
            return b''
        return raw if peer == self.peer else b''
    def close(self): self.socket.close()


class Client:
    def __init__(self, transport, *, timeout=.75, clock=time.monotonic, sequence=None):
        if not math.isfinite(timeout) or not 0 < timeout <= 1:
            raise ValueError('Observation timeout must be >0 and <=1 second')
        self.transport, self.timeout, self.clock = transport, timeout, clock
        self.sequence = secrets.randbelow(50000) + 1 if sequence is None else sequence
        if type(self.sequence) is not int or not 0 <= self.sequence < 65535:
            raise ValueError('Invalid initial sequence')
    def request(self, operation, *fields, cancel=None, on_send=None):
        if operation not in ALLOWED or tuple(fields) != ALLOWED[operation]:
            raise ValueError('This client permits only fixed Pitch calculation-preview operations')
        if self.sequence >= 65535: raise ProtocolError('Sequence exhausted; no wrap/reconnect')
        if cancel is not None and cancel.is_set(): raise Cancelled('Cancelled before send')
        self.sequence += 1
        wire = ('cfg:' + operation + ',' + str(self.sequence) +
                ''.join(',' + f for f in fields) + '\n').encode('ascii')
        if on_send is not None: on_send()
        self.transport.send(wire)  # Once only. No mutation retry.
        deadline = self.clock() + self.timeout
        while self.clock() < deadline:
            if cancel is not None and cancel.is_set(): raise Cancelled('Stop requested')
            raw = self.transport.receive(max(.0001, min(.05, deadline - self.clock())))
            if self.clock() > deadline: break
            if not raw or len(raw) > 511 or not raw.endswith(b'\n') or raw.count(b'\n') != 1: continue
            try: parts = raw.decode('ascii').rstrip('\r\n').split(',')
            except UnicodeDecodeError: continue
            if parts[0] != 'rsp:' + str(self.sequence): continue
            if len(parts) < 3 or parts[1] not in ('ok', 'err'):
                raise ProtocolError('Malformed matching reply')
            if parts[1] == 'err': raise ProtocolError('MCU rejected: ' + ','.join(parts[2:]))
            if parts[2] != operation: raise ProtocolError('Matching sequence has wrong operation')
            return parts[3:]
        raise ProtocolError(operation + ' timed out; result unknown, no automatic retry')


class PreviewSession:
    def __init__(self, host='192.168.0.111', port=5001, bind='0.0.0.0', local_port=8081,
                 emit=lambda event, data: None, *, transport_factory=UDPTransport,
                 clock=time.monotonic):
        self.endpoint = (host, port, bind, local_port)
        self.emit, self.factory, self.clock = emit, transport_factory, clock
        self.start_attempted = False

    def run(self, cancel_event):
        transport = None
        attempted = False
        confirmed = False
        outcome = 'Not started'
        previous = None
        try:
            self.emit('state', {'text': 'Checking firmware and zero-output profile'})
            transport = self.factory(*self.endpoint)
            client = Client(transport, clock=self.clock)
            ident = client.request('hello', cancel=cancel_event)
            if ident != ['1', FIRMWARE_ID, '63']:
                raise ProtocolError('Firmware identity differs; no start sent')
            profile = validate_cbench(client.request('cbench_status', cancel=cancel_event))
            idle = parse_preview(client.request('preview_status', cancel=cancel_event))
            if idle['active']: raise ProtocolError('A preview is already active; no start sent')
            self.emit('identity', {'firmware': ident[1], **profile})
            self.emit('log', {'message': 'Protocol identity matched, not a firmware hash or physical-output certification.'})
            if cancel_event.is_set(): raise Cancelled('Cancelled before preview')
            def mark_attempted():
                nonlocal attempted
                attempted = True
                self.start_attempted = True
            sent_at = self.clock()
            ack = client.request('preview_start', 'pitch', 'rate', cancel=cancel_event, on_send=mark_attempted)
            if ack != ['1', 'ZERO_OUTPUT']: raise ProtocolError('Unexpected preview_start ACK')
            self.emit('state', {'text': 'Pitch/rate preview; existing firmware PID; maximum 10 seconds'})
            deadline = sent_at + 10.0
            while self.clock() < deadline:
                if cancel_event.is_set(): raise Cancelled('Operator requested stop')
                s = parse_preview(client.request('preview_status', cancel=cancel_event))
                now = self.clock()
                if s['axis'] != 1 or s['ring'] != 0:
                    raise ProtocolError('Preview axis/ring mismatch')
                if previous is not None and s['elapsed_ms'] < previous['elapsed_ms']:
                    raise ProtocolError('MCU elapsed time moved backwards; do not merge sessions')
                if previous is not None and s['active'] and s['elapsed_ms'] == previous['elapsed_ms']:
                    raise ProtocolError('MCU preview time is not advancing; stale snapshot rejected')
                gap = previous is not None and (now - previous['host_monotonic_s'] > .25 or
                                                  s['elapsed_ms'] - previous['elapsed_ms'] > 250)
                row = {**s, 'host_monotonic_s': now, 'gap': gap}
                self.emit('sample', row)
                previous = row
                if not s['active']:
                    outcome = 'MCU preview ended, reason=' + str(s['reason'])
                    break
                if cancel_event.wait(.1): raise Cancelled('Operator requested stop')
            else:
                outcome = 'Host 10-second deadline reached'
        except (ProtocolError, OSError, ValueError) as exc:
            outcome = str(exc)
            self.emit('log', {'message': outcome})
        finally:
            if attempted and transport is not None:
                self.emit('state', {'text': 'Sending one preview_stop and reading fresh preview status'})
                stop_ack = False
                try:
                    stop = client.request('preview_stop')  # Cancellation cannot suppress the one stop.
                    if stop: raise ProtocolError('Unexpected preview_stop ACK payload')
                    stop_ack = True
                except (ProtocolError, OSError, ValueError) as exc:
                    outcome += '; STOP ACK UNCONFIRMED: ' + str(exc)
                # Read once even if the stop ACK was lost; never repeat the mutation.
                try:
                    final = parse_preview(client.request('preview_status'))
                    if final['active']: raise ProtocolError('MCU still reports preview active')
                    confirmed = stop_ack
                    message = 'MCU reported preview inactive after stop; physical stop is not measured.'
                    if not stop_ack:
                        outcome += '; fresh status reports inactive, but stop ACK was not confirmed'
                    self.emit('log', {'message': message})
                except (ProtocolError, OSError, ValueError) as exc:
                    outcome += '; STOP UNCONFIRMED: ' + str(exc)
            if transport is not None: transport.close()
            self.emit('finish', {'confirmed': confirmed, 'message': outcome,
                                 'start_attempted': attempted})
        return {'confirmed': confirmed, 'message': outcome, 'start_attempted': attempted}
