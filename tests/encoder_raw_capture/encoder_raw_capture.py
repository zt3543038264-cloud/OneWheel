"""Capture encoder telemetry only; no motor, parameter, mode or flash commands.

Python 3.9+, standard library only. A software observation is not a hardware
safety interlock. Stop the board, exit open-loop tests and isolate motor power.
"""
import argparse
from datetime import datetime, timezone
import ipaddress
import json
import math
from pathlib import Path
import socket
import struct
import sys
import time

ROOT = Path(__file__).resolve().parent
VERSION = '1.1'
KNOWN_FIELDS = frozenset(json.loads((ROOT / 'protocol_fields.json').read_text(encoding='utf-8'))['fields'])
TRAILER = b'\x00\x00\x80\x7f'
STOP_ZERO = ('control_mode', 'rate_state', 'rate_enabled', 'pwm_debug_enabled',
             'motor_left_pwm', 'motor_right_pwm', 'motor_ground_pwm', 'fault_reason')
FIELDS = ('telemetry_ms',) + STOP_ZERO + (
    'wheel_encoder_raw', 'wheel_encoder_delta', 'wheel_encoder_seq',
    'wheel_encoder_dt_ms', 'wheel_encoder_valid', 'wheel_encoder_errors',
    'wheel_encoder_sign', 'wheel_speed_cps', 'wheel_feedback_age_ms',
    'wheel_feedback_valid', 'wheel_position_count', 'wheel_position_valid')
SUB_COMMAND = ('SUB ' + ','.join(FIELDS) + '\n').encode('ascii')
ALLOWED_COMMANDS = (b'GET?\n', SUB_COMMAND, b'RATE 20\n', b'STREAM 1\n')


class ProtocolError(ValueError):
    pass


def parse_packet(data, schema):
    binary_shape = schema is not None and len(data) == 4 * len(schema) + 4 and data[-4:] == TRAILER
    if data.startswith(b'MCAR ') and not binary_shape:
        try:
            text = data.decode('ascii').rstrip('\r\n')
        except UnicodeDecodeError as exc:
            raise ProtocolError('Non-ASCII control response') from exc
        if '\n' in text or '\r' in text:
            raise ProtocolError('Multiple control responses in one packet')
        kind, sep, value = text[5:].partition(' ')
        if not sep:
            raise ProtocolError('Missing control response value')
        if kind == 'SUB':
            fields = tuple(value.split(','))
            if not 1 <= len(fields) <= 48 or len(set(fields)) != len(fields):
                raise ProtocolError('Invalid subscription size or duplicate fields')
            if any(k not in KNOWN_FIELDS for k in fields):
                raise ProtocolError('Unknown telemetry field; firmware version mismatch')
            return kind, fields
        if kind in ('RATE', 'STREAM'):
            try:
                number = int(value)
            except ValueError as exc:
                raise ProtocolError('Invalid telemetry acknowledgment') from exc
            if (kind == 'RATE' and not 2 <= number <= 1000) or (kind == 'STREAM' and number not in (0, 1)):
                raise ProtocolError('Out-of-range telemetry acknowledgment')
            return kind, number
        if kind in ('ERR', 'ERROR'):
            raise ProtocolError('Firmware rejected request: ' + value)
        raise ProtocolError('Unexpected control response: ' + kind)
    if schema is None:
        return 'UNMAPPED', None
    if len(data) != 4 * len(schema) + 4 or data[-4:] != TRAILER:
        raise ProtocolError('Frame length/trailer mismatch; refusing to guess fields')
    values = struct.unpack('<%df' % len(schema), data[:-4])
    if not all(math.isfinite(v) for v in values):
        raise ProtocolError('Non-finite telemetry value')
    return 'DATA', dict(zip(schema, values))


class CaptureSession:
    def __init__(self, ip, module_port, bind, listen_port, output, timeout=5.):
        address = ipaddress.ip_address(ip)
        if address.version != 4 or address.is_multicast or address.is_unspecified or str(address) == '255.255.255.255':
            raise ValueError('Use the board IPv4 address, not broadcast/multicast')
        if not all(1 <= port <= 65535 for port in (module_port, listen_port)):
            raise ValueError('Invalid UDP port')
        self.peer = (str(address), module_port)
        self.timeout = timeout
        self.schema = None
        self.expecting = False
        self.last_time = None
        self.socket = None
        self.log = Path(output).open('x', encoding='utf-8')
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # No port reuse: fail instead of competing with the original GUI.
            self.socket.bind((bind, listen_port))
            self.socket.settimeout(.1)
        except Exception:
            self.close()
            raise

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        if not self.log.closed:
            self.log.close()

    def event(self, direction, **kwargs):
        self.log.write(json.dumps({'timestamp': datetime.now(timezone.utc).isoformat(),
                                   'direction': direction, **kwargs}, ensure_ascii=False) + '\n')
        self.log.flush()

    def send(self, data):
        if data not in ALLOWED_COMMANDS:
            raise ProtocolError('Command blocked: capture-only allowlist')
        self.event('TX', text=data.decode('ascii'))
        self.socket.sendto(data, self.peer)

    def receive(self, deadline):
        while time.monotonic() < deadline:
            try:
                data, peer = self.socket.recvfrom(8192)
            except socket.timeout:
                continue
            if peer != self.peer:
                self.event('IGNORE', reason='unexpected endpoint', address=peer)
                continue
            kind, value = parse_packet(data, self.schema)
            self.event('RX', kind=kind, value=value)
            if kind == 'SUB':
                if self.expecting and value != FIELDS:
                    self.schema = None
                    raise ProtocolError('Subscription changed; capture aborted')
                if self.expecting and self.schema is not None:
                    # Firmware announces the current profile periodically. An
                    # EXACT same ordered schema is metadata, not a new session.
                    # Never reset last_time: stale/reordered data and reboot
                    # must remain detectable across a repeated announcement.
                    self.event('METADATA_REPEAT', kind='SUB', note='Exact same ordered schema; board-time baseline retained')
                else:
                    self.schema = value
                    self.last_time = None
            if kind == 'DATA' and self.expecting:
                stamp = value.get('telemetry_ms')
                if stamp is None or (self.last_time is not None and stamp <= self.last_time):
                    raise ProtocolError('Non-increasing board time; possible reboot or reordered frame')
                self.last_time = stamp
                if any(value[k] != 0. for k in STOP_ZERO):
                    raise ProtocolError('Stopped-status check failed; no Stop command was sent. Use your local safe-stop procedure.')
            return kind, value
        raise TimeoutError('UDP response timeout; no automatic retry')

    def wait_ack(self, kind, value):
        deadline = time.monotonic() + self.timeout
        while True:
            actual_kind, actual_value = self.receive(deadline)
            if actual_kind == kind:
                if actual_value != value:
                    raise ProtocolError('Acknowledgment does not match request')
                return

    def configure(self):
        self.send(b'GET?\n')
        deadline = time.monotonic() + self.timeout
        original = {}
        while not all(k in original for k in ('SUB', 'RATE', 'STREAM')):
            kind, value = self.receive(deadline)
            if kind in ('SUB', 'RATE', 'STREAM'):
                original[kind] = value
        self.event('ORIGINAL_TELEMETRY', settings=original)
        self.expecting = True
        self.schema = None
        self.send(SUB_COMMAND)
        self.wait_ack('SUB', FIELDS)
        self.send(b'RATE 20\n')
        self.wait_ack('RATE', 20)
        self.send(b'STREAM 1\n')
        self.wait_ack('STREAM', 1)
        return original

    def capture(self, seconds, progress=True):
        end = time.monotonic() + seconds
        data_deadline = time.monotonic() + self.timeout
        frames = 0
        next_print = 0.
        while time.monotonic() < end:
            try:
                kind, row = self.receive(min(end, data_deadline))
            except TimeoutError:
                if time.monotonic() < end - .05:
                    raise
                break
            if kind != 'DATA':
                expected_metadata = {'SUB': FIELDS, 'RATE': 20, 'STREAM': 1}
                if kind not in expected_metadata or row != expected_metadata[kind]:
                    raise ProtocolError('Telemetry settings changed or unexpected control response during capture')
                # Repeated identical SUB/RATE/STREAM metadata is legitimate.
                # It is neither a sample nor evidence that fresh data exists.
                continue
            frames += 1
            data_deadline = time.monotonic() + self.timeout
            if progress and time.monotonic() >= next_print:
                print('raw={wheel_encoder_raw:g}  delta={wheel_encoder_delta:g}  seq={wheel_encoder_seq:g}  '
                      'dt_ms={wheel_encoder_dt_ms:g}  valid={wheel_encoder_valid:g}  '
                      'speed_cps={wheel_speed_cps:g}'.format(**row), flush=True)
                next_print = time.monotonic() + .5
        if frames == 0:
            raise TimeoutError('No data frames captured')
        return frames


def main(argv=None):
    parser = argparse.ArgumentParser(description='Stopped, motor-power-isolated encoder raw/delta telemetry capture only.')
    parser.add_argument('--device-ip', default='192.168.0.119', help='board IPv4 address, not the computer address')
    parser.add_argument('--seconds', type=float, default=20.)
    parser.add_argument('--output', type=Path, help='new JSONL filename; existing files are never overwritten')
    parser.add_argument('--module-port', type=int, default=5001)
    parser.add_argument('--listen-port', type=int, default=8081)
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--timeout', type=float, default=5.)
    parser.add_argument('--confirm-stopped-isolated', action='store_true',
                        help='operator has stopped the board, exited open-loop tests, isolated motor power and closed other senders')
    args = parser.parse_args(argv)
    session = None
    output = args.output or Path('encoder_raw_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.jsonl')
    try:
        if not args.confirm_stopped_isolated:
            raise ValueError('Physically verify Stop and motor-power isolation first, then use --confirm-stopped-isolated. No socket opened.')
        if not math.isfinite(args.seconds) or not .2 <= args.seconds <= 300:
            raise ValueError('--seconds must be 0.2..300')
        if not math.isfinite(args.timeout) or not .5 <= args.timeout <= 30:
            raise ValueError('--timeout must be 0.5..30')
        session = CaptureSession(args.device_ip, args.module_port, args.bind, args.listen_port, output, args.timeout)
        original = session.configure()
        print('CAPTURING v' + VERSION + '. No motor/parameter commands. Keep motor power isolated.', flush=True)
        frames = session.capture(args.seconds)
        result = {'status': 'CAPTURE_COMPLETE', 'version': VERSION, 'frames': frames, 'log': str(output.resolve()),
                  'motion_commands_sent': 0, 'parameter_commands_sent': 0,
                  'original_telemetry': original}
        session.event('RESULT', **result)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except KeyboardInterrupt:
        if session:
            session.event('ABORTED', reason='KeyboardInterrupt')
        print('Capture interrupted. Partial log retained. No motor Stop was sent.', file=sys.stderr)
        return 130
    except (ValueError, OSError, TimeoutError, KeyError, struct.error) as exc:
        if session:
            session.event('ERROR', reason=str(exc))
        print('ERROR: ' + str(exc), file=sys.stderr)
        print('Keep motor power isolated. Save any partial log; do not use Start to test the connection.', file=sys.stderr)
        return 2
    finally:
        if session:
            session.close()
            print('Telemetry may remain on this raw/delta selection at 20 ms. Re-select your GUI profile before GUI recording.')


if __name__ == '__main__':
    raise SystemExit(main())
