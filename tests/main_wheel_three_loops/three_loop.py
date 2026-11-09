"""Offline ground-wheel analysis and explicitly requested, stopped-only RAM writes.

No motor, Start, Stop, flash, raw PWM, polarity or mechanical-zero commands.
Python 3.9+, standard library only. Not a safety-rated hardware interlock.
"""
import argparse
import csv
import ipaddress
import json
import math
from pathlib import Path
import socket
import statistics
import struct
import sys
import time
from datetime import datetime, timezone
from xml.sax.saxutils import escape

ROOT = Path(__file__).resolve().parent
FIELDS = frozenset(json.loads((ROOT / 'protocol_fields.json').read_text(encoding='utf-8'))['fields'])
TRAILER = b'\x00\x00\x80\x7f'
COMMON = ['telemetry_ms', 'control_mode', 'rate_state', 'rate_enabled', 'rate_axis',
          'rate_only', 'dual_enabled', 'pwm_debug_enabled', 'startup_scale', 'fault_reason',
          'imu_status', 'imu_seq', 'rate_seq', 'motor_left_pwm', 'motor_right_pwm', 'motor_ground_pwm']
GROUND = ['ground_rate_kp', 'ground_pwm_limit', 'ground_sign', 'ground_angle_kp',
          'ground_rate_limit_dps', 'ground_deadzone_pos_pwm', 'ground_deadzone_neg_pwm']
SINGLE = ['rate_target_dps', 'rate_actual_dps', 'rate_error_dps', 'rate_pwm',
          'balance_target_deg', 'balance_actual_deg', 'balance_error_deg']
DUAL = ['dual_pitch_actual_deg', 'dual_pitch_target_deg', 'dual_pitch_error_deg',
        'dual_pitch_target_rate_dps', 'dual_pitch_rate_dps', 'dual_pitch_rate_error_dps', 'dual_pitch_pwm']
SPEED = ['wheel_speed_cps', 'wheel_target_cps', 'wheel_speed_error', 'wheel_tilt_offset',
         'wheel_speed_integral', 'wheel_feedback_age_ms', 'wheel_feedback_valid',
         'wheel_encoder_seq', 'wheel_encoder_dt_ms', 'wheel_encoder_valid',
         'wheel_position_count', 'wheel_position_valid', 'wheel_hold_state',
         'wheel_speed_kp', 'wheel_speed_ki', 'wheel_tilt_limit', 'wheel_i_limit',
         'wheel_pos_kp', 'wheel_speed_limit', 'wheel_pos_error', 'wheel_ref_count']
PROFILES = {'rate': COMMON + GROUND + SINGLE + DUAL,
            'angle': COMMON + GROUND + SINGLE + DUAL,
            'speed': COMMON + SPEED + DUAL + ['ground_rate_kp', 'ground_angle_kp',
                     'ground_pwm_limit', 'ground_rate_limit_dps']}
# write_name: (minimum, maximum, dedicated telemetry readback name)
PARAMETERS = {
    'rate': {'rate_kp': (0., 500., 'ground_rate_kp'),
             'rate_pwm_limit': (0., 8000., 'ground_pwm_limit')},
    'angle': {'ground_angle_kp': (0., 100., 'ground_angle_kp'),
              'ground_rate_limit_dps': (0.1, 200., 'ground_rate_limit_dps')},
    'speed': {'wheel_speed_kp': (0., 0.1, 'wheel_speed_kp'),
              'wheel_speed_ki': (0., 0.1, 'wheel_speed_ki'),
              'wheel_tilt_limit': (0., 5., 'wheel_tilt_limit'),
              'wheel_i_limit': (0., 5., 'wheel_i_limit')},
}
MODES = {'rate': (7,), 'angle': (4, 5, 2, 10), 'speed': (2, 10)}
STOP_ZERO = ['control_mode', 'rate_state', 'rate_enabled', 'pwm_debug_enabled',
             'motor_left_pwm', 'motor_right_pwm', 'motor_ground_pwm', 'fault_reason']


class ProtocolError(ValueError):
    pass


class UncertainWrite(RuntimeError):
    """A write may have executed; never retry automatically."""


def finite(value):
    if isinstance(value, bool):
        raise ValueError('boolean is not a measurement')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('non-finite number')
    return value


def parameter(loop, text):
    name, sep, raw = text.partition('=')
    if not sep or name not in PARAMETERS[loop]:
        raise ValueError('unsupported parameter for this loop; see parameters command')
    number = finite(raw)
    lo, hi, readback = PARAMETERS[loop][name]
    if not lo <= number <= hi:
        raise ValueError(f'{name}: permitted range {lo}..{hi}')
    # Firmware stores float32 and replies with 9 significant decimal digits.
    number = struct.unpack('<f', struct.pack('<f', number))[0]
    command = f'[slider,{name},{number:.9g}]\n'.encode('ascii')
    return name, number, readback, command


def close_value(left, right):
    # Firmware uses float32 and %.9g: nine digits round-trip to the exact f32.
    # Do not accept tiny nonzero Ki as zero, or a nearby different gain.
    try:
        return struct.pack('<f', finite(left)) == struct.pack('<f', finite(right))
    except (ValueError, TypeError, OverflowError, struct.error):
        return False


def parse_packet(data, schema):
    binary_shape = schema is not None and len(data) == 4 * len(schema) + 4 and data[-4:] == TRAILER
    if data.startswith(b'MCAR ') and not binary_shape:
        try:
            text = data.decode('ascii').rstrip('\r\n')
        except UnicodeDecodeError as exc:
            raise ProtocolError('non-ASCII control response') from exc
        if '\n' in text or '\r' in text:
            raise ProtocolError('multiple control messages in one packet')
        kind, sep, value = text[5:].partition(' ')
        if kind == 'SUB':
            names = value.split(',')
            if not sep or not 1 <= len(names) <= 48 or len(set(names)) != len(names):
                raise ProtocolError('invalid schema length/duplicates')
            if any(n not in FIELDS for n in names):
                raise ProtocolError('unknown firmware field; version mismatch')
            return kind, tuple(names)
        if kind in ('ERR', 'ERROR'):
            raise ProtocolError('firmware rejected request: ' + value)
        if kind == 'SLIDER':
            name, sep, val = value.partition(' ')
            if not sep:
                raise ProtocolError('invalid slider acknowledgment')
            try:
                return kind, (name, finite(val))
            except ValueError as exc:
                raise ProtocolError('invalid slider acknowledgment value') from exc
        if kind in ('RATE', 'STREAM'):
            try:
                val = int(value)
            except ValueError as exc:
                raise ProtocolError('invalid telemetry setting acknowledgment') from exc
            if (kind == 'RATE' and not 2 <= val <= 1000) or (kind == 'STREAM' and val not in (0, 1)):
                raise ProtocolError('out-of-range telemetry setting acknowledgment')
            return kind, val
        if kind == 'LIST':
            return kind, tuple(value.split(','))
        raise ProtocolError('unexpected control response: ' + kind)
    if schema is None:
        return 'UNMAPPED', None
    if len(data) != 4 * len(schema) + 4 or data[-4:] != TRAILER:
        raise ProtocolError('frame length/trailer mismatch; schema not safe to assume')
    values = struct.unpack('<%df' % len(schema), data[:-4])
    if not all(math.isfinite(v) for v in values):
        raise ProtocolError('non-finite telemetry')
    return 'DATA', dict(zip(schema, values))


def is_stopped(row):
    return all(k in row and row[k] == 0. for k in STOP_ZERO)


class Session:
    def __init__(self, ip, port, bind, listen_port, event_path, timeout=5.):
        addr = ipaddress.ip_address(ip)
        if addr.version != 4 or addr.is_multicast or addr.is_unspecified or str(addr) == '255.255.255.255':
            raise ValueError('provide the board IPv4 address, not broadcast/multicast')
        if not 1 <= port <= 65535 or not 1 <= listen_port <= 65535:
            raise ValueError('invalid UDP port')
        self.peer = (str(addr), port)
        self.timeout = timeout
        self.schema = None
        self.expected = None
        self.pending_original_schema = None
        self.last_time = None
        self.last_rx = None
        self.socket = None
        self.log = Path(event_path).open('x', encoding='utf-8')
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # No SO_REUSEADDR: never silently steal/share an existing GUI listener.
            self.socket.bind((bind, listen_port))
            self.socket.settimeout(0.1)
        except Exception:
            self.close()
            raise

    def event(self, direction, **kwargs):
        row = {'timestamp': datetime.now(timezone.utc).isoformat(), 'direction': direction, **kwargs}
        self.log.write(json.dumps(row, ensure_ascii=False) + '\n')
        self.log.flush()

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        if not self.log.closed:
            self.log.close()

    def send(self, data):
        self.event('TX', text=data.decode('ascii'))
        self.socket.sendto(data, self.peer)

    def receive(self, deadline):
        while time.monotonic() < deadline:
            try:
                data, address = self.socket.recvfrom(8192)
            except socket.timeout:
                continue
            if address != self.peer:
                self.event('IGNORE', reason='unexpected endpoint', address=address)
                continue
            now = time.monotonic()
            kind, value = parse_packet(data, self.schema)
            self.event('RX', kind=kind, value=value)
            if kind == 'SUB':
                if self.expected is not None and value != self.expected:
                    self.schema = None
                    self.last_time = None
                    self.last_rx = None
                    # GET? and periodic announcements carry no request ID.
                    # Before the exact new SUB is acknowledged, an old,
                    # byte-for-byte equivalent field order may still arrive.
                    # Never map data with it or extend the handshake deadline.
                    if self.pending_original_schema is not None and value == self.pending_original_schema:
                        self.event('OLD_SUB_BEFORE_ACK', fields=value,
                                   note='Known original schema; ignored before exact requested SUB acknowledgment')
                        return 'OLD_SUB', value
                    self.pending_original_schema = None
                    raise ProtocolError('subscription changed; abort, do not assume same-length field order')
                self.pending_original_schema = None
                self.schema = value
                self.last_time = None
                self.last_rx = None
            # GET? can describe the stock Mode10 seven-channel profile, which
            # has no board clock. Timing checks begin only after our requested
            # subscription is acknowledged, not while querying old metadata.
            if kind == 'DATA' and self.expected is not None:
                stamp = value.get('telemetry_ms')
                if stamp is None or (self.last_time is not None and stamp <= self.last_time):
                    raise ProtocolError('non-increasing board time; reboot, duplicate or reordered frame')
                self.last_time = stamp
                self.last_rx = now
            return kind, value
        raise TimeoutError('UDP response timeout; no automatic retransmission')

    def wait_ack(self, kind, value):
        deadline = time.monotonic() + self.timeout
        while True:
            got_kind, got_value = self.receive(deadline)
            if got_kind == kind:
                if got_value != value:
                    raise ProtocolError('acknowledgment does not match request')
                return

    def configure(self, loop):
        # Query actual schema first. GET? is not a parameter readback.
        self.send(b'GET?\n')
        deadline = time.monotonic() + self.timeout
        original = {}
        while not all(k in original for k in ('SUB', 'RATE', 'STREAM')):
            kind, value = self.receive(deadline)
            if kind in ('SUB', 'RATE', 'STREAM'):
                original[kind] = value
        self.event('ORIGINAL_TELEMETRY', settings=original)
        self.expected = tuple(PROFILES[loop])
        self.schema = None  # Old binary frames are ignored until exact SUB acknowledgment.
        self.pending_original_schema = original['SUB'] if original['SUB'] != self.expected else None
        try:
            self.send(('SUB ' + ','.join(self.expected) + '\n').encode('ascii'))
            self.wait_ack('SUB', self.expected)
        finally:
            # Once acknowledged, timed out or failed, no old schema is allowed.
            self.pending_original_schema = None
        self.send(b'RATE 20\n')
        self.wait_ack('RATE', 20)
        self.send(b'STREAM 1\n')
        self.wait_ack('STREAM', 1)
        return original

    def stopped_guard(self):
        deadline = time.monotonic() + self.timeout
        count = 0
        first = first_rx = None
        while True:
            kind, row = self.receive(deadline)
            if kind != 'DATA':
                if kind == 'SUB':
                    count, first, first_rx = 0, None, None
                elif kind == 'SLIDER':
                    raise ProtocolError('unexpected parameter ACK before write; another sender may be active')
                continue
            if not is_stopped(row):
                raise ProtocolError('write blocked: board is not confirmed stopped with all outputs zero')
            if first is None:
                first = row['telemetry_ms']
                first_rx = self.last_rx
            count += 1
            if count >= 10 and row['telemetry_ms'] - first >= 200. and self.last_rx-first_rx >= .2:
                return row

    def write_one(self, loop, spec):
        name, value, readback, command = parameter(loop, spec)
        before = self.stopped_guard()
        if self.last_rx is None or time.monotonic() - self.last_rx > 0.15:
            raise ProtocolError('write blocked: stopped observation is stale')
        self.event('WRITE_PLAN', name=name, old_value=before[readback], requested=value,
                   readback=readback, note='One explicit RAM write; no retry or rollback')
        # From here any exception is an uncertain/partial transaction, never retry.
        try:
            self.send(command)
            deadline = time.monotonic() + self.timeout
            ack = False
            matches = 0
            while True:
                kind, row = self.receive(deadline)
                if kind == 'SLIDER':
                    if row[0] != name or not close_value(row[1], value):
                        raise ProtocolError('mismatched parameter acknowledgment')
                    ack = True
                    matches = 0
                elif kind == 'SUB':
                    raise ProtocolError('schema announcement during write; readback is uncertain')
                elif kind == 'DATA':
                    if not is_stopped(row):
                        raise ProtocolError('board state changed after write; inspect locally')
                    if ack:
                        matches = matches + 1 if close_value(row[readback], value) else 0
                        if matches >= 3:
                            result = {'status': 'ACK_AND_TELEMETRY_READBACK_CONFIRMED',
                                      'parameter': name, 'old_value': before[readback],
                                      'value': value, 'readback_field': readback,
                                      'storage': 'RAM only',
                                      'physical_motion_verified': False}
                            self.event('RESULT', **result)
                            return result
        except (Exception, KeyboardInterrupt) as exc:
            self.event('WRITE_UNCERTAIN', reason=str(exc), parameter=name,
                       note='Do not retry blindly. Inspect current RAM value and board state first.')
            raise UncertainWrite(str(exc)) from exc


def read_rows(path):
    path = Path(path)
    with path.open(encoding='utf-8-sig', newline='') as stream:
        if path.suffix.lower() == '.csv':
            records = list(csv.DictReader(stream))
        elif path.suffix.lower() in ('.jsonl', '.ndjson'):
            records = []
            for line in stream:
                if not line.strip():
                    continue
                item = json.loads(line)
                # Existing host export: {timestamp, address, values:{...}}.
                if item.get('direction') and not (item.get('direction') == 'RX' and item.get('kind') == 'DATA'):
                    continue
                record = item.get('values', item.get('value', item))
                if not isinstance(record, dict):
                    raise ValueError('measurement record must be an object')
                records.append(record)
        else:
            raise ValueError('only CSV or JSONL/NDJSON is supported; pickle is not loaded')
    rows = []
    for i, record in enumerate(records, 1):
        row = {}
        for key, val in record.items():
            if key not in FIELDS:
                continue
            if val in ('', None):
                continue
            try:
                row[key] = finite(val)
                if abs(row[key]) > 3.4028234663852886e38:
                    raise ValueError('outside float32 telemetry range')
            except (ValueError, TypeError, OverflowError) as exc:
                raise ValueError(f'record {i}, {key}: invalid numeric value') from exc
        if row:
            rows.append(row)
    if not rows:
        raise ValueError('no numeric telemetry rows')
    return rows


def stats(values):
    return {'samples': len(values), 'mean': statistics.fmean(values),
            'rms': math.sqrt(statistics.fmean(v*v for v in values)),
            'absolute_peak': max(abs(v) for v in values), 'min': min(values), 'max': max(values)}


def analyze(loop, rows):
    report = {'loop': loop, 'status': 'INCOMPLETE', 'total_rows': len(rows),
              'warnings': ['Offline observations only. No stability guarantee or automatic gain recommendation.'],
              'axis': 'ground wheel = displayed Pitch = internal imu_roll_deg / gyro.x'}
    required = {'telemetry_ms', 'control_mode', 'rate_state', 'imu_status',
                'startup_scale', 'fault_reason', 'pwm_debug_enabled', 'rate_enabled'}
    missing = sorted(required - set.intersection(*(set(r) for r in rows)))
    if missing:
        report['missing_required_fields'] = missing
        report['warnings'].append('Cannot validate mode/state for this export. Recapture with this tool; no assumed mode.')
        return report, []
    times = [r['telemetry_ms'] for r in rows]
    gaps = [b-a for a, b in zip(times, times[1:])]
    if any(dt <= 0 for dt in gaps):
        report['warnings'].append('Board time resets, duplicates or out-of-order frames: split into continuous sessions first.')
        return report, []
    report['timing'] = {'duration_s': (times[-1]-times[0])/1000,
                        'median_packet_ms': statistics.median(gaps) if gaps else None,
                        'max_packet_ms': max(gaps) if gaps else None}
    report['observed_modes'] = sorted(set(r['control_mode'] for r in rows))
    selected = [r for r in rows if r['control_mode'] in MODES[loop] and r['rate_state'] == 2
                and r['imu_status'] == 1 and .999 <= r['startup_scale'] <= 1. and r['rate_enabled'] == 1
                and r['fault_reason'] == 0 and r['pwm_debug_enabled'] == 0]
    report['selected_rows'] = len(selected)
    report['excluded_rows'] = len(rows)-len(selected)
    if not selected:
        report['warnings'].append('No running, fault-free, fully ramped samples of the required mode(s).')
        return report, []
    modes = set(r['control_mode'] for r in selected)
    if len(modes) != 1:
        report['warnings'].append('Multiple running control modes: split the recording by mode before comparison.')
        return report, []
    mode = next(iter(modes))
    report['selected_mode'] = mode
    if mode in (2, 5, 10):
        if any(r.get('dual_enabled') != 1 or r.get('rate_axis') != 2 for r in selected):
            report['warnings'].append('Dual-mode axis flags disagree; recapture with a verified schema.')
            return report, []
        report['warnings'].append('Coupled mode: flywheel also runs; this is not an isolated ground-wheel test.')
    signals = []
    if loop == 'rate':
        needs = {'rate_axis', 'rate_only', 'rate_target_dps', 'rate_actual_dps',
                 'rate_pwm', 'motor_ground_pwm', 'ground_pwm_limit'}
    elif loop == 'angle':
        prefix = 'dual_pitch_' if mode in (2, 5, 10) else 'balance_'
        needs = {'motor_ground_pwm', 'ground_pwm_limit', 'ground_angle_kp', 'ground_rate_limit_dps',
                 prefix+'actual_deg', prefix+'target_deg'}
        needs |= {'dual_pitch_target_rate_dps', 'dual_pitch_rate_dps'} if mode in (2, 5, 10) else {'rate_target_dps', 'rate_actual_dps', 'rate_axis', 'rate_only'}
    else:
        needs = set(SPEED) | {'motor_ground_pwm', 'ground_pwm_limit'}
    missing = sorted(needs - set.intersection(*(set(r) for r in selected)))
    if missing:
        report['missing_required_fields'] = missing
        return report, []
    if loop == 'rate' and any(r['rate_axis'] != 1 or r['rate_only'] != 1 for r in selected):
        report['warnings'].append('Mode7 axis/rate-only flags do not match; do not trust this schema.')
        return report, []
    if loop == 'angle' and mode == 4 and any(r['rate_axis'] != 1 or r['rate_only'] != 0 for r in selected):
        report['warnings'].append('Mode4 axis/angle flags do not match; do not trust this schema.')
        return report, []
    if loop == 'speed':
        good = [r for r in selected if r['wheel_feedback_valid'] == 1 and r['wheel_encoder_valid'] == 1
                and r['wheel_position_valid'] == 1 and r['wheel_hold_state'] == 2
                and 0 <= r['wheel_feedback_age_ms'] < 60 and 0 < r['wheel_encoder_dt_ms'] <= 60]
        report['invalid_encoder_rows'] = len(selected)-len(good)
        unique, last = [], None
        for r in good:
            if r['wheel_encoder_seq'] != last:
                unique.append(r)
                last = r['wheel_encoder_seq']
        selected = unique
        report['unique_encoder_windows'] = len(unique)
        if not selected:
            report['warnings'].append('No fresh, valid encoder windows.')
            return report, []
        report['position_loop_active_rows'] = sum(r['wheel_pos_kp'] > 0 and r['wheel_speed_limit'] > 0 for r in selected)
        report['warnings'].append('Speed units are counts/s; 4096 counts/wheel revolution. No assumed wheel diameter.')
        errors = [r['wheel_target_cps']-r['wheel_speed_cps'] for r in selected]
        report['speed_error_cps'] = stats(errors)
        report['speed_rpm'] = stats([r['wheel_speed_cps']*60/4096 for r in selected])
        report['tilt_limit_fraction'] = statistics.fmean(r['wheel_tilt_limit'] > 0 and abs(r['wheel_tilt_offset']) >= r['wheel_tilt_limit']-.0001 for r in selected)
        report['max_feedback_age_ms'] = max(r['wheel_feedback_age_ms'] for r in selected)
        signals = [('target counts/s', [r['wheel_target_cps'] for r in selected]),
                   ('actual counts/s', [r['wheel_speed_cps'] for r in selected])]
    elif loop == 'angle':
        errors = [r[prefix+'target_deg']-r[prefix+'actual_deg'] for r in selected]
        target_key = 'dual_pitch_target_rate_dps' if mode in (2, 5, 10) else 'rate_target_dps'
        actual_key = 'dual_pitch_rate_dps' if mode in (2, 5, 10) else 'rate_actual_dps'
        report['angle_error_deg'] = stats(errors)
        report['rate_error_dps'] = stats([r[target_key]-r[actual_key] for r in selected])
        expected = [max(-r['ground_rate_limit_dps'], min(r['ground_rate_limit_dps'], r['ground_angle_kp']*e)) for r,e in zip(selected, errors)]
        report['P_only_target_residual_dps'] = stats([r[target_key]-x for r,x in zip(selected, expected)])
        report['warnings'].append('P-only residual assumes the inspected firmware has zero ground angle Ki/Kd; it is not a plant model.')
        signals = [('target deg', [r[prefix+'target_deg'] for r in selected]),
                   ('actual deg', [r[prefix+'actual_deg'] for r in selected])]
    else:
        report['rate_error_dps'] = stats([r['rate_target_dps']-r['rate_actual_dps'] for r in selected])
        report['pid_pwm'] = stats([r['rate_pwm'] for r in selected])
        signals = [('target deg/s', [r['rate_target_dps'] for r in selected]),
                   ('actual deg/s', [r['rate_actual_dps'] for r in selected])]
    report['final_ground_pwm'] = stats([r['motor_ground_pwm'] for r in selected])
    report['final_pwm_limit_fraction'] = statistics.fmean(r['ground_pwm_limit'] > 0 and abs(r['motor_ground_pwm']) >= r['ground_pwm_limit']*r['startup_scale']-.5 for r in selected)
    report['warnings'].append('Final PWM includes direction-dependent friction compensation; PID PWM saturation flags alone miss it.')
    report['warnings'].append('Packet telemetry is slower than the 2 ms controller; unobserved peaks/oscillations remain possible.')
    report['status'] = 'OBSERVATIONS_ONLY'
    return report, ([r['telemetry_ms'] for r in selected], signals)


def write_svg(path, plot):
    if not plot:
        raise ValueError('no validated data to plot')
    times, signals = plot
    values = [v for _, ys in signals for v in ys]
    t0, t1 = min(times), max(times)
    lo, hi = min(values), max(values)
    if hi == lo:
        lo, hi = lo-1, hi+1
    padding = (hi-lo)*.05
    lo, hi = lo-padding, hi+padding
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="460" viewBox="0 0 1000 460">',
             '<rect width="1000" height="460" fill="white"/>',
             '<text x="70" y="28" font-family="sans-serif" font-size="18">Ground wheel recorded telemetry (offline)</text>',
             '<path d="M70 50V390H970" stroke="#444" fill="none"/>']
    for index, (name, ys) in enumerate(signals):
        points = ' '.join(f'{70+(t-t0)/max(1.,t1-t0)*900:.2f},{390-(y-lo)/(hi-lo)*330:.2f}' for t,y in zip(times,ys))
        color = ('#1370d8', '#d65314')[index % 2]
        parts += [f'<polyline fill="none" stroke="{color}" stroke-width="1.4" points="{points}"/>',
                  f'<text x="{70+index*260}" y="440" fill="{color}" font-family="sans-serif">{escape(name)}</text>']
    parts += [f'<text x="8" y="65" font-family="sans-serif">{hi:.3g}</text>',
              f'<text x="8" y="390" font-family="sans-serif">{lo:.3g}</text>',
              f'<text x="70" y="412" font-family="sans-serif">0 s</text>',
              f'<text x="880" y="412" font-family="sans-serif">{(t1-t0)/1000:.3g} s</text>', '</svg>']
    Path(path).write_text('\n'.join(parts), encoding='utf-8')


def main(loop):
    parser = argparse.ArgumentParser(description=f'{loop} loop: offline analysis / telemetry / explicit stopped-only RAM write')
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('parameters', help='show supported write names, bounds and readback fields')
    p = commands.add_parser('analyze', help='read exported CSV/JSONL; no network')
    p.add_argument('file', type=Path)
    p.add_argument('--output', type=Path)
    p.add_argument('--svg', type=Path)
    for name in ('capture', 'diagnose', 'send'):
        p = commands.add_parser(name)
        p.add_argument('--device-ip', help='explicit board IPv4; never auto-discovered')
        p.add_argument('--module-port', type=int, default=5001)
        p.add_argument('--listen-port', type=int, default=8081)
        p.add_argument('--bind', default='0.0.0.0')
        p.add_argument('--timeout', type=float, default=5.)
        p.add_argument('--output', type=Path, help='new JSONL path; refuses overwriting')
        p.add_argument('--seconds', type=float, default=10.)
        if name == 'send':
            p.add_argument('--set', required=True, metavar='NAME=VALUE', help='exactly one allowlisted parameter')
            p.add_argument('--apply', action='store_true', help='without this flag: offline preview only')
            p.add_argument('--confirm-board-stopped', action='store_true', help='operator checked physical Stop, exited all open-loop tests, isolated motor power, and closed other senders')
    args = parser.parse_args()
    session = None
    try:
        if args.command == 'parameters':
            print(json.dumps({'loop':loop, 'intended_modes':MODES[loop], 'writes_require_stopped_mode':0,
                              'parameters':PARAMETERS[loop]}, indent=2))
            return 0
        if args.command == 'analyze':
            report, plot = analyze(loop, read_rows(args.file))
            report['source_file'] = args.file.name
            result = json.dumps(report, ensure_ascii=False, indent=2)
            print(result)
            if args.output:
                with args.output.open('x', encoding='utf-8') as f:
                    f.write(result+'\n')
            if args.svg:
                if args.svg.exists():
                    raise ValueError('SVG output exists; choose a new filename')
                write_svg(args.svg, plot)
            return 0 if report['status'] == 'OBSERVATIONS_ONLY' else 2
        if args.command == 'send':
            name, value, readback, command = parameter(loop, args.set)
            print(json.dumps({'preview_only':not args.apply, 'command':command.decode().strip(),
                              'readback_field':readback, 'storage':'RAM', 'no_start_or_flash':True}, indent=2))
            if not args.apply:
                return 0
            if not args.confirm_board_stopped:
                raise ValueError('requires --confirm-board-stopped after physically stopping and isolating motor power')
        if not args.device_ip:
            raise ValueError('--device-ip is required for network operations')
        if not math.isfinite(args.timeout) or not .5 <= args.timeout <= 30:
            raise ValueError('timeout must be 0.5..30 seconds')
        if not math.isfinite(args.seconds) or not .2 <= args.seconds <= 3600:
            raise ValueError('seconds must be 0.2..3600')
        output = args.output or Path(f'{loop}_{args.command}_{time.time_ns()}.jsonl')
        session = Session(args.device_ip, args.module_port, args.bind, args.listen_port, output, args.timeout)
        original = session.configure(loop)
        print('Telemetry configured only. No motor Start/Stop. Original settings saved in log.')
        if args.command == 'send':
            print(json.dumps(session.write_one(loop, args.set), indent=2))
        else:
            end = time.monotonic()+args.seconds
            count = 0
            while time.monotonic() < end:
                try:
                    kind, row = session.receive(min(end, time.monotonic()+args.timeout))
                except TimeoutError:
                    if time.monotonic() < end-.05:
                        raise
                    break
                if kind == 'DATA':
                    count += 1
            print(json.dumps({'frames':count, 'log':str(output), 'motion_commands_sent':0,
                              'original_telemetry':original}, indent=2))
            if count == 0:
                return 2
        print('Telemetry selection is left active. Re-select your GUI profile before recording there.')
        return 0
    except UncertainWrite as exc:
        print('WRITE OUTCOME UNCERTAIN: '+str(exc)+'. Do not retry blindly; inspect board and current RAM values.', file=sys.stderr)
        return 3
    except (ValueError, OSError, TimeoutError, KeyError) as exc:
        print('ERROR: '+str(exc), file=sys.stderr)
        return 2
    finally:
        if session:
            session.close()


if __name__ == '__main__':
    raise SystemExit('Run rate_loop.py, angle_loop.py or speed_loop.py instead.')
