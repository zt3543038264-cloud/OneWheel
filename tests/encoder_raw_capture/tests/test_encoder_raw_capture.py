import contextlib
import io
import json
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import encoder_raw_capture as m


def pack(fields, row):
    return struct.pack('<%df' % len(fields), *(row.get(k, 0.) for k in fields)) + m.TRAILER


def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


class MockBoard:
    """Synthetic loopback-only firmware; never contacts real hardware."""
    def __init__(self, active=False, wrong_schema=False, malformed=False, announcements=False, stale_after_announcement=False, metadata_only=False, changed_metadata=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.port = self.sock.getsockname()[1]
        self.sock.settimeout(.005)
        self.fields = ('pitch_deg', 'roll_deg')
        self.commands = []
        self.target = None
        self.streaming = False
        self.running = True
        self.active = active
        self.wrong_schema = wrong_schema
        self.malformed = malformed
        self.announcements = announcements
        self.stale_after_announcement = stale_after_announcement
        self.metadata_only = metadata_only
        self.changed_metadata = changed_metadata
        self.seq = 0
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, data):
        self.sock.sendto(data, self.target)

    def run(self):
        last = time.monotonic()
        while self.running:
            try:
                data, self.target = self.sock.recvfrom(8192)
            except socket.timeout:
                data = None
            except OSError:
                break
            if data:
                self.commands.append(data)
                if data == b'GET?\n':
                    self.send(('MCAR SUB ' + ','.join(self.fields) + '\n').encode())
                    self.send(b'MCAR RATE 50\n')
                    self.send(b'MCAR STREAM 0\n')
                elif data == m.SUB_COMMAND:
                    self.fields = tuple(reversed(m.FIELDS)) if self.wrong_schema else m.FIELDS
                    self.send(('MCAR SUB ' + ','.join(self.fields) + '\n').encode())
                elif data == b'RATE 20\n':
                    self.send(b'MCAR RATE 20\n')
                elif data == b'STREAM 1\n':
                    self.send(b'MCAR STREAM 1\n')
                    self.streaming = True
                else:
                    self.send(b'MCAR ERR forbidden\n')
            now = time.monotonic()
            if self.streaming and now - last >= .02:
                last = now
                self.seq += 1
                row = dict(telemetry_ms=100 + self.seq * 20,
                           wheel_encoder_raw=(65530 + self.seq * 3) % 65536,
                           wheel_encoder_delta=3 if self.seq <= 7 else -3,
                           wheel_encoder_seq=self.seq, wheel_encoder_dt_ms=20,
                           wheel_encoder_valid=1, wheel_feedback_valid=1,
                           wheel_encoder_sign=1, wheel_feedback_age_ms=2,
                           wheel_speed_cps=150 if self.seq <= 7 else -150,
                           control_mode=7 if self.active else 0)
                if self.announcements and self.seq % 3 == 0:
                    self.send(('MCAR SUB ' + ','.join(self.fields) + '\n').encode())
                    self.send(b'MCAR RATE 20\n')
                    self.send(b'MCAR STREAM 1\n')
                    if self.changed_metadata:
                        self.send(self.changed_metadata)
                    if self.stale_after_announcement:
                        row['telemetry_ms'] = 100
                if not self.metadata_only:
                    self.send(b'broken' if self.malformed else pack(self.fields, row))

    def close(self):
        self.running = False
        self.thread.join(timeout=1.)
        self.sock.close()


class CaptureTests(unittest.TestCase):
    def test_profile_contains_raw_delta_and_known_fields(self):
        self.assertIn('wheel_encoder_raw', m.FIELDS)
        self.assertIn('wheel_encoder_delta', m.FIELDS)
        self.assertEqual(len(m.FIELDS), len(set(m.FIELDS)))
        self.assertLessEqual(len(m.FIELDS), 48)
        self.assertTrue(set(m.FIELDS) <= m.KNOWN_FIELDS)

    def test_command_allowlist_is_only_telemetry(self):
        self.assertEqual(m.ALLOWED_COMMANDS, (b'GET?\n', m.SUB_COMMAND, b'RATE 20\n', b'STREAM 1\n'))
        obj = object.__new__(m.CaptureSession)
        for command in (b'START\n', b'STOP\n', b'[slider,rate_kp,5]\n', b'SAVE\n', b'SUB pitch_deg\n'):
            with self.assertRaises(m.ProtocolError):
                obj.send(command)

    def test_cli_requires_confirmation_before_socket(self):
        with patch.object(m.socket, 'socket', side_effect=AssertionError('socket must not open')):
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(m.main([]), 2)

    def test_nonfinite_duration_rejected_before_socket(self):
        with patch.object(m.socket, 'socket', side_effect=AssertionError('socket must not open')):
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(m.main(['--confirm-stopped-isolated', '--seconds', 'nan']), 2)

    def test_binary_roundtrip_preserves_raw_delta(self):
        kind, row = m.parse_packet(pack(m.FIELDS, {'wheel_encoder_raw': 4094, 'wheel_encoder_delta': -7}), m.FIELDS)
        self.assertEqual(kind, 'DATA')
        self.assertEqual(row['wheel_encoder_raw'], 4094)
        self.assertEqual(row['wheel_encoder_delta'], -7)

    def test_old_data_is_ignored_until_schema(self):
        self.assertEqual(m.parse_packet(pack(('pitch_deg',), {}), None), ('UNMAPPED', None))

    def test_malformed_packets_rejected(self):
        for packet in (b'broken', pack(m.FIELDS, {})[:-1], b'MCAR SUB not_a_field\n',
                       b'MCAR SUB telemetry_ms,telemetry_ms\n', b'MCAR RATE 1\n',
                       b'MCAR STREAM 2\n', b'MCAR SLIDER rate_kp 5\n',
                       b'MCAR RATE 20\nMCAR STREAM 1\n'):
            with self.subTest(packet=packet[:60]):
                with self.assertRaises(m.ProtocolError):
                    m.parse_packet(packet, m.FIELDS)

    def test_nonfinite_binary_rejected(self):
        with self.assertRaises(m.ProtocolError):
            m.parse_packet(pack(m.FIELDS, {'wheel_encoder_raw': float('nan')}), m.FIELDS)

    def test_existing_log_not_overwritten(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'existing.jsonl'
            path.write_text('keep', encoding='utf-8')
            with self.assertRaises(FileExistsError):
                m.CaptureSession('127.0.0.1', 5001, '127.0.0.1', free_udp_port(), path)
            self.assertEqual(path.read_text(), 'keep')

    def test_udp_capture_and_exact_transmitted_commands(self):
        board = MockBoard()
        try:
            with tempfile.TemporaryDirectory() as d:
                log = Path(d) / 'capture.jsonl'
                session = m.CaptureSession('127.0.0.1', board.port, '127.0.0.1', free_udp_port(), log, .5)
                try:
                    original = session.configure()
                    self.assertEqual(original, {'SUB': ('pitch_deg', 'roll_deg'), 'RATE': 50, 'STREAM': 0})
                    frames = session.capture(.35, progress=False)
                    self.assertGreaterEqual(frames, 8)
                finally:
                    session.close()
                events = [json.loads(s) for s in log.read_text().splitlines()]
                rows = [e['value'] for e in events if e.get('kind') == 'DATA' and 'wheel_encoder_raw' in e['value']]
                self.assertTrue(any(r['wheel_encoder_raw'] < 10 for r in rows))
                self.assertTrue(any(r['wheel_encoder_delta'] < 0 for r in rows))
                self.assertEqual(board.commands, list(m.ALLOWED_COMMANDS))
        finally:
            board.close()

    def test_live_status_failure_aborts_without_stop(self):
        board = MockBoard(active=True)
        try:
            with tempfile.TemporaryDirectory() as d:
                session = m.CaptureSession('127.0.0.1', board.port, '127.0.0.1', free_udp_port(), Path(d)/'x.jsonl', .5)
                try:
                    with self.assertRaisesRegex(m.ProtocolError, 'Stopped-status'):
                        session.configure()
                        session.capture(.25, progress=False)
                    self.assertEqual(board.commands, list(m.ALLOWED_COMMANDS))
                finally:
                    session.close()
        finally:
            board.close()

    def test_wrong_schema_aborts(self):
        board = MockBoard(wrong_schema=True)
        try:
            with tempfile.TemporaryDirectory() as d:
                session = m.CaptureSession('127.0.0.1', board.port, '127.0.0.1', free_udp_port(), Path(d)/'x.jsonl', .5)
                try:
                    with self.assertRaisesRegex(m.ProtocolError, 'Subscription changed'):
                        session.configure()
                    self.assertEqual(board.commands, [b'GET?\n', m.SUB_COMMAND])
                finally:
                    session.close()
        finally:
            board.close()

    def test_live_malformed_frame_aborts(self):
        board = MockBoard(malformed=True)
        try:
            with tempfile.TemporaryDirectory() as d:
                session = m.CaptureSession('127.0.0.1', board.port, '127.0.0.1', free_udp_port(), Path(d)/'x.jsonl', .5)
                try:
                    with self.assertRaisesRegex(m.ProtocolError, 'length/trailer'):
                        session.configure()
                        session.capture(.25, progress=False)
                finally:
                    session.close()
        finally:
            board.close()

    def run_announcement_case(self, **kwargs):
        board = MockBoard(announcements=True, **kwargs)
        try:
            with tempfile.TemporaryDirectory() as d:
                session = m.CaptureSession('127.0.0.1', board.port, '127.0.0.1', free_udp_port(), Path(d)/'x.jsonl', .12)
                try:
                    session.configure()
                    frames = session.capture(.3, progress=False)
                    self.assertGreaterEqual(frames, 8)
                    self.assertEqual(board.commands, list(m.ALLOWED_COMMANDS))
                finally:
                    session.close()
        finally:
            board.close()

    def test_periodic_same_sub_rate_stream_announcements_are_allowed(self):
        self.run_announcement_case()

    def test_repeated_sub_does_not_reset_clock_or_accept_stale_data(self):
        with self.assertRaisesRegex(m.ProtocolError, 'Non-increasing board time'):
            self.run_announcement_case(stale_after_announcement=True)

    def test_metadata_cannot_keep_capture_alive_without_data(self):
        with self.assertRaises(TimeoutError):
            self.run_announcement_case(metadata_only=True)

    def test_changed_rate_during_announcement_aborts(self):
        with self.assertRaisesRegex(m.ProtocolError, 'settings changed'):
            self.run_announcement_case(changed_metadata=b'MCAR RATE 50\n')

    def test_stream_off_during_announcement_aborts(self):
        with self.assertRaisesRegex(m.ProtocolError, 'settings changed'):
            self.run_announcement_case(changed_metadata=b'MCAR STREAM 0\n')

    def test_reordered_same_length_schema_after_data_aborts(self):
        wrong = ('MCAR SUB ' + ','.join(reversed(m.FIELDS)) + '\n').encode()
        with self.assertRaisesRegex(m.ProtocolError, 'Subscription changed'):
            self.run_announcement_case(changed_metadata=wrong)


if __name__ == '__main__':
    unittest.main()
