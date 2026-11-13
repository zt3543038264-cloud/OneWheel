from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest
import contextlib
import io
from unittest.mock import patch
import powered_rate_cli as p
from test_powered_rate_cli import Clock, FakeTransport, status, PARAMS

def ack(seq=2, operation='cbench_status', **changes):
    s = status(**changes)
    return (','.join(['rsp:' + str(seq), 'ok', operation] + [str(s[k]) for k in p.FIELDS]) + '\n').encode()

class StreamTests(unittest.TestCase):
    def client(self, chunks):
        clock = Clock()
        transport = FakeTransport(clock)
        chunks = iter(chunks)
        transport.behavior = lambda timeout: next(chunks, b'')
        return p.Client(transport, clock=clock, sequence=1)
    def test_every_split_boundary(self):
        line = ack()
        for i in range(1, len(line)):
            c = self.client([line[:i], line[i:]])
            self.assertEqual(c.request('cbench_status')['profile'], 2)
    def test_single_byte_fragments_bounded(self):
        c = self.client([bytes([b]) for b in ack()])
        with self.assertRaisesRegex(p.ProtocolError, 'budget'): c.request('cbench_status')
    def test_multiline_telemetry_and_ack(self):
        c = self.client([b'telemetry,123\n' + ack(1) + ack() + b'tail\n'])
        self.assertEqual(c.request('cbench_status')['session'], 10)
    def test_partial_old_sequence_survives_request(self):
        old = ack()
        c = self.client([old[:35], b'', old[35:] + ack(3)])
        with self.assertRaises(p.ProtocolError): c.request('cbench_status')
        self.assertEqual(bytes(c.framer.partial), old[:35])
        self.assertEqual(c.request('cbench_status')['profile'], 2)
    def test_queued_future_ack_not_used(self):
        c = self.client([ack() + ack(3), b''])
        c.request('cbench_status')
        with self.assertRaises(p.ProtocolError): c.request('cbench_status')
    def test_overlong_suffix_never_resynchronized(self):
        c = self.client([b'x' * 512, ack(), b''])
        with self.assertRaises(p.ProtocolError): c.request('cbench_status')
        self.assertEqual(len(c.framer.partial), 0)
    def test_overlong_line_then_real_boundary(self):
        c = self.client([b'x' * 512, ack() + ack()])
        self.assertEqual(c.request('cbench_status')['profile'], 2)
    def test_embedded_ack_prefix_not_accepted(self):
        c = self.client([b'garbage' + ack()])
        with self.assertRaises(p.ProtocolError): c.request('cbench_status')
    def test_no_newline_memory_bounded(self):
        c = self.client([b'x' * 4096] * 10)
        with self.assertRaisesRegex(p.ProtocolError, 'budget'): c.request('cbench_status')
        self.assertLessEqual(len(c.framer.partial), 511)
        self.assertLessEqual(len(c.framer.lines), 64)
    def test_queue_flood_bounded(self):
        c = self.client([b'\n' * 65 + ack()])
        with self.assertRaisesRegex(p.ProtocolError, 'queue'): c.request('cbench_status')
        self.assertLessEqual(len(c.framer.lines), 64)
    def test_line_budget_bounded(self):
        c = self.client([b'x\n' * 64] * 4)
        with self.assertRaisesRegex(p.ProtocolError, 'line processing'): c.request('cbench_status')
    def test_datagram_flood_bounded(self):
        c = self.client([b'x'] * 65)
        with self.assertRaisesRegex(p.ProtocolError, 'datagram processing'): c.request('cbench_status')
    def test_oversized_datagram_no_truncated_suffix(self):
        c = self.client([b'x' * 4097, ack(3), ack(3)])
        with self.assertRaisesRegex(p.ProtocolError, 'Oversized'): c.request('cbench_status')
        self.assertEqual(c.request('cbench_status')['profile'], 2)
    def test_crlf(self):
        c = self.client([ack()[:-1] + b'\r', b'\n'])
        self.assertEqual(c.request('cbench_status')['profile'], 2)
    def test_fake_peer_cannot_pollute_partial(self):
        line = ack()
        class Sock:
            def __init__(self):
                self.frames = [(line[:30], ('192.168.0.111', 5001)),
                               (b'poison\n' + ack(), ('192.168.0.112', 5001)),
                               (line[30:], ('192.168.0.111', 5001))]
            def settimeout(self, timeout): pass
            def recvfrom(self, size): return self.frames.pop(0)
            def sendto(self, data, peer): return len(data)
        transport = p.UDPTransport.__new__(p.UDPTransport)
        transport.peer = ('192.168.0.111', 5001)
        transport.sock = Sock()
        c = p.Client(transport, sequence=1)
        self.assertEqual(c.request('cbench_status')['profile'], 2)
    def test_unverified_peer_flood_bounded(self):
        class Sock:
            calls = 0
            def settimeout(self, timeout): pass
            def recvfrom(self, size):
                self.calls += 1
                return b'poison', ('192.168.0.112', 5001)
        t = p.UDPTransport.__new__(p.UDPTransport)
        t.peer = ('192.168.0.111', 5001); t.sock = Sock()
        with patch.object(p.time, 'monotonic', return_value=0):
            self.assertEqual(t.receive(.04), b'')
        self.assertEqual(t.sock.calls, 64)
    def test_invalid_integer_ranges(self):
        for key, value in [('heartbeat_age_ms', -1), ('imu_age_ms', 0x100000000),
                           ('elapsed_ms', -1), ('duration_ms', -1), ('limit', -1),
                           ('encoder_20ms', 0x80000000), ('encoder_20ms', -0x80000001),
                           ('applied', 1)]:
            with self.subTest(key=key, value=value):
                c = self.client([ack(**{key: value})])
                with self.assertRaises(p.ProtocolError): c.request('cbench_status')
    def test_stop_ack_active_rejected(self):
        c = self.client([ack(operation='cbench_stop', active=1, state=1)])
        with self.assertRaises(p.ProtocolError): c.request('cbench_stop')
    def test_status_state_and_sensor_bounds(self):
        for change in ({'sensor_reason': -1}, {'sensor_reason': 20},
                       {'active': 1, 'state': 0}, {'active': 0, 'state': 1},
                       {'active': 1, 'state': 1, 'reason': 7}):
            with self.subTest(change=change):
                c = self.client([ack(**change)])
                with self.assertRaises(p.ProtocolError): c.request('cbench_status')
    def final_trial(self, final, duration=40):
        c = self.client([ack(), ack(3, 'cbench_start', active=1, session=11,
                                   duration_ms=duration),
                         ack(4, 'cbench_stop', session=11, duration_ms=duration,
                             state=2, reason=2, **final)])
        trace = []
        try:
            p.run(c, **(PARAMS | {'duration_ms': duration}), clock=c.clock,
                  sleep=c.clock.sleep, trace=trace)
        finally:
            self.last_client, self.last_trace = c, trace
    def test_short_no_keep_trial_normal_stop(self):
        self.final_trial({})
        self.assertEqual(len(self.last_client.transport.sent), 3)
        self.assertEqual(self.last_trace[-1][0], 'stop')
    def test_short_no_keep_trial_fault(self):
        c = self.client([ack(), ack(3, 'cbench_start', active=1, session=11, duration_ms=40),
                         ack(4, 'cbench_stop', session=11, duration_ms=40, state=3, reason=7)])
        trace = []
        with self.assertRaisesRegex(p.ProtocolError, 'Software output zero confirmed'):
            p.run(c, **(PARAMS | {'duration_ms': 40}), clock=c.clock, sleep=c.clock.sleep, trace=trace)
        self.assertEqual([event for event, _ in trace][-2:], ['stop', 'trial_failed'])
        self.assertEqual(sum(b'cfg:cbench_stop,' in x for x in c.transport.sent), 1)
        self.assertFalse(any(b'cbench_keep' in x for x in c.transport.sent))
    def test_final_session_change(self):
        c = self.client([ack(), ack(3, 'cbench_start', active=1, session=11, duration_ms=40),
                         ack(4, 'cbench_stop', session=12, duration_ms=40, state=2, reason=2)])
        trace = []
        with self.assertRaises(p.ProtocolError):
            p.run(c, **(PARAMS | {'duration_ms': 40}), clock=c.clock, sleep=c.clock.sleep, trace=trace)
        self.assertEqual(trace[-1][0], 'trial_failed')
        self.assertNotIn('stop_unconfirmed', [event for event, _ in trace])
    def test_final_configuration_change(self):
        with self.assertRaises(p.ProtocolError): self.final_trial({'kp': 3})
        self.assertEqual(self.last_trace[-1][0], 'trial_failed')
    def test_final_expiration_success(self):
        c = self.client([ack(), ack(3, 'cbench_start', active=1, session=11, duration_ms=40),
                         ack(4, 'cbench_stop', session=11, duration_ms=40, state=2, reason=3)])
        trace = p.run(c, **(PARAMS | {'duration_ms': 40}), clock=c.clock, sleep=c.clock.sleep)
        self.assertEqual(trace[-1][0], 'stop')
    def test_cli_fault_returns_nonzero_without_uncertain_stop(self):
        clock = Clock()
        transport = FakeTransport(clock)
        transport.close = lambda: None
        def reply(timeout):
            parts = transport.sent[-1].decode().strip().split(',')
            op, seq = parts[0][4:], int(parts[1])
            if op == 'cbench_status': return ack(seq, op)
            if op == 'cbench_start':
                return ack(seq, op, active=1, session=11, duration_ms=1)
            return ack(seq, op, session=11, duration_ms=1, state=3, reason=7)
        transport.behavior = reply
        out = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(io.StringIO()):
            code = p.main(['run', '--ack-powered-bench', '--polarity', '1',
                           '--kp', '2', '--duration-ms', '1', '--limit', '50'],
                          transport_factory=lambda *args: transport)
        self.assertEqual(code, 2)
        self.assertIn('trial_failed', out.getvalue())
        self.assertNotIn('stop_unconfirmed', out.getvalue())
        self.assertEqual(sum(b'cfg:cbench_stop,' in x for x in transport.sent), 1)
    def test_final_stop_unknown_is_distinct(self):
        c = self.client([ack(), ack(3, 'cbench_start', active=1, session=11, duration_ms=40), b''])
        trace = []
        with self.assertRaises(p.ProtocolError):
            p.run(c, **(PARAMS | {'duration_ms': 40}), clock=c.clock, sleep=c.clock.sleep, trace=trace)
        self.assertEqual(trace[-1][0], 'stop_unconfirmed')
    def test_bad_stream_during_run_stops_once(self):
        c = self.client([ack(), b'x\n' * 65, ack(4, 'cbench_stop')])
        clock = c.clock
        with self.assertRaises(p.ProtocolError):
            p.run(c, **PARAMS, clock=clock, sleep=clock.sleep)
        sent = c.transport.sent
        self.assertEqual(len(sent), 3)
        self.assertIn(b'cfg:cbench_start,3,', sent[1])
        self.assertEqual(sent[2], b'cfg:cbench_stop,4\n')

if __name__ == '__main__': unittest.main()
