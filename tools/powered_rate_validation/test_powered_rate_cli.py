from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import contextlib
import io
import unittest
from unittest.mock import patch
import powered_rate_cli as p

PARAMS = dict(ack_powered_bench=True, polarity=1, kp=2., duration_ms=150, limit=50)
def status(**kw):
    s = dict.fromkeys(p.FIELDS, 0)
    s.update(version=1, profile=2, mode=4, duration_ms=150, limit=50, kp=2., session=10)
    if kw.get('active') == 1:
        s.update(state=1, reason=1)
    s.update(kw)
    return s

class Clock:
    def __init__(self):
        self.t = 0.
        self.jitter = 0.
    def __call__(self):
        return self.t
    def sleep(self, n):
        self.t += n + self.jitter

class FakeClient:
    def __init__(self, clock, failure=None):
        self.clock, self.failure, self.calls = clock, failure, []
    def request(self, op, *fields):
        self.calls.append((op, fields, self.clock()))
        if self.failure:
            result = self.failure(op, fields)
            if result is not None:
                return result
        if op == 'cbench_status':
            return status()
        if op == 'cbench_start':
            return status(active=1, session=11)
        if op == 'cbench_keep':
            return status(active=1, session=11, keep_seq=fields[1])
        return status(session=11, state=2, reason=2)

class FakeTransport:
    def __init__(self, clock, behavior=None):
        self.clock, self.behavior, self.sent = clock, behavior, []
    def send(self, data):
        self.sent.append(data)
    def receive(self, timeout):
        if self.behavior:
            return self.behavior(timeout)
        fields = self.sent[-1].decode().strip().split(',')
        return (','.join(['rsp:' + fields[1], 'ok', fields[0][4:]] + [str(status()[k]) for k in p.FIELDS]) + '\n').encode()

class BenchTests(unittest.TestCase):
    def run_fake(self, failure=None, jitter=0., **params):
        clock = Clock()
        clock.jitter = jitter
        client = FakeClient(clock, failure)
        trace = []
        try:
            p.run(client, **(PARAMS | params), clock=clock, sleep=clock.sleep, trace=trace)
        finally:
            self.client, self.trace = client, trace
        return client
    def assert_stop_once(self):
        self.assertEqual([c[0] for c in self.client.calls].count('cbench_stop'), 1)
        self.assertEqual(self.client.calls[-1][0], 'cbench_stop')
    def test_anchored_schedule(self):
        c = self.run_fake()
        self.assertEqual([round(t, 3) for op, _, t in c.calls if op == 'cbench_keep'], [.05, .1])
        self.assertEqual(c.calls[1][1], (10, 'powered_rate_pos', 0, '2', 0, 150, 50))
        self.assert_stop_once()
    def test_negative_polarity(self):
        def reply(op, fields):
            if op == 'cbench_start': return status(active=1, session=11, mode=5)
            if op == 'cbench_keep': return status(active=1, session=11, mode=5, keep_seq=fields[1])
            if op == 'cbench_stop': return status(session=11, mode=5, state=2, reason=2)
        self.run_fake(reply, polarity=-1)
        self.assertEqual(self.client.calls[1][1][1], 'powered_rate_neg')
    def test_invalid_inputs_never_send(self):
        for overrides in ({'kp': 0}, {'kp': 20.1}, {'kp': float('nan')}, {'kp': float('inf')},
                          {'duration_ms': 1001}, {'duration_ms': 0}, {'limit': 201}, {'limit': 0},
                          {'ack_powered_bench': False}, {'polarity': 0}, {'polarity': True}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                self.run_fake(**overrides)
            self.assertEqual(self.client.calls, [])
    def test_start_timeout_no_retry(self):
        def fail(op, fields):
            if op == 'cbench_start': raise p.ProtocolError('timeout')
        with self.assertRaises(p.ProtocolError): self.run_fake(fail)
        self.assert_stop_once()
        self.assertEqual(sum(op == 'cbench_start' for op, _, _ in self.client.calls), 1)
    def test_ctrl_c_stop(self):
        def fail(op, fields):
            if op == 'cbench_keep': raise KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt): self.run_fake(fail)
        self.assert_stop_once()
    def test_preflight_timeout_stop(self):
        def fail(op, fields):
            if op == 'cbench_status': raise p.ProtocolError('timeout')
        with self.assertRaises(p.ProtocolError): self.run_fake(fail)
        self.assert_stop_once()
    def test_wrong_start_session(self):
        def fail(op, fields):
            if op == 'cbench_start': return status(active=1, session=10)
        with self.assertRaises(p.ProtocolError): self.run_fake(fail)
        self.assert_stop_once()
    def test_wrong_start_configuration(self):
        for change in ({'profile': 1}, {'mode': 5}, {'ki': 1}, {'target': 1}, {'limit': 51}, {'kp': 3}):
            def fail(op, fields):
                if op == 'cbench_start': return status(active=1, session=11, **change)
            with self.subTest(change=change), self.assertRaises(p.ProtocolError): self.run_fake(fail)
            self.assert_stop_once()
    def test_wrong_keep_ack(self):
        def fail(op, fields):
            if op == 'cbench_keep': return status(active=1, session=11, keep_seq=999)
        with self.assertRaises(p.ProtocolError): self.run_fake(fail)
        self.assert_stop_once()
    def test_early_firmware_end(self):
        def fail(op, fields):
            if op == 'cbench_keep': return status(session=11, keep_seq=fields[1])
        with self.assertRaises(p.ProtocolError): self.run_fake(fail)
        self.assert_stop_once()
    def test_late_wakeup_no_keep(self):
        with self.assertRaises(p.ProtocolError): self.run_fake(jitter=.002)
        self.assertFalse(any(op == 'cbench_keep' for op, _, _ in self.client.calls))
        self.assert_stop_once()
    def test_stop_failure_preserves_primary(self):
        def fail(op, fields):
            if op == 'cbench_start': raise KeyboardInterrupt()
            if op == 'cbench_stop': raise p.ProtocolError('stop timeout')
        with self.assertRaises(KeyboardInterrupt): self.run_fake(fail)
        self.assert_stop_once()
        self.assertEqual(self.trace[-1][0], 'stop_unconfirmed')
    def test_nonzero_stop_rejected(self):
        def fail(op, fields):
            if op == 'cbench_stop': return status(applied=1)
        with self.assertRaises(p.ProtocolError): self.run_fake(fail)
        self.assert_stop_once()
    def test_preflight_profile_idle_session_guards(self):
        for change in ({'profile': 1}, {'active': 1}, {'session': 0xffffffff}):
            def fail(op, fields):
                if op == 'cbench_status': return status(**change)
            with self.subTest(change=change), self.assertRaises(p.ProtocolError): self.run_fake(fail)
            self.assertFalse(any(op == 'cbench_start' for op, _, _ in self.client.calls))
    def test_protocol_request(self):
        c = Clock(); t = FakeTransport(c); client = p.Client(t, clock=c, sequence=1)
        self.assertEqual(client.request('cbench_status')['profile'], 2)
        self.assertEqual(t.sent, [b'cfg:cbench_status,2\n'])
    def test_forbidden_command(self):
        c = Clock(); t = FakeTransport(c); client = p.Client(t, clock=c, sequence=1)
        with self.assertRaises(ValueError): client.request('motor_enable')
        self.assertEqual(t.sent, [])
    def test_ack_deadline(self):
        c = Clock(); t = FakeTransport(c)
        normal = t.receive
        def late(timeout):
            t.behavior = None
            data = normal(timeout)
            c.t += .041
            return data
        t.behavior = late
        client = p.Client(t, clock=c, sequence=1)
        with self.assertRaises(p.ProtocolError): client.request('cbench_start')
        self.assertEqual(len(t.sent), 1)
    def test_timeout_cap(self):
        for timeout in (.041, 0, float('nan')):
            with self.assertRaises(ValueError): p.Client(None, timeout=timeout)
    def test_status_rejects_nonfinite_or_output(self):
        for change in ({'kp': float('nan')}, {'applied': 501}, {'profile': 3}, {'session': -1}):
            s = status(**change)
            with self.assertRaises(p.ProtocolError): p.parse_status([str(s[k]) for k in p.FIELDS])
    def test_parser_requires_all_explicit_controls(self):
        with contextlib.redirect_stderr(io.StringIO()):
            for args in (['run'], ['run', '--ack-isolated-signal'],
                         ['run', '--ack-powered-bench', '--polarity', '1', '--kp', '2', '--duration-ms', '100']):
                with self.assertRaises(SystemExit): p.build_parser().parse_args(args)
    def test_verified_udp_peer(self):
        class Sock:
            def __init__(self):
                self.frames = [(b'wrong', ('192.168.0.112', 5001)),
                               (b'wrongport', ('192.168.0.111', 5002)),
                               (b'correct', ('192.168.0.111', 5001))]
            def settimeout(self, timeout): pass
            def recvfrom(self, size): return self.frames.pop(0)
        transport = p.UDPTransport.__new__(p.UDPTransport)
        transport.peer = ('192.168.0.111', 5001)
        transport.sock = Sock()
        self.assertEqual(transport.receive(.04), b'correct')
    def test_stale_ack_is_not_accepted(self):
        c = Clock(); t = FakeTransport(c)
        calls = [b'rsp:1,ok,cbench_status\n', b'']
        t.behavior = lambda timeout: calls.pop(0)
        client = p.Client(t, clock=c, sequence=1)
        with self.assertRaises(p.ProtocolError): client.request('cbench_status')
        self.assertEqual(len(t.sent), 1)
    def test_matching_wrong_operation_rejected(self):
        c = Clock(); t = FakeTransport(c)
        t.behavior = lambda timeout: b'rsp:2,ok,cbench_start\n'
        client = p.Client(t, clock=c, sequence=1)
        with self.assertRaises(p.ProtocolError): client.request('cbench_status')
    def test_long_ack_latency_does_not_shift_schedule(self):
        clock = Clock()
        def slow(op, fields):
            if op != 'cbench_status': clock.t += .039
        client = FakeClient(clock, slow)
        p.run(client, **PARAMS, clock=clock, sleep=clock.sleep)
        self.assertEqual([round(t, 3) for op, _, t in client.calls if op == 'cbench_keep'], [.05, .1])
    def test_no_socket_in_suite(self):
        with patch.object(p.socket, 'socket', side_effect=AssertionError('Network forbidden')):
            self.run_fake()

if __name__ == '__main__':
    unittest.main()
