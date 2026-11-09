"""Offline metadata-event replay and simulated subscription transition tests.

The fixture preserves the failed capture's events. UNMAPPED datagram bytes
were not recorded by v1.0, so replay uses synthetic binary placeholders.
An optional successful ACK tail is explicitly synthetic, not observed.
"""
import json
from pathlib import Path
import time
import unittest

import test_three_loop as base

m = base.m
packed = base.packed

PEER = ('127.0.0.1', 5001)
FIXTURE = Path(__file__).parent / 'fixtures' / 'pre_ack_old_sub.jsonl'


def sub(fields):
    return ('MCAR SUB ' + ','.join(fields) + '\n').encode('ascii')


class SubscriptionHandshakeTests(unittest.TestCase):
    def make_session(self, packets, target='angle'):
        session = base.ReceiveTests().harness(list(packets))
        session.schema = session.expected = None
        session.pending_original_schema = None
        session.timeout = .02
        session.sent = []
        session.logged = []
        session.send = session.sent.append
        session.event = lambda direction, **kw: session.logged.append({'direction': direction, **kw})
        return session

    def replay(self):
        events = [json.loads(line) for line in FIXTURE.read_text(encoding='utf-8').splitlines()]
        original = tuple(next(e['value'] for e in events if e.get('kind') == 'SUB'))
        packets = []
        for event in events:
            if event['direction'] != 'RX':
                continue
            kind, value = event['kind'], event['value']
            if kind == 'UNMAPPED':
                data = packed(original, dict.fromkeys(original, 0.))
            elif kind == 'SUB':
                data = sub(value)
            else:
                data = f'MCAR {kind} {value}\n'.encode('ascii')
            packets.append((data, PEER))
        return events, original, packets

    def initial(self, original):
        return [(sub(original), PEER), (b'MCAR RATE 20\n', PEER), (b'MCAR STREAM 1\n', PEER)]

    def tail(self, target):
        return [(sub(target), PEER), (b'MCAR RATE 20\n', PEER), (b'MCAR STREAM 1\n', PEER)]

    def test_failed_log_alone_does_not_claim_configuration_success(self):
        events, original, packets = self.replay()
        self.assertEqual(len(events), 12)
        self.assertEqual(len(original), 39)
        self.assertEqual(sum(e.get('kind') == 'DATA' for e in events), 0)
        session = self.make_session(packets)
        with self.assertRaises(TimeoutError):
            session.configure('angle')
        self.assertIsNone(session.schema)
        self.assertIsNone(session.pending_original_schema)
        self.assertEqual(len(session.sent), 2)
        self.assertEqual(sum(e['direction'] == 'OLD_SUB_BEFORE_ACK' for e in session.logged), 1)
        self.assertFalse(any(e.get('kind') == 'DATA' for e in session.logged))

    def test_failed_log_plus_explicit_synthetic_ack_tail_succeeds(self):
        events, original, packets = self.replay()
        target = tuple(m.PROFILES['angle'])
        self.assertEqual(len(target), 37)
        session = self.make_session(packets + self.tail(target))
        result = session.configure('angle')
        self.assertEqual(result['SUB'], original)
        self.assertEqual(session.schema, target)
        self.assertIsNone(session.pending_original_schema)
        self.assertEqual(session.sent, [b'GET?\n', b'SUB ' + ','.join(target).encode() + b'\n', b'RATE 20\n', b'STREAM 1\n'])
        self.assertFalse(any(e.get('kind') == 'DATA' for e in session.logged))

    def test_same_length_old_binary_remains_unmapped_until_exact_ack(self):
        target = tuple(m.PROFILES['angle'])
        original = tuple(reversed(target))
        old_data = packed(original, dict.fromkeys(original, 0.))
        packets = self.initial(original) + [(sub(original), PEER), (old_data, PEER)] + self.tail(target)
        session = self.make_session(packets)
        session.configure('angle')
        self.assertEqual(session.schema, target)
        self.assertEqual(sum(e.get('kind') == 'UNMAPPED' for e in session.logged), 1)
        self.assertFalse(any(e.get('kind') == 'DATA' for e in session.logged))

    def test_unrecognized_third_schema_before_ack_still_aborts(self):
        original = ('telemetry_ms',)
        session = self.make_session(self.initial(original) + [(sub(('imu_seq',)), PEER)])
        with self.assertRaisesRegex(m.ProtocolError, 'subscription changed'):
            session.configure('angle')
        self.assertIsNone(session.schema)
        self.assertIsNone(session.pending_original_schema)

    def test_reordered_original_before_ack_still_aborts(self):
        original = ('telemetry_ms', 'imu_seq')
        session = self.make_session(self.initial(original) + [(sub(tuple(reversed(original))), PEER)])
        with self.assertRaisesRegex(m.ProtocolError, 'subscription changed'):
            session.configure('angle')

    def test_old_schema_after_exact_ack_aborts_even_during_rate_ack(self):
        target = tuple(m.PROFILES['angle'])
        original = ('telemetry_ms',)
        session = self.make_session(self.initial(original) + [(sub(target), PEER), (sub(original), PEER)])
        with self.assertRaisesRegex(m.ProtocolError, 'subscription changed'):
            session.configure('angle')
        self.assertEqual(session.sent[-1], b'RATE 20\n')
        self.assertIsNone(session.schema)

    def test_changed_schema_during_capture_still_aborts(self):
        target = tuple(m.PROFILES['angle'])
        original = tuple(reversed(target))
        session = self.make_session(self.initial(original) + self.tail(target) + [(sub(original), PEER)])
        session.configure('angle')
        with self.assertRaisesRegex(m.ProtocolError, 'subscription changed'):
            session.receive(time.monotonic() + .02)

    def test_identical_target_announcement_is_accepted(self):
        target = tuple(m.PROFILES['angle'])
        session = self.make_session(self.initial(('telemetry_ms',)) + self.tail(target) + [(sub(target), PEER)])
        session.configure('angle')
        self.assertEqual(session.receive(time.monotonic() + .02), ('SUB', target))

    def test_repeated_known_old_metadata_does_not_trigger_retransmit(self):
        original = ('telemetry_ms',)
        session = self.make_session(self.initial(original) + [(sub(original), PEER)] * 20)
        start = time.monotonic()
        with self.assertRaises(TimeoutError):
            session.configure('angle')
        self.assertLess(time.monotonic() - start, .3)
        self.assertEqual(len(session.sent), 2)
        self.assertIsNone(session.pending_original_schema)

    def test_no_old_exception_when_original_already_equals_target(self):
        target = tuple(m.PROFILES['angle'])
        wrong = tuple(reversed(target))
        session = self.make_session(self.initial(target) + [(sub(wrong), PEER)])
        with self.assertRaisesRegex(m.ProtocolError, 'subscription changed'):
            session.configure('angle')


if __name__ == '__main__':
    unittest.main()
