import csv
import json
import math
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
import three_loop as m


def stopped(schema):
    row = dict.fromkeys(schema, 0.)
    row.update(telemetry_ms=100., imu_status=1., ground_rate_kp=5.3,
               ground_pwm_limit=6000., ground_angle_kp=10., ground_rate_limit_dps=100.)
    return {k:row.get(k,0.) for k in schema}


def active(loop):
    row = stopped(m.PROFILES[loop])
    row.update(control_mode=m.MODES[loop][0], rate_state=2., rate_enabled=1., rate_axis=1.,
               startup_scale=1., rate_only=float(loop=='rate'),
               rate_target_dps=0., rate_actual_dps=2., rate_pwm=-11.,
               balance_target_deg=0., balance_actual_deg=.2,
               wheel_feedback_valid=1., wheel_encoder_valid=1., wheel_position_valid=1.,
               wheel_hold_state=2., wheel_encoder_dt_ms=20., wheel_feedback_age_ms=4.,
               wheel_encoder_seq=1., wheel_target_cps=0., wheel_speed_cps=50.,
               wheel_tilt_limit=1., wheel_speed_limit=300., wheel_pos_kp=.5)
    if loop == 'speed':
        row['dual_enabled']=1.;row['rate_axis']=2.
    return row


def packed(schema, row):
    return struct.pack('<%df' % len(schema), *(row[k] for k in schema))+m.TRAILER


class FakeSession(m.Session):
    def __init__(self, events):
        self.events = iter(events)
        self.last_rx = time.monotonic()
        self.timeout = .1
        self.sent = []
        self.logged = []
    def event(self, direction, **kwargs):
        self.logged.append((direction, kwargs))
    def send(self, data):
        self.sent.append(data)
    def receive(self, deadline):
        try:
            item = next(self.events)
        except StopIteration:
            raise TimeoutError('test timeout')
        if isinstance(item, Exception):
            raise item
        self.last_rx += .021
        return item
    def stopped_guard(self):
        return stopped(m.PROFILES['rate'])


class LocalFakeBoard:
    """Real UDP loopback transport, deterministic simulated firmware, never hardware."""
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1',0))
        self.port = self.sock.getsockname()[1]
        self.sock.settimeout(.005)
        self.schema = ('telemetry_ms',)
        self.target = None
        self.rate = 20
        self.stream = 1
        self.t = 100.
        self.running = True
        self.commands = []
        self.values = {'ground_rate_kp':5.3}
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
    def response(self, text):
        self.sock.sendto(text.encode('ascii'),self.target)
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
                cmd = data.decode().strip()
                self.commands.append(cmd)
                if cmd == 'GET?':
                    self.response('MCAR SUB '+','.join(self.schema)+'\n')
                    self.response('MCAR RATE 20\n')
                    self.response('MCAR STREAM 1\n')
                elif cmd.startswith('SUB '):
                    self.schema = tuple(cmd[4:].split(','))
                    self.response('MCAR SUB '+','.join(self.schema)+'\n')
                elif cmd == 'RATE 20':
                    self.response('MCAR RATE 20\n')
                elif cmd == 'STREAM 1':
                    self.response('MCAR STREAM 1\n')
                elif cmd.startswith('[slider,'):
                    _, name, val = cmd.strip('[]').split(',')
                    key = m.PARAMETERS['rate'][name][2]
                    self.values[key] = float(val)
                    self.response(f'MCAR SLIDER {name} {val}\n')
                else:
                    self.response('MCAR ERR unknown\n')
            now = time.monotonic()
            if self.target and now-last >= .02:
                last = now
                self.t += 20.
                row = stopped(self.schema)
                row.update(self.values)
                row['telemetry_ms']=self.t
                self.sock.sendto(packed(self.schema,row),self.target)
    def close(self):
        self.running = False
        self.thread.join(timeout=1)
        self.sock.close()


class ParameterTests(unittest.TestCase):
    def test_profiles_exact(self):
        for fields in m.PROFILES.values():
            self.assertLessEqual(len(fields),48)
            self.assertEqual(len(fields),len(set(fields)))
            self.assertFalse(set(fields)-m.FIELDS)
    def test_rate_readback_is_ground_not_generic(self):
        self.assertEqual(m.parameter('rate','rate_kp=5.3')[2],'ground_rate_kp')
    def test_limits(self):
        for loop, params in m.PARAMETERS.items():
            for name, (lo,hi,_) in params.items():
                for value in (lo,hi):
                    m.parameter(loop,f'{name}={value}')
                for value in (lo-1,hi+1,'nan','inf','-inf'):
                    with self.assertRaises(ValueError):
                        m.parameter(loop,f'{name}={value}')
    def test_disallowed(self):
        for name in ('debug_ground_pwm','fly_test_left','rate_sign','balance_angle_x','debug_step_run','wheel_pos_kp','START'):
            with self.assertRaises(ValueError):
                m.parameter('rate',name+'=1')
    def test_wrong_loop(self):
        with self.assertRaises(ValueError):
            m.parameter('angle','rate_kp=1')
    def test_preview_has_no_socket(self):
        with patch.object(sys,'argv',['rate_loop.py','send','--set','rate_kp=5.3']), patch.object(socket,'socket', side_effect=AssertionError('network forbidden')):
            self.assertEqual(m.main('rate'),0)


class ProtocolTests(unittest.TestCase):
    def test_binary_roundtrip(self):
        schema=tuple(m.PROFILES['rate'])
        kind,row=m.parse_packet(packed(schema,stopped(schema)),schema)
        self.assertEqual(kind,'DATA')
        self.assertAlmostEqual(row['ground_rate_kp'],5.3,5)
    def test_bad_frames(self):
        schema=('telemetry_ms',)
        for data in (b'bad',struct.pack('<f',float('nan'))+m.TRAILER,b'\0'*8):
            with self.assertRaises(m.ProtocolError):
                m.parse_packet(data,schema)
    def test_schema_rejects_duplicate_unknown_overflow(self):
        for text in ('MCAR SUB telemetry_ms,telemetry_ms\n','MCAR SUB no_such_field\n','MCAR SUB '+','.join(['telemetry_ms']*49)+'\n'):
            with self.assertRaises(m.ProtocolError):
                m.parse_packet(text.encode(),None)
    def test_err(self):
        with self.assertRaises(m.ProtocolError):
            m.parse_packet(b'MCAR ERR parameter out of range\n',None)
    def test_guard_rejects_missing_and_running(self):
        row=stopped(m.PROFILES['rate'])
        self.assertTrue(m.is_stopped(row))
        for key in m.STOP_ZERO:
            bad=row.copy();bad[key]=1.
            self.assertFalse(m.is_stopped(bad))
            bad=row.copy();del bad[key]
            self.assertFalse(m.is_stopped(bad))
    def test_actual_guard_needs_time_and_ten_frames(self):
        rows=[]
        for i in range(11):
            row=stopped(m.PROFILES['rate']);row['telemetry_ms']=100+i*20
            rows.append(('DATA',row))
        f=FakeSession(rows)
        out=m.Session.stopped_guard(f)
        self.assertEqual(out['telemetry_ms'],300)
        self.assertEqual(f.sent,[])
    def test_active_guard_no_write(self):
        row=stopped(m.PROFILES['rate']);row['control_mode']=7
        f=FakeSession([('DATA',row)])
        with self.assertRaises(m.ProtocolError):
            m.Session.stopped_guard(f)
        self.assertEqual(f.sent,[])


class WriteTests(unittest.TestCase):
    def match(self):
        row=stopped(m.PROFILES['rate']);row['ground_rate_kp']=6.
        return ('DATA',row)
    def test_ack_and_three_post_ack_frames(self):
        f=FakeSession([('SLIDER',('rate_kp',6.)),self.match(),self.match(),self.match()])
        self.assertEqual(f.write_one('rate','rate_kp=6')['status'],'ACK_AND_TELEMETRY_READBACK_CONFIRMED')
        self.assertEqual(f.sent,[b'[slider,rate_kp,6]\n'])
    def test_pre_ack_data_cannot_confirm(self):
        f=FakeSession([self.match(),self.match(),self.match(),('SLIDER',('rate_kp',6.))])
        with self.assertRaises(m.UncertainWrite):
            f.write_one('rate','rate_kp=6')
        self.assertEqual(len(f.sent),1)
    def test_no_ack_but_changed_ram_uncertain(self):
        f=FakeSession([self.match()]*5)
        with self.assertRaises(m.UncertainWrite):
            f.write_one('rate','rate_kp=6')
        self.assertEqual(len(f.sent),1)
    def test_distinct_float32_values_not_accepted(self):
        self.assertFalse(m.close_value(0.,1e-9))
        self.assertFalse(m.close_value(500.,499.9995))
        self.assertTrue(m.close_value(5.3,5.30000019))
        self.assertFalse(m.close_value(float('inf'),1.))
        f=FakeSession([('SLIDER',('rate_kp',6.000000476837158))])
        with self.assertRaises(m.UncertainWrite):
            f.write_one('rate','rate_kp=6')
        self.assertEqual(len(f.sent),1)

    def test_mismatched_ack_uncertain_once(self):
        for ack in [('wrong',6.),('rate_kp',9.)]:
            f=FakeSession([('SLIDER',ack)])
            with self.assertRaises(m.UncertainWrite):
                f.write_one('rate','rate_kp=6')
            self.assertEqual(len(f.sent),1)
    def test_schema_or_err_abort_no_retry(self):
        for event in [('SUB',tuple(m.PROFILES['rate'])),m.ProtocolError('firmware rejected')]:
            f=FakeSession([event])
            with self.assertRaises(m.UncertainWrite):
                f.write_one('rate','rate_kp=6')
            self.assertEqual(len(f.sent),1)
    def test_active_after_write_unknown(self):
        row=self.match()[1];row['rate_state']=2.
        f=FakeSession([('SLIDER',('rate_kp',6.)),('DATA',row)])
        with self.assertRaises(m.UncertainWrite):
            f.write_one('rate','rate_kp=6')
        self.assertEqual(len(f.sent),1)
    def test_wrong_generic_readback_not_accepted(self):
        row=stopped(m.PROFILES['rate']);row['rate_kp']=6.
        f=FakeSession([('SLIDER',('rate_kp',6.))]+[('DATA',row)]*3)
        with self.assertRaises(m.UncertainWrite):
            f.write_one('rate','rate_kp=6')


class AnalysisTests(unittest.TestCase):
    def rows(self,loop):
        rows=[]
        for i in range(20):
            row=active(loop);row['telemetry_ms']=i*20.;row['wheel_encoder_seq']=i//2+1
            rows.append(row)
        return rows
    def test_three_loops(self):
        for loop in m.PROFILES:
            report,plot=m.analyze(loop,self.rows(loop))
            self.assertEqual(report['status'],'OBSERVATIONS_ONLY')
            self.assertTrue(plot)
    def test_speed_dedup(self):
        report,_=m.analyze('speed',self.rows('speed'))
        self.assertEqual(report['unique_encoder_windows'],10)
    def test_legacy_missing_mode_rejected(self):
        rows=self.rows('angle')
        for r in rows:del r['control_mode']
        report,_=m.analyze('angle',rows)
        self.assertEqual(report['status'],'INCOMPLETE')
    def test_fault_and_ramp_excluded(self):
        rows=self.rows('rate');rows[0]['fault_reason']=3;rows[1]['startup_scale']=.5
        report,_=m.analyze('rate',rows)
        self.assertEqual(report['selected_rows'],18)
    def test_restart_rejected(self):
        rows=self.rows('rate');rows[5]['telemetry_ms']=0
        report,_=m.analyze('rate',rows)
        self.assertEqual(report['status'],'INCOMPLETE')
    def test_encoder_invalid(self):
        rows=self.rows('speed')
        for r in rows:r['wheel_feedback_age_ms']=60
        report,_=m.analyze('speed',rows)
        self.assertEqual(report['status'],'INCOMPLETE')
    def test_jsonl_host_export_and_event_log(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'data.jsonl'
            p.write_text(json.dumps({'timestamp':'x','values':active('rate')})+'\n',encoding='utf-8')
            self.assertEqual(len(m.read_rows(p)),1)
            p.write_text(json.dumps({'direction':'TX','text':'GET?\n'})+'\n'+json.dumps({'direction':'RX','kind':'DATA','value':active('rate')})+'\n',encoding='utf-8')
            self.assertEqual(len(m.read_rows(p)),1)
    def test_pickle_forbidden(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'data.pkl';p.write_bytes(b'not loaded')
            with self.assertRaises(ValueError):m.read_rows(p)
    def test_invalid_numeric_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'data.jsonl'
            for value in ('NaN', 1e308, True):
                p.write_text(json.dumps({'values':{'telemetry_ms':value}})+'\n')
                with self.assertRaises(ValueError):m.read_rows(p)


class ReceiveTests(unittest.TestCase):
    def harness(self, packets):
        class Sock:
            def recvfrom(self, n):
                if packets:
                    return packets.pop(0)
                raise socket.timeout()
        obj=object.__new__(m.Session)
        obj.socket=Sock();obj.peer=('127.0.0.1',5001)
        obj.schema=tuple(m.PROFILES['rate']);obj.expected=obj.schema
        obj.pending_original_schema=None
        obj.last_time=None;obj.last_rx=None
        obj.event=lambda *args,**kwargs:None
        return obj
    def test_wrong_endpoint_ignored(self):
        schema=tuple(m.PROFILES['rate']);data=packed(schema,stopped(schema))
        f=self.harness([(data,('127.0.0.2',5001)),(data,('127.0.0.1',5001))])
        self.assertEqual(f.receive(time.monotonic()+.01)[0],'DATA')
    def test_same_length_reordered_schema_rejected(self):
        fields=list(m.PROFILES['rate']);fields[0],fields[1]=fields[1],fields[0]
        f=self.harness([(('MCAR SUB '+','.join(fields)+'\n').encode(),('127.0.0.1',5001))])
        with self.assertRaises(m.ProtocolError):f.receive(time.monotonic()+.01)
        self.assertIsNone(f.schema)
    def test_default_seven_channel_schema_rejected(self):
        f=self.harness([(b'MCAR SUB dual_roll_actual_deg,dual_roll_target_deg,dual_roll_target_rate_dps,dual_roll_rate_dps,dual_roll_pwm,motor_left_pwm,motor_right_pwm\n',('127.0.0.1',5001))])
        with self.assertRaises(m.ProtocolError):f.receive(time.monotonic()+.01)
    def test_original_mode10_without_clock_during_get_handshake(self):
        original=('dual_roll_actual_deg','dual_roll_target_deg','dual_roll_target_rate_dps',
                  'dual_roll_rate_dps','dual_roll_pwm','motor_left_pwm','motor_right_pwm')
        peer=('127.0.0.1',5001)
        target=tuple(m.PROFILES['rate'])
        packets=[(('MCAR SUB '+','.join(original)+'\n').encode(),peer),
                 (packed(original,dict.fromkeys(original,0.)),peer),
                 (b'MCAR RATE 20\n',peer),(b'MCAR STREAM 1\n',peer),
                 (('MCAR SUB '+','.join(target)+'\n').encode(),peer),
                 (b'MCAR RATE 20\n',peer),(b'MCAR STREAM 1\n',peer)]
        f=self.harness(packets);f.schema=None;f.expected=None;f.timeout=.1
        commands=[];f.send=commands.append
        saved=f.configure('rate')
        self.assertEqual(saved['SUB'],original)
        self.assertEqual(f.schema,target)
        self.assertEqual(commands[0],b'GET?\n')
        self.assertFalse(any(command.startswith(b'[slider') for command in commands))

    def test_duplicate_and_reversed_time_rejected(self):
        for old in (100.,101.):
            schema=tuple(m.PROFILES['rate']);row=stopped(schema)
            f=self.harness([(packed(schema,row),('127.0.0.1',5001))]);f.last_time=old
            with self.assertRaises(m.ProtocolError):f.receive(time.monotonic()+.01)
    def test_burst_old_frames_fail_host_duration_guard(self):
        schema=tuple(m.PROFILES['rate']);packets=[]
        for i in range(11):
            row=stopped(schema);row['telemetry_ms']=100+i*20
            packets.append((packed(schema,row),('127.0.0.1',5001)))
        f=self.harness(packets);f.timeout=.02
        with self.assertRaises(TimeoutError):f.stopped_guard()
    def test_open_loop_claim_not_part_of_protocol(self):
        self.assertNotIn('wheel_test_pwm',m.FIELDS)
        self.assertNotIn('fly_test_left',m.FIELDS)



class NetworkTests(unittest.TestCase):
    def test_loopback_full_transaction(self):
        board=LocalFakeBoard()
        session=None
        try:
            probe=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            probe.bind(('127.0.0.1',0));port=probe.getsockname()[1];probe.close()
            with tempfile.TemporaryDirectory() as td:
                session=m.Session('127.0.0.1',board.port,'127.0.0.1',port,Path(td)/'events.jsonl',1.)
                session.configure('rate')
                out=session.write_one('rate','rate_kp=6')
                self.assertEqual(out['status'],'ACK_AND_TELEMETRY_READBACK_CONFIRMED')
                self.assertEqual(sum(c.startswith('[slider') for c in board.commands),1)
                self.assertFalse(any(c.startswith(('START','STOP','DTRACE')) or 'debug_' in c for c in board.commands if c.startswith('[slider')))
                session.close();session=None
        finally:
            if session:session.close()
            board.close()


if __name__=='__main__':unittest.main()
