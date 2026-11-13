"""Offline deterministic protocol tests. No socket, firmware, or motor access."""
import unittest
from preview_core import Client, ProtocolError, Cancelled, PreviewSession, parse_preview, validate_cbench, FIRMWARE_ID

IDLE = ['1','0','0','0','0','0','0','0','0','0']
BENCH = ['1','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0']

class Clock:
    def __init__(self): self.now=1.0
    def __call__(self): return self.now

class Cancel:
    def __init__(self, clock, after=None): self.clock=clock; self.after=after; self.flag=False
    def is_set(self): return self.flag or (self.after is not None and self.clock() >= self.after)
    def wait(self, timeout): self.clock.now += timeout; return self.is_set()

class Peer:
    def __init__(self, clock, *, delays=None, drop=None, overrides=None):
        self.clock=clock; self.writes=[]; self.pending=[]; self.closed=False
        self.delays=delays or {}; self.drop=set(drop or []); self.overrides=overrides or {}
        self.active=False; self.started=0; self.last_elapsed=0
    def send(self, wire):
        self.writes.append(wire)
        operation,seq,*fields=wire.decode().strip().removeprefix('cfg:').split(',')
        if operation=='hello': payload=['1',FIRMWARE_ID,'63']
        elif operation=='cbench_status': payload=BENCH[:]
        elif operation=='preview_start':
            self.active=True; self.started=self.clock(); payload=['1','ZERO_OUTPUT']
        elif operation=='preview_stop': self.active=False; payload=[]
        elif operation=='preview_status':
            elapsed=min(10000, int(round((self.clock()-self.started)*1000))) if self.started else 0
            self.active=self.active and elapsed<10000
            payload=['1',str(int(self.active)),str(int(bool(self.started))),'0', '0' if self.active else '16' if self.started else '0',str(elapsed), '1.25','-2.5','3.75' if self.active else '0','0']
        else: raise AssertionError(operation)
        override=self.overrides.get(operation)
        if override is not None: payload=override(payload) if callable(override) else override
        if operation not in self.drop:
            line=('rsp:'+seq+',ok,'+operation+('' if not payload else ','+','.join(payload))+'\n').encode()
            self.pending.append((self.clock()+self.delays.get(operation,0),line))
    def receive(self, timeout):
        if self.pending and self.pending[0][0] <= self.clock()+timeout:
            at,line=self.pending.pop(0); self.clock.now=max(self.clock(),at); return line
        self.clock.now+=timeout; return b''
    def close(self): self.closed=True

class Tests(unittest.TestCase):
    def run_case(self, **kwargs):
        clock=Clock(); peer=Peer(clock,**kwargs); events=[]
        session=PreviewSession(emit=lambda e,d:events.append((e,d)),clock=clock,transport_factory=lambda *a:peer)
        result=session.run(Cancel(clock))
        return peer,events,result
    def ops(self,peer): return [w.decode().split(',')[0][4:] for w in peer.writes]
    def test_happy_path_and_whitelist(self):
        p,e,r=self.run_case(); ops=self.ops(p)
        self.assertTrue(r['confirmed']); self.assertTrue(p.closed)
        self.assertEqual(ops.count('preview_start'),1); self.assertEqual(ops.count('preview_stop'),1)
        self.assertEqual(set(ops),{'hello','cbench_status','preview_status','preview_start','preview_stop'})
        samples=[d for k,d in e if k=='sample']; self.assertGreater(len(samples),80)
        self.assertNotIn('applied',samples[0]); self.assertEqual(samples[0]['axis'],1)
    def test_450ms_observation_latency_does_not_require_keep(self):
        p,e,r=self.run_case(delays={'preview_status':.45})
        self.assertTrue(r['confirmed']); self.assertTrue(any(d['gap'] for k,d in e if k=='sample'))
        self.assertNotIn('cbench_keep',self.ops(p))
    def test_lost_start_ack_stops_once_no_restart(self):
        p,e,r=self.run_case(drop={'preview_start'})
        self.assertTrue(r['start_attempted']); self.assertTrue(r['confirmed'])
        self.assertIn('timed out',r['message']); self.assertEqual(self.ops(p).count('preview_start'),1)
        self.assertEqual(self.ops(p).count('preview_stop'),1)
    def test_stop_ack_loss_unconfirmed(self):
        p,e,r=self.run_case(drop={'preview_stop'})
        self.assertFalse(r['confirmed']); self.assertIn('STOP ACK UNCONFIRMED',r['message']); self.assertIn('fresh status reports inactive',r['message'])
        self.assertEqual(self.ops(p).count('preview_stop'),1)
    def test_no_start_wrong_identity(self):
        p,e,r=self.run_case(overrides={'hello':['1','wrong','63']})
        self.assertFalse(r['start_attempted']); self.assertNotIn('preview_start',self.ops(p))
        self.assertNotIn('preview_stop',self.ops(p))
    def test_no_start_signal_profile(self):
        v=BENCH[:]; v[-1]='1'
        p,e,r=self.run_case(overrides={'cbench_status':v})
        self.assertFalse(r['start_attempted']); self.assertNotIn('preview_start',self.ops(p))
    def test_no_start_busy_preview(self):
        v=IDLE[:]; v[1]='1'
        p,e,r=self.run_case(overrides={'preview_status':v})
        self.assertFalse(r['start_attempted'])
    def test_cancel_before_network_send(self):
        clock=Clock(); peer=Peer(clock); cancel=Cancel(clock); cancel.flag=True
        r=PreviewSession(clock=clock,transport_factory=lambda *a:peer).run(cancel)
        self.assertFalse(r['start_attempted']); self.assertEqual(peer.writes,[]); self.assertTrue(peer.closed)
    def test_cancel_running_sends_stop(self):
        clock=Clock(); peer=Peer(clock); cancel=Cancel(clock,after=1.35)
        r=PreviewSession(clock=clock,transport_factory=lambda *a:peer).run(cancel)
        self.assertTrue(r['confirmed']); self.assertEqual(self.ops(peer).count('preview_stop'),1)
    def test_cancel_while_waiting_start_ack_stops(self):
        clock=Clock(); peer=Peer(clock,delays={'preview_start':.5}); cancel=Cancel(clock,after=1.1)
        r=PreviewSession(clock=clock,transport_factory=lambda *a:peer).run(cancel)
        self.assertTrue(r['start_attempted']); self.assertTrue(r['confirmed'])
    def test_reject_bad_preview_values(self):
        for index,value in [(0,'2'),(1,'2'),(2,'2'),(3,'2'),(4,'20'),(5,'10001'),(6,'nan'),(7,'inf'),(8,'1'),(9,'2')]:
            with self.subTest(index=index):
                v=IDLE[:];v[index]=value
                with self.assertRaises(ProtocolError): parse_preview(v)
    def test_reject_bad_cbench_values(self):
        for index,value in [(0,'2'),(1,'1'),(3,'4'),(4,'13'),(5,'20'),(6,'-1'),(12,'501'),(17,'nan'),(20,'1'),(21,'1')]:
            with self.subTest(index=index):
                v=BENCH[:];v[index]=value
                with self.assertRaises(ProtocolError): validate_cbench(v)
    def test_operations_blocked_before_wire(self):
        clock=Clock(); p=Peer(clock); c=Client(p,clock=clock)
        for op,args in [('cbench_start',()),('cbench_keep',()),('set',()),('preview_start',('roll','rate')),('preview_start',('pitch','angle')),('preview_stop',('extra',))]:
            with self.assertRaises(ValueError):c.request(op,*args)
        self.assertEqual(p.writes,[])
    def test_sequence_no_wrap(self):
        p=Peer(Clock()); c=Client(p,sequence=65534); c.sequence=65535
        with self.assertRaises(ProtocolError):c.request('hello')
        self.assertEqual(p.writes,[])
    def test_old_wrong_and_multiline_replies_ignored(self):
        clock=Clock(); p=Peer(clock); c=Client(p,clock=clock,sequence=10)
        p.pending=[(clock(),b'rsp:9,ok,hello,1,wrong,63\n'),(clock(),b'rsp:11,ok,hello,1,wrong,63\natt:0,0,0\n')]
        self.assertEqual(c.request('hello'),['1',FIRMWARE_ID,'63'])
    def test_wrong_operation_matching_sequence_fails(self):
        clock=Clock(); p=Peer(clock); c=Client(p,clock=clock,sequence=10)
        p.pending=[(clock(),b'rsp:11,ok,preview_stop\n')]
        with self.assertRaises(ProtocolError):c.request('hello')
    def test_stale_mcu_elapsed_fails_closed(self):
        def freeze(v):
            if v[1]=='1':v[5]='0'
            return v
        p,e,r=self.run_case(overrides={'preview_status':freeze})
        self.assertIn('not advancing',r['message']); self.assertTrue(r['confirmed'])
        self.assertEqual(self.ops(p).count('preview_start'),1)
    def test_axis_switch_stops(self):
        def switch(v):
            if v[1]=='1':v[2]='0'
            return v
        p,e,r=self.run_case(overrides={'preview_status':switch})
        self.assertIn('axis/ring mismatch',r['message']); self.assertTrue(r['confirmed'])
    def test_error_reply_fails(self):
        clock=Clock();p=Peer(clock);c=Client(p,clock=clock,sequence=10)
        p.pending=[(clock(),b'rsp:11,err,BLOCKED,12\n')]
        with self.assertRaisesRegex(ProtocolError,'MCU rejected'):c.request('hello')

if __name__=='__main__':unittest.main(verbosity=2)
