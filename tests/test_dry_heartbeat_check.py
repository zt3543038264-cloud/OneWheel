import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import dry_heartbeat_check as d

class Clock:
    t=0
    late=False
    def now(self): return self.t
    def sleep(self,n): self.t+=n+(.002 if self.late else 0)
class Fake:
    def __init__(self, fail=None, modify=None):
        self.calls=[]; self.fail=fail; self.modify=modify
        self.s=dict(profile=2,active=0,applied=0,session=0,angle_deg=0,mode=0,
                    duration_ms=0,limit=0,target=0,kp=0,ki=0,calculated=0,keep_seq=0,state=0,reason=0)
    def request(self,op,*args):
        self.calls.append((op,args))
        if op==self.fail: raise d.ProtocolError('simulated timeout')
        if op=='cbench_start': self.s.update(active=1,state=1,reason=1,session=1,duration_ms=1000,limit=1)
        if op=='cbench_keep': self.s['keep_seq']=args[1]
        if op=='cbench_stop': self.s.update(active=0,state=2,reason=2,applied=0)
        s=dict(self.s)
        if self.modify: self.modify(op,s)
        return s
class Tests(unittest.TestCase):
    def run_check(self,f,c=None):
        c=c or Clock(); trace=[]
        d.check(f,trace,clock=c.now,sleep=c.sleep)
        return trace
    def test_success(self):
        f=Fake(); trace=self.run_check(f)
        self.assertEqual(len([x for x in f.calls if x[0]=='cbench_keep']),19)
        self.assertEqual(f.calls[1],('cbench_start',(0,'dry_raw',0,0,0,1000,1)))
        self.assertEqual(f.calls[-1][0],'cbench_stop')
        self.assertTrue(all(v['applied']==0 for _,v in trace))
    def test_wrong_profile_no_start(self):
        f=Fake(modify=lambda op,s:s.update(profile=0))
        with self.assertRaises(d.ProtocolError):self.run_check(f)
        self.assertEqual(len(f.calls),1)
    def test_tilt_no_start(self):
        f=Fake(modify=lambda op,s:s.update(angle_deg=42))
        with self.assertRaises(d.ProtocolError):self.run_check(f)
        self.assertEqual(len(f.calls),1)
    def test_busy_no_start(self):
        f=Fake(modify=lambda op,s:s.update(active=1))
        with self.assertRaises(d.ProtocolError):self.run_check(f)
        self.assertEqual(len(f.calls),1)
    def test_start_timeout_stop_once(self):
        f=Fake(fail='cbench_start')
        with self.assertRaises(d.ProtocolError):self.run_check(f)
        self.assertEqual([x[0] for x in f.calls],['cbench_status','cbench_start','cbench_stop'])
    def test_keep_timeout_stop_once(self):
        f=Fake(fail='cbench_keep')
        with self.assertRaises(d.ProtocolError):self.run_check(f)
        self.assertEqual([x[0] for x in f.calls],['cbench_status','cbench_start','cbench_keep','cbench_stop'])
    def test_nonzero_keep_aborts(self):
        f=Fake(modify=lambda op,s:s.update(applied=1) if op=='cbench_keep' else None)
        with self.assertRaises(d.ProtocolError):self.run_check(f)
        self.assertEqual(f.calls[-1][0],'cbench_stop')
    def test_wrong_session_aborts(self):
        f=Fake(modify=lambda op,s:s.update(session=999) if op=='cbench_start' else None)
        with self.assertRaises(d.ProtocolError):self.run_check(f)
        self.assertEqual(len(f.calls),3)
    def test_fault_at_stop_not_complete(self):
        f=Fake(modify=lambda op,s:s.update(state=3,reason=4) if op=='cbench_stop' else None)
        with self.assertRaises(d.ProtocolError):self.run_check(f)
    def test_stop_timeout_not_complete(self):
        f=Fake(fail='cbench_stop')
        with self.assertRaises(d.ProtocolError):self.run_check(f)
    def test_late_no_catchup(self):
        f=Fake(); c=Clock(); c.late=True
        with self.assertRaises(d.ProtocolError):self.run_check(f,c)
        self.assertFalse(any(op=='cbench_keep' for op,args in f.calls))
        self.assertEqual(f.calls[-1][0],'cbench_stop')
if __name__=='__main__':unittest.main()
