import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import graded_wheel_pulse as p
from powered_rate_cli import parse_status,FIELDS,ProtocolError
class Clock:
    t=0
    late=False
    def now(self):return self.t
    def sleep(self,n):self.t+=n+(.002 if self.late else 0)
class Fake:
    def __init__(self,fail=None,modify=None):
        self.calls=[];self.fail=fail;self.modify=modify
        self.s=dict(profile=2,active=0,applied=0,session=0,angle_deg=0,rate_dps=0,mode=0,duration_ms=0,limit=0,target=0,kp=0,ki=0,calculated=0,keep_seq=0,state=0,reason=0)
    def request(self,op,*args):
        self.calls.append((op,args))
        if op==self.fail:raise ProtocolError('simulated timeout')
        if op=='cbench_start':self.s.update(mode=6,session=1,active=1,state=1,reason=1,target=args[2],limit=args[-1],duration_ms=400)
        if op=='cbench_keep':self.s['keep_seq']=args[1]
        if op=='cbench_stop':self.s.update(active=0,state=2,reason=2,applied=0,calculated=0)
        s=dict(self.s)
        if self.modify:self.modify(op,s)
        return s
class Tests(unittest.TestCase):
    def check(self,f,level=1,direction=1,c=None):
        c=c or Clock();trace=[];p.run(f,level=level,direction=direction,trace=trace,clock=c.now,sleep=c.sleep);return trace
    def test_all_levels_directions(self):
        for l in range(1,7):
            for direction in (-1,1):
                f=Fake();trace=self.check(f,l,direction)
                self.assertEqual(f.calls[1],('cbench_start',(0,'powered_pulse',direction*l*200,0,0,400,l*200)))
                self.assertEqual(sum(x[0]=='cbench_keep' for x in f.calls),7)
                self.assertEqual(sum(x[0]=='cbench_stop' for x in f.calls),1)
    def test_bad_levels_no_send(self):
        for l in (0,7,1.5,True):
            f=Fake()
            with self.assertRaises(ValueError):self.check(f,l)
            self.assertEqual(f.calls,[])
    def test_bad_direction_no_send(self):
        f=Fake()
        with self.assertRaises(ValueError):self.check(f,direction=0)
        self.assertEqual(f.calls,[])
    def test_preflight_blocked(self):
        for key,value in [('profile',0),('active',1),('applied',1),('angle_deg',15),('rate_dps',5),('session',0xffffffff)]:
            f=Fake(modify=lambda op,s:s.update({key:value}))
            with self.assertRaises(ProtocolError):self.check(f)
            self.assertEqual(len(f.calls),1)
    def test_timeout_start_keep_stop(self):
        for op in ('cbench_start','cbench_keep','cbench_stop'):
            f=Fake(fail=op)
            with self.assertRaises(ProtocolError):self.check(f)
            self.assertEqual(sum(x[0]=='cbench_stop' for x in f.calls),1)
    def test_wrong_start_and_keep(self):
        for op in ('cbench_start','cbench_keep'):
            for key,value in [('session',99),('mode',4),('profile',1),('target',0),('applied',-1),('calculated',-1),('limit',201)]:
                f=Fake(modify=lambda x,s:s.update({key:value}) if x==op else None)
                with self.assertRaises(ProtocolError):self.check(f)
                self.assertEqual(f.calls[-1][0],'cbench_stop')
    def test_fault_stop(self):
        f=Fake(modify=lambda op,s:s.update(state=3,reason=4) if op=='cbench_stop' else None)
        with self.assertRaises(ProtocolError):self.check(f)
    def test_lateness(self):
        c=Clock();c.late=True;f=Fake()
        with self.assertRaises(ProtocolError):self.check(f,c=c)
        self.assertFalse(any(x[0]=='cbench_keep' for x in f.calls))
    def test_mode6_parser_bounds(self):
        s={k:0 for k in FIELDS};s.update(version=1,profile=2,mode=6,session=1,state=2,reason=2,duration_ms=400,target=1200,limit=1200)
        self.assertEqual(parse_status([str(s[k]) for k in FIELDS])['limit'],1200)
        for key,value in [('mode',4),('limit',1201),('target',100),('duration_ms',401),('kp',1),('applied',1201),('profile',1)]:
            v=dict(s);v[key]=value
            with self.assertRaises(ProtocolError):parse_status([str(v[k]) for k in FIELDS])
if __name__=='__main__':unittest.main()
