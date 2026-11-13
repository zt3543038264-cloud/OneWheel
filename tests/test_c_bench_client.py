"""Deterministic tests of the real patched client; no network or vehicle access."""
import heapq
import importlib.util
from pathlib import Path
import unittest
import sys
import tempfile
from unittest.mock import patch

PATH=Path(__file__).parent/'tools/c_bench_cli.py'
if not PATH.exists(): PATH=Path(__file__).resolve().parents[1]/'tools/c_bench_cli.py'
sys.path.insert(0,str(PATH.parent))
spec=importlib.util.spec_from_file_location('bench_under_test',PATH)
cli=importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)

class Clock:
    def __init__(self): self.now=10.0
    def __call__(self): return self.now
    def sleep(self,seconds): self.now+=seconds

class Transport:
    def __init__(self,clock,delays=None,profile=0):
        self.clock=clock;self.delays=delays or {};self.sent=[];self.inbox=[];self.closed=False
        self.status=dict.fromkeys(cli.STATUS_FIELDS,0)
        self.status.update(version=1,profile=profile)
        self.receive_stall=0
    def send(self,wire):
        fields=wire.decode().strip().split(',')
        op=fields[0][4:];seq=int(fields[1]);args=fields[2:]
        self.sent.append((self.clock(),op,seq,args))
        s=self.status
        if op=='cbench_start':
            s.update(active=1,mode=cli.MODES[args[1]],state=1,reason=1,session=s['session']+1,
                     keep_seq=0,target=float(args[2]),kp=float(args[3]),ki=float(args[4]),
                     duration_ms=int(args[5]),limit=int(args[6]))
        elif op=='cbench_keep':
            s['keep_seq']=int(args[1])
        elif op=='cbench_stop': s.update(active=0,applied=0,state=2,reason=2)
        keep=s['keep_seq'] if op=='cbench_keep' else 0
        delay=self.delays.get((op,keep),.002)
        if delay is None: return
        values=','.join(str(s[key]) for key in cli.STATUS_FIELDS)
        raw=f'rsp:{seq},ok,{op},{values}\n'.encode()
        heapq.heappush(self.inbox,(self.clock()+delay,seq,raw))
    def receive(self,timeout):
        end=self.clock()+timeout
        if self.receive_stall:
            self.clock.now+=self.receive_stall;self.receive_stall=0;return b''
        if self.inbox and self.inbox[0][0]<=end:
            at,_,raw=heapq.heappop(self.inbox)
            self.clock.now=max(self.clock.now,at)
            return raw
        self.clock.now=end
        return b''
    def close(self): self.closed=True

class Tests(unittest.TestCase):
    def setup_run(self,delays=None,profile=0):
        clock=Clock();transport=Transport(clock,delays,profile)
        client=cli.Client(transport,clock=clock,sequence=100)
        return clock,transport,client
    def run_test(self,clock,client,emit=lambda *args:None,profile='calculation-only'):
        cli.run(client,profile=profile,mode='rate',target=0,kp=1,ki=0,duration_ms=1000,
                limit=100,isolated=True,clock=clock,sleep=clock.sleep,emit=emit)
    def check_once(self,transport):
        ops=[op for _,op,_,_ in transport.sent]
        self.assertEqual(ops.count('cbench_start'),1)
        self.assertEqual(ops.count('cbench_stop'),1)
        sequences=[seq for _,_,seq,_ in transport.sent]
        self.assertEqual(len(sequences),len(set(sequences)))
        keeps=[int(args[1]) for _,op,_,args in transport.sent if op=='cbench_keep']
        self.assertEqual(keeps,list(range(1,len(keeps)+1)))
    def test_normal_run(self):
        c,t,client=self.setup_run();events=[]
        self.run_test(c,client,lambda e,s:events.append((e,dict(s))))
        self.check_once(t)
        self.assertGreaterEqual(sum(e=='keep' for e,_ in events),18)
        self.assertEqual(events[-1][0],'stopped')
    def test_90ms_ack_does_not_block_next_50ms_send(self):
        c,t,client=self.setup_run({('cbench_keep',i):.09 for i in range(1,21)})
        self.run_test(c,client);self.check_once(t)
        times=[at for at,op,_,_ in t.sent if op=='cbench_keep']
        self.assertGreaterEqual(len(times),19)
        for a,b in zip(times,times[1:]): self.assertAlmostEqual(b-a,.05,places=7)
    def test_reordered_ack_correlated_to_own_sequence(self):
        c,t,client=self.setup_run({('cbench_keep',1):.11,('cbench_keep',2):.002})
        keeps=[];self.run_test(c,client,lambda e,s:keeps.append(s['keep_seq']) if e=='keep' else None)
        self.assertEqual(keeps[:2],[2,3]);self.assertNotIn(1,keeps);self.check_once(t)
    def test_no_ack_stops_before_200ms(self):
        c,t,client=self.setup_run({('cbench_keep',i):None for i in range(1,21)})
        with self.assertRaisesRegex(cli.ProtocolError,'ACK timed out'): self.run_test(c,client)
        first=next(at for at,op,_,_ in t.sent if op=='cbench_keep')
        stopped=next(at for at,op,_,_ in t.sent if op=='cbench_stop')
        self.assertLess(stopped-first,.2);self.check_once(t)
    def test_fresh_valid_newer_ack_supersedes_lost_old_ack(self):
        c,t,client=self.setup_run({('cbench_keep',1):None})
        keeps=[];self.run_test(c,client,lambda e,s:keeps.append(s['keep_seq']) if e=='keep' else None)
        self.assertEqual(keeps[:2],[2,3]);self.check_once(t)
    def test_all_ack_lost_after_initial_progress_still_stops(self):
        c,t,client=self.setup_run({('cbench_keep',i):None for i in range(2,21)})
        with self.assertRaisesRegex(cli.ProtocolError,'#2 ACK timed out'):self.run_test(c,client)
        self.check_once(t)
    def test_late_ack_is_rejected(self):
        c,t,client=self.setup_run({('cbench_keep',i):.121 for i in range(1,21)})
        with self.assertRaisesRegex(cli.ProtocolError,'ACK timed out'): self.run_test(c,client)
        self.check_once(t)
    def test_wrong_session_newer_ack_cannot_supersede_old(self):
        c,t,client=self.setup_run({('cbench_keep',1):None})
        original=t.receive
        def corrupt(timeout):
            raw=original(timeout)
            if b',ok,cbench_keep,' in raw:
                fields=raw.decode().strip().split(',');fields[3+6]='99'
                return (','.join(fields)+'\n').encode()
            return raw
        t.receive=corrupt
        with self.assertRaisesRegex(cli.ProtocolError,'does not match current run'):self.run_test(c,client)
        self.check_once(t)
    def test_duplicate_old_ack_does_not_cover_missing_new_ack(self):
        c,t,client=self.setup_run({('cbench_keep',i):None for i in range(2,21)})
        original=t.send
        def duplicate(wire):
            original(wire)
            if b'cbench_keep' in wire and wire.endswith(b',1\n'):
                _,seq,raw=t.inbox[-1]
                for delay in (.09,.14,.19):heapq.heappush(t.inbox,(c()+delay,seq,raw))
        t.send=duplicate
        with self.assertRaisesRegex(cli.ProtocolError,'#2 ACK timed out'):self.run_test(c,client)
        self.check_once(t)
    def test_start_timeout_sends_stop_once(self):
        c,t,client=self.setup_run({('cbench_start',0):None})
        with self.assertRaisesRegex(cli.ProtocolError,'cbench_start timed out'): self.run_test(c,client)
        self.check_once(t)
    def test_signal_profile_retains_original_40ms_limit(self):
        c,t,client=self.setup_run({('cbench_keep',1):.06},profile=1)
        with self.assertRaisesRegex(cli.ProtocolError,'cbench_keep timed out'):
            self.run_test(c,client,profile='signal-diagnostic')
        self.check_once(t)
    def test_wrong_profile_never_starts(self):
        c,t,client=self.setup_run(profile=1)
        with self.assertRaisesRegex(cli.ProtocolError,'profile differs'): self.run_test(c,client)
        self.assertEqual([op for _,op,_,_ in t.sent],['cbench_status'])
    def test_host_stall_stops_without_catchup_burst(self):
        c,t,client=self.setup_run()
        def emit(event,status):
            if event=='started': t.receive_stall=.11
        with self.assertRaisesRegex(cli.ProtocolError,'schedule missed'):self.run_test(c,client,emit)
        self.check_once(t)
    def test_cli_slow_console_runs_after_stop_with_original_timestamps(self):
        c,t,client=self.setup_run();records=[]
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
        def slow_print(event,status):
            self.assertEqual(t.sent[-1][1],'cbench_stop')
            self.assertTrue(t.closed)
            records.append(event);c.now+=.1
        with patch.object(cli,'Client',return_value=client),patch.object(cli,'print_status',side_effect=slow_print),\
             patch.object(cli.time,'monotonic',c),patch.object(cli.time,'sleep',c.sleep):
            result=cli.main(['--wire-log',str(Path(temporary.name)/'wire.jsonl'),'run','--profile','calculation-only','--mode','rate','--target','0',
                             '--kp','1','--ki','0','--duration-ms','1000','--limit','100'],
                            transport_factory=lambda *a:t)
        self.assertEqual(result,0);self.assertIn('stopped',records);self.check_once(t)
    def test_unknown_or_malformed_unmatched_frames_are_ignored(self):
        for raw in (b'',b'att:1,2,3\n',b'rsp:999,ok,cbench_keep\n',b'\xff\n',b'x'*512+b'\n'):
            self.assertIsNone(cli.Client.decode_reply(raw,1,'cbench_keep'))
    def test_matching_wrong_operation_is_rejected(self):
        with self.assertRaisesRegex(cli.ProtocolError,'wrong operation'):
            cli.Client.decode_reply(b'rsp:1,ok,cbench_start\n',1,'cbench_keep')

if __name__=='__main__': unittest.main(verbosity=2)
