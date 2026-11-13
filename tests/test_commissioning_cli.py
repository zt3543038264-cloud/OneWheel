"""No serial device. Verify explicit action, session binding and no retry."""
import importlib.util
from pathlib import Path
import pytest
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('commissioning_cli',ROOT/'tools/commissioning_cli.py')
cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)

class Port:
    def __init__(self):
        self.writes=[];self.replies=[];self.drop=None;self.bad=None;self.closed=False
        self.status=['1','0','1','1','0','0','0','123','0','0','0']
    def write(self,data):
        self.writes.append(data)
        if data==b'stop\n':return len(data)
        op,seq,*args=data.decode().strip().removeprefix('cfg:').split(',')
        if op!=self.drop:
            payload=','.join(self.status) if op=='bench_status' else ('1,'+','.join('0' for _ in range(len(cli.STATUS_FIELDS[op])-1)) if op in cli.STATUS_FIELDS else '1')
            self.replies.append(f'rsp:{seq},ok,{self.bad or op},{payload}\n'.encode())
        return len(data)
    def read_until(self,*args):return self.replies.pop(0) if self.replies else b''
    def close(self):self.closed=True
    def reset_input_buffer(self):pass

class Clock:
    def __init__(self):self.now=0
    def __call__(self):self.now+=.1;return self.now

def client(port):return cli.Client(port,clock=Clock(),timeout=.5)

def test_status_is_read_only():
    p=Port();result=cli.status(client(p));assert set(result)=={'preview_status','bench_status','timing'}
    assert all(b'start' not in w and w!=b'stop\n' for w in p.writes)

def test_pulse_requires_explicit_matching_nonce_and_ack():
    p=Port();c=client(p)
    with pytest.raises(ValueError):cli.pulse_b(c,100,50,123,acknowledged=False)
    with pytest.raises(ValueError):cli.pulse_b(c,501,50,123,acknowledged=True)
    assert not p.writes
    with pytest.raises(cli.ProtocolError):cli.pulse_b(c,100,50,124,acknowledged=True)
    assert len(p.writes)==1 and b'bench_status' in p.writes[0]
    assert cli.pulse_b(c,-100,50,123,acknowledged=True)==['1']
    assert any(b',-100,50,B,123\n' in w for w in p.writes)

def test_fresh_local_grant_can_follow_latched_completed_pulse():
    p=Port();p.status[1]='4';p.status[5]='8'
    cli.pulse_b(client(p),100,50,123,acknowledged=True)
    assert sum(b'bench_start' in w for w in p.writes)==1

def test_uncertain_mutation_is_never_retried_and_requests_stop():
    p=Port();p.drop='bench_start'
    with pytest.raises(cli.ProtocolError):cli.pulse_b(client(p),100,50,123,acknowledged=True)
    assert sum(b'bench_start' in w for w in p.writes)==1
    assert p.writes[-1]==b'stop\n'

def test_no_hidden_reconnect_or_port_probe_on_invalid_arguments():
    opened=[]
    with pytest.raises(SystemExit):
        cli.main(['--port','NOT-A-DEVICE','pulse-b','--pwm','501','--milliseconds','50','--session','123','--ack-isolated-bench'],port_factory=lambda *a:opened.append(a))
    assert not opened
    p=Port();assert cli.main(['--port','NOT-A-DEVICE','status'],port_factory=lambda *a:p)==0
    assert p.closed and len(p.writes)==3

def test_bad_or_oversized_request_and_mismatched_response_fail_closed():
    p=Port();c=client(p)
    with pytest.raises(ValueError):c.request('bench_start','1\nstop')
    with pytest.raises(ValueError):c.request('preview_start','x'*80)
    assert not p.writes
    p.bad='different'
    with pytest.raises(cli.ProtocolError):c.request('preview_status')
    assert len(p.writes)==1
