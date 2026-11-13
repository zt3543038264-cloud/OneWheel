import csv
import importlib.util
from pathlib import Path
import socket
import subprocess
import sys
import threading
import pytest

SCRIPT=Path(__file__).parents[1]/'scripts/capture_roll_trace.py'
spec=importlib.util.spec_from_file_location('capture_roll_trace',SCRIPT)
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)

def test_decode_timestamp_wrap_and_missing_index():
    rows=mod.decode_samples([[0,4294967200,1,2,3,4,5,6,7,8,9,10,7],
                             [1,99904,1,2,3,4,5,6,7,8,9,10,7]],100000000)
    assert rows[1][2]==1000
    with pytest.raises(ValueError):mod.decode_samples([[1,0]+[0]*11],100000000)
    with pytest.raises(ValueError):mod.decode_samples([],0)

def test_udp_capture_download_and_read_retry(tmp_path):
    server=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);server.bind(('127.0.0.1',0));server.settimeout(.2)
    reserve=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);reserve.bind(('127.0.0.1',0));local=reserve.getsockname()[1];reserve.close()
    done=threading.Event();commands=[];dropped=False
    def serve():
        nonlocal dropped
        while not done.is_set():
            try:data,peer=server.recvfrom(2048)
            except socket.timeout:continue
            for line in data.decode().splitlines():
                if not line:continue
                commands.append(line)
                if line=='stop':continue
                f=line.split(',');op=f[0][4:];seq=f[1];prefix=''
                if op=='hello':payload='hello,1,test,63'
                elif op=='status':prefix='stat:10,0,0,0,0,7,1,0,1,0,0,1,0,0,7\n';payload='status'
                elif op=='trace_arm':payload='trace_arm,1,256,1000'
                elif op=='trace_status':payload='trace_status,1,3,256,256,1000,100000000,1,1,0,40,0,0'
                elif op=='trace_get':
                    i=int(f[2])
                    if i==7 and not dropped:dropped=True;continue
                    ticks=(4294967200+i*100000)&0xffffffff
                    payload=f'trace_get,{i},{ticks},0,1,2,3,0,0,1,40,-20,20,7'
                else:raise AssertionError(op)
                server.sendto((prefix+f'rsp:{seq},ok,{payload}\n').encode(),peer)
    thread=threading.Thread(target=serve);thread.start()
    try:
        result=subprocess.run([sys.executable,str(SCRIPT),'--host','127.0.0.1','--port',str(server.getsockname()[1]),
                               '--local-port',str(local),'--delay-ms','0','--out',str(tmp_path)],capture_output=True,text=True,timeout=15)
        assert result.returncode==0,result.stdout+result.stderr
        path=next(tmp_path.glob('*.csv'));rows=list(csv.DictReader(path.open()))
        assert len(rows)==256 and float(rows[-1]['time_us'])==255000
        assert sum('cfg:trace_arm,' in x for x in commands)==1
        assert sum(x.endswith(',7') and x.startswith('cfg:trace_get,') for x in commands)==2
        assert not any(x.startswith(('cfg:set,','cfg:save,','speed:')) for x in commands)
        assert 'STOP confirmed' in result.stdout
    finally:
        done.set();thread.join(timeout=2);server.close()
