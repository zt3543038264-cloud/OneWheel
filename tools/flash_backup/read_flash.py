"""Read-only UDP backup. Only transmits cfg:flash_read requests; no hello/set/save."""
import argparse, hashlib, json, socket, struct, time, zlib
from pathlib import Path

def request(sock, peer, seq, offset):
    command = f'cfg:flash_read,{seq},{offset}\n'.encode('ascii')
    for _ in range(3):
        sock.sendto(command, peer)
        until = time.monotonic() + 2
        pending = b""
        while time.monotonic() < until:
            try:
                packet, sender = sock.recvfrom(65535)
            except socket.timeout:
                continue
            if sender != peer:
                continue
            pending += packet
            if len(pending) > 65536: raise RuntimeError('Unterminated response stream')
            lines = pending.split(b'\n'); pending = lines.pop()
            for raw_line in lines:
                line = raw_line.decode('ascii', 'strict').rstrip('\r')
                fields = line.split(',')
                if fields[:1] != [f'rsp:{seq}']:
                    continue
                if len(fields) > 1 and fields[1] == 'err':
                    raise RuntimeError(line)
                if len(fields) != 12 or fields[1:3] != ['ok', 'flash_read'] or fields[3] != str(offset):
                    raise RuntimeError('Malformed reply: ' + line)
                if any(len(x) != 8 or any(c not in '0123456789abcdefABCDEF' for c in x) for x in fields[4:]):
                    raise RuntimeError('Invalid word encoding')
                return [int(x, 16) for x in fields[4:]]
    raise RuntimeError(f'Timeout reading offset {offset}; no complete backup saved')

def download(sock, peer, start_seq):
    words = []
    for i, offset in enumerate(range(0, 1024, 8)):
        words.extend(request(sock, peer, start_seq+i, offset))
    return words

def validate(words):
    count=words[1]
    result={'magic': f'{words[0]:08X}', 'count_word': count}
    if 1 <= count <= 510:
        at=2+2*count
        expected=zlib.crc32(struct.pack('<'+'I'*at,*words[:at])) & 0xffffffff
        result.update(crc_index=at, stored_crc=f'{words[at]:08X}', computed_crc=f'{expected:08X}', crc_matches=words[at]==expected, zero_tail=not any(words[at+1:]))
    else:
        result['crc_matches']=False
    return result

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host');ap.add_argument('--remote-port',type=int);ap.add_argument('--local-port',type=int)
    ap.add_argument('--output',type=Path,default=Path('flash_backups'))
    a=ap.parse_args()
    host=a.host or input('Module IPv4 (same as station): ').strip()
    remote=a.remote_port or int(input('Module UDP port: '))
    local=a.local_port or int(input('Computer listening UDP port: '))
    socket.inet_aton(host)
    if not all(1<=p<=65535 for p in (remote,local)):raise ValueError('Invalid port')
    peer=(host,remote)
    print('Close station first. Keep vehicle STOP. Reading twice; no write commands are sent.')
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
        s.bind(('0.0.0.0',local));s.settimeout(.2)
        first=download(s,peer,1000);second=download(s,peer,2000)
    if first!=second:raise RuntimeError('Snapshots differ; no verified backup saved')
    # Firmware checks every high half is zero before returning ANY low words.
    raw=b''.join(struct.pack('<II',w,0) for w in first)
    result=validate(first)
    result.update(address='AF016000',size_bytes=8192,layout='8-byte page: low uint32 + verified zero high uint32',double_read_identical=True,sha256=hashlib.sha256(raw).hexdigest(),words=[f'{w:08X}' for w in first])
    a.output.mkdir(parents=True,exist_ok=True)
    name='param_page11_'+time.strftime('%Y%m%d_%H%M%S')
    path=a.output/(name+'.bin')
    with path.open('xb') as f:f.write(raw)
    with (a.output/(name+'.json')).open('x') as f:json.dump(result,f,indent=2)
    print('Verified backup:',path.resolve());print(json.dumps({k:v for k,v in result.items() if k!='words'},indent=2))
if __name__=='__main__':
    try:main()
    except Exception as e:print('FAILED:',e);raise SystemExit(1)
