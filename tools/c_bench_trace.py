"""Read MCU keep logs after STOP and correlate with the host wire JSONL.

Only klog_status/klog_get can be sent. No start, keep, stop, parameter write,
or automatic retry exists here. A missing download row is incomplete evidence.
"""
import argparse
import json
import math
import secrets
import time
from pathlib import Path
from c_bench_cli import UDPTransport,ProtocolError
from keep_wire_log import WireLog

ROW_FIELDS=('version','id','cfg_seq','session','keep_seq','result','rx_hi','rx_lo',
    'reply_hi','reply_lo','queue_result','reply_bytes','reply_hash','tx_end','spi_hi','spi_lo','spi_seen')

def fnv1a(raw):
    value=2166136261
    for byte in raw:value=((value^byte)*16777619)&0xffffffff
    return value

def parse_row(values,expected):
    if len(values)!=len(ROW_FIELDS):raise ProtocolError('Unsupported MCU log row schema')
    try:row={key:int(value) for key,value in zip(ROW_FIELDS,values)}
    except ValueError as e:raise ProtocolError('Noninteger MCU log row') from e
    if (row['version']!=1 or row['id']!=expected or any(v<0 or v>0xffffffff for v in row.values())
            or not 1<=row['cfg_seq']<=65535 or row['queue_result'] not in (0,1,2)
            or row['spi_seen'] not in (0,1) or row['result'] not in (1,2,3)
            or row['reply_bytes']>511):raise ProtocolError('Invalid MCU log row')
    return row

class Reader:
    def __init__(self,transport,timeout=.12,clock=time.monotonic):
        if not math.isfinite(timeout) or not 0 < timeout <= 5.0:
            raise ValueError("read-timeout must be >0..5s")
        self.transport=transport;self.timeout=timeout;self.clock=clock;self.seq=secrets.randbelow(40000)+1000
    def request(self,op,*args):
        if op not in ('klog_status','klog_get'):raise ValueError('Read-only log operations only')
        if len(args)!=(1 if op=='klog_get' else 0):raise ValueError('Wrong read operation arity')
        if args and (type(args[0]) is not int or not 1<=args[0]<=0xffffffff):raise ValueError('Invalid record id')
        self.seq+=1
        if self.seq>65535:raise ProtocolError('Read sequence exhausted')
        deadline=self.clock()+self.timeout
        self.transport.send((f'cfg:{op},{self.seq}'+''.join(','+str(a) for a in args)+'\n').encode('ascii'))
        while self.clock()<deadline:
            raw=self.transport.receive(max(0,deadline-self.clock()))
            if self.clock()>=deadline:break
            try:fields=raw.decode('ascii').rstrip('\r\n').split(',')
            except UnicodeDecodeError:continue
            if not raw.endswith(b'\n') or len(raw)>511 or raw.count(b'\n')!=1:continue
            if fields[0]!=f'rsp:{self.seq}':continue
            if len(fields)<3 or fields[1]!='ok' or fields[2]!=op:
                raise ProtocolError('MCU log query rejected: '+','.join(fields[1:]))
            return fields[3:]
        raise ProtocolError(op+' timed out; row remains unknown, no retry')

def status(reader):
    values=reader.request('klog_status')
    if len(values)!=7:raise ProtocolError('Unsupported MCU log status schema')
    try:version,cap,first,last,overwritten,hz,exhausted=map(int,values)
    except ValueError as e:raise ProtocolError('Noninteger MCU log status') from e
    if (version!=1 or cap!=64 or not 0<=first<=last<=0xffffffff or
            (last-first+1 if first else 0)>cap or not 1000<=hz<=1000000000 or
            overwritten<0 or exhausted not in (0,1)):raise ProtocolError('Invalid MCU log status')
    return dict(version=version,capacity=cap,first=first,last=last,overwritten=overwritten,clock_hz=hz,exhausted=exhausted)

def compare(host_rows,mcu):
    meta=next((r for r in host_rows if r.get('event')=='metadata'),{})
    tx={int(r['cfg_seq']):r for r in host_rows if r.get('event')=='tx_attempt' and r.get('operation')=='cbench_keep'}
    sent={int(r['cfg_seq']):r for r in host_rows if r.get('event')=='tx_sent'}
    rx={r['id']:r for r in host_rows if r.get('event')=='rx_datagram'}
    filters={}
    for r in host_rows:
        if r.get('event')=='rx_filtered':filters.setdefault(r.get('rx_id'),[]).append(r)
    validated={int(r['cfg_seq']):r for r in host_rows if r.get('event')=='keep_validated'}
    records={}
    for row in mcu.get('records',[]):records.setdefault((row['cfg_seq'],row['session'],row['keep_seq']),[]).append(row)
    output=[];hz=mcu.get('status',{}).get('clock_hz')
    for seq,t in tx.items():
        key=(seq,int(t['session']),int(t['keep_seq']));found=records.get(key,[])
        incoming=[]
        for rid,row in rx.items():
            raw=bytes.fromhex(row.get('raw_hex',''))
            if raw.split(b',',1)[0]==f'rsp:{seq}'.encode('ascii'):incoming.append((rid,row,raw))
        item={'cfg_seq':seq,'session':key[1],'keep_seq':key[2],
              'host_send_attempt_ns':t['host_monotonic_ns'],
              'host_send_return_ns':sent.get(seq,{}).get('host_monotonic_ns'),
              'host_replies':[dict(rx_id=rid,host_monotonic_ns=r['host_monotonic_ns'],raw_hex=raw.hex(),
                  filtered_reasons=[f['reason'] for f in filters.get(rid,[])]) for rid,r,raw in incoming],
              'mcu_records':found,'host_keep_validated':seq in validated}
        if seq not in sent:item['classification']='HOST_SEND_NOT_CONFIRMED'
        elif not found:item['classification']='MCU_PARSER_EVIDENCE_MISSING'
        else:
            record=found[-1]
            fingerprint=[rid for rid,_,raw in incoming if len(raw)==record['reply_bytes'] and fnv1a(raw)==record['reply_hash']]
            item['matching_reply_fingerprint_rx_ids']=fingerprint
            if record['result']!=1:item['classification']='MCU_REJECTED_KEEP'
            elif record['queue_result']!=1:item['classification']='MCU_REPLY_NOT_QUEUED'
            elif not record['spi_seen']:item['classification']='MCU_REPLY_NOT_OBSERVED_SUBMITTED_TO_MODULE'
            elif not incoming:item['classification']='MODULE_SUBMITTED_HOST_REPLY_NOT_OBSERVED'
            elif not fingerprint:item['classification']='HOST_REPLY_DIFFERS_FROM_MCU_FINGERPRINT'
            elif seq not in validated:item['classification']='HOST_REPLY_SEEN_NOT_VALIDATED'
            else:item['classification']='HOST_AND_MCU_EVIDENCE_MATCH'
            if hz:
                def ticks(prefix):return (record[prefix+'_hi']<<32)|record[prefix+'_lo']
                if record['queue_result']:item['mcu_parse_to_reply_us']=((ticks('reply')-ticks('rx'))&0xffffffffffffffff)*1e6/hz
                if record['spi_seen']:item['mcu_parse_to_spi_us']=((ticks('spi')-ticks('rx'))&0xffffffffffffffff)*1e6/hz
        output.append(item)
    return {'schema_version':1,'host_log_complete':bool(meta.get('complete')),
        'mcu_download_complete':mcu.get('complete',False),'keeps':output,
        'limits':['Host and MCU clocks are not synchronized; no cross-clock latency subtraction.',
                  'MCU RX is parser dispatch, not radio ingress. SPI submission is not UDP delivery.',
                  'Missing/overwritten/download-failed MCU rows do not prove downlink packet loss.',
                  'Raw reply hash is FNV-1a for accidental-corruption comparison, not authentication.']}

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',required=True);p.add_argument('--port',type=int,default=5001)
    p.add_argument('--local-port',type=int,default=8081);p.add_argument('--bind',default='0.0.0.0')
    p.add_argument('--host-log',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--read-timeout',type=float,default=.12,
                   help='Read-only klog reply timeout >0..5s; default: 0.12s; no retries')
    args=p.parse_args(argv)
    if not math.isfinite(args.read_timeout) or not 0 < args.read_timeout <= 5.0:
        p.error("read-timeout must be >0..5s")
    host_rows=[json.loads(line) for line in args.host_log.read_text(encoding='utf-8').splitlines() if line]
    # Reserve output before opening a socket; never overwrite prior evidence.
    file=args.output.open('x',encoding='utf-8');transport=None
    mcu={'records':[],'missing_ids':[],'complete':False};wire=WireLog()
    try:
        transport=UDPTransport(args.host,args.port,args.bind,args.local_port);transport.audit=wire
        reader=Reader(transport,timeout=args.read_timeout);mcu['status']=status(reader)
        first,last=mcu['status']['first'],mcu['status']['last']
        for index in range(first,last+1) if first else ():
            try:mcu['records'].append(parse_row(reader.request('klog_get',index),index))
            except ProtocolError as error:mcu['missing_ids'].append({'id':index,'error':str(error)})
        mcu['status_after']=status(reader)
        mcu['complete']=not mcu['missing_ids'] and mcu['status']==mcu['status_after'] and not mcu['status']['exhausted']
    except (ProtocolError,OSError,KeyboardInterrupt) as error:mcu['error']=str(error)
    finally:
        if transport:transport.close()
        result={'mcu':mcu,'comparison':compare(host_rows,mcu),'retrieval_rx_log':wire.events}
        json.dump(result,file,indent=2);file.close()
    print('Saved read-only MCU/host comparison:',args.output)
    print('MCU download complete:',mcu['complete'],'missing rows:',len(mcu['missing_ids']))
    return 0 if mcu['complete'] else 2

if __name__=='__main__':raise SystemExit(main())
