"""Bounded in-memory wire evidence. Files are written only after control stops."""
import datetime
import json
import time

def datagram_reason(raw):
    if not raw:return 'EMPTY_DATAGRAM'
    if len(raw)>511:return 'OVERSIZE_DATAGRAM'
    if not raw.endswith(b'\n'):return 'MISSING_LF'
    if raw.count(b'\n')!=1:return 'MULTIPLE_LINES'
    try:text=raw.decode('ascii').rstrip('\r\n')
    except UnicodeDecodeError:return 'NON_ASCII'
    fields=text.split(',')
    if not fields[0].startswith('rsp:'):return 'NOT_RSP'
    token=fields[0][4:]
    if not token.isdecimal() or not 1<=int(token)<=65535:return 'INVALID_CFG_SEQUENCE'
    if len(fields)<3 or fields[1] not in ('ok','err'):return 'MALFORMED_RESPONSE_HEADER'
    return None

class WireLog:
    MAX_EVENTS=2048
    MAX_RAW_BYTES=2*1024*1024
    def __init__(self,clock=time.monotonic):
        self.clock=clock;self.events=[];self.raw_bytes=0;self.dropped=0;self.counter=0
        self.utc_started=datetime.datetime.now(datetime.timezone.utc).isoformat()
    def add(self,event,raw=None,**fields):
        self.counter+=1
        cost=0 if raw is None else len(raw)
        if len(self.events)>=self.MAX_EVENTS or self.raw_bytes+cost>self.MAX_RAW_BYTES:
            self.dropped+=1;return None
        row={'id':self.counter,'host_monotonic_ns':round(self.clock()*1e9),'event':event,**fields}
        if raw is not None:
            self.raw_bytes+=len(raw)
            row.update(byte_count=len(raw),raw_hex=raw.hex(),raw_ascii=raw.decode('ascii',errors='backslashreplace'))
        self.events.append(row)
        return row['id']
    def write(self,file):
        file.write(json.dumps({'event':'metadata','schema_version':1,'utc_started':self.utc_started,
            'clock':'host monotonic; not synchronized with MCU STM',
            'dropped_event_count':self.dropped,'complete':self.dropped==0})+'\n')
        for row in self.events:file.write(json.dumps(row,ensure_ascii=True)+'\n')
