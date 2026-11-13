import io
import json
import socket
import unittest
from unittest.mock import patch
import test_c_bench_client as base
from keep_wire_log import WireLog
from c_bench_trace import compare,fnv1a,Reader,ProtocolError,parse_row

cli=base.cli

class Tests(unittest.TestCase):
    def run_capture(self,delays=None):
        clock=base.Clock();transport=base.Transport(clock,delays)
        client=cli.Client(transport,clock=clock,sequence=100);wire=WireLog(clock)
        client.audit=wire
        cli.run(client,profile='calculation-only',mode='rate',target=0,kp=1,ki=0,
                duration_ms=1000,limit=100,clock=clock,sleep=clock.sleep)
        return wire
    def test_every_keep_has_exact_raw_sequence_and_send_boundaries(self):
        wire=self.run_capture()
        attempts=[r for r in wire.events if r['event']=='tx_attempt' and r['operation']=='cbench_keep']
        sent={r['cfg_seq']:r for r in wire.events if r['event']=='tx_sent'}
        self.assertGreaterEqual(len(attempts),19)
        for keep,row in enumerate(attempts,1):
            self.assertEqual(int(row['keep_seq']),keep)
            self.assertEqual(bytes.fromhex(row['raw_hex']),f"cfg:cbench_keep,{row['cfg_seq']},1,{keep}\n".encode())
            self.assertGreaterEqual(sent[row['cfg_seq']]['host_monotonic_ns'],row['host_monotonic_ns'])
        self.assertEqual(len({r['cfg_seq'] for r in attempts}),len(attempts))
    def test_unknown_and_superseded_packets_have_filter_reason(self):
        wire=self.run_capture({('cbench_keep',1):.11})
        self.assertTrue(any(r.get('reason')=='NO_PENDING_SEQUENCE' for r in wire.events))
        self.assertTrue(any(r.get('reason')=='SUPERSEDED_BY_NEWER_ACK' for r in wire.events))
    def test_transport_preserves_all_rejected_raw_datagrams(self):
        good=b'rsp:9,ok,cbench_status\n';peer=('192.168.0.112',5001)
        incoming=[(b'',peer),(good,('192.168.0.113',5001)),(b'abc',peer),
                  (b'x'*512+b'\n',peer),(b'a\nb\n',peer),(good,peer)]
        class Sock:
            def bind(self,*args):pass
            def settimeout(self,*args):pass
            def recvfrom(self,*args):return incoming.pop(0)
            def close(self):pass
        with patch.object(cli.socket,'socket',return_value=Sock()):
            transport=cli.UDPTransport(peer[0]);wire=WireLog();transport.audit=wire
            self.assertEqual(transport.receive(.1),good)
        reasons={r['reason'] for r in wire.events if r['event']=='rx_filtered'}
        self.assertEqual(reasons,{'EMPTY_DATAGRAM','WRONG_SOURCE','MISSING_LF','OVERSIZE_DATAGRAM','MULTIPLE_LINES'})
        self.assertEqual(len([r for r in wire.events if r['event']=='rx_datagram']),6)
    def test_decode_rejection_keeps_raw_link_and_error(self):
        clock=base.Clock();transport=base.Transport(clock);client=cli.Client(transport,clock=clock)
        wire=WireLog(clock);client.audit=wire
        client.last_rx_id=wire.add('rx_datagram',raw=b'rsp:8,ok,cbench_keep,broken\n')
        with self.assertRaises(cli.ProtocolError):client.decode_checked(b'rsp:8,ok,cbench_keep,broken\n',8,'cbench_keep')
        self.assertEqual(wire.events[-1]['reason'],'MATCHING_REPLY_REJECTED')
        self.assertEqual(wire.events[-1]['rx_id'],client.last_rx_id)
    def test_bounded_log_marks_incomplete_instead_of_hiding_drops(self):
        wire=WireLog();wire.MAX_EVENTS=2
        for i in range(3):wire.add('packet',raw=b'a')
        file=io.StringIO();wire.write(file)
        meta=json.loads(file.getvalue().splitlines()[0])
        self.assertFalse(meta['complete']);self.assertEqual(meta['dropped_event_count'],1)
    def test_reader_cannot_send_control_operations(self):
        class NeverSend:
            def send(self,data):raise AssertionError('must not send')
        reader=Reader(NeverSend())
        for op in ('cbench_start','cbench_keep','cbench_stop','set','save'):
            with self.assertRaises(ValueError):reader.request(op)
    def matched_evidence(self):
        wire=self.run_capture();events=[{'event':'metadata','complete':True}]+wire.events
        tx=next(r for r in events if r['event']=='tx_attempt' and r['operation']=='cbench_keep')
        seq=tx['cfg_seq'];rx=next(r for r in events if r['event']=='rx_datagram' and bytes.fromhex(r['raw_hex']).startswith(f'rsp:{seq},'.encode()))
        raw=bytes.fromhex(rx['raw_hex'])
        row=dict(version=1,id=1,cfg_seq=seq,session=1,keep_seq=1,result=1,rx_hi=0,rx_lo=1000,
                 reply_hi=0,reply_lo=1020,queue_result=1,reply_bytes=len(raw),reply_hash=fnv1a(raw),tx_end=200,
                 spi_hi=0,spi_lo=1040,spi_seen=1)
        mcu={'records':[row],'status':{'clock_hz':1000000},'complete':True}
        return events,mcu,row,rx
    def test_compare_matches_raw_reply_and_separate_clock_intervals(self):
        events,mcu,row,rx=self.matched_evidence();result=compare(events,mcu)['keeps'][0]
        self.assertEqual(result['classification'],'HOST_AND_MCU_EVIDENCE_MATCH')
        self.assertEqual(result['mcu_parse_to_reply_us'],20)
        self.assertEqual(result['mcu_parse_to_spi_us'],40)
        self.assertEqual(result['matching_reply_fingerprint_rx_ids'],[rx['id']])
    def test_compare_missing_download_does_not_claim_downlink_loss(self):
        events,mcu,row,rx=self.matched_evidence();mcu['records']=[];mcu['complete']=False
        result=compare(events,mcu)
        self.assertFalse(result['mcu_download_complete'])
        self.assertEqual(result['keeps'][0]['classification'],'MCU_PARSER_EVIDENCE_MISSING')
    def test_compare_can_distinguish_queue_and_module_submission(self):
        events,mcu,row,rx=self.matched_evidence();row['queue_result']=2
        self.assertEqual(compare(events,mcu)['keeps'][0]['classification'],'MCU_REPLY_NOT_QUEUED')
        row['queue_result']=1;row['spi_seen']=0
        self.assertEqual(compare(events,mcu)['keeps'][0]['classification'],'MCU_REPLY_NOT_OBSERVED_SUBMITTED_TO_MODULE')
    def test_compare_detects_reply_fingerprint_mismatch(self):
        events,mcu,row,rx=self.matched_evidence();row['reply_hash']^=1
        self.assertEqual(compare(events,mcu)['keeps'][0]['classification'],'HOST_REPLY_DIFFERS_FROM_MCU_FINGERPRINT')

if __name__=='__main__':unittest.main(verbosity=2)
