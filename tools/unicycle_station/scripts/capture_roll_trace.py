"""Arm a local 1 kHz Roll Rate capture; manually start the test, STOP, download CSV.
No PID/Flash writes or remote motor starts. Uses only Python's standard library.
"""
import argparse
import csv
import json
from pathlib import Path
import socket
import time

class Link:
    def __init__(self, host, port, local_port):
        self.peer = (socket.gethostbyname(host), port)
        self.seq = 5000
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        self.sock.bind(('0.0.0.0', local_port))
        self.sock.settimeout(.2)

    def request(self, op, *args, retries=0):
        self.seq = self.seq % 65535 + 1
        seq = self.seq
        wire = (','.join(['cfg:'+op, str(seq), *map(str,args)])+'\r\n').encode('ascii')
        for attempt in range(retries+1):
            self.sock.sendto(wire, self.peer)
            deadline = time.monotonic()+2
            status = None
            while time.monotonic()<deadline:
                try:
                    data, peer = self.sock.recvfrom(8192)
                except socket.timeout:
                    continue
                if peer != self.peer:
                    continue
                for line in data.decode('ascii', errors='replace').splitlines():
                    if line.startswith('stat:'):
                        status = [int(x) for x in line[5:].split(',')]
                    if line.startswith(f'rsp:{seq},'):
                        fields = line.split(',')
                        if fields[1] != 'ok':
                            raise RuntimeError(line)
                        return fields[2:], status
        raise TimeoutError(f'No reply to {op}; write commands are not retried')

    def stop(self):
        self.sock.sendto(b'\r\nstop\r\n', self.peer)
        time.sleep(.15)
        _, status = self.request('status', retries=1)
        if status is None or any(status[1:5]):
            raise RuntimeError('STOP not confirmed; use local stop')


def decode_samples(samples, tick_hz):
    if not samples or tick_hz <= 0:
        raise ValueError('No samples or invalid timestamp clock')
    base = int(samples[0][1])
    rows = []
    for expected, sample in enumerate(samples):
        if len(sample) != 13 or int(sample[0]) != expected:
            raise ValueError('Missing or misordered trace sample')
        rows.append([expected, int(sample[1]),
                     ((int(sample[1])-base) & 0xffffffff)*1e6/tick_hz,
                     *sample[2:]])
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host', required=True, help='Vehicle WiFi IP')
    ap.add_argument('--port', type=int, default=5001)
    ap.add_argument('--local-port', type=int, default=8081)
    ap.add_argument('--delay-ms', type=int, default=2000)
    ap.add_argument('--out', type=Path, default=Path('artifacts/wifi-tuning'))
    ap.add_argument('--label', choices=['ground','airborne','other'], default='other')
    args = ap.parse_args(argv)
    if not 0 <= args.delay_ms <= 10000:
        ap.error('--delay-ms must be 0..10000')
    link = Link(args.host, args.port, args.local_port)
    armed = False
    try:
        link.request('hello', retries=1)
        _, status = link.request('status', retries=1)
        if status is None or any(status[1:5]):
            raise RuntimeError('Start from STOP; this script does not start motors')
        armed = True  # arm may apply even if its reply is lost
        link.request('trace_arm', args.delay_ms)
        print('Armed. Hold the chassis and manually start Roll > Rate. Do not edit PID.', flush=True)
        deadline = time.monotonic()+45
        while time.monotonic()<deadline:
            meta, _ = link.request('trace_status', retries=1)
            if meta[0] != 'trace_status' or len(meta) != 13:
                raise RuntimeError('Incompatible trace metadata')
            if int(meta[2]) == 3:
                break
            time.sleep(.2)
        else:
            raise TimeoutError('Capture did not freeze in time')
        link.stop()
        armed = False
        n, tick_hz = int(meta[3]), int(meta[6])
        reason = int(meta[8])
        if not 0 < n <= 256:
            raise RuntimeError(f'No samples: finish reason={reason}')
        samples = []
        for index in range(n):
            fields, _ = link.request('trace_get', index, retries=2)
            if fields[0] != 'trace_get':
                raise RuntimeError('Unexpected sample response')
            samples.append([float(x) for x in fields[1:]])
        rows = decode_samples(samples, tick_hz)
        args.out.mkdir(parents=True, exist_ok=True)
        base = args.out/('trace1000-'+args.label+'-'+time.strftime('%Y%m%d-%H%M%S'))
        with base.with_suffix('.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['index','stm_ticks','time_us','roll_deg','gyro_x_dps','gyro_y_dps',
                             'gyro_z_dps','acc_x_g','acc_y_g','acc_z_g','roll_duty',
                             'a_rpm','b_rpm','flags'])
            writer.writerows(rows)
        intervals = [rows[i][2]-rows[i-1][2] for i in range(1,len(rows))]
        metadata = dict(zip(['operation','version','state','count','capacity','nominal_hz','tick_hz',
                             'parameter_revision','finish_reason','delay_ms','rate_kp','rate_ki','rate_kd'], meta))
        metadata.update(host=args.host, label=args.label, min_interval_us=min(intervals,default=0),
                        max_interval_us=max(intervals,default=0), hardware_timing_verified=False)
        base.with_suffix('.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        print(f'STOP confirmed; saved {n} samples to {base.with_suffix(".csv")}', flush=True)
        print(f'Finish reason={reason}; intervals {metadata["min_interval_us"]:.1f}..{metadata["max_interval_us"]:.1f} us')
        if reason != 1:
            print('Partial trace retained; inspect finish reason before comparison')
    finally:
        try:
            if armed:
                try:
                    link.request('trace_cancel')
                finally:
                    link.stop()
        finally:
            link.sock.close()

if __name__ == '__main__':
    main()
