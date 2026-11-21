"""Download one frozen right flywheel recording; never start or arm the vehicle."""
import base64
from binascii import crc_hqx
import csv
import os
from pathlib import Path
import re
import struct
from tempfile import NamedTemporaryFile
import time

_META = struct.Struct("<IBBHHHIIHH")
_ROW = struct.Struct("<IIhHHHHhHBB")
_COLUMNS = (
    "capture_id", "capture_stop_reason", "sample", "relative_ms", "driver_ticks",
    "accepted_sequence", "command_b", "right_pwm_a_counts", "right_pwm_b_counts",
    "right_pwm_c_counts", "board_bus_mv", "encoder_delta", "encoder_angle",
    "right_pwm_valid", "voltage_valid", "encoder_error", "battery_protect",
    "right_motor_protect", "right_protect_cause", "safe_state", "safe_reason",
    "flags_raw", "safe_status_raw",
)


def _frame(fields, mode, capture=None, index=None):
    prefix = f"BT,{mode},"
    if mode == "D":
        prefix += f"{capture:08X},{index:04X},"
    line = ",".join(fields)
    match = re.fullmatch(re.escape(prefix) + r"([A-Za-z0-9+/]{32})\*([0-9A-F]{4})", line)
    if not match:
        raise ValueError("记录应答格式或编号不符")
    if crc_hqx(line[:-5].encode("ascii"), 0xffff) != int(match[2], 16):
        raise ValueError("记录校验失败")
    return base64.b64decode(match[1], validate=True)


def _write_csv(path, capture, stop_reason, trigger, hz, rows):
    path = Path(path)
    temp = None
    try:
        with NamedTemporaryFile(mode="w", encoding="utf-8-sig", newline="",
                                suffix=".tmp", dir=path.parent, delete=False) as out:
            temp = Path(out.name)
            writer = csv.writer(out)
            writer.writerow(_COLUMNS)
            for i, (ticks, seq, command, pa, pb, pc, mv, delta, angle, flags, status) in enumerate(rows):
                bit = lambda n: (flags >> n) & 1
                writer.writerow((
                    capture, stop_reason, i, f"{(ticks-trigger)*1000/hz:.3f}",
                    ticks, seq, command, pa if bit(0) else "", pb if bit(0) else "",
                    pc if bit(0) else "", mv if bit(1) else "", delta, angle,
                    bit(0), bit(1), bit(2), bit(3), bit(4), flags >> 5,
                    status & 15, status >> 4, flags, status,
                ))
        os.replace(temp, path)
    finally:
        if temp is not None:
            temp.unlink(missing_ok=True)


class RightTraceDownload:
    """Called only by the device I/O thread; GUI reads a progress string."""
    def __init__(self, engine):
        self.engine = engine
        self.busy = False
        self.message = "右飞轮诊断：车身 STOP、飞轮停稳后导出"
        self.rows = []
        self.attempts = 0
        self.retry = None

    def start(self, path):
        e = self.engine
        if self.busy:
            return
        if (e.replay or e.mode not in ("serial", "wifi") or not e.stopped()
                or e.remote.armed or e.stop_requested.is_set()):
            self.message = "导出未启动：连接实车，确认 STOP 且飞轮停稳后再试"
            return
        if e.pitch_preview.busy or e.requests.pending or e.requests.queue:
            self.message = "导出未启动：请等当前请求或预览结束"
            return
        self.busy = True
        self.path = Path(path)
        self.rows = []
        self.attempts = 0
        self.retry = None
        self.message = "正在读取右飞轮诊断状态…"
        self._request_meta()

    def _finish(self, message, success=False):
        self.busy = False
        self.rows = []
        self.retry = None
        self.message = message
        self.engine.event("RIGHT_TRACE_SAVED" if success else "RIGHT_TRACE_STOPPED", message)

    def tick(self):
        if not self.busy or self.retry is None:
            return
        e = self.engine
        if not e.stopped() or e.stop_requested.is_set():
            self._finish("导出已中止：STOP 状态变化或过期")
            return
        deadline, next_request = self.retry
        if time.monotonic() >= deadline and not e.requests.pending and not e.requests.queue:
            self.retry = None
            next_request()

    def _retry_read(self, fields, next_request):
        # A firmware BUSY/TIMEOUT never replays an actuator command.
        # Queue only on a later tick, after RequestManager's error cleanup.
        if not fields or fields[0] not in ("BUSY", "TIMEOUT") or self.attempts >= 4:
            return False
        self.attempts += 1
        self.retry = time.monotonic() + (.043, .067, .103, .147)[self.attempts-1], next_request
        return True

    def _request_meta(self):
        self.engine.requests.enqueue("rtrace", "M", callback=self._meta)

    def _meta(self, ok, fields):
        try:
            if not ok and self._retry_read(fields, self._request_meta):
                return
            if not ok:
                raise ValueError(" ".join(map(str, fields)) or "主控未应答")
            (capture, state, reason, pre, post, size, trigger, hz, capacity, reserved) = (
                _META.unpack(_frame(fields[1:], "M"))
            )
            if size != _ROW.size or capacity != 1024 or reserved or hz != 8000000:
                raise ValueError("驱动板记录版本不匹配")
            if state != 3 or reason == 3 or not post:
                raise ValueError("尚无冻结的右轮记录；先确认点动完成且飞轮停稳")
            if (not capture or reason not in (1, 2) or trigger >= 0xffffffff
                    or pre > 128 or post > 896 or pre+post > capacity):
                raise ValueError("记录头不完整")
            self.capture, self.trigger, self.hz = capture, trigger, hz
            self.count, self.stop_reason = pre+post, reason
            self.attempts = 0
            self._next()
        except ValueError as exc:
            self._finish(f"未导出右飞轮记录：{exc}")

    def _next(self):
        if not self.engine.stopped() or self.engine.stop_requested.is_set():
            self._finish("导出已中止：STOP 状态变化或过期")
            return
        index = len(self.rows)
        self.message = f"正在读取右飞轮记录：{index}/{self.count}"
        self.engine.requests.enqueue("rtrace", "D", f"{self.capture:08X}", f"{index:04X}",
                                     callback=self._row)

    def _row(self, ok, fields):
        try:
            if not ok and self._retry_read(fields, self._next):
                return
            if not ok:
                raise ValueError(" ".join(map(str, fields)) or "主控未应答")
            if not self.engine.stopped() or self.engine.stop_requested.is_set():
                raise ValueError("STOP 状态变化或过期")
            index = len(self.rows)
            row = _ROW.unpack(_frame(fields[1:], "D", self.capture, index))
            if self.rows and row[0] <= self.rows[-1][0]:
                raise ValueError("样本时间乱序")
            self.rows.append(row)
            self.attempts = 0
            if len(self.rows) == self.count:
                _write_csv(self.path, self.capture, self.stop_reason, self.trigger, self.hz, self.rows)
                self._finish(f"右飞轮记录已导出：{self.count} 点 · {self.path}", True)
            else:
                self._next()
        except (ValueError, OSError) as exc:
            self._finish(f"未导出右飞轮记录：{exc}")
