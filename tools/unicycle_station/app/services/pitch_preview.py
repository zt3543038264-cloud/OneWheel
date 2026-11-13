"""Existing MCU Pitch controller preview; never generates keep or motor commands."""
import time
from app.core.models import Frame
from app.core.store import DataStore
from app.protocols.firewater import number


class PreviewProfile:
    data = {"telemetry_period_ms": 100}
    channels = {"preview": [
        {"name": "preview_pitch", "label": "Pitch 角度", "unit": "°"},
        {"name": "preview_pitch_rate", "label": "Pitch 角速度", "unit": "°/s"},
        {"name": "preview_output", "label": "控制器计算输出（未施加）", "unit": ""},
    ]}

    def channel_names(self, tag, count):
        return [p["name"] for p in self.channels["preview"][:count]]


def parse_status(fields):
    if len(fields) != 11 or fields[0] != "preview_status":
        raise ValueError("预览状态字段不完整")
    values = [number(str(x)) for x in fields[1:]]
    for index in (0, 1, 2, 3, 4, 5, 9):
        if values[index] != int(values[index]) or not 0 <= values[index] <= 0xffffffff:
            raise ValueError("预览状态整数非法")
    if (values[0] != 1 or values[1] not in (0, 1) or values[2] not in (0, 1)
            or values[3] not in (0, 1) or values[5] > 10000 or values[9] not in (0, 1)):
        raise ValueError("不支持的预览状态")
    return dict(active=bool(values[1]), axis=int(values[2]), ring=int(values[3]),
                reason=int(values[4]), elapsed_ms=int(values[5]), angle=values[6],
                rate=values[7], output=values[8], flywheel_link=bool(values[9]))


class PitchPreview:
    """All mutations run on StationEngine's existing I/O thread.

    Status polling reads only. It does not renew any firmware lease, and cannot
    extend the firmware's 10-second preview. Late callbacks cannot start a run
    after STOP, disconnect or a newer user action.
    """
    def __init__(self, engine, clock=time.monotonic):
        self.engine, self.clock = engine, clock
        self.store = DataStore(PreviewProfile(), capacity=1000)
        self.generation = 0
        self.reset(clear=True)

    def reset(self, clear=False):
        self.generation += 1
        self.phase = "idle"
        self.latest = None
        self.received = 0
        self.pending = False
        self.next_poll = 0
        self.changed_at = self.clock()
        self.supported = None
        self.message = "读取主控状态后，可运行 Pitch 角速度环或角度环的零输出预览。"
        if clear:
            self.store.reset()

    @property
    def busy(self):
        return self.phase in ("preflight", "starting", "active", "stopping", "unknown")

    def permission(self, name):
        if not self.busy:
            return None
        if (self.phase != "active" or not self.latest or self.latest["axis"] != 1
                or self.clock() - self.received >= .7):
            return False, "预览状态未确认，请先停止或读取状态"
        rings = ("rate", "angle") if self.latest["ring"] == 1 else ("rate",)
        allowed = {f"p_{ring}_{gain}" for ring in rings for gain in ("kp", "ki", "kd")}
        return (name in allowed, "当前 Pitch 预览 PID" if name in allowed else "预览期间仅可修改参与计算的 Pitch PID")

    def cancel_for_stop(self):
        self.generation += 1
        self.pending = False
        if self.busy:
            self.phase = "stopping"
            self.changed_at = self.clock()
            self.next_poll = self.clock()
            self.message = "已请求停止，等待主控预览状态确认。"

    def _stop_unknown(self, message):
        self.cancel_for_stop()
        self.message = message + "；已请求立即停止，结果待确认。"
        self.engine.event("PITCH_PREVIEW", self.message)
        self.engine.emergency_stop()

    def _accept(self, fields):
        status = parse_status(fields)
        self.latest, self.received = status, self.clock()
        self.supported = True
        self.phase = "active" if status["active"] else "idle"
        if status["active"]:
            self.message = ("Pitch 零输出预览运行中" if status["axis"] == 1 else "其他轴正在预览，请先停止")
        else:
            self.message = f"主控预览已停止 · 原因码 {status['reason']}"
        if status["axis"] == 1:
            self.store.accept(Frame("preview", [status["angle"], status["rate"], status["output"]],
                                    "", self.received))
        if self.engine.recorder:
            self.engine.recorder.put("pitch_preview", **status,
                                     source="mock" if self.engine.mode == "mock" else "firmware")
        self.next_poll = self.clock() + .1
        return status

    def _read(self, after=None):
        generation = self.generation
        self.pending = True
        def result(ok, fields):
            if generation != self.generation:
                return
            self.pending = False
            if not ok:
                if self.phase in ("starting", "active"):
                    self._stop_unknown("预览状态读取失败：" + ",".join(map(str, fields)))
                elif self.phase in ("stopping", "unknown"):
                    self.phase = "unknown"
                    self.next_poll = self.clock() + .1
                    self.message = "停止状态尚未确认：" + ",".join(map(str, fields))
                else:
                    self.phase = "idle"
                    self.supported = False if fields and fields[0] == "UNKNOWN_COMMAND" else self.supported
                    self.message = "未确认预览状态，未启动：" + ",".join(map(str, fields))
                return
            try:
                status = self._accept(fields)
            except ValueError as error:
                if self.phase in ("starting", "active"):
                    self._stop_unknown(str(error))
                elif self.phase in ("stopping", "unknown"):
                    self.phase = "unknown"
                    self.message = "停止状态尚未确认：" + str(error)
                else:
                    self.phase = "idle"
                    self.message = str(error)
                return
            if after:
                after(status)
        self.engine.requests.enqueue("preview_status", callback=result)

    def action(self, operation, ring="rate"):
        if self.engine.profile.data.get("id") != "tc264_unicycle":
            self.message = "当前设备未提供主轮 Pitch 预览。"
            return
        if self.engine.replay or not self.engine.connected or not self.engine.protocol_ready:
            self.message = "请先连接主控并完成握手；回放不能启动预览。"
            return
        if self.engine.requests.pending or self.engine.requests.queue:
            self.message = "请等待当前参数或状态请求完成。"
            return
        if operation == "read":
            self._read()
            return
        if operation != "start" or ring not in ("rate", "angle"):
            raise ValueError("仅支持 Pitch 角速度环和角度环预览")
        if self.busy or not self.engine.stopped() or self.engine.remote.armed:
            self.message = "请先确认 STOP、停止其他预览，并关闭遥控。"
            return
        self.generation += 1
        generation = self.generation
        self.phase, self.changed_at = "preflight", self.clock()
        self.message = "正在核对主控预览状态……"
        def checked(status):
            if generation != self.generation:
                return
            if status["active"]:
                self.message = "主控已有预览，请先停止，未发送新的启动命令。"
                return
            self.store.reset()
            self.phase, self.changed_at = "starting", self.clock()
            self.message = "启动已请求，等待主控确认零输出预览。"
            def started(ok, fields):
                if generation != self.generation:
                    return
                if not ok or fields != ["preview_start", "1", "ZERO_OUTPUT"]:
                    self._stop_unknown("启动未得到匹配的零输出确认：" + ",".join(map(str, fields)))
                    return
                self.phase = "active"
                self.received = self.clock()
                self.next_poll = self.clock()
                self.latest = None
                self.message = "零输出预览已确认，正在读取反馈。"
                self.engine.event("PITCH_PREVIEW", "正式 Pitch 控制器零输出预览：" + ring)
            self.engine.requests.enqueue("preview_start", "pitch", ring, callback=started)
        self._read(after=checked)

    def tick(self):
        if self.engine.replay or not self.engine.connected or not self.engine.protocol_ready:
            return
        now = self.clock()
        if self.phase in ("starting", "active") and now - max(self.received, self.changed_at) >= .7:
            self._stop_unknown("预览反馈超时")
            return
        if self.phase == "preflight" and now - self.changed_at >= .7:
            self.generation += 1
            self.phase, self.pending = "idle", False
            self.message = "预检查超时，未启动预览。"
            self.engine.requests.cancel("PREVIEW_PREFLIGHT_TIMEOUT")
            return
        if (self.phase in ("active", "stopping", "unknown") and not self.pending
                and now >= self.next_poll and not self.engine.stop_requested.is_set()
                and not self.engine.requests.pending and not self.engine.requests.queue):
            self._read()
