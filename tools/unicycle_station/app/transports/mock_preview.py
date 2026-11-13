"""GUI demonstration only; these samples are not a firmware PID simulation."""
import math
import time


class MockPreview:
    def __init__(self, transport):
        self.transport = transport
        self.reset()

    def reset(self):
        self.active = False
        self.ring = 0
        self.started = 0
        self.elapsed = 0
        self.reason = 0

    def stop(self, reason=17):
        if self.active:
            self.elapsed = min(10000, int((time.monotonic()-self.started)*1000))
        self.active = False
        self.reason = reason

    def snapshot(self):
        if self.active:
            self.elapsed = min(10000, int((time.monotonic()-self.started)*1000))
            if self.elapsed >= 10000:
                self.stop(16)
        t = time.monotonic() - self.transport.origin
        angle, rate = 3*math.sin(t*.7), 2.1*math.cos(t*.7)
        kp = float(self.transport.params.get("p_rate_kp", {}).get("value", 1))
        target = -angle*float(self.transport.params.get("p_angle_kp", {}).get("value", 1)) if self.ring else 0
        output = kp*(target-rate) if self.active else 0
        return [1, int(self.active), 1, self.ring, self.reason, self.elapsed,
                angle, rate, output, 1]

    def command(self, operation, seq, args):
        if operation not in ("preview_status", "preview_start", "preview_stop"):
            return False
        prefix = f"rsp:{seq},"
        if operation == "preview_status" and not args:
            self.transport.append(prefix+"ok,preview_status,"+",".join(map(str,self.snapshot())))
        elif operation == "preview_start" and len(args)==2 and args[0]=="pitch" and args[1] in ("rate","angle"):
            if self.active or self.transport.mode != "STOP":
                self.transport.append(prefix+"err,BLOCKED,19")
            else:
                self.active=True;self.ring=int(args[1]=="angle");self.reason=0
                self.started=time.monotonic();self.elapsed=0
                self.transport.manual_stopped=True
                self.transport.append(prefix+"ok,preview_start,1,ZERO_OUTPUT")
        elif operation == "preview_stop" and not args:
            self.stop()
            self.transport.append(prefix+"ok,preview_stop")
        else:
            self.transport.append(prefix+"err,BAD_FORMAT,Invalid preview command")
        return True
