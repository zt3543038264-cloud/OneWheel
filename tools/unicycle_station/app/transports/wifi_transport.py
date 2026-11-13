"""Peer-bound WiFi UDP transport; one or more complete protocol lines per datagram."""
import socket
import threading
from dataclasses import asdict


class WifiTransport:
    MAX_DATAGRAM = 2048

    def __init__(self, config):
        self.config = config
        self.device = None
        self.write_lock = threading.Lock()
        self.dropped_datagrams = 0

    def open(self):
        self.config.validate()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Exclusive bind: two stations must never silently share vehicle replies.
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            sock.bind((self.config.wifi_bind, self.config.wifi_local_port))
            sock.connect((self.config.wifi_host, self.config.wifi_port))
            sock.settimeout(self.config.timeout)
        except Exception:
            sock.close()
            raise
        self.device = sock

    def close(self):
        sock, self.device = self.device, None
        if sock:
            sock.close()

    def read(self):
        if self.device is None:
            raise OSError("WiFi 未连接")
        try:
            # Extra byte detects truncation; never feed a partial datagram to the parser.
            data = self.device.recv(self.MAX_DATAGRAM + 1)
        except socket.timeout:
            return b""
        except OSError as exc:
            if getattr(exc, "winerror", None) == 10040:
                self.dropped_datagrams += 1
                return b""
            raise
        if not data or len(data) > self.MAX_DATAGRAM or not data.endswith(b"\n"):
            self.dropped_datagrams += 1
            return b""
        return data

    def write(self, data):
        if not data or len(data) > self.MAX_DATAGRAM or not data.endswith(b"\n"):
            raise ValueError("WiFi 命令必须是完整换行文本，且不超过 2048 字节")
        with self.write_lock:
            if self.device is None:
                raise OSError("WiFi 未连接")
            count = self.device.send(data)
            if count != len(data):
                raise OSError("WiFi 发送不完整")
            return count

    def emergency_stop(self):
        # Stop is idempotent; sets and Flash writes are NEVER repeated here.
        return sum(self.write(b"\r\nstop\r\n") for _ in range(3))

    def status(self):
        return {"connected": self.device is not None, "config": asdict(self.config),
                "dropped_datagrams": self.dropped_datagrams}
