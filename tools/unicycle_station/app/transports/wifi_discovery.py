"""Passive peer discovery adapted from master QcarMonitor._accept_peer.

Only validated current-protocol telemetry is considered. No commands, broadcasts,
parameter writes or motor actions are sent. Source endpoints are candidates, not
cryptographically authenticated device identities.
"""
import ipaddress
import socket

from app.protocols.firewater import FireWaterParser


class WifiDiscovery:
    MAX_DATAGRAM = 2048
    MAX_PEERS = 32

    def __init__(self):
        self.device = None
        self.peers = {}

    def open(self, bind, port):
        address = ipaddress.IPv4Address(bind)
        if address.is_multicast or int(address) == 0xffffffff:
            raise ValueError("本机接收地址必须是本机 IPv4 或 0.0.0.0")
        if not 1024 <= port <= 65535:
            raise ValueError("本机接收端口必须在 1024～65535")
        self.close()
        self.peers.clear()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            sock.bind((str(address), port))
            sock.setblocking(False)
        except Exception:
            sock.close()
            raise
        self.device = sock

    def poll(self):
        if self.device is None:
            return []
        added = []
        # Bound each GUI tick even on a busy UDP port.
        for _ in range(64):
            try:
                data, peer = self.device.recvfrom(self.MAX_DATAGRAM + 1)
            except BlockingIOError:
                break
            except OSError as exc:
                if getattr(exc, "winerror", None) == 10040:
                    continue
                raise
            if len(data) > self.MAX_DATAGRAM or not data.endswith(b"\n"):
                continue
            if not 1024 <= peer[1] <= 65535:
                continue
            parser = FireWaterParser()
            parser.feed(data)
            frames = parser.parsed_frames()
            if not frames or any(f.error or f.tag not in ("att", "run", "stat", "fly", "nav", "task", "taskevt") for f in frames):
                continue
            if peer not in self.peers and len(self.peers) < self.MAX_PEERS:
                self.peers[peer] = frames[0].tag
                added.append(peer)
        return added

    def close(self):
        sock, self.device = self.device, None
        if sock is not None:
            sock.close()
