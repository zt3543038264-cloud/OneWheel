#!/usr/bin/env python3
"""Fixed, observational-only UDP link probe. Python 3 standard library only.

This program cannot start a C-wheel bench test, change settings, or authorize
powered testing. It sends one begin, up to 200 unacknowledged indexed ticks at
50 ms intervals, then one status query (at most one collecting-state follow-up).
Importing this module opens no socket and sends nothing.
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import secrets
import socket
import sys
import time
from pathlib import Path


SAMPLE_COUNT = 200
PERIOD_SECONDS = 0.050
SAMPLE_WINDOW_MS = 10000
REPLY_TIMEOUT_SECONDS = 5.0
POST_SEND_WAIT_SECONDS = 1.0
MAX_RESPONSE_BYTES = 511
UINT32_MAX = 0xFFFFFFFF
ALLOWED_OPERATIONS = frozenset(("lprobe_begin", "lprobe_tick", "lprobe_status"))
STATE_NAMES = {0: "idle", 1: "armed", 2: "collecting", 3: "complete",
               4: "busy", 5: "clock", 6: "no_samples"}
STATUS_FIELDS = (
    "version", "run_id", "state", "received", "last", "missing", "duplicate",
    "reordered", "invalid", "max_gap_us", "mean_gap_us", "gaps_ge200",
    "shadow_expired", "elapsed_ms", "remaining_ms", "rx_errors", "tx_drops",
)
NOTICE = "Observation only. No powered-test permission is granted by this result."


class ProtocolError(RuntimeError):
    """No confirmed probe result; do not retry a begin or tick."""


def _uint(value, maximum, name, minimum=0):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in {minimum}..{maximum}")
    return value


def ipv4_address(value, *, peer=False):
    try:
        address = ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError as error:
        raise ValueError("Use a literal IPv4 address; DNS/discovery is not used") from error
    if address.is_multicast or int(address) == UINT32_MAX or (peer and address.is_unspecified):
        raise ValueError("Use a unicast IPv4 peer or a unicast/unspecified bind address")
    return str(address)


class UDPTransport:
    """Exclusive local bind, one exact IPv4/port peer, one datagram per receive.

    Source filtering returns control after every datagram so inbound traffic
    cannot keep a caller inside an unbounded receive/drain loop.
    """
    def __init__(self, host="192.168.0.111", port=5001, bind="0.0.0.0", local_port=8081,
                 *, socket_factory=socket.socket):
        self.peer = (ipv4_address(host, peer=True), _uint(port, 65535, "port", 1))
        local = (ipv4_address(bind), _uint(local_port, 65535, "local port", 1))
        self.discarded_sources = 0
        self.socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Windows otherwise permits another socket to steal our local port.
            # Never enable SO_REUSEADDR or SO_REUSEPORT on any platform.
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            self.socket.bind(local)
        except BaseException:
            self.socket.close()
            raise

    def send(self, data):
        self.socket.settimeout(0.0)  # No blocking send that could create a catch-up burst.
        if self.socket.sendto(data, self.peer) != len(data):
            raise ProtocolError("Partial UDP send; outcome uncertain, no retry")

    def receive(self, timeout):
        if timeout <= 0:
            return b""
        self.socket.settimeout(timeout)
        try:
            # One byte beyond the maximum makes oversized/truncated frames invalid.
            raw, source = self.socket.recvfrom(MAX_RESPONSE_BYTES + 1)
        except (socket.timeout, BlockingIOError):
            return b""
        if source != self.peer:
            self.discarded_sources += 1
            return b""
        return raw

    def close(self):
        self.socket.close()


def _response_parts(raw):
    """Only complete, bounded, printable-ASCII response frames may be read."""
    if (not isinstance(raw, bytes) or not 1 <= len(raw) <= MAX_RESPONSE_BYTES or
            not raw.endswith(b"\n") or raw.count(b"\n") != 1 or b"\r" in raw):
        return None
    try:
        text = raw[:-1].decode("ascii")
    except UnicodeDecodeError:
        return None
    if any(ord(ch) < 32 or ord(ch) > 126 for ch in text):
        return None
    return text.split(",")


def parse_response(raw, *, sequence, operation, run_id):
    """Return one exact, bounded version-1 response, or None for unwanted data."""
    parts = _response_parts(raw)
    if (parts is None or len(parts) != 3 + len(STATUS_FIELDS) or
            parts[:3] != [f"rsp:{sequence}", "ok", operation]):
        return None
    fields = parts[3:]
    # int() alone accepts signs, whitespace, Unicode digits and unlimited sizes.
    if any(not value or len(value) > 10 or any(ch < "0" or ch > "9" for ch in value)
           for value in fields):
        return None
    status = dict(zip(STATUS_FIELDS, map(int, fields)))
    if (any(value > UINT32_MAX for value in status.values()) or
            status["version"] != 1 or status["run_id"] != run_id or
            status["state"] not in STATE_NAMES or
            any(status[key] > SAMPLE_COUNT for key in ("received", "last", "missing")) or
            status["elapsed_ms"] > SAMPLE_WINDOW_MS or
            status["remaining_ms"] > SAMPLE_WINDOW_MS or status["shadow_expired"] not in (0, 1)):
        return None
    received, last, state = status["received"], status["last"], status["state"]
    elapsed, remaining = status["elapsed_ms"], status["remaining_ms"]
    expected_missing = (SAMPLE_COUNT if state == 3 else last) - received
    if (received > last or (received == 0) != (last == 0) or
            status["missing"] != expected_missing or
            (received > 0 and remaining != SAMPLE_WINDOW_MS - elapsed) or
            (received == 0 and (elapsed != 0 or remaining != 0)) or
            (state in (0, 1, 6) and received != 0) or
            (state == 2 and (received == 0 or elapsed >= SAMPLE_WINDOW_MS)) or
            (state == 3 and (received == 0 or elapsed != SAMPLE_WINDOW_MS or remaining != 0))):
        return None
    return status


class Client:
    """A single run, with no retransmission or per-tick acknowledgement path."""
    def __init__(self, transport, *, clock=time.monotonic, sequence=None, run_id=None):
        self.transport, self.clock = transport, clock
        # Reserve begin + 200 ticks + two final queries without sequence wrap.
        self.sequence = _uint(secrets.randbelow(60000) + 1 if sequence is None else sequence,
                              65535 - SAMPLE_COUNT - 3, "initial sequence")
        self.run_id = _uint(secrets.randbelow(UINT32_MAX) + 1 if run_id is None else run_id,
                            UINT32_MAX, "run ID", 1)
        self._begun = False
        self._armed = False
        self._last_index = 0
        self.board = None
        self.stats = {"sent": 0, "skipped": 0, "max_gap_us": 0,
                      "discarded_frames": 0, "status_queries": 0}

    def _send(self, operation, index=None):
        if operation not in ALLOWED_OPERATIONS:
            raise ValueError("Only observational lprobe operations are permitted")
        if operation == "lprobe_tick":
            _uint(index, SAMPLE_COUNT, "tick index", 1)
        elif index is not None:
            raise ValueError("Only a tick accepts an index")
        if self.sequence >= 65535:
            raise ProtocolError("Request sequence exhausted; no wrap or retry")
        self.sequence += 1
        wire = f"cfg:{operation},{self.sequence},{self.run_id}"
        if index is not None:
            wire += f",{index}"
        self.transport.send((wire + "\n").encode("ascii"))
        return self.sequence

    def _request(self, operation):
        sent_at = self.clock()
        sequence = self._send(operation)
        deadline = sent_at + REPLY_TIMEOUT_SECONDS
        while self.clock() < deadline:
            raw = self.transport.receive(max(0.0, deadline - self.clock()))
            if self.clock() >= deadline:
                break  # A late reply cannot retrospectively authorize sampling.
            parts = _response_parts(raw)
            if (parts is not None and len(parts) >= 3 and
                    parts[:2] == [f"rsp:{sequence}", "err"] and all(parts[2:])):
                # Error replies have no OP/version/run fields. They may abort,
                # never arm a run or authorize ticks, and never trigger retries.
                raise ProtocolError("Firmware rejected " + operation + ": " + ",".join(parts[2:]))
            status = parse_response(raw, sequence=sequence, operation=operation, run_id=self.run_id)
            if status is not None:
                self.board = status
                return status
            if raw:
                self.stats["discarded_frames"] += 1
        raise ProtocolError(f"{operation} timed out; no retry")

    def begin(self):
        if self._begun:
            raise ProtocolError("Begin already attempted; no retry")
        self._begun = True  # Also prevents retry after send failure/uncertainty.
        result = self._request("lprobe_begin")
        if result["state"] != 1:
            raise ProtocolError("Begin did not arm: " + STATE_NAMES[result["state"]])
        self._armed = True
        return result

    def tick(self, index):
        if not self._armed:
            raise ProtocolError("No exact armed begin response; no ticks permitted")
        _uint(index, SAMPLE_COUNT, "tick index", 1)
        if index <= self._last_index:
            raise ProtocolError("Tick index already attempted; no retry or reordering")
        self._last_index = index
        self._send("lprobe_tick", index)
        self.stats["sent"] += 1

    def status(self):
        if not self._armed or self.stats["status_queries"] >= 2:
            raise ProtocolError("No additional status query permitted")
        self.stats["status_queries"] += 1
        return self._request("lprobe_status")

    def drain_until(self, deadline):
        while self.clock() < deadline:
            raw = self.transport.receive(max(0.0, deadline - self.clock()))
            if raw:
                self.stats["discarded_frames"] += 1


def run_probe(client):
    """200 fixed scheduled slots; skipped host slots are never sent in a burst."""
    client.begin()
    origin = client.clock()
    index = 1
    last_sent = None
    while index <= SAMPLE_COUNT:
        scheduled = origin + (index - 1) * PERIOD_SECONDS
        # Preserve a minimum inter-send interval even after sub-period lateness.
        due = scheduled if last_sent is None else max(scheduled, last_sent + PERIOD_SECONDS)
        client.drain_until(due)
        now = client.clock()
        # Small tolerance only cancels binary floating-point boundary noise.
        late_slots = max(0, int((now - scheduled + 1e-9) / PERIOD_SECONDS))
        skipped = min(late_slots, SAMPLE_COUNT - index + 1)
        client.stats["skipped"] += skipped
        if last_sent is not None:
            # Include observed silence if a host stall consumes every remaining
            # slot, even though there will be no next tick to close that gap.
            client.stats["max_gap_us"] = max(client.stats["max_gap_us"],
                                             round((now - last_sent) * 1_000_000))
        index += skipped
        if index > SAMPLE_COUNT:
            break
        client.tick(index)  # Exactly once; never wait for a tick ACK.
        last_sent = now
        index += 1
    # Includes the fixed 10 s board window, which starts at its first valid tick.
    client.drain_until((client.clock() if last_sent is None else last_sent) + POST_SEND_WAIT_SECONDS)
    status = client.status()
    if status["state"] == 2:
        client.drain_until(client.clock() + status["remaining_ms"] / 1000.0 + 0.250)
        status = client.status()  # The only permitted read-only follow-up.
    if status["state"] != 3:
        raise ProtocolError("Probe did not complete: " + STATE_NAMES[status["state"]])
    return status


def _port(value):
    try:
        if not value.isascii() or not value.isdecimal():
            raise ValueError()
        return _uint(int(value), 65535, "port", 1)
    except ValueError as error:
        raise argparse.ArgumentTypeError("Port must be an integer in 1..65535") from error


def _address(value, *, peer=False):
    try:
        return ipv4_address(value, peer=peer)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def argument_parser():
    parser = argparse.ArgumentParser(description=__doc__, epilog=NOTICE)
    parser.add_argument("--host", default="192.168.0.111", type=lambda value: _address(value, peer=True))
    parser.add_argument("--port", default=5001, type=_port)
    parser.add_argument("--bind", default="0.0.0.0", type=_address)
    parser.add_argument("--local-port", default=8081, type=_port)
    parser.add_argument("--output", type=Path, help="Create a new JSON summary file; never overwrite")
    return parser


def summary(client, *, complete, error=None):
    return {"schema_version": 1, "observation_complete": complete, "powered_test_permission": False,
            "notice": NOTICE, "sample_count": SAMPLE_COUNT, "interval_ms": 50,
            "sample_window_ms": SAMPLE_WINDOW_MS, "run_id": None if client is None else client.run_id,
            "host": None if client is None else dict(client.stats,
                discarded_sources=getattr(client.transport, "discarded_sources", 0)),
            "board": None if client is None else client.board, "error": error}


def print_summary(result):
    host, board = result["host"], result["board"]
    if host is not None:
        print(f"host sent={host['sent']}/200 skipped={host['skipped']} "
              f"max_gap_us={host['max_gap_us']} discarded={host['discarded_frames']} "
              f"wrong_source={host['discarded_sources']}")
    if board is not None:
        print(f"board state={STATE_NAMES[board['state']]} received={board['received']}/200 "
              f"last={board['last']} missing={board['missing']} duplicate={board['duplicate']} "
              f"reordered={board['reordered']} invalid={board['invalid']}")
        print(f"board max_gap_us={board['max_gap_us']} mean_gap_us={board['mean_gap_us']} "
              f"gaps_ge200={board['gaps_ge200']} shadow_expired={board['shadow_expired']} "
              f"elapsed_ms={board['elapsed_ms']} remaining_ms={board['remaining_ms']} "
              f"rx_errors={board['rx_errors']} tx_drops={board['tx_drops']}")
    if result["error"]:
        print("Probe incomplete: " + result["error"], file=sys.stderr)
    print(NOTICE)


def main(argv=None, *, transport_factory=UDPTransport, clock=time.monotonic):
    parser = argument_parser()
    args = parser.parse_args(argv)  # Validate every argument before opening a socket.
    output = None
    if args.output is not None:
        try:
            output = args.output.open("x", encoding="utf-8")
        except OSError as error:
            parser.error(f"Cannot create summary file: {error}")
    transport = client = None
    code, error = 1, None
    try:
        transport = transport_factory(args.host, args.port, args.bind, args.local_port)
        client = Client(transport, clock=clock)
        run_probe(client)
        code = 0
    except KeyboardInterrupt:
        code, error = 130, "Interrupted; no retries or stop command sent"
    except (OSError, ProtocolError, ValueError) as failure:
        error = str(failure)
    finally:
        # Observational probe has no STOP, keepalive, recovery, or actuation path.
        try:
            if transport is not None:
                transport.close()
        finally:
            result = summary(client, complete=code == 0, error=error)
            print_summary(result)
            if output is not None:
                try:
                    json.dump(result, output, indent=2, sort_keys=True, allow_nan=False)
                    output.write("\n")
                except OSError as failure:
                    print(f"Could not write summary: {failure}", file=sys.stderr)
                    code = 1
                finally:
                    output.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
