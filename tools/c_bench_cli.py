#!/usr/bin/env python3
"""Explicit C-wheel bench client. UDP only; importing/connecting never starts.

Default endpoint is 192.168.0.111:5001, bound locally on UDP 8081. All endpoints
are configurable. A reply is not evidence of physical power isolation or stop.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
from keep_wire_log import WireLog, datagram_reason
import ipaddress
import math
import secrets
import socket
import sys
import time
from typing import Callable


class ProtocolError(RuntimeError):
    pass


MODES = {"dry_raw": 0, "dry_rate": 1, "signal_raw": 2, "signal_rate": 3}
MODE_NAMES = {value: key for key, value in MODES.items()}
PROFILES = {0: "calculation-only", 1: "signal-diagnostic"}
# Order is versioned by the firmware. No field is inferred from the request.
STATUS_FIELDS = (
    "version", "active", "mode", "state", "reason", "sensor_reason",
    "session", "keep_seq", "elapsed_ms", "heartbeat_age_ms", "imu_age_ms",
    "duration_ms", "limit", "target", "kp", "ki", "angle_deg", "rate_dps",
    "encoder_20ms", "calculated", "applied", "profile",
)
INTEGER_FIELDS = {"version", "active", "mode", "reason", "session", "keep_seq",
                  "elapsed_ms", "encoder_20ms", "applied", "profile", "state", "sensor_reason",
                  "heartbeat_age_ms", "imu_age_ms", "duration_ms", "limit"}


class UDPTransport:
    """One UDP peer, complete newline-terminated frames, no discovery/reconnect."""
    def __init__(self, host="192.168.0.111", port=5001, bind="0.0.0.0", local_port=8081):
        self.audit = None
        self.last_rx_id = None
        self.peer = (str(ipaddress.IPv4Address(host)), port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind((str(ipaddress.IPv4Address(bind)), local_port))
        except BaseException:
            self.socket.close()
            raise

    def send(self, data):
        if self.socket.sendto(data, self.peer) != len(data):
            raise ProtocolError("Partial UDP send; outcome uncertain, no retry")

    def receive(self, timeout):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return b""
            self.socket.settimeout(remaining)
            try:
                raw, source = self.socket.recvfrom(65535)
            except socket.timeout:
                return b""
            self.last_rx_id = self.audit.add("rx_datagram", raw=raw, source=list(source)) if self.audit else None
            if source != self.peer:
                if self.audit: self.audit.add("rx_filtered", rx_id=self.last_rx_id, reason="WRONG_SOURCE")
                continue
            # Firmware emits exactly one complete response line per datagram.
            if not raw.endswith(b"\n") or raw.count(b"\n") != 1 or len(raw) > 511:
                if self.audit: self.audit.add("rx_filtered", rx_id=self.last_rx_id, reason=datagram_reason(raw))
                continue
            return raw

    def close(self):
        self.socket.close()


class Client:
    def __init__(self, transport, *, timeout=.12, clock: Callable[[], float] = time.monotonic,
                 sequence=None):
        if not math.isfinite(timeout) or not 0 < timeout <= .15:
            raise ValueError("timeout must be >0 and <=0.15 seconds")
        self.transport, self.timeout, self.clock = transport, timeout, clock
        self.audit = None
        self.last_rx_id = None
        self.seq = secrets.randbelow(59000) + 1 if sequence is None else sequence
        if not isinstance(self.seq, int) or not 0 <= self.seq < 65535:
            raise ValueError("Invalid initial sequence")

    def send_request(self, operation, *fields):
        if operation not in ("cbench_status", "cbench_start", "cbench_keep", "cbench_stop"):
            raise ValueError("Only C bench operations are permitted")
        if self.seq >= 65535:
            raise ProtocolError("Configuration sequence exhausted; no wrap/retry")
        pieces = [str(f) for f in fields]
        if any(not x or any(ch in x for ch in ",\r\n") for x in pieces):
            raise ValueError("Invalid command field")
        seq = self.seq + 1
        wire = (f"cfg:{operation},{seq}" + "".join("," + x for x in pieces) + "\n").encode("ascii")
        if len(wire) > 191:
            raise ValueError("Oversized command")
        self.seq = seq
        sent_at = self.clock()
        if self.audit:
            self.audit.add("tx_attempt", raw=wire, cfg_seq=seq, operation=operation,
                           session=pieces[0] if operation=="cbench_keep" else None,
                           keep_seq=pieces[1] if operation=="cbench_keep" else None)
        try:
            self.transport.send(wire)  # Exactly once. Never retransmit mutations.
        except BaseException as error:
            if self.audit: self.audit.add("tx_failed", cfg_seq=seq, error=str(error))
            raise
        if self.audit: self.audit.add("tx_sent", cfg_seq=seq, operation=operation)
        return seq, sent_at

    def request(self, operation, *fields, timeout=None):
        seq, sent_at = self.send_request(operation, *fields)
        deadline = sent_at + (self.timeout if timeout is None else timeout)
        while self.clock() < deadline:
            raw = self.receive(max(0, deadline - self.clock()))
            if self.clock() > deadline:
                self.filtered(raw, "REPLY_AFTER_DEADLINE", cfg_seq=seq)
                break  # A late ACK cannot retroactively satisfy this request.
            status = self.decode_checked(raw, seq, operation)
            if status is not None:
                return status
        if self.audit: self.audit.add("request_timeout", cfg_seq=seq, operation=operation)
        raise ProtocolError(f"{operation} timed out; outcome unknown, no retry")

    def receive(self, timeout):
        raw = self.transport.receive(timeout)
        if raw:
            if (getattr(self.transport, "audit", None) is self.audit and self.audit is not None
                    and getattr(self.transport, "last_rx_id", None) is not None):
                self.last_rx_id = self.transport.last_rx_id
            elif self.audit:
                self.last_rx_id = self.audit.add("rx_datagram", raw=raw, source=None)
        elif self.audit:
            self.audit.add("receive_timeout", wait_seconds=timeout)
        return raw

    def filtered(self, raw, reason, **fields):
        if self.audit and raw:
            self.audit.add("rx_filtered", rx_id=self.last_rx_id,
                           reason=datagram_reason(raw) or reason, **fields)

    def decode_checked(self, raw, seq, operation):
        try:
            status = self.decode_reply(raw, seq, operation)
        except ProtocolError as error:
            self.filtered(raw, "MATCHING_REPLY_REJECTED", cfg_seq=seq, detail=str(error))
            raise
        if status is None:
            self.filtered(raw, "CFG_SEQUENCE_MISMATCH", expected_cfg_seq=seq)
        elif self.audit:
            self.audit.add("ack_matched", rx_id=self.last_rx_id, cfg_seq=seq,
                           operation=operation, session=status["session"], keep_seq=status["keep_seq"])
        return status

    @staticmethod
    def decode_reply(raw, seq, operation):
        if len(raw) > 511:
            return None
        if not raw or not raw.endswith(b"\n") or raw.count(b"\n") != 1:
            return None
        try:
            text = raw.decode("ascii").rstrip("\r\n")
        except UnicodeDecodeError:
            return None
        parts = text.split(",")
        if parts[0] != f"rsp:{seq}":
            return None  # Old status or duplicated previous ACK cannot confirm this request.
        if len(parts) < 3 or parts[1] not in ("ok", "err"):
            raise ProtocolError("Malformed matching response; no retry")
        if parts[1] == "err":
            raise ProtocolError("Firmware rejected request: " + ",".join(parts[2:]))
        if parts[2] != operation:
            raise ProtocolError("Matching sequence has wrong operation; no retry")
        return parse_status(parts[3:])


def parse_status(fields):
    if len(fields) != len(STATUS_FIELDS):
        raise ProtocolError("Unsupported C bench status schema")
    try:
        out = {key: (int(value) if key in INTEGER_FIELDS else float(value))
               for key, value in zip(STATUS_FIELDS, fields)}
        if not all(math.isfinite(value) for value in out.values()):
            raise ValueError("Nonfinite telemetry")
        if out["version"] != 1 or out["active"] not in (0, 1) or out["mode"] not in MODE_NAMES:
            raise ValueError("Unknown state/mode/version")
        if out["profile"] not in PROFILES:
            raise ValueError("Unknown output profile")
        if not 0 <= out["session"] <= 0xffffffff or not 0 <= out["keep_seq"] <= 0xffffffff:
            raise ValueError("Bad session/keep sequence")
        if out["reason"] not in range(13) or out["state"] not in range(4) or not 0 <= out["elapsed_ms"] <= 0xffffffff:
            raise ValueError("Bad reason/time")
        if out["active"] and (not 1 <= out["duration_ms"] <= 3000 or not 1 <= out["limit"] <= 500):
            raise ValueError("Invalid active duration/limit")
        if (abs(out["applied"]) > 500 or
                ((out["profile"] == 0 or out["mode"] in (0, 1)) and out["applied"] != 0) or
                (out["limit"] > 0 and abs(out["applied"]) > out["limit"])):
            raise ValueError("Unexpected applied output")
        return out
    except (ValueError, OverflowError) as error:
        raise ProtocolError("Invalid C bench status: " + str(error)) from error


def validate_run(*, profile, mode, target, kp, ki, duration_ms, limit, isolated=False):
    if profile not in PROFILES.values() or mode not in ("raw", "rate"):
        raise ValueError("Explicit known --profile and --mode are required")
    if not all(math.isfinite(x) for x in (target, kp, ki)):
        raise ValueError("Target and gains must be finite")
    if (type(duration_ms) is not int or type(limit) is not int or
            not 1 <= duration_ms <= 3000 or not 1 <= limit <= 500):
        raise ValueError("Duration must be 1..3000ms and PWM limit 1..500")
    if mode == "raw" and (abs(target) > limit or kp != 0 or ki != 0):
        raise ValueError("Raw mode requires |target| <= limit, kp=0, ki=0")
    if mode == "rate" and (abs(target) > 30 or not 0 <= kp <= 100 or not 0 <= ki <= 1000):
        raise ValueError("Rate target must be +/-30deg/s, kp0..100, ki0..1000")
    if profile == "signal-diagnostic" and not isolated:
        raise ValueError("Signal diagnostic requires --ack-isolated-signal (motor power disconnected)")
    return ("signal_" if profile == "signal-diagnostic" else "dry_") + mode


def dry_heartbeats(client, *, session, started_at, duration_ms, matches, emit, clock):
    """Calculation-only: send each keep once on schedule; correlate ACKs separately.

    At most three requests may be in flight. A correlated, fully validated newer
    keep ACK supersedes older keeps: it proves the current session/configuration
    is active, without asserting that the older packets arrived. No progress
    within the oldest pending request's 120ms deadline still stops the run.
    Firmware lease remains 200ms; signal-diagnostic rules remain unchanged.
    """
    period, ack_timeout = .05, .12
    deadline = started_at + duration_ms / 1000
    next_keep = started_at + period
    heartbeat = 0
    pending = {}
    while True:
        now = clock()
        for seq, (keep, sent_at) in pending.items():
            if now >= sent_at + ack_timeout:
                if client.audit: client.audit.add("ack_timeout", cfg_seq=seq, keep_seq=keep, deadline_ns=round((sent_at+ack_timeout)*1e9))
                raise ProtocolError(f"dry keep #{keep} ACK timed out (120ms); stopping, no retry")
        if now >= deadline:
            return
        if now >= next_keep:
            if now - next_keep >= period:
                raise ProtocolError("Keep schedule missed; stopping, no burst catch-up")
            if len(pending) >= 3:
                raise ProtocolError("Too many unconfirmed dry keeps; stopping")
            heartbeat += 1
            seq, sent_at = client.send_request("cbench_keep", session, heartbeat)
            pending[seq] = (heartbeat, sent_at)
            next_keep += period
        wake = min(deadline, next_keep, *(sent + ack_timeout for _, sent in pending.values()))
        raw = client.receive(max(0.0, wake - clock()))
        received_at = clock()
        matched = False
        for seq, (keep, sent_at) in tuple(pending.items()):
            if raw.split(b",",1)[0] != f"rsp:{seq}".encode("ascii"): continue
            matched = True
            status = client.decode_checked(raw, seq, "cbench_keep")
            if status is None:
                continue
            if received_at >= sent_at + ack_timeout:
                client.filtered(raw,"ACK_AFTER_120MS",cfg_seq=seq,keep_seq=keep)
                raise ProtocolError(f"dry keep #{keep} ACK arrived late; stopping, no retry")
            if (status["profile"] != 0 or status["session"] != session or
                    status["keep_seq"] != keep or not matches(status)):
                client.filtered(raw,"SESSION_OR_CONFIGURATION_MISMATCH",cfg_seq=seq,keep_seq=keep)
                raise ProtocolError("Keep acknowledgement does not match current run")
            if not status["active"]:
                raise ProtocolError(f"Firmware ended run early (reason={status['reason']})")
            if client.audit: client.audit.add("keep_validated", cfg_seq=seq, keep_seq=keep, session=session, rx_id=client.last_rx_id)
            # A newer accepted heartbeat proves current lease renewal even if
            # an earlier UDP request/ACK was lost. Never renew on an unrelated,
            # duplicate, wrong-session, or malformed response.
            for older_seq, (older_keep, _) in tuple(pending.items()):
                if older_keep <= keep:
                    if client.audit: client.audit.add("pending_retired", cfg_seq=older_seq, keep_seq=older_keep, confirmed_by_cfg_seq=seq, reason="ACKED" if older_seq==seq else "SUPERSEDED_BY_NEWER_ACK")
                    del pending[older_seq]
            emit("keep", status)
            break
        if raw and not matched: client.filtered(raw,"NO_PENDING_SEQUENCE")


def run(client, *, profile, mode, target, kp, ki, duration_ms, limit, isolated=False,
        emit=lambda event, status: None, clock=time.monotonic, sleep=time.sleep,
        preflight_timeout=None):
    """Preflight read, one start, monotonic keep, one stop even after uncertainty."""
    wire_mode = validate_run(profile=profile, mode=mode, target=target, kp=kp, ki=ki,
                             duration_ms=duration_ms, limit=limit, isolated=isolated)
    if preflight_timeout is not None and (
            not math.isfinite(preflight_timeout) or not 0 < preflight_timeout <= 5.0):
        raise ValueError("preflight-timeout must be >0..5s")
    # This read is before any start attempt. Control ACK deadlines are unchanged.
    preflight = client.request("cbench_status", timeout=preflight_timeout)
    emit("preflight", preflight)
    if PROFILES[preflight["profile"]] != profile:
        raise ProtocolError("Firmware profile differs from explicit --profile; no start sent")
    if preflight["active"]:
        raise ProtocolError("C bench already active; stop explicitly first, no start sent")
    if preflight["session"] == 0xffffffff:
        raise ProtocolError("Session exhausted; no start sent")
    def configuration_matches(status):
        return (status["mode"] == MODES[wire_mode] and PROFILES[status["profile"]] == profile and
                status["duration_ms"] == duration_ms and status["limit"] == limit and
                all(math.isclose(status[key], value, rel_tol=1e-6, abs_tol=1e-6)
                    for key, value in (("target", target), ("kp", kp), ("ki", ki))))
    attempted = False
    failed = False
    try:
        attempted = True
        started_at = clock()  # Duration includes awaiting start ACK, never extends on keep.
        current = client.request("cbench_start", preflight["session"], wire_mode,
                                 format(target, ".9g"), format(kp, ".9g"), format(ki, ".9g"),
                                 duration_ms, limit)
        session = preflight["session"] + 1
        if (current["session"] != session or not configuration_matches(current) or not current["active"] or
                current["keep_seq"] != 0):
            raise ProtocolError("Start acknowledgement does not match session/configuration/profile")
        emit("started", current)
        if profile == "calculation-only":
            dry_heartbeats(client, session=session, started_at=started_at,
                           duration_ms=duration_ms, matches=configuration_matches,
                           emit=emit, clock=clock)
        else:
            deadline = started_at + duration_ms / 1000
            next_keep = started_at + .05
            heartbeat = 0
            while clock() < deadline:
                sleep(max(0, min(next_keep, deadline) - clock()))
                if clock() >= deadline:
                    break
                heartbeat += 1
                current = client.request("cbench_keep", session, heartbeat, timeout=.04)
                if (current["session"] != session or current["keep_seq"] != heartbeat or
                        not configuration_matches(current)):
                    raise ProtocolError("Keep acknowledgement does not match current run")
                emit("keep", current)
                if not current["active"]:
                    raise ProtocolError(f"Firmware ended run early (reason={current['reason']})")
                next_keep += .05
                if next_keep < clock():
                    raise ProtocolError("Keep schedule missed; stopping, no burst catch-up")
    except BaseException:
        failed = True
        raise
    finally:
        if attempted:
            try:
                final = client.request("cbench_stop")  # Once, including start timeout/Ctrl-C.
                emit("stopped", final)
                if final["active"] or final["applied"] != 0:
                    raise ProtocolError("STOP replied with nonzero/active state")
            except BaseException as error:
                if not failed:
                    raise
                print(f"STOP unconfirmed: {error}; use independent physical cutoff", file=sys.stderr)


def print_status(event, status):
    s = status
    print(f"{event}: {PROFILES[s['profile']]} {MODE_NAMES[s['mode']]} active={s['active']} "
          f"session={s['session']} keep={s['keep_seq']} state={s['state']} reason={s['reason']} "
          f"sensor_reason={s['sensor_reason']} t={s['elapsed_ms']}ms/{s['duration_ms']}ms "
          f"keep_age={s['heartbeat_age_ms']}ms imu_age={s['imu_age_ms']}ms "
          f"kp={s['kp']:g} ki={s['ki']:g} limit={s['limit']} "
          f"target={s['target']:g} angle={s['angle_deg']:g}deg rate={s['rate_dps']:g}deg/s "
          f"encoder20ms={s['encoder_20ms']} calculated={s['calculated']:g} applied={s['applied']}",
          flush=True)


def build_parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="192.168.0.111", help="Actual module IPv4, configurable")
    p.add_argument("--port", type=int, default=5001, help="Module UDP port")
    p.add_argument("--bind", default="0.0.0.0", help="Local IPv4 interface")
    p.add_argument("--local-port", type=int, default=8081, help="Local UDP receive port")
    p.add_argument("--timeout", type=float, default=.12, help="Reply timeout >0..0.15s; no retries")
    p.add_argument("--csv", help="Optional local CSV trace; exclusive-create, never overwrite")
    p.add_argument("--wire-log", help="Raw packet JSONL; default: CSV basename + .wire.jsonl for runs")
    sub = p.add_subparsers(dest="operation", required=True)
    s = sub.add_parser("status", help="Read only; no start/keep/stop side effects")
    s.add_argument("--status-timeout", type=float, default=None,
                   help="Status-only reply timeout >0..5s; default: --timeout; no retries")
    sub.add_parser("stop", help="One explicit stop request, no retry")
    r = sub.add_parser("run", help="One bounded C run; never use with unqualified powered hardware")
    r.add_argument("--preflight-timeout", type=float, default=None,
                   help="Preflight status-only timeout >0..5s; default: --timeout; no retries")
    r.add_argument("--profile", required=True, choices=tuple(PROFILES.values()))
    r.add_argument("--mode", required=True, choices=("raw", "rate"))
    r.add_argument("--target", type=float, required=True, help="Raw PWM or Pitch Euler deg/s")
    r.add_argument("--kp", type=float, required=True)
    r.add_argument("--ki", type=float, required=True)
    r.add_argument("--duration-ms", type=int, required=True)
    r.add_argument("--limit", type=int, required=True, help="Absolute raw PWM limit 1..500")
    r.add_argument("--ack-isolated-signal", "--ack-power-disconnected", dest="ack_power_disconnected",
                   action="store_true", help="Motor power is physically disconnected; signal verification only")
    return p


def main(argv=None, *, transport_factory=UDPTransport):
    parser = build_parser()
    args = parser.parse_args(argv)
    params = {key: getattr(args, key) for key in
              ("profile", "mode", "target", "kp", "ki", "duration_ms", "limit")} if args.operation == "run" else {}
    try:
        ipaddress.IPv4Address(args.host)
        ipaddress.IPv4Address(args.bind)
        if not 1 <= args.port <= 65535 or not 0 <= args.local_port <= 65535:
            raise ValueError("Invalid UDP port")
        if not math.isfinite(args.timeout) or not 0 < args.timeout <= .15:
            raise ValueError("timeout must be >0..0.15s")
        if args.operation == "status" and args.status_timeout is not None:
            if not math.isfinite(args.status_timeout) or not 0 < args.status_timeout <= 5.0:
                raise ValueError("status-timeout must be >0..5s")
        if args.operation == "run" and args.preflight_timeout is not None:
            if not math.isfinite(args.preflight_timeout) or not 0 < args.preflight_timeout <= 5.0:
                raise ValueError("preflight-timeout must be >0..5s")
        if params:
            params["isolated"] = args.ack_power_disconnected
            validate_run(**params)
    except ValueError as error:
        parser.error(str(error))
    transport = None
    trace = None
    events = []
    audit = None
    wire_file = None
    try:
        if args.csv:
            trace = open(args.csv, "x", newline="", encoding="utf-8")
        writer = csv.DictWriter(trace, fieldnames=("host_monotonic_s", "event", *STATUS_FIELDS)) if trace else None
        if writer:
            writer.writeheader()
        def emit(event, status):
            # Never let console output or CSV flush delay a control heartbeat.
            # Runs last at most 3s, so this buffer holds at most 64 events.
            events.append((time.monotonic(), event, dict(status)))
        transport = transport_factory(args.host, args.port, args.bind, args.local_port)
        client = Client(transport, timeout=args.timeout)
        if args.operation == "run" or args.wire_log:
            wire_path = Path(args.wire_log) if args.wire_log else (Path(args.csv).with_suffix(".wire.jsonl") if args.csv else Path("cbench_wire_"+str(time.time_ns())+".jsonl"))
            wire_file = wire_path.open("x",encoding="utf-8")
            audit = WireLog(clock=client.clock)
            client.audit = transport.audit = audit
            print("Wire log: " + str(wire_path),flush=True)
        if args.operation == "run":
            print("No physical power isolation or hardware safety is inferred from a reply.", flush=True)
            print("Run telemetry is buffered and printed after STOP.", flush=True)
            run(client, **params, emit=emit, clock=client.clock, sleep=time.sleep,
                preflight_timeout=args.preflight_timeout)
        else:
            operation = "cbench_status" if args.operation == "status" else "cbench_stop"
            if args.operation == "status":
                result = client.request(operation, timeout=args.status_timeout)
            else:
                result = client.request(operation)
            emit(args.operation, result)
            if args.operation == "stop" and (result["active"] or result["applied"]):
                raise ProtocolError("Stop not confirmed by firmware; use physical cutoff")
        return 0
    except (ProtocolError, OSError, KeyboardInterrupt) as error:
        if audit: audit.add("run_error",error=str(error))
        print(f"ERROR: {error or 'Interrupted'}; no automatic retry or fallback start", file=sys.stderr)
        return 2
    finally:
        if transport:
            transport.close()
        if wire_file:
            try:
                if audit: audit.write(wire_file)
            finally: wire_file.close()
        try:
            for stamp, event, status in events:
                print_status(event, status)
                if trace:
                    writer.writerow({"host_monotonic_s": stamp, "event": event, **status})
        finally:
            if trace:
                trace.close()


if __name__ == "__main__":
    raise SystemExit(main())
