# Staged SAFE UART commissioning transport

This source adds one manual B/right-channel pulse path. Ordinary balance, Run,
Test, old Jog, `W_Motor_SetDuty` and `W_Motor_Release` remain zero-only. A/left
Yaw and the DRV8701E ground-wheel C remain disabled. It is not a balanced-vehicle
release and has not been flashed or electrically verified by these tests.

## Wire contract

Use the configured TC264 UART3, 460800 baud, P15.6 TX/P15.7 RX. SAFE requests
and legacy text requests have CRLF line endings. The bounded version1 commands:

- `SAFE-STATUS,1`: observation only.
- `SAFE-ZERO,1,session`: consume the current locally granted session into ZEROED.
- `SAFE-ARM,1,session`: requires positive ZEROED status first; still zero output.
- `SAFE-DUTY,1,session,seq,0,right`: A is exactly0; signed raw B is within±500
  of the driver's10000 full scale. First sequence1, subsequent sequence increases.
- `SAFE-DISARM,1`: cancel authority and inhibit output.

Full status is `SAFE-STATUS,1,state,session,last_seq,reason,1`. The final1 is
capability1, specifically this bounded right-only bench mode. State values are
0 DISARMED,1 LOCAL_READY,2 ZEROED,3 ARMED,4 LATCHED. A valid old text/binary RPM
reply never supplies identity, capability, a session, or permission to actuate.

Driver reasons:0 none,1 boot,2 manual/disarm,3 command timeout,4 local grant
window expired,5 pulse limit,6 invalid stored calibration,7 motor/battery fault,
8 sequence exhausted,9 main heartbeat timeout,10 control ISR heartbeat timeout.
Host reason codes are separate and defined in `W_Motor.h`.

A fresh driver-board KEY1 release/press gives LOCAL_READY. Query status before
issuing a manual pulse and echo that exact nonzero session in the controller
request. The host rejects stale/mismatched sessions, missing fields, wrong
version/capability, extra fields, overflow, and nonzero sequence in ready/zeroed
states. Grants are one-shot. There is no reconnect retry or automatic rearming.
Session numbers are monotonic only within one driver boot and can repeat after
power reset. They are not authentication or cross-reboot replay protection.

## Controller integration contract

`W_Motor_BenchRequestB(raw_duty,duration_ms,expected_session)` accepts only
nonzero raw duty within±500 and duration1..150ms, with fresh LOCAL_READY status.
It queues ZERO only. `W_Motor_BenchService()` may advance to ARM and then DUTY
only after the controller's current sensor, state and scheduling checks pass.
Both positive ZEROED and ARMED acknowledgments must belong to the same session.
Each handshake stage and status/sequence progress is bounded to60ms. Active
commands are sent every20ms with increasing sequence numbers.

`W_Motor_Tick1ms()` may observe or abort, never create an ARM or nonzero duty.
The TX IRQ may finish previously authorized queued bytes. Pulse expiry uses a
64-bit STM timestamp, not a count of control ticks; duration cannot exceed150ms.
The driver separately enforces200 serviced safety-ISR ticks for its pulse cap
and100 ticks for its command watchdog, nominally200ms and100ms. Missed/delayed
PIT ticks extend wall-clock time, so these are not hard hardware deadlines. Host and driver supervision are complementary, not proof against a
complete hardware clock/CPU failure.

During a valid dedicated bench branch, force C/PWM and public ordinary control
commands to zero, but do not call the W ordinary zero/stop path. Outside that
branch use `W_Motor_KeepZero()` for idle observation: it preserves LOCAL_READY
without granting authority. A call to it during an active bench is an abort.
Explicit Stop always calls `W_Motor_Stop()` and cancels the grant.

Stop replaces unsent queued commands with priority DISARM then legacy zero.
It cannot retract a partially transmitted line or bytes already in the UART
FIFO/on the wire. The current line must finish before emergency DISARM can be
parsed. Therefore150ms is the host stop-request deadline, not a measured
physical gate-off deadline. Use a physical power cutoff and restrained, unloaded
bench setup for commissioning; software Stop is not a safety-rated emergency stop.

## Verification included

- `python tests/test_wmotor_safe_bench.py`: six real-source host test groups,
  including text/binary zero-only ordinary paths, strict parsing, stale and
  wrong sessions, positive acknowledgments, missed ticks, lower STM wrap,
  deadline, link/sequence failures, pending-command stop priority and no rearm.
- `python tests/test_wmotor_driver_integration.py`: byte-for-byte integration of
  delivered TC264 transport, CYT2BL3 parser/safety code and real PWM gate
  functions, with mocked hardware, under AddressSanitizer/UndefinedBehaviorSanitizer.
- `python tests/test_external_driver_safety.py`: driver-side source/gate tests.

The host GCC object uses approximately400bytes static storage, no heap. This
is only a host-side size indicator; it does not replace the final TASKING linker
map or worst-case stack/interrupt-latency measurement. Target firmware build,
real UART timing, electrical polarity, wiring, effective gate inhibition and
motor direction still require controlled hardware verification.
