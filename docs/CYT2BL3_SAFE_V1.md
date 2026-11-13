# CYT2BL3 commissioning driver: SAFE v1, capability 1

This source is the **matching driver for a short right/B pulse commissioning test**, not a rideable/balancing firmware release. It does not prove which image is installed on an existing driver board. No board was flashed, connected, or actuated while preparing this patch.

## Fixed scope

- Only the existing FAST_FOC + UART configuration is supported. Changing either default causes a compile-time error.
- A/left is permanently inhibited at its physical PWM pin mux. It never receives a nonzero command.
- B/right command range is -500 to +500 on the existing -10000…10000 scale. Values outside the range, or nonzero A, are rejected, not clamped or acknowledged as accepted.
- The nonzero pulse window is 200 safety-ISR ticks, nominally 200 ms, from the first accepted nonzero command. Sending zero or changing the command does not restart that window. The matching TC264 host has its separate 150 ms deadline. The driver value is not a hard wall-clock guarantee: missed/delayed PIT ticks extend real elapsed time.
- Command timeout is 100 safety-ISR ticks, nominally 100 ms. Status, malformed commands, repeated sequences and other traffic never refresh it.
- The command cap is **not a measured current, speed, power or phase-PWM limit**. The existing FAST_FOC calculation, protections and back-EMF feedforward remain in place.
- Original RA/EKF and TC264 motor gates are outside this driver patch. Existing calibration contents are read-only and are never migrated or overwritten.

## Wire format

460800 baud, existing UART pins, ASCII. Requests require exactly CRLF, with no spaces, extra fields or prefix/suffix text. The receive frame is limited to 95 bytes including CRLF; one byte is reserved for its NUL terminator in the existing 128-byte buffer. Overlong, binary/control-byte, embedded-NUL or expired partial frames are discarded through the next LF. A partial frame expires after 100 ms; it cannot be completed later as a new valid frame.

Requests:

- `SAFE-STATUS,1\r\n`
- `SAFE-ZERO,1,<session>\r\n`
- `SAFE-ARM,1,<session>\r\n`
- `SAFE-DUTY,1,<session>,<sequence>,0,<right>\r\n`
- `SAFE-DISARM,1\r\n`

Response:

`SAFE-STATUS,1,<state>,<session>,<last_sequence>,<reason>,1\r\n`

The final `1` is capability 1: right-only, command magnitude at most 500, nonzero pulse window at most 200 ms, command timeout 100 ms, fresh physical permission plus zero then arm required. A host must match the version and this capability before starting the handshake. Speed telemetry alone is not capability evidence. Replies are sent from the main loop on state changes, queries and every 20 ms while in states 1–3. The line is at most 44 bytes with full-width fields and the current two-digit reason range, below its 64-byte stack buffer.

Session and sequence are canonical unsigned decimal uint32 fields: no sign, whitespace or leading zeros except the number zero itself. Sessions must be nonzero to arm/command. Sequence starts at 1 and must strictly increase; duplicates and backwards sequences do not refresh the command timeout. Sequence UINT32_MAX immediately exhausts/latches the session instead of allowing a wrap. Signed duty accepts an optional minus, not plus, and rejects negative zero. Physical permission expires after 5000 ms whether or not ZERO was received. An accepted ARM begins the 100 ms command timeout with zero outputs.

States:

| Value | State |
|---|---|
| 0 | Disarmed |
| 1 | Fresh local permission, waiting for zero |
| 2 | Zero accepted, waiting for explicit arm |
| 3 | Armed, still gated until an accepted nonzero right command |
| 4 | Latched, new local permission required |

Reasons:

| Value | Reason |
|---|---|
| 0 | None |
| 1 | Boot |
| 2 | Explicit disarm/manual stop/legacy motor command |
| 3 | Command timeout, 100 ms |
| 4 | Local permission/arm window expired, 5000 ms |
| 5 | Nonzero pulse window exhausted, 200 ms |
| 6 | Invalid/missing stored calibration |
| 7 | Existing motor, encoder or battery protection fault |
| 8 | Session/sequence counter exhausted |
| 9 | Main loop heartbeat absent for 100 ms |
| 10 | Either FOC ISR heartbeat absent for 10 ms |

A stop invalidates the session and returns sequence to zero. State 0 and 4 cannot be armed remotely. A stale command or reconnect is never a reason to grant permission.

## Deliberate local permission and compatibility changes

KEY_1/P06_6 no longer calibrates the motor. It is a local permission/stop button. The driver must observe two released samples followed by two pressed samples, sampled from the existing 10 ms main loop. A button held at boot or held through a timeout cannot rearm. A fresh press while armed disarms instead of granting another session.

After the local event, the matching host must explicitly request zero for that session, observe state 2, explicitly request arm, observe state 3, and only then transmit its bounded right pulse. At least 20 dedicated PIT ticks must have occurred before ARM can succeed. The host must not automate a new zero/arm handshake following timeout, reconnect or driver reboot.

Legacy `SET-DUTY,0,0` is harmless and preserves a pending local/zeroed grant, but stops an already armed session. A fully valid legacy nonzero SET-DUTY stops/latches permission and never actuates. Invalid legacy frames do not renew the watchdog. Legacy binary commands never actuate. Remote SET-ZERO, phase tests and calibration commands are inert. Existing exact read-only telemetry commands require CRLF. The matching host now emits CRLF for those commands as well.

All calibration entry points, phase-test and motor-music functions are inert in this commissioning image. Flash read checks magic and plausible zero locations, pole-pair counts and the original signed direction values +1/-1. Invalid data inhibits arm and avoids calling FAST_FOC initialization with invalid divisors. A valid magic/range check does not certify motor calibration accuracy. Calibration writes are disabled, including the old invalid-magic fallback that wrote defaults. A separate, reviewed calibration workflow is required if stored data is invalid; this patch does not calibrate automatically.

## Output and timing implementation

`driver_safety.inc` is included by the already-built UART source, so an existing IDE source list does not need an extra C translation unit. `driver_safety.h` declares the small adapter interface. The parser reuses the existing 128-byte read buffer, removes the old 128-byte FIFO buffer and does not allocate a new target receive queue. Status formatting uses a 64-byte stack buffer.

After clock initialization, safety initialization drives all 12 MOS control pins to GPIO-low before debug, ADC, flash and encoder initialization. PWM initialization keeps those pins in GPIO mode while the counters continue running for encoder sampling. Only the right/B pins can be restored to PWM, and only inside the final right duty writer with active permission, a fresh command and a bounded right command. Stop sets commanded duties and active/buffer compares to zero and returns all pins to GPIO-low. Short critical sections prevent a timeout/command race from restoring an output after a stop.

The CM4 priority grouping is explicitly set to zero (preemption bits). Dedicated PIT_CH0 uses CPUIntIdx2 priority 0. The old PIT0 handler's wrong CH1 clear was fixed. Both PWM/FOC interrupts are priority 1 and UART CPUIntIdx7 is priority 2. This lets the safety PIT interrupt an encoder/SPI polling stall in a PWM ISR. Its heartbeats, command age and pulse window are independent of main-loop/UART progress. `safe_ms` counts serviced PIT interrupts, not a free-running hardware timestamp: an interrupt held pending for several periods contributes only one tick when serviced. All driver millisecond thresholds in this document are nominal tick-based thresholds. Missed or delayed ticks can lengthen actual elapsed pulse/timeout durations, so these values are not hard wall-clock guarantees. The old duty-loss counters remain as a redundant saturating check with an FPWM-derived threshold, but are not the permission authority.

## Verification performed

Run from the commissioning directory:

`python3 tests/test_external_driver_safety.py`

The test is also discoverable by pytest where installed. It compiles the actual delivered UART parser and included state machine, and extracts the exact final PWM/gate functions from the delivered driver source into mocked registers. GCC uses `-Wall -Wextra -Werror`, AddressSanitizer and UndefinedBehaviorSanitizer. LeakSanitizer is disabled because this host runs under ptrace; no dynamic allocation is used by the driver under test.

Covered: malformed/overlong/out-of-range/partial/binary/NUL commands; strict CRLF; exact left/right field routing; permanently inhibited A; right cap rejection; default startup and held-key refusal; zero-before-arm; session/sequence replay; fresh local permission after timeout; reboot/reconnect without automatic arm; non-restarting tick-based pulse cap despite refresh/zero; status not refreshing duty; main/control heartbeat failures; disarm/legacy stop; direct PWM gate denial; and source assertions for startup order, priorities, disabled calibration/music/alternate modes. A separate test compiles the actual flash reader/write stub and verifies valid forward/reverse calibration, invalid record rejection and no record modification.

A further reproducible check, `python3 tests/test_external_driver_sdk_syntax.py`, parses all eight complete modified C translation units with the bundled real SDK headers: main_cm4, cm4_isr, motor_control, pwm_output, motor_driver_uart_control (including driver_safety.inc), motor_flash, motor_voice and driver_gpio. Native GCC syntax-only checks passed with missing API declarations, incompatible pointer types and implicit integer conversions treated as errors. Selectors follow the bundled CYT2BL3CAE/TVIIBE4M CM4 headers; native 64-bit pointer-size warnings are suppressed. No mock SDK headers are used in this check.

These are source/mock and native syntax/API tests, not a target build or hardware test. The supplied source subset does not include an IAR project/build configuration or installed CYT2BL3 toolchain in this workspace. ARM assembly, target code generation/ABI, linking, reset/startup files, RAM/stack placement, interrupt routing and timing remain unverified on the target.

## Required bench qualification before any powered pulse

- Confirm the exact driver image and capability response, actual B/right wiring, calibration and wheel restraint; use an independent physical power cutoff.
- Verify with gate-drive measurements that GPIO-low actually disables all MOS paths for this board, including active-low/inverting driver possibilities. GPIO register mocks do not prove electrical off-state. Verify all left pins remain inhibited.
- Verify reset/bootloader/power-up pin behavior before `main`, arm/zero transitions, timeout, physical stop and queued-command replay. Confirm there is no startup pulse or stale compare pulse when reconnecting right PWM pins.
- Measure PIT period, priority preemption, command/ISR/main timeouts, pulse cap and worst-case IRQ-masked intervals. A blocked FOC SPI read must not prevent the independent PIT from cutting the gates.
- A CPU hang, globally masked interrupts, failed PIT clock or wrong interrupt routing can defeat the software timer. The included SDK offers `Cy_WDT_Init`, `Cy_WDT_SetUpperLimit`, reset action, `Cy_WDT_ClearWatchdog` and `Cy_WDT_Enable`, but an independent watchdog/reset/kill path is **not configured or qualified by this patch**. The SDK's nominal 32000-tick default is about one second and does not itself establish the required bound. Qualify its independent clock, reset action and gate behavior before treating this as a safety-rated stop.
- Session counters prevent old-session replay within one boot. They are not cryptographic nonces and can repeat after reboot. Fresh physical permission is mandatory after every boot/stop; this is not protection against a malicious active UART participant or cross-boot replay after a new local grant. A qualified entropy source or persistent monotonic boot epoch would be required for that stronger property.

Until those checks pass, keep motor power isolated. Do not infer production/riding readiness from these host tests or from receiving a status line.
