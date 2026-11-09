# Validation record

## Scope

Standalone PC tools only. No firmware modifications, hardware connection, motor actuation, flashing, parameter writes to real devices, or user-computer access occurred. All network tests used an ephemeral 127.0.0.1 fake board. Synthetic example data is not a vehicle measurement.

## Passed

- v1.1 Python standard-library test suite: 48 distinct tests, `python -m unittest discover -s tests -v`. Full output is in `validation/tests_v1_1.txt`.
- Ten new handshake tests cover the uploaded failed event sequence, no success without an exact requested ACK, an explicitly synthetic successful ACK tail, same-length old binary remaining UNMAPPED, an unknown third schema, reordered original fields, any old schema after exact ACK (including RATE acknowledgment), changes during capture, identical target announcements, repeated old announcements without retransmit/deadline extension, and no old exception when original already equals target.
- The uploaded log has 12 events, 0 DATA rows, and no target ACK. Original control messages are replayed in order; raw UNMAPPED bytes were absent from the log and are replaced with synthetic placeholders. The log alone correctly times out under v1.1; only the explicitly synthetic completed tail succeeds. This is not a claim of successful real-board capture.
- Python compilation: all package and test modules compile.
- Real UDP loopback integration: GET/SUB/RATE/STREAM handshake, distinct incoming data, stopped observation, exactly one allowlisted slider write, matching ACK, three post-ACK dedicated telemetry readbacks.
- Parameter bounds/finite values, rejected raw PWM/sign/mechanical-zero/position/unknown writes, cross-loop mismatch, and preview opening no socket.
- Corrupt packet/trailer, NaN, unknown/duplicate/oversized schema, wrong endpoint, same-length reordered schema, default shorter schema, original Mode10 seven-channel frames without telemetry_ms interleaved during GET metadata query, duplicate/reversed board time, burst delivery of old frames, active state and missing safety fields.
- ACK-before-data, data-before-ACK, missing ACK with changed RAM, wrong ACK name/value, zero-versus-tiny-nonzero and neighboring distinct float32 rejection, schema change/error after write, running state after write, and generic rate_kp incorrectly standing in for ground_rate_kp. Uncertain writes are never replayed.
- Three-loop offline metrics, encoder deduplication, invalid/stale encoder exclusion, calibration/fault/ramp filtering, board-time reset rejection, original-host JSONL and event-log import, and refusal to load pickle.
- Synthetic CLI examples for all three entry points generated JSON reports and well-formed SVG XML.
- Historical 892-row original-host CSV correctly returns INCOMPLETE rather than assuming absent control_mode, rate_enabled or pwm_debug_enabled.
- Historical v1.0 verification (not rerun for this PC handshake-only change): existing source regressions executed read-only against the inspected new checkout for generic positional/incremental PID, wheel speed hold, and wheel position hold. All three passed then. They compile real C against fake hardware boundaries, not on-target firmware.

## Not verified

- Actual board firmware identity, IP address, network route, serial/driver behavior, timing under WiFi loss, and real hardware safety.
- Windows/macOS execution and the user's installed Python version; tested in the provided Linux execution environment.
- GUI interoperability while both listeners are active: deliberately unsupported on the same UDP port. Close one listener first.
- Hardware outputs under menu open-loop previews: existing telemetry cannot establish those outputs. Operator must exit tests and isolate motor power.
- Full TC264 firmware build or full repository suite. Only the three named existing C regressions were run, and no firmware was changed.
- Browser rendering of generated SVG; structural XML validation passed, but the environment lacked the SVG rasterization delegate. SVG files contain standard paths, polylines and text.
- Stability margins, recommended gains, safe operating range or real three-loop control quality. Tool reports are observations, not a commissioning certificate.

## Protocol limits carried into the design

- Binary telemetry contains no schema ID. A delayed old binary frame of exactly the new schema's length arriving after the target ACK is fundamentally indistinguishable at the framing layer. This patch does not claim to solve that firmware protocol limit.
- Pre-existing v1.0 behavior is retained: an identical target SUB resets the timing baseline. Duplicate/reversed timestamps between consecutive mapped DATA are rejected, but a time rollback separated by an identical SUB is not reliably detected. Do not interpret these checks as complete reboot/reordering protection.
- Firmware has no transaction ID, source authentication or atomic stopped-only write interlock. Received ACK and readback confirm an observed RAM value; they cannot prove causality against an interfering client or physical motor state.
- GET? returns schema/rate/stream, not parameter values. Dedicated ground_* telemetry is required for ground writes.
- CLI exposes no arbitrary-command option or write queue. It exposes one allowlisted scalar per invocation and never starts motors or changes mode.
- A schema change or failure after send is treated as uncertain. No automatic retry or rollback occurs.
