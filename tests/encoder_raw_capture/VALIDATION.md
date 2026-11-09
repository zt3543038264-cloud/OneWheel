# Validation v1.1

Validated on 2026-11-07 in the cloud Linux workspace with Python standard library only.

All 19 automated tests passed. Coverage:

- Fixed profile contains raw/delta and all fields are in the delivered firmware field list.
- Four exact telemetry commands are the complete send allowlist; motor, parameter, save and arbitrary subscription commands are rejected.
- Missing operator confirmation and invalid duration are rejected before opening a socket.
- Float32 packet decoding preserves raw/delta; malformed frames, unknown/duplicate fields, malformed ACKs, non-finite values and unexpected slider ACKs are rejected.
- Unmapped old telemetry is ignored until an exact subscription acknowledgment arrives.
- Existing log files are not overwritten.
- A real UDP mock on 127.0.0.1 verifies GET/SUB/RATE/STREAM, original profile logging, raw wrap and positive/negative delta capture.
- Changed subscription order, an active control mode and malformed live frames abort; no motor Stop or other control command is transmitted.
- Exact repeated SUB/RATE/STREAM announcements are accepted without counting as data or resetting board time.
- A stale data frame after an identical repeated SUB is rejected.
- Metadata-only traffic cannot postpone the data timeout.
- Changed RATE, STREAM 0 and a same-length reordered SUB after valid data all abort.

The reported partial log was replayed locally: all 19 DATA records passed; its final identical repeated SUB was accepted while retaining telemetry_ms=1518713.375. An injected earlier data frame after that announcement was rejected. Replay reconstructed datagrams from logged parsed fields; original unmapped packet bytes are absent from the log and were not claimed to be replayed byte-for-byte.

Source verification against the provided dcce786 working tree:
- code/config.h: APP_WIFI_ANNOUNCE_MS = 1000u.
- code/menu.c: menu_auto_wifi_service calls menu_wifi_announce at that cadence.
- code/wifispi.c: wifispi_telemetry_announce sends current SUB, RATE and STREAM metadata.
- code/wheel_encoder.c: raw is a uint16_t hardware counter, with modulo-65536 displacement handling.
- SHA256 of inspected code/wifispi.c: fd671a34e769bf58efbccba94f6c6a6b10910e531b8936fd213be8758ae01ef4.

The user's currently flashed firmware version was not independently confirmed. The log establishes identical metadata, not the identity of its sender beyond the configured UDP endpoint.

No connection to real hardware, no motor operation, no physical rotation test, no firmware compilation/flashing and no original GUI modification took place.
The Windows .cmd launcher is supplied for convenience but was not executed on Windows. Its Python capture target was tested on Linux.
UDP telemetry is unauthenticated, and software stopped-status observation is not a hardware interlock.
