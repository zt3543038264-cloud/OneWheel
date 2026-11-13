# A/B actual speed telemetry

`fly:uptime_ms,a_rpm,b_rpm,link_ok`

The firmware emits this numeric frame alongside each `run:` frame from the same
control snapshot, including Roll Rate/Angle tests with recovery bypassed.
RPM comes from `W_Motor_GetSpeed1/2()` (driver feedback with MOTOR_DIR_A/B applied),
not commanded PWM or a PID speed target. `link_ok=0` means feedback is stale;
retained RPM must not be treated as live measurement. The two getters expose the
latest driver feedback, not a new measurement for every 20 ms telemetry frame.

The source version of Embedded Station exposes `flywheel_a_rpm`,
`flywheel_b_rpm`, and `flywheel_link_ok` in the waveform channel list and logs.
Use `fly_uptime_ms` to align with `run.uptime_ms`. Existing packaged executables
need rebuilding to include these channel names. Existing `run:` stays 25 fields;
its A-B and common RPM now update even when the recovery loop is bypassed.

Build and flash the updated firmware before collecting another trial. This
change does not alter PID gains, control output, or flash parameters.
