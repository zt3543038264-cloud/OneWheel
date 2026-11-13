# Roll 1 kHz short trace

The CPU0 1 ms control ISR captures 256 samples (nominal 0.256 s) into a
11264-byte static RAM buffer. There is no network output, formatting, Flash
write, allocation or additional IMU transaction in the sample path. The hook
runs after control output calculation via the existing `vofa_snapshot()` call,
even with regular telemetry disabled. This feature does not change control gains,
feedback filters, output limits or motor-start permissions.

## Use

Build and flash the updated Qcar main-controller firmware. Disconnect the GUI
so this script can exclusively bind the receiving UDP port. Start from STOP,
hold the chassis, and run from the project root:

```powershell
python tools/unicycle_station/scripts/capture_roll_trace.py --host 192.168.0.111 --label ground --delay-ms 2000
```

After the script reports Armed, manually start **Roll > Rate** on the car.
Keep all parameters fixed. Sampling begins after 2000 ms in that mode; hold the
support condition fixed during that instant. The buffer freezes when full.
The script sends stop, confirms STOP, then downloads the indexed samples to
CSV plus metadata JSON. Run again with `--label airborne` and the same parameters
for the suspended comparison. Do not change support while sampling.
The physical start, support and emergency stop remain with the operator.

`--port` defaults to vehicle port 5001; `--local-port` defaults to PC port 8081.
Vehicle IP can change with DHCP; the firmware's configured PC address must match
the current PC address. This standalone script needs no third-party Python packages.
It never changes PID/Flash or remotely starts motors. On capture errors or Ctrl+C
it attempts to cancel and stop; network loss cannot guarantee stop delivery.

## Samples and timing

CSV: index, raw 32-bit STM ticks, relative time_us, Roll angle (deg), raw-body
six-axis gyro (deg/s) and acceleration (g), calculated Roll output (signed duty),
latest A/B driver RPM (signed, MOTOR_DIR applied), validity flags.
Gyro is before startup bias removal; control output is the calculated Roll command,
not measured driver PWM/current. RPM is the latest driver reply and is **not** a
fresh 1 kHz motor-speed measurement. Flags: bit0 raw sample valid, bit1 motor-link
valid, bit2 IMU initialized and link valid; invalid six-axis entries are zero-filled
and must be excluded. Adjacent STM deltas reveal actual ISR timing/jitter. A
nominal 1 kHz schedule alone does not prove timing performance on hardware.
For 256 samples frequency-bin spacing is about 3.9 Hz; frequencies above Nyquist
can alias. Do not infer a vibration frequency from ordinary 50 Hz telemetry.

## Indexed cfg commands (no streaming flood)

- `cfg:trace_arm,seq,delay_ms`: STOP-only, delay integer 0..10000 ms; clears the
  old trace and arms a new one. Does not start motors. Retrigger requires new arm.
- `cfg:trace_status,seq`: response payload is
  `trace_status,1,state,count,256,1000,tick_hz,param_revision,reason,delay_ms,kp,ki,kd`.
- `cfg:trace_get,seq,index`: STOP + frozen only, index 0..count-1. Response:
  `trace_get,index,ticks,roll,gx,gy,gz,ax,ay,az,output,a_rpm,b_rpm,flags`.
- `cfg:trace_cancel,seq`: freezes any partial buffer, without changing motor state.

States: 0 idle, 1 armed, 2 recording, 3 frozen. Reasons: 1 full, 2 test stopped,
3 parameters changed, 4 no trigger within 30 s, 5 wrong test/mode, 6 cancelled.
Wrong mode, stopping the test and parameter edits terminate the recording rather
than mixing conditions. STOP cancels an armed record as well. Read-only requests
may be retried; arm is never automatically retried. The buffer persists until
rearm/reset and can be re-downloaded after packet loss. No Flash storage is used.
