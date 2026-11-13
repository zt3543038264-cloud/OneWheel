# New-branch IMU/EKF preserved in master

The user's final instruction is to keep the current solving method. The live
pipeline therefore uses the **new branch's IMU660RA driver, body mapping and
500 Hz EKF**, not master A+r10h. The ADS project excludes the retained r10h source tree
from the firmware build; standalone historical host tests compile it explicitly. The earlier raw-only prototype has been superseded and is not part of this
review package.

## Source and changes

`code/Estimation/Attitude/new_ekf/` imports new branch `imu.c`, `attitude.c`,
`attitude_algorithm.c`, their headers, and the exact IMU/EKF configuration block.
Implementation files have `.inc` extensions and are included by the existing
master `imu.c`, so no additional firmware translation units are needed.
Only collision shims are applied to imported implementation text:

- new `imu_init()` -> `new_imu_init()` (master has its own return-value API).
- new math `attitude_init(q,norm,acc)` -> `new_attitude_init(...)` (master has a
  no-argument publication initializer).
- `Axis3f` in the compatibility-state header becomes an identical local
  three-float struct because master lacks that typedef.

All original EKF/filter/calibration equations, parameters, RA register accesses,
quality handling, navigation-sample getter, recovery/service functions and
body mapping remain unchanged. The original `imu_sample_sequence` variable is
retained; the master shim accessor is `imu_get_sample_sequence()`.

## Coordinates

The estimator retains **body X/Y/Z = sensor Y/-Z/-X**, for gyro and acceleration.
Master presentation/control aliases follow the new branch display convention:

- master Roll = new `imu_pitch_deg`; master roll_rate = new prepared gyro Y.
- master Pitch = new `imu_roll_deg`; master pitch_rate = new prepared gyro X.
- yaw is unchanged, with continuous yaw from original `attitude_state.True_yaw`.

The shim `imu_data_t` X/Y swap is a display/control alias, not a right-handed
3D estimator frame. Never feed it into r10h or another quaternion estimator.
Master forward-direction parameters do not alter the imported mapping.

## Schedule and calibration

The parent calls `imu_update_gyro()` every 1 ms. A divide-by-two gate calls the
original `imu_update()` every **2 ms**. The EKF receives the original STM-measured
dt; there is no synthetic dt or r10h integration. BMI270 hardware remains at
800 Hz as in new branch. A 500 Hz solver does not mean a fresh 1 kHz stream.
`attitude_tick1ms()` is a compatibility no-op. `attitude_update()` publishes
only a new navigation sequence/generation. Intervening 1 ms ticks hold the last
rates/angles; diagnostics identify freshness, sequence and actual sample dt.

Boot `imu_calibrate()` runs the original 500-good-sample calibration, with 2 ms
waits and a **10-second elapsed STM deadline**. Run it only with sampling timers
stopped and motors disabled. If it times out, the wrapper retains
`IMU_CALIB_MOVED`, displays `CALIB TIMEOUT / REBOOT`, and stops feeding the
imported pipeline. Later stillness cannot change the wrapper to READY or create
a startup mean. `attitude_converged()` also requires `IMU_CALIB_OK`.

Once calibrated, the application wrapper checks a **25 ms elapsed STM deadline**
from the last newly accepted navigation sequence/generation, not a count of
scheduled calls. It reads STM TIM6/TIM0/TIM6 coherently, avoiding the shared CAP
latch and retaining the upper counter bits. Lower-32-bit wrap and scheduler gaps
therefore do not make an old publication fresh. Foreground status queries and
both sides of an EKF poll check the deadline, so an already-overdue new frame
cannot erase the outage. A slow SPI/read/solve operation is checked before its
result is accepted. CPU0 status checks protect their 64-bit publication snapshot
against the CPU0 sampling ISR.

A pipeline error, publication age **>=25 ms**, unexpected pipeline restart, or
boot-calibration timeout is **latched for this boot**. Fault text identifies the
first observed cause and requests reboot. Subsequent wrapper polls do not call
the imported solver, rates are invalid/zeroed at the attitude adapter, and
startup-mean/raw-sample getters reject the data. Existing displayed angles can
remain held but `attitude_converged()` is false and `attitude_diverged()` is true.
The imported diagnostic status can still contain its last RUNNING value: it is
not the application readiness flag. Use `imu_link_lost()` and
`attitude_converged()` when deciding whether the data may be used.

**Recovery policy: stop, inspect the mounting/bus, then reboot with the vehicle
stationary and motors physically isolated.** Only boot-only `imu_init()` clears
the application latch, followed by a new successful calibration/publication.
There is no runtime/UI recovery command and no automatic motor re-enable.
Reconnecting a failed bus, returning a clipped gyro to range, calling
`imu_calibrate()` again, or calling the original imported recovery/service APIs
does not clear a latched wrapper fault. Do not add a runtime caller of
`imu_init()` or the imported recovery APIs without a separate stopped-motor
recovery design and validation.

Brief interruptions below the deadline retain the last accepted values; this is
a bounded hold, not an assertion that a new measurement arrived. A dropped,
clipped, repeated, or original-dt-skipped frame cannot advance the wrapper's
freshness timestamp. `att_diag.sample_fresh` is true only on a newly accepted
poll. Normal intervening 1 ms polls remain valid holds without re-solving.

## Limits and acceptance

**Motor output remains globally disabled by the parent project.** The unchanged
master 1 ms controller consumes held 2 ms publications. Control gains, filtering,
startup timing, sign conventions and rate scaling require separate validation
before enabling it. This is not a claim of road readiness or stability.

New branch's bounded `spi_transfer_8bit_timeout()` is imported into the common
SPI driver by the parent. The vendor BMI270 configuration-image initialization
still uses its existing startup functions; startup blocking behavior is not
fixed here. No hardware flashing, TASKING build, physical mount verification,
SPI electrical test or on-target execution-time validation has been performed.

Run `python3 tests/new_ekf/run.py`: strict GCC integration, original EKF tests and
original preprocessing tests. The old `tests/imu_ra/run.py` redirects here.
`tools/attitude_r10h/run_tests.py` additionally runs historical r10h core tests;
those do not mean r10h is active in firmware.

The STM deadline is checked only when firmware executes a poll/status check; it
cannot stop hardware while the CPU is hung or interrupts are blocked. The STM
must run continuously at its initialized clock frequency. The imported EKF
still uses its unchanged lower-32-bit dt arithmetic, but the full-width wrapper
deadline rejects prolonged gaps before allowing another normal wrapper solve.
On-target timer semantics, ISR duration/overrun behavior, electrical disconnect,
reset behavior, and an independent motor-driver watchdog remain unlock blockers.

The host integration test covers normal/tilted boot, 2 ms cadence and axes,
short restored dropouts with sequence freshness, a >110 ms outage, repeated
gyro clipping, raw-burst/SPI faults, delayed calibration, original BAD_DT and
filter errors, direct imported recovery calls, deadline checks without polls,
SPI overruns, lower-counter rollover including a rollover during the timer
read, gaps longer than one lower-counter period, and successful reboot recovery.

The old master `imu_spi.inc` and `imu_r10h.inc` files are retained unchanged
for reference and are not included by the live adapter.
