# A/B angle response changes

Current revision (2026-10-27): all extra controller angle and body-rate LPFs have been removed, including their state and config macros. Modes 3..7 consume EKF angles and corrected gyro rates directly. EKF and its internal accel LPF are retained. Compiled rate Kp defaults are flywheel 36 and ground 20. The A/B results below describe historical revisions, not the current feedback path.

- A: angle LPFs are now independent: `FLY_ANGLE_LPF_TAU_S=0.010f` for the flywheel (previously 0.040 s), and `GROUND_ANGLE_LPF_TAU_S=0.040f` for the ground wheel. Set either to 0 to bypass that axis. Flywheel rate LPF remains 0.005 s.
- B: `FLY_ANGLE_FF_KP` in `code/config.h` defaults to 300 PWM/degree. Set 0 to disable; supported range 0..1000. This is a compile-time parameter, not stored in Flash or controlled by the UDP PID sliders.
- Active only for the flywheel in Modes 3/5 with nonzero angle Kp. Ground control, rate-only Modes 6/7 and raw Mode8 do not receive the compensation.
- Formula: `Sign * clamp(rate_PID + FLY_ANGLE_FF_KP * (target - angle_lpf), +/- PWM_limit * startup_scale)`. The rate PID retains its own clamp. The added term uses the same filtered angle error and output direction.
- Stop, calibration, IMU stale/failure, invalid inputs and tilt protection still gate the entire output. PID gains, mechanical zeros, signs and Flash format are unchanged.
- Rebuild and flash the TC264 to activate. Flash PID values apply only when `QCAR_BOOT_LOAD_FLASH=1` (currently 0); enabling this term does not validate physical stability.

The 10ms change affects the Mode3/5 flywheel angle loop and its angle-PWM compensation together. Ground-wheel control retains its 40ms response. PID gains, limits, zeros, signs, startup ramp and output protocol are unchanged by this filter split. Host zero-crossing replay is a prescribed sensor input, not a motor/body simulation or an on-board balance test.

Validation after splitting the angle filters: 21/21 host regressions pass. The added test checks independent axis/bypass settings, Mode3/4/5 consistency, Arm seeding and raw tilt protection. In the prescribed zero-crossing replay (true target angle, body rate +20deg/s), 40ms commands about +1095 PWM while 10ms commands about -370 PWM; no motor/body plant is simulated. ADS 1.10.10 rebuilt `Debug/OneWheelUpright.elf` and `.hex` with 0 errors and 2 existing W525 warnings in the Flywheel menu's volatile-pointer bindings. The firmware has not been flashed or physically validated.
