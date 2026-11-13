#ifndef VEHICLE_PROFILE_H_
#define VEHICLE_PROFILE_H_

/* Main-wheel C bench review profile. Ordinary balance/Run/Test/Jog and both
 * flywheels stay locked. This delivery adds explicit bounded POWERED_RATE modes.
 * Historical SIGNAL-only builds were for isolated-motor-power electrical checks;
 * this macro is not proof of a hardware inhibit, current limit or stable loop.
 * A = left horizontal Yaw; B = right vertical Roll; C = ground Pitch.
 * Preserve new branch RA + 500 Hz EKF. See C_POWERED_RATE_README.md for this build. */
/* This delivered build enables only explicitly named POWERED_RATE modes.
 * It does not certify the power stage, wiring, current limit or plant polarity.
 * Legacy SIGNAL modes are locked in this profile. No automatic restart. */
#ifndef C_BENCH_POWERED_RATE_PROFILE
#define C_BENCH_POWERED_RATE_PROFILE 1
#endif
#if (C_BENCH_POWERED_RATE_PROFILE != 0) && (C_BENCH_POWERED_RATE_PROFILE != 1)
#error "C_BENCH_POWERED_RATE_PROFILE must be 0 or 1"
#endif
#ifndef C_BENCH_ONLY_PROFILE
#define C_BENCH_ONLY_PROFILE 1
#endif
#ifndef C_BENCH_SIGNAL_OUTPUT_ALLOWED
#define C_BENCH_SIGNAL_OUTPUT_ALLOWED C_BENCH_POWERED_RATE_PROFILE
#endif
#define C_BENCH_RAW_DUTY_MAX 500
#define C_BENCH_DIRECTION_ZERO_MS 2
#if (C_BENCH_ONLY_PROFILE != 0) && (C_BENCH_ONLY_PROFILE != 1)
#error "C_BENCH_ONLY_PROFILE must be 0 or 1"
#endif
#if (C_BENCH_SIGNAL_OUTPUT_ALLOWED != 0) && (C_BENCH_SIGNAL_OUTPUT_ALLOWED != 1)
#error "C_BENCH_SIGNAL_OUTPUT_ALLOWED must be 0 or 1"
#endif
#define VEHICLE_MOTOR_OUTPUT_ALLOWED 0
#if C_BENCH_ONLY_PROFILE && VEHICLE_MOTOR_OUTPUT_ALLOWED
#error "C-only bench may not enable the ordinary whole-vehicle output path"
#endif
#ifndef VEHICLE_YAW_ENABLED
#define VEHICLE_YAW_ENABLED 0
#endif
#if (VEHICLE_YAW_ENABLED != 0) && (VEHICLE_YAW_ENABLED != 1)
#error "VEHICLE_YAW_ENABLED must be 0 or 1"
#endif

#if C_BENCH_POWERED_RATE_PROFILE && (!C_BENCH_ONLY_PROFILE || VEHICLE_YAW_ENABLED)
#error "Powered rate bench requires C-only profile and Yaw locked"
#endif

/* Geometry only, before the original channel direction parameters.
 * Roll retains master's B component (+roll), never drives A.
 * No shared saturation budget: the flywheel axes are orthogonal. */
static inline float vehicle_roll_command(float roll) { return roll; }
static inline float vehicle_yaw_command(float yaw)
{
#if VEHICLE_YAW_ENABLED
    return yaw;
#else
    (void)yaw; return 0.0f;
#endif
}
/* Preserve the sign of master's A-B feedback for its retained B actuator.
 * Magnitude is one measured wheel, not a synthetic doubled/differential RPM.
 * The original recovery gains are retained, NOT recalibrated for this plant. */
static inline float vehicle_roll_recovery_rpm(float a, float b)
{ (void)a; return -b; }
static inline float vehicle_yaw_momentum_rpm(float a, float b)
{ (void)b; return a; }
#endif
