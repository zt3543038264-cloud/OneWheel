#ifndef VEHICLE_PROFILE_H_
#define VEHICLE_PROFILE_H_

/* Explicit-action ordinary output profile. Boot remains stopped.
 * Ordinary A/Yaw, B/Roll and C/Pitch are available only after controller Start.
 * External A/B driver must implement SAFE version1 capability2.
 * C diagnostic modes retain their existing bounds and lease checks. */
#ifndef C_BENCH_POWERED_RATE_PROFILE
#define C_BENCH_POWERED_RATE_PROFILE 1
#endif
#if (C_BENCH_POWERED_RATE_PROFILE != 0) && (C_BENCH_POWERED_RATE_PROFILE != 1)
#error "C_BENCH_POWERED_RATE_PROFILE must be 0 or 1"
#endif
#ifndef C_BENCH_ONLY_PROFILE
#define C_BENCH_ONLY_PROFILE 0
#endif
#ifndef C_BENCH_SIGNAL_OUTPUT_ALLOWED
#define C_BENCH_SIGNAL_OUTPUT_ALLOWED C_BENCH_POWERED_RATE_PROFILE
#endif
#define C_BENCH_RAW_DUTY_MAX 500
/* Dedicated bounded pulse sink; legacy raw/rate limits remain unchanged. */
#define C_BENCH_PULSE_RAW_DUTY_MAX 2000
#define C_BENCH_DIRECTION_ZERO_MS 2
#if (C_BENCH_ONLY_PROFILE != 0) && (C_BENCH_ONLY_PROFILE != 1)
#error "C_BENCH_ONLY_PROFILE must be 0 or 1"
#endif
#if (C_BENCH_SIGNAL_OUTPUT_ALLOWED != 0) && (C_BENCH_SIGNAL_OUTPUT_ALLOWED != 1)
#error "C_BENCH_SIGNAL_OUTPUT_ALLOWED must be 0 or 1"
#endif
#define VEHICLE_MOTOR_OUTPUT_ALLOWED 1
#if C_BENCH_ONLY_PROFILE && VEHICLE_MOTOR_OUTPUT_ALLOWED
#error "C-only bench may not enable the ordinary whole-vehicle output path"
#endif
#ifndef VEHICLE_YAW_ENABLED
#define VEHICLE_YAW_ENABLED 1
#endif
#if (VEHICLE_YAW_ENABLED != 0) && (VEHICLE_YAW_ENABLED != 1)
#error "VEHICLE_YAW_ENABLED must be 0 or 1"
#endif

#if C_BENCH_POWERED_RATE_PROFILE && C_BENCH_ONLY_PROFILE && VEHICLE_YAW_ENABLED
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
