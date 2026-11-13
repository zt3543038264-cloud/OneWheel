/* r10h portable facade. Retain original core notices; see docs/来源与修改说明.md. */
#ifndef R10H_ATTITUDE_H
#define R10H_ATTITUDE_H
#include "../core/candidate/ahrs_types.h"
#ifdef __cplusplus
extern "C" {
#endif
#define R10H_SAMPLE_RATE_HZ 1000u
#define R10H_STARTUP_SAMPLES 6000u
/* Physical units: gyro deg/s; accelerometer g. Same right-handed body frame.
 * One singleton, one sensor, one execution context. Never call concurrently.
 * This staged target uses an identity gyro matrix, not a measured RA calibration.
 * The portable 1 kHz core is retained for reference/tests; the live new-branch EKF adapter does not run it.
 */
/* Mean of 6000 valid stationary RAW physical samples, BEFORE bias removal
 * and BEFORE the core's gyro matrix. Returns 0 for invalid mean.
 * A new init clears estimator state and previous output availability.
 */
uint8_t r10h_init(const ahrs_sample_t *stationary_raw_mean);
/* Exactly one call per 1 ms sample. Raw physical input, no external bias
 * subtraction or scale matrix. Returns 1 when accepted, 0 otherwise.
 * On rejection no new output exists: previous output is stale.
 */
uint8_t r10h_update(const ahrs_sample_t *raw_sample);
/* Returns 0 until first accepted update. Degrees; yaw is continuous.
 * Copy in the same execution context as update (or protect the copy).
 */
uint8_t r10h_get_output(ahrs_output_t *out);
/* Optional profiling only. Does NOT change integration dt (fixed 1 ms). */
void r10h_set_clock(uint32_t (*clock_ticks)(void), float us_per_tick);
#ifdef __cplusplus
}
#endif
#endif
