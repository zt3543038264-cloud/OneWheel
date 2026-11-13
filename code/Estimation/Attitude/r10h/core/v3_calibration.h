/* TEMPLATE for an UNCALIBRATED new specimen, not the bench-accepted profile.
 * Copy over core/v3_calibration.h in your new project only.
 * Keep C_MOTION_KP: it selects the tested r10h bias classifier path.
 * Identify a new physical calibration before claiming bench-level accuracy.
 */
#define GYRO_CAL_MATRIX {1.0f,0.0f,0.0f,0.0f,1.0f,0.0f,0.0f,0.0f,1.0f}
#define C_MOTION_KP (1)
