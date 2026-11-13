#ifndef NEW_EKF_CONFIG_H
#define NEW_EKF_CONFIG_H
/* ===== IMU / EKF：保留现有算法参数和机体轴映射 ===== */
/* EKF noise densities (rad^2/s, rad^2/s^3), normalized gravity variance.
 * Gate uses normalized innovation squared, 3 DOF, 99 percent threshold.
 * No small-gyro deadband: preserve slow real yaw motion. */
#define EKF_GYRO_NOISE           0.0001f
#define EKF_BIAS_NOISE           0.00000001f
#define EKF_ACCEL_VARIANCE       0.0025f
#define EKF_CHI_SQUARE_LIMIT     11.345f
/* Single software preprocessing stage BEFORE fusion. No post-EKF LPF. */
#define IMU_GYRO_INPUT_TAU_S     0.002f
#define IMU_ACCEL_INPUT_TAU_S    0.004f
#define IMU_GYRO_SPIKE_DPS       300.0f
#define IMU_ACCEL_SPIKE_G        2.0f
#define EKF_REACQUIRE_STABLE_S   0.5f
#define EKF_REACQUIRE_GYRO_RADPS 0.03490659f
#define EKF_REACQUIRE_VIB_G      0.015f
#define EKF_REACQUIRE_NORM_G     0.03f
#define EKF_CORRECTION_MAX_RADPS 0.34906585f
#define EKF_BIAS_LIMIT_RADPS     0.08726646f
/* IMU quality: seconds, g and rad/s. Independent of the control-rate LPF. */
#define EKF_QUALITY_TAU_S         1.0f
#define EKF_VIBRATION_TAU_S       0.05f
#define EKF_ACCEL_SOFT_ERROR_G    0.04f
#define EKF_ACCEL_SOFT_VIB_G      0.05f
#define EKF_NIS_SOFT_LIMIT        3.0f
#define EKF_R_SCALE_MAX           16.0f
#define EKF_BIAS_LEARN_MAX_RADPS  0.17453293f
#define EKF_BIAS_SLEW_RADPS2      0.01f
#define EKF_GYRO_STEP_MAX_RADPS   0.34906585f
#define IMU_PREDICT_LIMIT_S       0.5f
#define IMU_POOR_ACCEPT_RATIO     0.5f
#define IMU_POOR_ACCEPT_LIMIT_S   1.0f
#define IMU_POOR_R_SCALE          8.0f
#define IMU_RAW_CLIP_COUNTS       32000

#define ACC_NORM_MIN             0.85f
#define ACC_NORM_MAX             1.15f
#define MAX_SAMPLE_DT            0.020f
/* 超过 MAX_SAMPLE_DT 但在本值以内：只丢这一帧，不锁止（总线偶发毛刺）。 */
#define MAX_SKIP_DT              0.050f

#define CALIBRATION_SAMPLES      500u /* 500Hz: keep a 1 second stillness window. */
/* BMI270 (IMU660RA): both sensors 800Hz; normal-bandwidth HW filtering. */
#define IMU_RA_ACC_CONF_CONFIG  0xABu /* Performance, normal filter, 800Hz. */
#define IMU_RA_GYR_CONF_CONFIG  0xEBu /* Filter/noise performance, normal, 800Hz. */
#define IMU_RA_PWR_CONF_CONFIG  0x00u /* Advanced power saving disabled. */
#define IMU_RA_PWR_CTRL_CONFIG  0x0Eu /* Gyro + accel + temperature enabled. */
#define CAL_GYRO_LIMIT_DPS       10.0f
#define CAL_GYRO_SPAN_DPS        6.0f
#define CAL_ACCEL_SPAN_G         0.12f
/* Reject sustained rotation even if its peak-to-peak span is small. */
#define CAL_GYRO_MEAN_MAX_DPS    3.5f
#define IMU_TIMEOUT_MS           100u

/* Body frame retained by the controllers: X=right, Y=front, Z=up.
 * New mount: sensor +X=down, +Z=rear, hence +Y=right.
 * Body X/Y/Z = sensor Y/-Z/-X, for BOTH gyro and accelerometer.
 * Axis indices 0/1/2 mean sensor X/Y/Z; signs must be integer +/-1. */
#define IMU_BODY_X_AXIS 1
#define IMU_BODY_Y_AXIS 2
#define IMU_BODY_Z_AXIS 0
#define IMU_BODY_X_SIGN 1
#define IMU_BODY_Y_SIGN -1
#define IMU_BODY_Z_SIGN -1
#if IMU_BODY_X_AXIS < 0 || IMU_BODY_X_AXIS > 2 || IMU_BODY_Y_AXIS < 0 || IMU_BODY_Y_AXIS > 2 || IMU_BODY_Z_AXIS < 0 || IMU_BODY_Z_AXIS > 2
#error IMU_BODY axes must be sensor indices 0, 1, 2
#endif
#if IMU_BODY_X_AXIS == IMU_BODY_Y_AXIS || IMU_BODY_X_AXIS == IMU_BODY_Z_AXIS || IMU_BODY_Y_AXIS == IMU_BODY_Z_AXIS
#error IMU_BODY axes must be distinct
#endif
#if (IMU_BODY_X_SIGN != 1 && IMU_BODY_X_SIGN != -1) || (IMU_BODY_Y_SIGN != 1 && IMU_BODY_Y_SIGN != -1) || (IMU_BODY_Z_SIGN != 1 && IMU_BODY_Z_SIGN != -1)
#error IMU_BODY signs must be integer +1 or -1
#endif
#if IMU_BODY_X_SIGN * IMU_BODY_Y_SIGN * IMU_BODY_Z_SIGN != (((IMU_BODY_X_AXIS > IMU_BODY_Y_AXIS) + (IMU_BODY_X_AXIS > IMU_BODY_Z_AXIS) + (IMU_BODY_Y_AXIS > IMU_BODY_Z_AXIS)) % 2 ? -1 : 1)
#error IMU_BODY mapping must preserve a right-handed coordinate frame
#endif
#define DEG_TO_RAD 0.01745329251994329577f
#define RAD_TO_DEG 57.29577951308232088f


#define APP_IMU_PERIOD_US 2000u
#endif
