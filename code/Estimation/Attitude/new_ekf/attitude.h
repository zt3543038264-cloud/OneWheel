/* Six-axis EKF attitude estimator and calibration.
 * Input: rad/s, g, seconds. Output: degrees. See IMU.md. */
#ifndef MCAR_ATTITIUDE_H
#define MCAR_ATTITIUDE_H
#include "config.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    unsigned count; /* 连续合格静止样本数 */
    float sum_g[3], sum_a[3]; /* 角速度/加速度累加和 */
    float min_g[3], max_g[3]; /* 角速度窗口极值，deg/s */
    float min_a[3], max_a[3]; /* 加速度窗口极值，g */
    float bias_dps[3], initial_accel[3]; /* 陀螺静止零偏和初始重力 */
} ImuCalibration;

void calibration_reset(ImuCalibration *c);
int calibration_push(ImuCalibration *c, const float gyro_dps[3], const float accel_g[3]);

/* 保留仓库原公共数学接口。静止 +Z=+1g，初始 yaw=0。 */
int new_attitude_init(float q[4], float *accel_norm, const float accel_g[3]);
void attitude_euler(const float q[4], float rpy_deg[3]);
/* Cube 的 Ry*Rx*Rz 分解，不可与常规 ZYX 欧拉角混用。 */
void attitude_cube_euler(const float q[4], float xyz_deg[3]);

/* 更新函数成功返回 1，失败返回 0；失败不提交新状态。 */
/* Quaternion nominal state, local angle error + X/Y residual gyro bias.
 * Z bias is determined by startup stationary calibration in imu.c.
 * Fixed-size storage; no allocator or matrix library required. */
typedef struct {
    float q[4], bias_xy[2], covariance[5][5], accel_lpf[3];
    float accel_norm, innovation_nis;
    int accel_used;
    float accel_vibration_sq, gyro_step_sq, previous_gyro[3];
    float observation_r_scale, bias_slew_dps2;
    float reject_age_s, accept_ratio, poor_accept_s;
    unsigned reject_count;
    int reject_reason, quality, have_gyro, bias_learning;
    float gyro_lpf[3], gyro_reference[3], previous_accel[3], input_accel_norm, gravity_stable_s;
    unsigned gyro_hold_mask, accel_hold_mask;
    int input_ready, reacquiring;
    float gravity_vibration_sq;
} Ekf6;
enum { EKF_REJECT_NONE=0, EKF_REJECT_NORM=1, EKF_REJECT_NIS=2,
       EKF_REJECT_GEOMETRY=3, EKF_REJECT_CLIP=4 };
enum { EKF_QUALITY_GOOD=0, EKF_QUALITY_PREDICT=1,
       EKF_QUALITY_WEAK=2, EKF_QUALITY_UNTRUSTED=3 };
int ekf6_init(Ekf6 *s, const float a[3]);
int ekf6_update(Ekf6 *s, const float g[3], const float a[3], float dt);
void ekf6_euler(const Ekf6 *s, float e[3]);
void ekf6_cube_euler(const Ekf6 *s, float e[3]);
#ifdef __cplusplus
}
#endif
#endif
