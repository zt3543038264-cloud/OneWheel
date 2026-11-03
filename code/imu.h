/** IMU660RA 采样、状态机与姿态发布；不依赖 WiFi 或显示驱动。 */
#ifndef MCAR_IMU_H
#define MCAR_IMU_H
#include "zf_common_typedef.h"
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

enum {
    IMU_ATTITUDE_INIT_FAILED=-1, IMU_ATTITUDE_TIMEOUT=-2,
    IMU_ATTITUDE_BAD_DT=-3, IMU_ATTITUDE_FILTER_ERROR=-4,
    IMU_ATTITUDE_UNTRUSTED=-5,
    IMU_ATTITUDE_CALIBRATING=0, IMU_ATTITUDE_RUNNING=1
};
/* 保留原变量名，Mymenu 不需改变数据绑定。多变量一致快照需短暂关中断。 */
extern volatile int32 imu_attitude_status;





/* 常规 ZYX 欧拉角，单位：度。六轴 yaw 长期会漂移。 */
extern volatile float imu_roll_deg;
extern volatile float imu_pitch_deg;
extern volatile float imu_yaw_deg;
extern volatile float imu_accel_norm_g; /* 加速度模长，g */
extern volatile float imu_calibration_percent; /* 静止标定进度，0~100 */
extern volatile float imu_accel_g[3]; /* 原始 X/Y/Z 加速度，g */
extern volatile float imu_gyro_dps[3]; /* 未扣静止零偏的角速度，deg/s */
extern volatile float imu_gyro_bias_z_dps; /* 当前生效的 z 轴零偏（含静止跟踪修正） */
/* Diagnostics: dt in ms, solve time in us (filter + Euler, excludes SPI).
 * accel_used: -1=no completed update, 0=rejected, 1=fused.
 * EKF NIS: -1=not evaluated; bias_xy is residual, not startup bias. */
extern volatile float imu_sample_dt_ms, imu_solve_us, imu_solve_max_us;
extern volatile int32 imu_accel_used;
extern volatile float imu_ekf_nis, imu_ekf_bias_xy_dps[2];
extern volatile uint32 imu_sample_sequence;
extern volatile uint32 imu_spi_timeouts,imu_not_ready_samples,imu_last_status_reg,imu_bad_bursts;
/* Quality 0=fused, 1=prediction, 2=weak observation, 3=low confidence (diagnostic only).
 * Reject reason 0=none, 1=norm, 2=NIS, 3=geometry, 4=gyro clipping. */
extern volatile int32 imu_quality,imu_reject_reason,imu_bias_learning;
extern volatile float imu_reject_ms,imu_accept_ratio,imu_accel_vibration_g;
extern volatile float imu_gyro_step_dps,imu_r_scale,imu_bias_slew_dps2;
extern volatile float imu_recovery_percent;
extern volatile uint32 imu_reject_count,imu_gyro_clips,imu_accel_clips,imu_recovery_count;
/* Legacy recovery fields stay zero; no quality-triggered stopped recovery FSM. */
extern volatile float imu_pre_gyro_dps[3],imu_pre_accel_g[3],imu_pre_norm_g;
extern volatile int32 imu_reacquiring;

/* Unwrapped calibrated sensor-Z rate integral (deg), not full 3D Euler yaw. */
extern volatile float imu_gyro_z_integral_deg;
extern const uint8 imu_attitude_method; /* Telemetry protocol ID: always 3 (EKF). */

/* 同一成功采样帧的姿态、加速度和扣零偏角速度，供定位融合使用。
 * 主循环调用 getter 时须短暂关中断；PIT 内可直接读取。 */
typedef struct {
    uint32 sequence, generation;
    float roll_deg, pitch_deg, yaw_deg, dt_s;
    float accel_g[3], gyro_dps[3];
} imu_navigation_sample_t;
bool imu_get_navigation_sample(imu_navigation_sample_t *out);

void imu_init(void); /* 启动阶段，先于 WiFi/定时采样启用 */
void imu_update(void); /* APP_IMU_PERIOD_US周期入口，当前2ms。 */
void imu_request_recalibration(void); /* 只置请求标志 */
void imu_recover_after_stall(void); /* 长时间关中断（如 Flash 擦写）后恢复采样 */
void imu_service(void); /* 主循环处理重标定 */
/* Reuse a successful calibration at Start; wait for fresh attitude. */
void imu_prepare_start(void);

#ifdef __cplusplus
}
#endif
#endif
