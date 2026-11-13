#ifndef IMU_H_
#define IMU_H_

#include "zf_common_headfile.h"

typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;       // 车体系加速度，单位 g

    float gyro_x;
    float gyro_y;
    float gyro_z;      // 车体系角速度，单位 deg/s
} imu_data_t;

/* Master control/display aliases: X=new body Y, Y=new body X, Z=new body Z.
 * Do not feed these swapped aliases into a 3D estimator. Original EKF body mapping
 * remains sensor Y/-Z/-X. imu rates are original prepared, bias-corrected EKF rates. */
extern imu_data_t imu;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查 IMU660RA 通信链路状态
// 参数说明     void
// 返回参数     uint8           1 表示链路失效，0 表示正常
// 使用示例     uint8 lost = imu_link_lost();
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_link_lost(void);
/* Original pipeline errors, boot-calibration timeout and a 25 ms STM elapsed
 * publication deadline latch failure until reboot/boot-only imu_init().
 * Reconnecting the bus or calling an imported recovery API cannot clear it.
 * Status queries enforce the deadline even without scheduled IMU polls.
 * Foreground/ISR use is CPU0 only. No automatic motor re-enable exists. */
const char *imu_fault_text(void);

// New 分支固定机体映射，不使用 master 的动态 forward 设置；固定返回 0。
uint8 imu_forward_pending(void);

typedef enum
{
    IMU_CALIB_NONE = 0,     // 未标定
    IMU_CALIB_OK,           // 静止标定通过
    IMU_CALIB_MOVED,        // 标定期间检测到运动
} imu_calib_state_t;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 IMU660RA 并配置采样率和滤波器
// 参数说明     void
// 返回参数     uint8           0 表示成功，非 0 表示失败
// 使用示例     uint8 state = imu_init();
//-------------------------------------------------------------------------------------------------------------------
/* Boot only: timers stopped, motors disabled. Never a runtime recovery API. */
uint8 imu_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动阶段以 2ms 周期运行原 EKF 静止标定，最长等待 10s
// 参数说明     void
// 返回参数     void
// 使用示例     imu_calibrate();
//-------------------------------------------------------------------------------------------------------------------
/* Timeout/error requires reboot; a later still interval does not clear it. */
void imu_calibrate(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 IMU 标定状态
// 参数说明     void
// 返回参数     imu_calib_state_t 当前标定状态
// 使用示例     imu_calib_state_t state = imu_calib_state();
//-------------------------------------------------------------------------------------------------------------------
imu_calib_state_t imu_calib_state(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取标定期间采集的车体系重力方向
// 参数说明     ax 加速度 X 输出；ay 加速度 Y 输出；az 加速度 Z 输出
// 返回参数     uint8           1 表示数据有效，0 表示尚未标定
// 使用示例     uint8 valid = imu_get_static_acc(&ax, &ay, &az);
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_get_static_acc(float *ax, float *ay, float *az);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     每 1ms 调用；每第二次运行原 2ms RA/EKF 管线
// 参数说明     void
// 返回参数     void
// 使用示例     imu_update_gyro();
//-------------------------------------------------------------------------------------------------------------------
void imu_update_gyro(void);

// CPU0 原始车体系六轴物理量：尚未扣启动零偏，尚未乘标定矩阵。
// 仅最近一次轮询接受新样本且链路正常时返回 1；重复读取不消耗样本。
uint8 imu_get_raw_sample(imu_data_t *out);
/* Original new-branch STM observation dt, seconds; 2ms nominal solver cadence. */
float imu_sample_dt_s(void);
uint32 imu_get_sample_sequence(void);
uint8 imu_get_startup_mean(imu_data_t *out);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取原 EKF 适配器发布的连续航向，仅 RUNNING 时有效
// 参数说明     void
// 返回参数     float           连续航向角，单位 deg
// 使用示例     float yaw = imu_get_angle_yaw();
//-------------------------------------------------------------------------------------------------------------------
float imu_get_angle_yaw(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取原 EKF 航向增量的独立累计转角
// 参数说明     void
// 返回参数     float           元素累计转角，单位 deg
// 使用示例     float angle = imu_get_angle_element();
//-------------------------------------------------------------------------------------------------------------------
float imu_get_angle_element(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发布姿态解算得到的航向角和本周期增量
// 参数说明     yaw_continuous 连续航向角；yaw_delta 本周期航向增量
// 返回参数     void
// 使用示例     imu_publish_attitude_yaw(att.yaw, yaw_delta);
//-------------------------------------------------------------------------------------------------------------------
void imu_publish_attitude_yaw(float yaw_continuous, float yaw_delta);

#endif
