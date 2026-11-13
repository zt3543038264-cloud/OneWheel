#ifndef ATTITUDE_H_
#define ATTITUDE_H_

#include "zf_common_headfile.h"

/* Master display axes: Roll=new EKF pitch, Pitch=new EKF roll, Yaw unchanged.
 * These aliases are not a new right-handed body frame. Roll/Yaw control still
 * uses the original body rates; Pitch control projects the original Euler rate.
 * Valid only while attitude_converged(); values can be held between 2ms solves.
 * Boot-calibration failure, pipeline error or >=25 ms STM publication age
 * latches invalid until reboot plus successful new calibration/publication. */
typedef struct
{
    float roll;         // 横滚角，车体左右倾，单位 deg
    float pitch;        // 俯仰角，车体前后倾，单位 deg
    float yaw;          // 连续航向角，永不清零，单位 deg
    float yaw_wrapped;  // 显示用航向角，范围 (-180, 180]

    float roll_rate;    // 原EKF body q/Y（shim gyro_x），deg/s；非欧拉Roll变化率
    float pitch_rate;   // 原EKF body p/X（shim gyro_y），deg/s；Pitch另作欧拉角速度投影
    float yaw_rate;     // 原EKF body r/Z（shim gyro_z），deg/s；非欧拉Yaw变化率
} attitude_t;

extern attitude_t att;

typedef struct
{
    float acc_norm;     // 加速度模长，单位 g
    float bias_z;       // 原 EKF 管线 Z 零偏（含启动均值），单位 rad/s
    float acc_weight;   // 兼容字段固定为零；EKF 使用 NIS/R-scale
    float period_us;    // Original new-branch STM sample interval, microseconds
    float update_us, max_update_us; // Original EKF solve timing, microseconds
    uint8 is_static;    // 内部零偏学习判定，不是电机安全许可
    uint8 acc_valid;    // Original EKF accel_used==1
    uint32 sample_sequence; // Published new-branch navigation sample sequence
    uint8 sample_fresh; // Current acquisition poll accepted; not an attitude-valid flag
} att_diag_t;

extern att_diag_t att_diag;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 master 显示接口，解算沿用 new 分支 EKF
// 参数说明     void
// 返回参数     void
// 使用示例     attitude_init();
//-------------------------------------------------------------------------------------------------------------------
void attitude_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     使用 IMU 原始数据更新车体系角速度
// 参数说明     void
// 返回参数     void
// 使用示例     attitude_update_rate();
//-------------------------------------------------------------------------------------------------------------------
void attitude_update_rate(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     仅发布新 EKF 样本的显示角度，不重复解算
// 参数说明     void
// 返回参数     void
// 使用示例     attitude_update();
//-------------------------------------------------------------------------------------------------------------------
void attitude_update(void);

// 1 ms 调度兼容空接口；解算已在每第二次 imu_update_gyro 调用中完成。
void attitude_tick1ms(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查姿态解算冷启动收敛状态
// 参数说明     void
// 返回参数     uint8           启动标定OK、原EKF为RUNNING、有效发布未满25ms且无锁存故障时返回1
// 使用示例     uint8 ready = attitude_converged();
//-------------------------------------------------------------------------------------------------------------------
uint8 attitude_converged(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查原 EKF 管线错误状态
// 参数说明     void
// 返回参数     uint8           标定超时、原EKF错误或发布满25ms后锁存1；仅重启重新标定可恢复
// 使用示例     uint8 fault = attitude_diverged();
//-------------------------------------------------------------------------------------------------------------------
uint8 attitude_diverged(void);

#endif
