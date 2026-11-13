#ifndef DIRECTION_CONFIG_H_
#define DIRECTION_CONFIG_H_

// REVIEW: 以下为旧 master 基准，未在新 RA 正交车上验证。
// 2026-09-21 旧车基准。菜单 +1=保持此基准，-1=相对反向。
// 这些符号属于硬件/控制适配，不能再次当作用户默认值写入参数。
// 与 PARAM_MAGIC 的 sign2 语义绑定；未来改基准须新增存储版本和迁移。
#define MOTOR_DIR_A_BASE      (1)
#define MOTOR_DIR_B_BASE      (1)
#define MOTOR_DIR_C_BASE      (-1)
#define ENC_DIR_C_BASE        (-1)
#define STEER_DIR_BASE        (-1)
#define IMU_FORWARD_DIR_BASE  (-1)

// 保留实车混合符号：Roll Rate P=+20, D=-8；Yaw Rate P=-56。
// 因此 Roll 只归一化 D，不能翻转整个 Roll 环。
#define R_RATE_KD_BASE        (-1.0f)
#define Y_RATE_BASE           (-1.0f)

#endif
