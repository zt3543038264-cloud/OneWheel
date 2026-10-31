# Mode10 控制结构

飞轮：速度 PD → Roll 目标角度 → 角度 PD → 专用增量式角速度 PI → 飞轮 PWM。

飞轮新增两个 RAM 参数，默认均为 0，可通过 WiFi 即时写入、同名遥测回读；现有 Flash Save 不保存它们：

```text
fly_vel_kd,fly_angle_kd
```

速度 D 对 `(左转速-右转速)*0.005` 作负微分。由驱动反馈帧计数识别新样本，dt 使用两帧之间累计的控制时间（秒）；一阶低通时间常数 30 ms。首次反馈只建立历史，无新帧时保持该 D 项，超过驱动反馈超时则清零 D 项，恢复首帧不产生微分尖峰。

角度 D 为 `-fly_angle_kd * 实际Roll角速度`，与角度 P 相加后再作目标角速度限幅。它对实际测量微分，不对目标角度微分，避免速度环修改目标倾角时产生 D 冲击。两个 Kd 仅影响 Mode10 飞轮，地轮四环不使用它们；菜单对应 Gyro test / Vel KD 与 Angle KD。

```text
[slider,fly_vel_kd,0]
[slider,fly_angle_kd,0]
SUB fly_vel_kd,fly_angle_kd,fly_vel_d_out,fly_angle_d_out,fly_vel_out,dual_roll_target_rate_dps,dual_roll_rate_dps
```

`fly_vel_d_out` 单位为度，`fly_angle_d_out` 单位为度/秒。Kd 按秒制计算，与专用增量式内环每周期直接累加的 Ki 数值尺度不同。Kd 增大会放大测量噪声，应分别从小值验证，不保证增大就一定减小振动。

地轮：位置 P → 目标行驶速度 → 速度 PI → Pitch 目标角度 → 角度 P → 位置式角速度 P → 静摩擦补偿 → 地轮 PWM。

Mode10 复用 Mode2 的地轮位置／速度控制器。每次 Start 重置编码器位置窗口与外环状态，在首个有效位置样本建立本次位置原点。编码器过期、位置不连续或参数无效时，两轴通过原有故障路径停止输出；Stop 清除外环状态。

可写参数名：

```text
wheel_pos_kp,wheel_speed_limit,wheel_speed_kp,wheel_speed_ki,wheel_tilt_limit,wheel_i_limit,ground_angle_kp,rate_kp,balance_angle_x
```

`wheel_pos_kp`、`wheel_speed_limit` 任一设为 0 可关闭位置回复，保留零速保持。位置／速度参数与 Mode2 共用，目前不包含在现有 Flash 保存记录内。各环编译默认值以 `code/config.h` 为准，仍需按实车验证，不能把当前配置直接视为其他车辆的稳定参数。

建议订阅以下字段观察地轮链路；上位机 SUB 可替换默认订阅，Mode10 默认 7 通道 Roll 遥测保持原顺序：

```text
wheel_position_count,wheel_ref_count,wheel_pos_error,wheel_target_cps,wheel_speed_cps,wheel_tilt_offset,dual_pitch_target_deg,dual_pitch_actual_deg,dual_pitch_target_rate_dps,dual_pitch_rate_dps,motor_ground_pwm,wheel_feedback_valid,wheel_position_valid
```

地轮静摩擦补偿支持 WiFi 即时修改：`ground_deadzone_pos_pwm` 默认 1800，`ground_deadzone_neg_pwm` 默认 3200，均为非负 PWM 幅值，按实际输出方向选择。设为 0 可分别关闭；补偿不改变零控制量，遵循启动斜坡及 PWM 限幅。两项只存于 RAM，现有 Flash Save 不包含它们。协议示例见 [WIFI.md](WIFI.md)。

主机集成验证：`python tests/check_mode10_hold.py`。

IMU 融合样本连续不更新的控制层超时为 50 ms（原 25 ms），由 `BALANCE_IMU_STALE_MS` 配置；新样本会重置计时。触发后仍锁存停机，需 Stop/Start。IMU 自身读取超时仍为 100 ms，非有限数据、滤波失败和超过 30°倾角的保护条件不变。
