# 地轮静摩擦（死区）标定

地轮使用独立的正、负 PWM 方向补偿，编译默认值位于 `code/config.h`：

```c
#define GROUND_DEADZONE_POS_PWM 1800.0f
#define GROUND_DEADZONE_NEG_PWM 3200.0f
```

它们是待实车标定的起点，不是所有负载下的固定门槛。补偿在闭环控制量非零时按实际下发 PWM 的方向叠加，跟随启动斜坡，最终仍受地轮 PWM 限幅约束；控制量为零时输出仍为零。方向按 `motor_ground_pwm` 正负判断，不直接等同于车身前后。

## 为什么分别实测

减速箱、轮胎接地载荷、电池电压会影响起转。正、反方向可能存在不同的起转门槛，起转与维持转动所需的 PWM 也可能不同。闭环调试中“较大 PWM、编码器速度为零”的区段，还可能受到外部限位、机械卡滞或编码器反馈异常影响，不能直接认定为静摩擦门槛。

## 台架准备

1. 地轮接地并承载正常车重，扶稳车架，使地轮能向两侧自由滚动。
2. 选择 Mode8，保持两个飞轮指令及地轮指令为零，再执行车端 Start。确认 `pwm_debug_enabled=1`、`rate_state=2`，等待 `startup_scale=1`。
3. Mode8 使用直接 PWM，不运行平衡 PID、倾角保护和静摩擦补偿，需持续扶持。
4. 上位机订阅下列公开字段名，不要加固件内部的 `TV_` 前缀：

   ```text
   SUB telemetry_ms,rate_state,fault_reason,imu_status,startup_scale,motor_ground_pwm,wheel_encoder_delta,wheel_speed_cps,wheel_encoder_valid,wheel_position_valid
   ```

## 测量步骤

1. 通过 `[slider,debug_ground_pwm,数值]` 从零开始，每档增加 100，保持约 1 秒，记录实际 PWM 和编码器变化。
2. 记下首次持续转动的 PWM，作为该方向的起转观测值。每次测量后先归零，等待停稳。
3. 换向重复，每个方向至少三次，保持接地与扶持条件一致。
4. 已转动后逐档减小 PWM，记录停止转动的门槛，用于评估起转与维持转动之间的回差。
5. 若实际轮子转动而编码器速度为零，先排查编码器；若轮子受到外部限制，该次记录不能用于标定。

Mode8 输入仍受 `DEBUG_PWM_LIMIT` 与启动斜坡约束，应以 `motor_ground_pwm` 的实际下发值为准。

## 应用结果

分别取两个方向起转观测值的中位数作为试调参考，不要把两方向合成一个值。可以修改上述编译默认值，或者在闭环调试时通过 WiFi 修改 RAM：

```text
[slider,ground_deadzone_pos_pwm,1800]
[slider,ground_deadzone_neg_pwm,3200]
SUB ground_deadzone_pos_pwm,ground_deadzone_neg_pwm,dual_pitch_pwm,motor_ground_pwm,wheel_speed_cps
```

两个值均填非负数，允许 0..8000。设为零关闭该方向补偿，Stop/Start 保留 RAM 设置，重启恢复编译默认值；现有 Storage/Save 不保存这两项。Mode8 和菜单开环测试不会应用它们。

## 闭环验证

1. 返回需要的闭环模式，观察 `dual_pitch_pwm`（补偿前）和 `motor_ground_pwm`（实际下发）以及编码器速度。
2. 小角度扰动后应及时动作；若仍长时间停转，核对负载、机械状态和反馈有效性。
3. 补偿过大可能让微小控制量产生较大动作、过冲及低频振荡。每次只调整一个方向，并记录测试条件。
4. Mode2/Mode10 位置保持还需验证返回位置时的过冲和摆动；补偿值匹配并不等于四环 PID 已调好。
5. 当前实现未分离起转补偿和维持转动补偿。若回差较大，需要进一步设计，而非持续提高常数。

完整控制结构见 [Mode10 控制说明](../MODE10_CONTROL.md)，协议见 [WiFi 遥测](../WIFI.md)。
