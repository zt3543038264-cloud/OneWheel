# 角度环判断与 WiFi 记录

已恢复原角度反馈：`目标角速度 = clamp(Angle KP × (目标角度 - 实际角度), ±Rate limit)`。角速度内环 Sign 未修改。

菜单 Connect 成功后默认选用 `tools/angle_telemetry.json` 的 28 通道，周期请求 20 ms，每帧 116 字节（28 个小端 float32 + JustFloat 帧尾）。设备通知当前 SUB 列表，Route_Run 上位机在 MCAR 模式下自动更新解析字段。上位机需开始监听本机 UDP 8081。

| 分组 | 通道 | 用途 |
| --- | --- | --- |
| 角度 | balance_target_deg、balance_actual_deg、balance_error_deg | 目标、实际、误差；看回正时间、稳态误差、超调 |
| 角速度 | rate_target_dps、rate_actual_dps、rate_error_dps | 看内环能否跟随外环要求 |
| 输出 | rate_pwm、motor_left_pwm、motor_right_pwm | 限幅后输出与左右电机指令，非电流反馈 |
| 参数 | balance_kp、rate_kp、rate_sign、balance_rate_limit_dps、rate_pwm_limit | 记录本次实际设置，避免 Flash 覆盖造成误判 |
| 饱和 | balance_rate_saturated、balance_pwm_saturated | 1 表示正在对应限幅附近，0 表示未饱和；仅 State=2 时有效 |
| 转速 | flywheel_left_speed_raw、flywheel_right_speed_raw、driver_speed_frames | 检查动量积累与转速回传是否更新；原始速度单位尚未确认为 RPM |
| 时序与状态 | telemetry_ms、rate_seq、imu_seq、imu_status、rate_state、rate_enabled、imu_dt_ms | 检查运行状态、采样与收包间隔、丢帧 |
| 原始角度名 | roll_deg、pitch_deg | 底层变量名保留；不要与已互换的屏幕标签混淆 |

`balance_actual_deg` 才是当前角度环用到、屏幕显示为 Roll 的角度，来自内部 `imu_pitch_deg`。它和目标角度、角度误差、目标角速度、实际角速度及 PWM 在同一控制更新中发布。判断闭环只使用 `rate_state=2` 且 `imu_status=1` 的片段。参数改变后的临界帧可能跨更新周期，采集时尽量不改参数。

建议记录 20～30 秒：静止约 3 秒，左右分别施加一次约 2～3° 小扰动，每次等待恢复；注明何时施力/松手、是否持续扶持。观察角度误差是否收敛、超调和持续振荡、目标/实际角速度跟随、输出和限速饱和，以及飞轮是否持续单向加速。不要仅根据 PWM 正负判断物理反馈方向。

如果使用采集脚本而非 GUI（不能同时占用 8081）：

```powershell
python tools/wifi_rate_capture.py --profile tools/angle_telemetry.json --seconds 30
```

CSV/JSON 保存在 validation。也可用 Route_Run 的 CSV 导出，设备已自动发布上述字段，通常不需要粘贴 SUB 命令。

`python tests/check_angle_wifi_host.py` 已使用固件实际生成的 116 字节数据，经本机 UDP 给指定 Route_Run 版本的接收器和解析器，检查全部 28 项、符号、目标和饱和标志。该测试证明协议可收可解析，不代表本次新固件已完成实车无线收包；实车还需烧录后确认 WiFi 就绪、Tx 和上位机接收计数增加。
