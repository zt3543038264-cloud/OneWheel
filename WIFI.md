# WiFi-SPI 直立遥测

Mode10 连接成功和 Start 时默认发送以下 7 通道，周期 20 ms；`MCAR SUB` 回执会公布相同顺序，上位机仍可用 `SUB` 替换订阅：

```text
dual_roll_actual_deg,dual_roll_target_deg,dual_roll_target_rate_dps,dual_roll_rate_dps,dual_roll_pwm,motor_left_pwm,motor_right_pwm
```

分别为 Roll 实际角度、目标角度、目标角速度、实际角速度、控制器 PWM、左飞轮实际下发 PWM、右飞轮实际下发 PWM。角度单位为 °，角速度为 °/s。其他模式沿用原有默认遥测。

地轮正、负 PWM 方向的静摩擦补偿现可分别在线修改，当前编译默认值为 1800、3200：

```text
[slider,ground_deadzone_pos_pwm,1800]
[slider,ground_deadzone_neg_pwm,3200]
```

两个值均填非负数，允许 0..8000；设为 0 关闭该方向补偿。方向按 `motor_ground_pwm` 的正负判断，不能直接等同于车身前后。闭环控制量为 0 时保持零输出；补偿随启动斜坡增加，最终输出仍受地轮 PWM 限幅约束。Mode8 和菜单开环测试不加此补偿。参数修改即时写入 RAM，Stop/Start 保留，重启恢复编译默认值，现有 Storage/Save 不保存这两项。

订阅补偿值、补偿前后 PWM 和地轮速度：

```text
SUB ground_deadzone_pos_pwm,ground_deadzone_neg_pwm,dual_pitch_pwm,motor_ground_pwm,wheel_speed_cps
```

Mode10 飞轮速度／角度环支持 `fly_vel_kd`、`fly_angle_kd`，默认均为 0，允许 0..1000，命令格式与 Kp 相同。它们可同名订阅回读，D 项输出分别为 `fly_vel_d_out`（度）和 `fly_angle_d_out`（度/秒）。两项仅保存在 RAM；算法及秒制尺度见 [Mode10 控制说明](MODE10_CONTROL.md)。

网络、SPI引脚和速率统一在 `code/config.h`。本地目标PC为192.168.0.112:8081，模块命令端口5001；WiFi名和密码保持原值。

先停车并退出IMU/开环预览，在Startup/WiFi connect选中后设On。连接失败可再次设On重试；连接后可Start或使用IMU预览。Stop停电机但保持遥测。WiFi等待期间仍处理菜单导航，硬件动作待SPI退出后执行。

数据：N个小端IEEE754 float32 + `00 00 80 7F`，N=1～40，每UDP报一帧。文本回执保留MCAR前缀。Connect/Start按模式选择34或40通道；SUB回执给出真实顺序，接收端不应猜字段偏移。

| 命令 | 作用 |
|---|---|
| LIST? | 当前可用直立变量 |
| GET? | 通道/周期/STREAM |
| SUB roll_deg,pitch_deg,yaw_deg | 替换通道列表 |
| RATE 20 | 2～1000ms发送间隔 |
| STREAM 0/1 | 暂停/恢复遥测，不是停车 |
| [slider,rate_kp,30] | 写行进轮内环KP（固定绑定） |
| [slider,fly_rate_kp,35] | 写飞轮内环KP |
| [slider,ground_angle_kp,40] | 写行进轮外环KP |
| [slider,balance_angle_x,3.9] | 写行进轮目标角度，范围±20° |
| [slider,debug_fly_left_pwm,300] | 写Mode8左飞轮原始PWM；范围由DEBUG_PWM_LIMIT决定 |
| [slider,debug_fly_right_pwm,-700] | 写Mode8右飞轮原始PWM；独立输入，不自动取反 |
| [slider,debug_ground_pwm,-500] | 写Mode8行进轮原始PWM |

其他可写名称见 `wifispi.c/g_slider_parameters`。非法、非有限或越界输入拒绝；PID方向只能通过有效值±1运行。下发修改RAM，停车后Storage/Save才保存支持的直立参数。Mode8的三个PWM值不保存，Start/Stop/切换模式会清零；因此先选择Mode8并Start，再发送PWM。旧的debug_fly_pwm配对命令已取消；改用左右独立名称。该模式不使用PID和IMU/倾角保护，需架空调试；STREAM 0仍只停数据。未增加远程启停、模式切换或远程Flash功能。旧cascade_* PID接口已删除。

Mode5和Mode8 Connect/Start选用同一40通道布局；Mode8的dual_enabled为0，motor_left_pwm/motor_right_pwm/motor_ground_pwm反映左飞轮/右飞轮/行进轮实际下发值，dual_roll_pwm仅为左飞轮兼容字段，dual_pitch_pwm仍为行进轮。可用 `SUB pwm_debug_enabled,debug_fly_left_pwm,debug_fly_right_pwm,debug_ground_pwm` 单独查看调试使能和三路设定值；这会替换当前订阅列表。

上位机实际UI是否暴露某个可写参数取决于其版本；固件支持字段不等于旧上位机已增加相应输入框。协议回环测试不代表无线实车已连通。
