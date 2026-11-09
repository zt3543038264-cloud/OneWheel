# 主轮三环电脑端工具 v1.1

v1.1 修复订阅握手期间收到旧配置公告而提前中止的问题。具体日志依据、修改边界和停车下的检查步骤见 `FIX_NOTES_CN.md`。请完整解压到新文件夹，先检查 `VERSION.txt` 为 1.1，不要覆盖原日志。

## 这份包是什么

三个独立入口，调用同一个经过测试的通信模块：

- `rate_loop.py`：主轮角速度环，严格分析 Mode7。
- `angle_loop.py`：主轮角度环，分析 Mode4，或明确标记 Mode2/5/10 中的耦合观察。
- `speed_loop.py`：主轮速度环，分析 Mode2/10；同时报告位置环是否在参与。

每个入口都有 `parameters`、`analyze`、`capture`、`diagnose`、`send` 命令。
这是新增的电脑端分析、采集和人工参数收发工具，**不是三份新固件，也不替换原上位机**。当前 new 分支已经有这些板端控制环。包不含修改过的 `config.h`、固件 HEX、启动电机程序或自动调参程序；不含 WiFi 密码。

核对源码：OneWheel `new` 分支 `dcce786`，包含原工作区的局部存储改动，但本工具没有覆盖这些改动。字段和范围以实际 `code/wifispi.c`、`code/config.h` 为依据，不采用过时 README 或旧上位机默认值。源码文件哈希见 `SOURCE_AUDIT.json`，以便区分后续不同版本。

## 先备份，再独立解压

1. 将原工程、原上位机文件夹和已有记录分别复制备份，不要覆盖正在使用的副本。
2. 把整个 `main_wheel_three_loops` 文件夹解压到新的目录，保留 `three_loop.py`、`protocol_fields.json` 和三个入口的相对位置。
3. 安装/使用 Python 3.9 或更高版本。只依赖标准库，不需要 pip 安装。
4. Windows PowerShell/终端进入解压目录。下面的 `python` 可按本机环境改成 `py -3`。
5. 先运行离线测试：

```text
python -m unittest discover -s tests -v
python rate_loop.py parameters
python angle_loop.py parameters
python speed_loop.py parameters
```

若今后更新这个工具包，只在这个独立文件夹内更新。不要把其中任何文件当作板端 C 文件覆盖，也不要替换原上位机的协议解析器。

## 不联网的示例与波形

```text
python rate_loop.py analyze examples/rate_demo.csv --output rate_report.json --svg rate_waveform.svg
python angle_loop.py analyze examples/angle_demo.csv --output angle_report.json --svg angle_waveform.svg
python speed_loop.py analyze examples/speed_demo.csv --output speed_report.json --svg speed_waveform.svg
```

`examples` 中全部是**合成数据**，只用于验证软件，不代表任何真实车辆效果。SVG 是离线波形，可用浏览器打开。程序拒绝覆盖已有日志/报告文件，请另取文件名。

支持：

- CSV：第一行是遥测字段名；如 `telemetry_ms,control_mode,...`。
- 原上位机 JSONL：每行 `{"timestamp":"...","address":["...",5001],"values":{"telemetry_ms":..., ...}}`。
- 本工具生成的 JSONL：自动忽略 TX、ACK 和状态记录，仅解析 DATA 测量。
- 不读取或反序列化 `.pkl`，避免不受信任 pickle 执行代码。

如果原记录缺少 `control_mode`、故障状态、启动斜坡或相应环的关键通道，会返回 `INCOMPLETE` 和缺失字段，而不是假定车辆处在某个模式。历史双轴记录没有编码器数据时不能分析速度环。

## 读取原上位机的波形

可以读取原上位机导出的 CSV/JSONL 原始记录，再重画和定量分析。也可查看截图判断形状，但截图不能可靠还原准确采样时间和数值。已有的 JSONL 导出结构已在本工具中适配。

电脑端正在绘制的实时曲线并不会自动传给远端助手。要检查具体波形，请导出原始记录并提供文件；若只有屏幕，提供截图。此包不远程控制电脑、不抓屏、不向其他服务上传记录。

同一台电脑通常不能让原上位机与本工具同时独占监听 UDP 8081。使用本工具收发前，关闭原上位机 UDP 接收以及其他会发控制命令的程序；或者只用原上位机记录，再离线分析。程序不启用端口复用，不偷偷抢占监听。不要为解决冲突随意换监听端口：板端目标端口也要匹配，默认是 8081。

## 第一步：只检查收发，不改控制参数

先保持板端 Stop，退出 PWM/飞轮开环测试，按硬件要求安全隔离电机动力。只保留板端通信所需供电；不能可靠隔离时不要做写参数实验。

把下面的 `BOARD_IP` 换成实际板子 WiFi 模块的 IPv4 地址，**不是电脑 IP**。可从原上位机记录的 `address` 字段或设备网络信息确认，不能把历史地址当成当前地址。板端目的电脑地址需正确，默认电脑接收端口 8081、板端命令端口 5001。此包不会改电脑或板子的网络配置，也不自动打开防火墙权限。

```text
python rate_loop.py diagnose --device-ip BOARD_IP --seconds 5 --output rate_link_check.jsonl
python angle_loop.py diagnose --device-ip BOARD_IP --seconds 5 --output angle_link_check.jsonl
python speed_loop.py diagnose --device-ip BOARD_IP --seconds 5 --output speed_link_check.jsonl
```

`diagnose` 与 `capture` 的通信行为相同，只发：

1. `GET?`：取得当前通道、周期和开关。
2. `SUB ...`：选择本环的确切通道顺序。
3. `RATE 20`：请求 20ms 遥测。
4. `STREAM 1`：允许发遥测。

**这些命令不启动或停止电机，不改变 PID 参数。`STREAM 0` 也不是停车。** 程序不发送 Start/Stop、模式切换、Flash 保存、DTRACE、开环 PWM、方向或机械零点命令。

本工具会记录原遥测配置，退出后保留所选通道和 20ms 周期，不自动恢复旧配置。恢复使用原上位机时请重新选择其波形订阅。选择新模式、Start 或连接动作可能让板端切换通道。v1.1 仅在请求新 SUB 后、收到其完全匹配的确认前，允许忽略与查询到的原字段列表及顺序完全相同的旧公告，日志标记 `OLD_SUB_BEFORE_ACK`；期间二进制数据仍不解码，不延长超时，不自动重发。收到目标 SUB 后立即关闭此例外。其他不一致的 SUB、未知字段、错长度或时间倒退仍中止，不猜同长度字段含义。

`capture` 用于记录人工选择的实验。它只采集，不指挥用户做任何启动动作：

```text
python speed_loop.py capture --device-ip BOARD_IP --seconds 10 --output speed_capture.jsonl
python speed_loop.py analyze speed_capture.jsonl --output speed_report.json --svg speed_waveform.svg
```

实际实车试验需另行安排可靠支撑、隔离、防护和人工停止手段，本包的离线测试不等于实车验证。

## 第二步：仅预览要发的一个参数

```text
python rate_loop.py send --set rate_kp=5.3
python angle_loop.py send --set ground_angle_kp=10
python speed_loop.py send --set wheel_speed_kp=0.001
```

不带 `--apply` 时完全离线，连 UDP socket 都不会打开，只显示指令、专用回读字段和 RAM 性质。这里的数值是源码中的示例，不是新的推荐调参值，不保证与你板上当前值相同。

本版本只支持以下参数，范围来自固件允许范围，**不是安全运行范围**：

| 入口 | 写入名称 | 允许范围 | 正确回读字段 |
|---|---|---|---|
| rate | rate_kp | 0～500 | ground_rate_kp |
| rate | rate_pwm_limit | 0～8000 | ground_pwm_limit |
| angle | ground_angle_kp | 0～100 | ground_angle_kp |
| angle | ground_rate_limit_dps | 0.1～200 | ground_rate_limit_dps |
| speed | wheel_speed_kp | 0～0.1 | wheel_speed_kp |
| speed | wheel_speed_ki | 0～0.1 | wheel_speed_ki |
| speed | wheel_tilt_limit | 0～5 | wheel_tilt_limit |
| speed | wheel_i_limit | 0～5 | wheel_i_limit |

重要：停止状态下旧的 `rate_kp` 遥测可能显示**飞轮**参数，所以本工具始终读 `ground_rate_kp`。不开放符号参数、原始 PWM、机械零点、位置参数或批量写入。`balance_angle_x` 缺少严格的同值参数遥测回读，本版本不发送它。

## 第三步：人工确认后写一个 RAM 参数

只在已完成这些检查后使用下面两个确认选项：

- 在板端实际执行 Stop，退出所有 PWM 预览和飞轮开环测试。
- 电机动力已按硬件要求隔离，不仅仅是屏幕显示零。
- 无人操作 Start/模式/菜单，没有其他上位机或网络发送器。
- 已保存当前参数记录，确认准确写入名称和值。

```text
python rate_loop.py send --device-ip BOARD_IP --set rate_kp=5.3 --apply --confirm-board-stopped --output rate_write_check.jsonl
```

上述仍为数值示例。第一次建议从通信日志读出当前 `ground_rate_kp`，用**当前相同的值**检查发送、ACK、回读路径，不故意改变闭环行为。不同参数分别执行，程序永远不会自动从角速度环切到角度环或速度环，也不会自动启用下一环。

写之前要求至少 10 个新鲜、严格递增的板端时间帧，且板端和本机接收时间都跨过至少 200ms，观察到：

`control_mode=0, rate_state=0, rate_enabled=0, pwm_debug_enabled=0, fault_reason=0`，以及三路已发布 `motor_*_pwm=0`。

这些是**软件观察门槛，不是硬件停转保证**。当前固件菜单的开环预览可能不更新这些遥测字段；UDP 没有认证、事务号或板端“只在停车时写”的原子联锁，最后一帧与发送之间也有时序窗口。因此仍必须人工 Stop、退出开环、隔离动力并保持单一发送器。程序不会尝试自动归零未知测试参数。

发送后同时要求：

1. 精确参数名的 `MCAR SLIDER name value` ACK，双方转换为 float32 后精确相等比较。
2. ACK 之后连续 3 个新鲜遥测帧在**专用参数字段**读到该值。
3. 这些帧继续满足停止状态观察。

成功只表示“RAM 写入 ACK 和参数回读已确认”，不表示 Flash 保存、物理电机状态、驱动响应或闭环稳定性已验证。ACK 格式来自 `wifispi.c` 中赋值后重新读 RAM 的结果。`GET?` 不返回参数值。

如果丢 ACK、超时、重连、错误、通道变化或状态变化，显示 `WRITE OUTCOME UNCERTAIN`：可能已写入，也可能未写入。**不自动重试，不自动回滚，不自动恢复默认值**。先检查板端当前 RAM 参数与状态，再决定下一步。单次运行只允许一个参数。

## 三环和字段的实际含义

- 主轮/地轮 = 屏幕 Pitch = 内部 `imu_roll_deg` / `gyro.x`，不是飞轮 Roll。
- Mode7：角速度目标固定零，只抑制转动，不回角度零点。
- Mode4：角度 P → 角速度 P → PWM。
- Mode2/10：位置 P → 目标速度 → 速度 PI → 倾角修正 → 角度 → 角速度 → PWM；同时有飞轮控制。本工具不会为了“单速度环”自动关位置环。
- 角度/角速度控制周期 2ms；编码器约 20ms 发布新窗口，速度 PI 只在新窗口更新；位置环约 100ms。
- 编码器速度单位为 counts/s，每轮一圈 4096 counts；RPM = counts/s × 60 / 4096。未确认轮径，不转换成 m/s。
- 速度 PI 输出是目标倾角偏置，不是电机 PWM。当前源码速度倾角映射符号为 -1。
- `rate_pwm` / `dual_pitch_pwm` 是死区补偿之前的 PID 值；`motor_ground_pwm` 才是最终已发布命令。当前源码正/负补偿不同，分析最终 PWM 限幅而不只看旧饱和标志。
- 遥测默认 20ms，慢于 2ms 内环。包不会声称从这类下采样记录求出了精确带宽、稳定裕量或所有尖峰。
- 本包只给观察统计、限幅比例、有效性和 P-only 一致性残差；不自动建议或下发新的增益。

## 退出码和问题定位

- 0：命令完成。分析状态 `OBSERVATIONS_ONLY`，不是“已稳定”。
- 2：缺字段、无有效样本、参数/协议/端口等错误；检查消息和日志。
- 3：参数写入结果不确定；不要盲目重试。
- Ctrl+C：退出，不自动发任何电机控制或回滚命令。写入中中断也应按结果不确定处理。

典型问题：

- 8081 已占用：先关原上位机 UDP 接收。不要同时启动多个本工具入口。
- 没有 ACK/数据：核对板端 IP、电脑目的地址、8081/5001、WiFi 和本机防火墙。保持停车，不用启动电机“试通”。
- 未知字段/通道不匹配：板端版本不兼容或有人换了模式/订阅；保存日志核对版本，不盲改字段顺序。
- 旧记录不足：重新订阅所需通道后采集。不要用缺编码器的飞轮/双轴旧记录声称速度环通过。

## 验证边界

包内测试使用合成数据与 127.0.0.1 本机伪设备，覆盖参数白名单、范围、浮点、模式/状态、协议、ACK/回读、丢包错误和三环离线分析。没有连接真实板子，没有改用户电脑或原工程，没有烧录和实车测试。详见 `VALIDATION.md`。
