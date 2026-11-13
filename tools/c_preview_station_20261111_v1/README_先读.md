# Pitch 只计算预览上位机 v1.0.0

这是独立、可手动放入工程的 Qt 工具。没有替换原上位机、没有修改任何固件，也没有连接或操作硬件。它使用现有固件的 **Pitch / rate 控制器预览**，不是 cbench dry_rate，也不是上位机模拟控制器。

## 安装和启动

1. 将完整 `c_preview_station_20261111_v1` 文件夹解压到已指定工程的 `tools` 目录，与 `unicycle_station` 并列。若同名目录已存在，先保留旧包，避免覆盖。
2. 目录结构应为 `tools/c_preview_station_20261111_v1/start_preview.bat`，以及已有的 `tools/unicycle_station/.venv/Scripts/python.exe`。
3. 先关闭通用上位机、CLI、探针等可能占用 UDP 8081 的程序。
4. 双击 `start_preview.bat`。启动只打开界面，不连接、不发命令。它只使用原上位机现有的 Python、PySide6、pyqtgraph，不安装依赖、不写用户配置。
5. **先物理断开电机动力电源，保留主控/传感器逻辑供电。** 确认设备稳定支撑、没有其他控制程序，再勾选界面的确认框。软件不能验证物理隔离。
6. 检查设备 IP/端口与本地绑定。默认 `192.168.0.111:5001`，本地 `0.0.0.0:8081`；这些只是原 CLI 默认值，不是本次已验证的在线地址。点“启动一次预览”。
7. 正常将显示 Pitch 角度、角速度和**计算值（未施加）**三条曲线。一次最多 10 秒。可以提前点停止；关闭窗口时会先请求停止。每次重新运行都需重新勾选，不自动重连或重启。
8. 使用“导出 CSV”保存到一个新文件。已有文件不会覆盖。CSV 包含 MCU elapsed_ms、主机收包单调时钟、真实预览计算值、状态和缺口标记。

## 本工具实际做什么

固定命令白名单仅包括：`hello`、`cbench_status`、`preview_status`、`preview_start,pitch,rate`、`preview_stop`。

- 先核验固件协议标识 `tc264-cfg1-task1-path1-sign2`、预览 schema v1、C bench profile=0、bench 非运行且报告 applied=0、没有已运行的其他 preview。任一不符，不启动。
- 仅发送一次 preview_start。不发送 cbench_start / cbench_keep，不写 PID、不写 Flash、不进入 signal、Jog、Run、Balance，也不改 MCU 的 200ms lease。
- 每次收到状态后间隔约100ms再查询，查询超时750ms。这个超时只用于观察请求，不改变固件时限。UDP帧必须来自指定端点，序号和命令精确匹配。
- 超时、坏报文、轴/环变化、MCU 时间倒退或停止前进时终止本次观察，发一次 preview_stop。start/stop 不自动重发。
- 停止后独立查询一次状态，即使 stop ACK 丢失也尝试查询。只有匹配 stop ACK 和后续 inactive 状态都收到，才显示“MCU 已报告预览停止”。这仍然不是物理停机证明。
- 若尚未发送 start 就失败，界面显示“未启动预览”，不误报停止失败。

## 与 C bench dry_rate 的区别

现有 `control_preview_run()` 调用 `pitch_test_ctrl()`，使用当前固件参数中的 `p_rate_pid`。rate 模式的角度/速度外环被复位，角速度目标为0，误差为 `-pitch_euler_rate()`；它不是 C bench 通过 target/kp/ki 指定的独立 PI。

在所读源码中，预览在约1ms控制循环中计算，保留原 IMU660RA 与 500Hz EKF 路径。计算结束后每拍调用 `control_zero_sink()`，普通整车输出编译锁保持0。预览自身最长10000ms，传感器异常或状态冲突会终止。C bench 独立的200ms保活保护完全未改。

本工具只显示经网络抽取的快照，不是1kHz/500Hz完整采样记录。无线间隔超过250ms时曲线断开，不补点、不插值。连续重复的 active elapsed_ms 会被视为失去进展并停止。界面超过750ms没有新收包显示过期，保留旧值而不是伪造归零。主机收包时间不是 MCU 数据产生时间。

## 必须知道的限制

- **没有修复 WiFi 丢包或长延迟。** 既有探针记录 MCU 最大收包间隔431292/482822微秒，并超过200ms影子保活边界。独立 C bench 连续记录问题仍未解决。
- 现有 hello 标识不是构建哈希，profile 状态也不能证明板上镜像与本次读取的源码一致。请核对你实际烧录的版本；本次没有烧录、联机、读取板上版本或进行物理输出测量。
- preview_status 没有实际电机电流、PWM引脚读回或物理停止字段。计算值不能当作实际输出；A/B链路新鲜标志也不表示物理安全。
- 现有 preview 协议没有运行 nonce 或启动包有效期。极端UDP乱序时，已发送但延迟到达的 start 理论上可晚于 stop 处理；软件停止确认仅描述那次查询时的状态。因此必须保持动力物理断开，不能将本工具用作带动力测试或独立紧急停止装置。
- `preview_stop` 在当前固件中也调用 `control_c_bench_stop()`。所以预览期间必须独占控制入口，不要从另一程序/本地菜单启动 C bench 或其他模式。
- 预览使用现有固件参数，工具不读取完整参数快照，也不允许调参。不要把不同参数下导出的曲线直接当作同一个控制器配置比较。
- 软件时限依赖 MCU 正常执行，不是硬件看门狗。停止未确认时保持物理隔离，不根据界面推断真实机械状态。

## 离线检查

- 双击 `test_offline.bat`，执行标准库 unittest 假传输测试，不打开socket、不接硬件。
- 命令行 `start_preview.bat --demo`：明确标记的离线合成数据演示，不接硬件。
- `start_preview.bat --smoke 新截图.png`：离线渲染截图，不接硬件；目标必须不存在。

## 本次交付验证

- 19项离线协议/生命周期测试通过，包括450ms观测延迟、丢start ACK、丢stop ACK、取消、错误profile、忙状态、错误序号/命令、多行报文、非有限值、MCU时间不前进等。
- Python文件语法检查通过。
- 独立代码审查发现的预览时间停滞、取消边界、未启动误报、旧identity残留问题均已修正。
- 云端没有PySide6/pyqtgraph，所以**本次没有运行Qt界面、渲染截图或完成真实Windows启动测试**。启动脚本只复用项目已有环境。没有运行固件构建、C集成夹具或硬件测试。

源码核查依据和哈希见 `SOURCE_AUDIT.json`；离线测试实录见 `TEST_RESULTS.txt`。这些记录对应2026-11-11所读文件，不等于板上固件版本证明。
