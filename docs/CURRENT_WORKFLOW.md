# 当前工程入口与整理边界

核对日期：2026-11-10。适用范围：当前 OneWheel_master_RA_EKF_review_fixed_20261109 工程目录。

## 先看这些

- [分阶段调试说明](../COMMISSIONING.md)：原阶段 A/B 的用途、物理隔离要求与限制。它不是后续 C 调试功能的完整说明。
- [构建配置](BUILD_CONFIGURATION.md)：唯一 Debug/TASKING 配置、源码排除规则和 78 项参数说明。
- [SAFE UART 台架约定](SAFE_UART_BENCH.md)、[外板 SAFE v1](CYT2BL3_SAFE_V1.md)、[存储安全](STORAGE_SAFETY.md)。
- [历史说明与本次恢复清单](history/2026-11-10_cleanup/README.md)：区分历史操作、旧参数和当前入口。
- [原 IMU660RA 适配状态](../code/HW_Drivers/IMU/RA_PORT_STATUS.md)。

本次只整理文档和一个可再生成缓存，不改变 IMU660RA、原 500 Hz EKF、主控默认输出锁或 200 ms 约束。历史文档中的 Mode3～8、IMU660RB/r10h、75 项参数和“已通过”等描述不能直接套到本工程。

## 当前实际存在的工具入口

| 路径（相对工程根目录） | 用途与边界 |
| --- | --- |
| tools/c_link_probe.py | 现有仅观测 UDP 链路探针；仍有已知主机调度漂移，不能据此宣称问题已修复或允许带动力。 |
| tools/c_bench_cli.py | C 调试命令行工具；存在文件不代表当前可以安全输出或进行有动力测试。 |
| tools/commissioning_cli.py | 原分阶段调试串口工具，适用范围见 COMMISSIONING.md。 |
| tools/unicycle_station/start_windows.bat | Embedded Station 0.6.1 通用 Qt 上位机源码入口，经 run_station.py 进入 app/main.py。首次启动可能创建 .venv 并安装依赖。 |
| tools/unicycle_station/start_app.bat | 旧打包入口，要求 dist/EmbeddedStation/EmbeddedStation.exe；本次盘点未发现该 dist 目录。 |
| tools/unicycle_station/build_windows.bat | 安装依赖、运行 pytest，再调用 scripts/build_windows.py 打包。 |
| tools/ads/build.py | ADS 构建辅助工具，说明见 tools/ads/README.md。 |

当前没有 start_c_probe.bat、app/ui/c_probe_window.py、app/services/c_probe_backend.py，app/main.py 也没有 --c-probe 参数。专用 C 探针 Qt 补丁尚未合并；本次整理没有合入 GUI 或其他新功能。

不要从旧 Mode 文档直接执行启动、PWM、Flash 或网络命令。整理完成不构成烧录、连接硬件或有动力测试许可。

## 测试入口与保留原因

- tests/run_review.py：集中引用当前评审用例及 tools/tests 中的 Flash、存储、路径测试。
- tests/run_all.py：根 tests 与 tools/unicycle_station/tests 的 pytest 入口。
- tests/new_ekf/run.py：原 IMU/EKF 的 C 夹具、模拟头和 .c.in 回归入口。
- tests/imu_ra/run.py：兼容转发入口，转到 tests/new_ekf/run.py。
- tools/attitude_r10h/run_tests.py：仍被上位机 test_imu_forward.py 引用，不能因目录名旧而整删。
- tools/unicycle_station/tests/firmware_host：真实固件协议测试夹具，包含 .c、.h、.inc，均不是临时输出。

测试和工具保留原目录。部分测试会编译 C、写临时文件或使用本机 UDP 回环，因此不属于纯只读检查。本次没有执行任何测试、构建、安装器或工具程序，也未进行硬件验证。

## 数据与构建输出

- 保留根目录 c_link_probe_params.json、c_link_probe_result.json，作为当前未解决链路问题的证据。
- 保留 validation 全部报告、日志、截图、备份及历史文件清单。
- 保留 Debug 的最新 ELF、HEX、MAP、MDF 及其他输出，不把它们当普通缓存清空。
- 保留 tools/unicycle_station/docs 的参数快照、分析 CSV/截图，以及 examples 中被测试读取的示例会话。
- 不读取私密 WiFi 头内容，不改校准、用户配置或工作源码。

.cproject 当前只有 Debug 配置，以工程根扫描源码，并排除 tools、tests、validation、docs、build、.venv 等目录。不要把 C 测试夹具搬到未排除的新根目录，否则可能被 ADS 编入固件。
