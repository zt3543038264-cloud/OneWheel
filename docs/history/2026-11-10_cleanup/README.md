# 历史说明索引与恢复方法

整理日期：2026-11-10。返回 [当前工程入口](../../CURRENT_WORKFLOW.md)。

本目录只收纳确定过时、没有活动依赖且没有 Markdown 相对链接的说明。原文保持逐字节不变，不将历史结论改写成当前验证结果。

## 本次归档

| 原工程根路径 | 本目录文件 | 归档依据 |
| --- | --- | --- |
| ANGLE_RESPONSE_AB.md | [ANGLE_RESPONSE_AB.md](ANGLE_RESPONSE_AB.md) | 原文明确区分历史角度滤波与旧 Mode3～7 的反馈路径。 |
| ANGLE_TUNING_DATA.md | [ANGLE_TUNING_DATA.md](ANGLE_TUNING_DATA.md) | 旧 28 通道协议与 Route_Run 操作；引用的 tools/angle_telemetry.json 当前不存在。 |
| MODE8_STEP_TEST.md | [MODE8_STEP_TEST.md](MODE8_STEP_TEST.md) | 旧 Mode8 阶跃操作，原文已有乱码及不存在的旧测试路径；保存原始字节供追溯。 |

工程内文本引用检索只发现 validation/FILE_SHA256.json 对上述原路径的历史记录，没有代码、测试或当前说明依赖。该冻结清单保持原样；整理后的路径对应关系以本目录 restore_manifest.json 为准，不能把旧清单误当整理后的现状。

归档文档中的代码路径和命令仍是历史的工程根目录写法，不应在本目录执行，也不代表当前功能受支持。

## 原位保留的历史说明

以下文件有入链、相对链接、报告属性或尚不能排除的用途，未移动、未改写：

- [ADAPTATION_REVIEW.md](../../../ADAPTATION_REVIEW.md)：有 COMMISSIONING.md 入链及导入说明相对链接。
- [ASC_PORT.md](../../../ASC_PORT.md)、[ASC_PORT_VALIDATION.md](../../../ASC_PORT_VALIDATION.md)：README、版权说明和彼此之间仍有引用。
- [VALIDATION.md](../../../VALIDATION.md)：历史验收报告保留原位。
- [FAST_FOC_DIAGNOSIS_20261028.md](../../../FAST_FOC_DIAGNOSIS_20261028.md)：实测结论和大量相对证据链接，原位保留。
- [DUAL_BALANCE.md](../../../DUAL_BALANCE.md)、[GROUND_BALANCE.md](../../../GROUND_BALANCE.md)、[GYRO_TEST.md](../../../GYRO_TEST.md)、[WIFI_GYRO_TEST.md](../../../WIFI_GYRO_TEST.md)：旧 Mode 文案，保留其 README/模式/菜单/WiFi 相对链接。
- [MODES_STARTUP.md](../../../MODES_STARTUP.md)：旧编号模式，仍被上述文档引用。
- [MODE10_CONTROL.md](../../../MODE10_CONTROL.md)：仍被 GROUND_DEADZONE.md 和 WIFI.md 引用。
- [IMU.md](../../../IMU.md)、[MENU.md](../../../MENU.md)、[PROTECTIONS.md](../../../PROTECTIONS.md)、[WIFI.md](../../../WIFI.md)、[ERRORS.md](../../../ERRORS.md)：混有历史状态和旧参数，暂作历史参考，不能替代当前源码和安全说明。
- docs/design 下的 r10h、500 Hz/1 kHz 试验和 IMU 前向切换说明：原位保留，历史算法/方向/时序不能套用于当前 RA 固定机体系。

## 工具、测试与证据

活动 tests、tools 及夹具原路径全部保留。旧 mode7/mode8/mode10、roll/ground 工具与 qcar_monitor_update 覆盖包也未移动或删除，避免破坏相对导入、原覆盖布局或尚未确认的用途。

validation、诊断 JSON、参数快照、分析数据、备份、Debug 产物均不变。相同的三份语法报告和零字节逐文件日志也保留，不能因为内容相同就抹掉不同阶段的证据。

本次只删除 tools/__pycache__/c_bench_cli.cpython-314.pyc。它可由保留的 tools/c_bench_cli.py 再生成。没有为整理而运行 Python、测试、构建、安装器、串口或网络操作。

## 恢复

restore_manifest.json 记录三份说明的原路径、新路径、字节数和 SHA-256，也记录 README 前后哈希、原始字节备份与唯一缓存清理。

恢复时先核对 SHA-256，并确认目标路径没有后来新增的文件，不能覆盖后续工作：
1. 将三份归档说明按清单移回工程根目录。
2. 若确需恢复旧 README，先备份后续 README，再将 README.before-cleanup.bytes 的原始字节恢复到工程根 README.md。该 .bytes 文件是恢复副本，不是可直接浏览的导航页。
3. 本次新增的入口和索引可保留；如移除，先核对没有后续引用。已清理 .pyc 不需要恢复，后续正常使用 Python 时可再生成。

以上仅为恢复说明，本次没有执行恢复或任何项目程序。
