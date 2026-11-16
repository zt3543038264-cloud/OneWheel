# 工具入口

整理日期：2026-11-16。此压缩包不包含 `unicycle_station` 主上位机，切勿用它覆盖/删除整个本机 tools 目录。

## 当前工具（原文件内容保留）

- `wifi_status_latency.py`：只读状态往返测量。
- `c_bench_cli.py`、`c_bench_trace.py`、`keep_wire_log.py`：C 台架与报文日志。
- `dry_heartbeat_check.py`、`c_link_probe.py`：心跳/链路观察，适用 profile 需与当前固件一致。
- `powered_rate_cli.py`、`graded_wheel_pulse.py`：有动力工具，本轮不运行；目前用户暂停主轮测试。
- `ads/`：构建工具。
- `tests/`、`powered_rate_validation/`：离线回归。

网络地址以当前实测为准；命令行选项以各工具 `--help` 为准。

## 有依赖的历史工具仍保留

- `attitude_r10h/`：旧数学核心回归和兼容链被其他测试引用，不能整删。pre_migration 源码备份已从本整理副本移除，原上传包仍保留。
- `cyt2bl3_driver/`：历史覆盖补丁，不能用它覆盖当前已解锁的新驱动；保留其许可证与源码参考。
- `cyt2bl3_trace/`、`driver_trace_client.py`、`recover_driver_trace.py`、`analyze_driver_trace.py`、`mode8_step_capture.py`、`analyze_mode8_step.py`：彼此关联的旧驱动追踪/分析链，保留以读取历史数据；不是当前菜单测试入口。

移除旧 McarPVC 上位机补丁、重复独立 Pitch 预览 GUI、旧 Mode7/10 调参脚本、旧 MCAR 订阅工具和 Python 缓存。主上位机、固件、原始测量数据不在本轮删除范围。旧预览目录的 `1.csv` 单独保留，不推断其是否为真实测量或模拟数据。
