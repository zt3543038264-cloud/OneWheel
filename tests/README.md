# 测试入口与适用范围

整理日期：2026-11-16。测试不是硬件操作步骤，不代表带动力测试通过。

## 入口

- `python tests/run_all.py`：pytest 总入口，依赖完整工程及上位机测试依赖。
- `python tests/run_review.py`：旧审查版重点入口，包含完整工程 `tools/tests` 的存储测试。
- `python tests/new_ekf/run.py`：IMU/EKF 回归。
- `tests/imu_ra/run.py`：兼容入口，转发至 new_ekf，不是第二套解算；保留避免破坏外部调用。

## 未删除的原因

本包不含最新完整固件、工具及上位机。C/H/INC、mocks 和模板都是测试依赖，不是缓存。
以下旧审查测试包含历史限定，当前输出解锁后可能需要按当前源码更新预期：
`test_qcar_driver.py`、`test_orthogonal_review.py`、`test_wmotor_safe_bench.py`、`test_wmotor_driver_integration.py`、`test_external_driver_safety.py`、`test_commissioning_control.py`。
其中也覆盖几何映射、解析边界、停止与故障处理，不能整文件删除，不能为得到绿色结果直接跳过或删掉失败断言。
其余协议、心跳、菜单、相机、SPI、EKF 测试也继续保留。

本轮完成 Python 语法检查、引用盘点及 6 套标准库离线单测（99 项）；分级脉冲测试夹具已同步当前 7 档、1 秒、19 次心跳与 20 ms 调度容差。工具和固件控制逻辑未改变。完整 pytest 回归未运行：当前环境无 pytest，上传包也不含完整主控与主上位机源码。
下一次整合最新完整工程时，应逐条区分“代码回归”与“历史契约变化”，再更新对应测试。
