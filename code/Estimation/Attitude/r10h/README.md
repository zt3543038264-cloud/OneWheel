# 独轮车的 A+ r10h 接入

来源：`D:/项目/姿态解算/660RB_Aplus_r10h_Portable`（2026-10-04）。本目录仅引入 core/include/src 与许可证，没有把 examples/tests/reference_tc264 加入固件。

六个 C 文件由 ADS 自动发现；`.inc` 只由其所属 C 文件包含。候选数学文件全部逐字节保留，入口增加 TASKING `cpu0_dsram` 分段，外层接口使用相对头文件路径；不用修改 ADS 搜索路径。初始化增益、A/B/C 内部状态、滤波、100 ms/1 s 零偏窗口不变。

唯一算法配置差异：`core/v3_calibration.h` 使用原包的单位矩阵模板，保留 `C_MOTION_KP` 和 `R10_NORMAL_STARTUP_GAIN`。原实测矩阵属于另一颗传感器及其测试坐标，未直接继承。标定应在独轮车实际车体系下进行；换前向/轴映射时，非单位矩阵须重新标定或作 `M_body = R M_sensor R^T` 变换。

`source_origin.json` 与 `UPSTREAM_SHA256SUMS.json` 描述来源原包，不是当前移植目录的有效哈希清单。当前差异和哈希见 `migration_manifest.json`。原作者声明保留，GPLv3 全文在 `LICENSES/GPL-3.0.txt`；本地新增/修改层按 GPL-3.0-or-later 提供，各原始文件许可声明保持原意。

完整接入及验证说明：[A+ r10h 移植说明](../../../../docs/design/Aplus-r10h移植说明.md)。
