# 审查修复包构建配置与上位机 Schema

日期：2026-11-09。所有修改及检查仅作用于独立云端副本；没有连接车辆、串口或烧录器，没有修改原始压缩包。

## 唯一支持的工程配置

- `.project` 工程名保持 **Qcar**。请在一个独立、干净的 ADS 工作区中导入本包，避免与原来的同名工程混用。导入后确认工程根目录确实是本包，再选择 **Debug**。
- 本包仅保留 **Debug / TASKING / TC26xB / BGA292**。优化级别 O0、fastSingle、现有链接脚本及 FPU/runtime 链接选项保持不变。
- 已删除旧 `Release`、`External GCC - Debug`、`External GCC - Release` 配置。它们指向另一工程 `ASC_SummerTraining_Unicycle`，缺少当前源码排除项，不能当作本包的可选构建方式。若需要 Release 或 GCC，应单独生成、审查并验证配置，不能直接恢复旧节点。
- Debug 的 86 个 include 目录均使用当前 `${ProjName}` 并在本包内实际存在；构建输出为当前工程的 `Debug/`。工程级库根目录、自动发现保留路径、刷新范围和扫描器引用均已与本树一致。
- 原有源码排除项保留，包括旧 r10h、`tools/`、`tests/`、`validation/`、独立双 FOC 工程。新 EKF 的三个 `.inc` 仍仅由 `code/HW_Drivers/IMU/imu.c` 引入，不独立编译，保留原 IMU660RA、500 Hz 解算与轴映射。
- 显式链接 `libraries/zf_device/zf_device_config.a`、`-lcs_fpu`、`-lfp_fpu`、`-lrt`、`--lsl-core=vtc`；CPU 启动代码保留路径不变。

本次配置修复没有开启任何电机输出或 Flash 写入验证闸门。导入和编译并不构成上车许可。

## 上位机 78 项参数同步

当前固件参数表为 **78 项**。新增补齐的 Roll 陷波参数如下，排列顺序与固件表一致：

| 参数 | 默认值 | 范围 | Schema flags |
| --- | ---: | --- | ---: |
| `r_rate_notch_hz` | 47 Hz | 5–200 Hz | 21 |
| `r_rate_notch_q` | 7 | 0.5–20 | 21 |
| `r_rate_notch_db` | 0 dB，严格旁路 | −40–0 dB | 21 |

内置 Profile、默认工作区、当前配置默认值、Mock Schema 和纯软件示例会话已同步。在线调参白名单与固件显式名单一致；运行中写入仍要求 Schema 权限及有效状态位，不能凭参数前缀推断权限。Mock 不再偷偷补写权限位来掩盖旧 Profile。

`tools/unicycle_station/docs/qcar-defaults.json` 是由当前源码推导的配置默认值，明确标记 `hardware_readback: false`，不是实车读回。原 `defaults-20260914.json` 和 `defaults-20260921.json` 作为历史证据保留，不改写为新固件的 78 项读回结果。`examples/demo_session` 也是重新生成的纯软件日志，不是测量记录。

生成器保留审查版的固定机体系提示、A Left/Yaw 与 B Right/Roll 通道语义、Path 状态位及安全警告；不会因重生成而恢复旧双轮混控或可变 IMU frame 文案。缺失默认值或默认值越界会直接失败，不会自动用 0 代替。

```sh
python tools/unicycle_station/scripts/generate_profiles.py
python tools/unicycle_station/scripts/generate_profiles.py --check
```

第二条命令只检查，不写文件。更新示例日志另行运行 `python tools/unicycle_station/scripts/generate_example_log.py`。

## 已完成的离线检查

- `test_build_configuration.py`：唯一支持配置、当前工程 include/库根目录、链接依赖、源码排除与 `.inc` 单一归属。
- `test_station_schema_contract.py`：编译执行实际固件参数表、默认加载函数和协议分组/权限函数，逐项比较全部 78 项名称、顺序、类型、上下界、默认值、分组与 flags；另查生成可复现、Mock 公布的 Schema 和不擅自增加权限。
- `test_default_snapshot.py`：配置默认值与实际 `param_load_defaults()` 一致，保持浮点值范围检查；lean 范围测试保留数值约束。
- `test_live_parameters.py`：包含三个 notch 参数的在线写入确认、RAM dirty、保持运行且不隐式保存；固定 frame 的拒绝/无操作行为仍保留验证。
- `validation/review_build_schema/syntax_ilp32.py`：使用本包 Debug include 路径及真实 SDK/驱动头，对 21 个活动应用/用户 C 文件及 SPI、Flash、IMU660RA 驱动进行 **host GCC ILP32 语法检查，24/24 通过**。具体结果在同目录 `syntax_ilp32.json` 和 `syntax-summary.log`。

## 仍未完成的目标机验收

云端没有 TASKING `cctc` 或 TriCore GCC；**没有完成 TASKING 编译/链接，也没有验证 ADS 干净工作区实际导入**。静态 XML 检查不能替代这两步。

ILP32 检查只使用主机 `gcc -m32 -std=c99 -fsyntax-only`，选择 HIGHTEC 预处理分支。为在 Linux 上解析 Windows 风格大小写头文件和缺失工具链头，需要临时大小写别名、两个 intrinsic 的声明及空 32 位 glibc stubs 头；ISR 属性可能被主机编译器忽略。它不生成目标代码，不链接可执行文件，不证明目标工具链兼容、ISR 向量正确或硬件安全。

交付后仍须人工用 ADS/TASKING 对唯一 Debug 做 clean build，检查 ELF/map 中的链接对象、段分配、DSPR、栈/CSA/FPU runtime 与实际余量，再进行独立的受控台架时序、方向及故障注入验收。不得据此文档直接开启默认关闭的电机或 Flash 闸门。
