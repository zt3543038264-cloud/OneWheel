# Qcar - complete reference-layout port

Read [ASC_PORT.md](ASC_PORT.md) for this vehicle, [validation](ASC_PORT_VALIDATION.md), and the retained upstream guide below. This revision uses the original reference menu, 8x16 LCD font, layered application and station protocol. Hardware pins, physical zeros, driver text protocol and local network configuration are Qcar adaptations.

2026-11-12：项目公共配置统一到 [`code/config.h`](code/config.h)，包含原 board/direction/中断/WiFi 公共参数。热点名、密码和电脑目标 IP 仅填写未跟踪的 `code/config_private.h`，模板为 [`config_private.example.h`](code/config_private.example.h)。公共端口为电脑1349、模块6666。本次保留原参数数值与控制逻辑。ADS工程改为UTF-8，原GBK库按目录/文件保留编码；打开旧编辑页时请先保存自己的修改，再关闭重开。详见 [配置与编码说明](docs/guides/配置与编码说明.md)。

2026-11-09：Roll角速度内环改为1ms增量式PI，独立配置左右编码器反馈极性，统一Speed回收方向及单轴测试目标。旧Rate KD固定为0，其他Flash参数继续加载。当前逻辑、验证边界和上板顺序见 [Roll方向与增量PI修订](docs/design/Roll方向与增量PI修订.md)；下方历史说明中的位置式Roll Rate和回收减号以该修订为准。

左右飞轮编码器极性现在可在`Params → Motor → Enc A / Enc B`分别修改为±1，停车修改后立即生效，通过Motor页`Save`保存并在重启后恢复。它们独立于`Dir A / Dir B`，不改变PWM输出方向。操作与验收见 [Motor编码器极性菜单](docs/design/Motor编码器极性菜单.md)。

最新Roll默认参数按用户2026-11-09提供的实车菜单照片固化：Rate KP/KI为**45/0.05**，Angle KP/KI/KD为**30/0/20**，Speed KP/KI/KD为**0.0001/0/0**，Rate陷波为**45Hz、Q=3、dB=0（旁路）**。用户报告当前参数可稳定直立；本次软件验收没有代替实车复测。完整参数及适用范围见 [Roll实车参数记录](docs/design/Roll实车参数20261109.md)。

2026-11-09：旧 Qcar 信标识别、时序追踪和三目标路径排序已恢复到 `code/Estimation/Beacon/`，通过 **Params → Beacon** 调试及启动运行；Mode=0 保留原循迹，Mode=1/2/3 为距离/时序/增强时序识别。当前 Roll 参数和增量 PI 保留。菜单操作、参数、相机标定及验证边界见 [信标识别与运行](docs/guides/信标识别与运行.md)。

2026-11-10：修复摄像头启动/曝光命令覆盖，支持OFF状态启动及初始化失败重试；重启后拒收旧快照、清除旧信标追踪历史，修复BIN+L的信标中心标记显示。Image新增RAW/LCD帧号、原始灰度min/avg/max、VS/DM/DROP计数和曝光请求/应答状态。`Params → Camera`提供`Diag 50/512`原始采图及`Normal Camera`恢复操作。正常PID、零点、引脚和WiFi配置保留。

本轮软件验收：**51项相关检查通过，ADS编译0错误、0警告**，新HEX与当前源码哈希已核对。用户随后反馈 **Image已有画面**；当时的显示模式、曝光和帧率未单独记录，尚不能确定此前黑屏的唯一原因，也未完成信标识别和自动行驶的实车验收。操作说明见 [摄像头黑屏诊断](docs/guides/摄像头黑屏诊断.md)，软件验收范围见 [本轮验证记录](docs/reference/camera-image-validation-20261110.json)。

---

# Q-SmartCar

TC264D 双核 Q 型独轮车工程，包含三轴平衡、视觉循迹、路径示教与惯性航向导航、车身菜单和 Windows 上位机。

> 更新日期：2026-11-10；当前源码为95项参数（含15项Beacon），本轮摄像头诊断选项不增加Flash参数。方向菜单 +1 代表当前已调好的车辆基准；详见 [方向基准整理](docs/design/方向基准整理.md)。编译默认值来自 `code/config.h`，上电实际参数以 Flash 加载后的 `g_param` 和 MCU Schema 为准。

## 文档入口

| 文档 | 内容 |
|---|---|
| [文件结构与程序架构](docs/文件结构与程序架构.md) | 分类目录、每个应用文件职责、双核调度、控制/视觉/路径数据流及维护入口 |
| [A+ r10h 移植说明](docs/design/Aplus-r10h移植说明.md) | 1 ms 解算、启动静止、单位矩阵、异常保护与验证边界 |
| [角度外环 1 kHz 试验说明](docs/design/角度外环1kHz试验说明.md) | 1 ms 同拍调度、参数自动匹配与对照试验 |
| [控制调试指南](docs/guides/控制调试指南.md) | 当前 PID、混控、模式、安全退出和调试顺序 |
| [视觉调试指南](docs/guides/视觉调试指南.md) | 整帧逐行提边、IPM、中线、元素与诊断 |
| [信标识别与运行](docs/guides/信标识别与运行.md) | Beacon模式、目标筛选、三目标局部路径排序与菜单操作 |
| [摄像头黑屏诊断](docs/guides/摄像头黑屏诊断.md) | 启动/曝光状态、RAW/LCD/灰度统计、例程诊断及恢复正常处理 |
| [菜单指南](docs/guides/菜单指南.md) | 当前页面、动作、参数与保存范围 |
| [路径导航指南](docs/guides/路径导航指南.md) | 手推/遥控采点、双槽保存、路线显示与低速自动复现 |
| [参数参考](docs/guides/参数参考.md) | 历史75项基础参数说明；新增参数和当前默认值以Schema及专项修订文档为准 |
| [调参命令](docs/guides/调参命令.md) | UART2 命令与遥测入口 |
| [上位机说明](tools/unicycle_station/README.md) | 安装、Mock、波形、参数、任务、日志和回放 |
| [源码核对记录](tools/unicycle_station/docs/source-audit.md) | 已修正文档差异及尚存的实现边界 |

项目 skill 位于 `.agents/skills/develop-tc264-unicycle/`，开发时以当前源码和具体接口文档为准。

2026-09-11 补充：在线调参和 Run Test 波形见 [实时调参说明](tools/unicycle_station/docs/live-tuning.md)；实车原地振荡记录见 [Pitch 分析](tools/unicycle_station/docs/pitch-analysis-20260911/分析与调参建议.md) 与 [Roll 分析](tools/unicycle_station/docs/roll-analysis-20260911/分析与调参建议.md)。构建辅助工具用法见 [ADS 工具说明](tools/ads/README.md)。

## 当前能力

| 模块 | 源码状态 |
|---|---|
| IMU660RB / A+ r10h | 1 kHz 六轴解算、6000 点静止标定、姿态收敛及链路/时序保护 |
| 三轴控制 | Roll 回收→角度→角速度；Pitch 速度→角度→角速度；Yaw 航向→角速度 |
| 调试入口 | A/B/C Jog、单轴 Rate/Angle/Speed Test、Balance、Remote、1m Test |
| Path 导航 | 相对里程/航向定位、前瞻跟踪、终点平衡、256 点、独立双槽 Flash；无需视觉 |
| 正式 Run | 视觉转向、分段速度规划、压弯、Yaw 共模动量降权、失帧/丢线/斑马线退出 |
| 视觉 | 188×120 整帧、大津二值化、逐行提边、来源标记、半宽表、中线拟合、IPM |
| 信标 | 距离/时序/增强时序识别、最多三目标局部访问排序、Beacon参数与Run接入 |
| 图像诊断 | 50帧/512曝光可选原始采图、灰度统计、采集/显示帧号、曝光应答；诊断状态不允许视觉Run启动 |
| 元素 | 斑马线、十字、左右环岛、坡道；没有路障实现 |
| 通信 | cfg v1 / task1；att、run、stat、par、rsp、task、taskevt |
| 参数 | 95 项运行参数（含 15 项 Beacon）、按名称及类型键保存、CRC 与逐字回读校验 |

当前 Roll rate 为增量 PI，默认 `(45, 0.05, 0)`；Angle 为 `(30, 0, 20)`；Speed 为 `(0.0001, 0, 0)`，Rate 陷波 45Hz/Q3/dB0（旁路）。Pitch rate 约为 `(20, 0.64, 0)`，Yaw rate 界面为 `(56, 0, 0)`。上电 Flash 可覆盖默认值，Reset 恢复当前源码默认值。历史文档中的 Roll 位置式 Rate 与旧参数不再适用。

## 双核与实时调度

| 执行位置 | 职责 |
|---|---|
| CPU0 前台 | 按顺序执行 `vofa_cmd_poll → menu_run → control_ipm_flush → vofa_poll` |
| CPU0 1 ms 中断 | 电机链路、六轴采样/A+ 核心解算/角速度、运行状态、安全闸、视觉结果接收、控制输出、定长遥测快照、有界 UART 搬运 |
| CPU0 1 ms 同拍 | 姿态角发布、Roll/Pitch/Yaw 角度外环（1 kHz）、Pitch 投影缓存按 Roll 更新 |
| CPU0 5 ms | C 编码器采样、Path 位姿/采点、压弯、向 CPU1 发布参数和运动反馈 |
| CPU0 10 ms | 按键扫描 |
| CPU0 20 ms | Pitch 速度环、Roll 回收/动量权限、方向控制、Path 前瞻规划和速度斜坡 |
| CPU1 前台 | 摄像头初始化及整帧图像、IPM、元素、结果和屏幕快照发布 |
| CPU1 中断 / DMA | UART1、VSYNC、DMA5 完成；PCLK 直接触发 DMA |

CPU1 不驱动电机。跨核通信使用 `vision_core.c` 的序号锁及 `__dsync()`；图像显示遵守 request/read/release 所有权协议。CPU0 状态在 `cpu0_dsram`，CPU1 图像状态在 `cpu1_dsram`，信箱在 `vision_shared`，部分 CPU1 算法在 `cpu1_psram`。

不调用 `debug_init()`，UART0 不初始化；`ips200_init()` 将断言/日志输出接到屏幕。

## 硬件接线（当前源码配置）

| 部件 | 接口 |
|---|---|
| IMU660RB | SPI0：SCK P20_11、MOSI P20_14、MISO P20_12、CS P20_13 |
| A/B 动量轮 | CYT2BL3 外置板磁编码器 + FOC，原 UART3 460800 接口：TX P15_6、RX P15_7 |
| C 行进轮 | DRV8701E：DIR P21_4、PWM P21_5，17 kHz |
| C 编码器 | TIM2：脉冲 P33_7、方向 P33_6；每 5 ms 采样 |
| MT9V03X | D0–D7 P00_0–P00_7；VSYNC P02_0、PCLK P02_1；UART1 P02_2/P02_3 |
| IPS200 | SCK P15_3、MOSI P15_5、CS P15_2、RST P15_1、DC P15_0、BL P15_4 |
| 按键 | 上 P20_6、下 P20_7、确认 P11_2、返回 P11_3 |
| 无线 UART2 | 115200：MCU TX P10_5、RX P10_6；模块 RTS 接 P10_2，高电平暂停上行 |

Roll 为左右倾斜，A/B 差动；Pitch 为前后倾斜，C 轮驱动；Yaw 为水平转向，A/B 同向。混控先保 Roll，再把剩余 A/B 余量给 Yaw。

原 `推荐IO分配.txt`、`尽量不要使用的引脚.txt` 当前已删除。接线应核对 `config.h`、`user/isr_config.h` 和实际设备驱动头文件；此表只列当前占用，不是其他引脚均可使用的保证。

## 运行与停止

主菜单为 `Params / Image / Run / Run Test / Path`。`Run Test` 内有 `Balance / Remote / Back`，参数页另有 Lean、Run、Odometry、Beacon。

- Balance：三轴原地平衡，目标速度 0，启动时锁存航向，视觉不参与。
- Run：要求 STOP、摄像头 READY、新鲜且有效、模式匹配的视觉结果，以及全部 Balance 安全条件；循迹模式额外要求 IPM 有效，信标模式使用自己的相机标定。按键两次确认。
- Remote：由车身两次确认启动后，才接受 `speed:turn,speed`；没有 cfg 远程发车命令。
- Path：手推或遥控采集路线，停车后保存；从相对起点两次确认启动导航，终点减速后保留 Balance。路线在 Path → View Route 显示，详见路径导航指南。
- 1m Test：使用三轴平衡直行验证里程，完成后仍保持平衡。
- Run 的人工结束、视觉超时、持续丢线和斑马线结束可退回 Balance；`run_active=0` 不等于电机停转。
- 基础运行状态非 STOP 时返回键、串口 `stop` 或控制安全故障走完全停机路径，清输出并锁 A/B 软件刹车。Jog 编辑参数时第一次返回仅退出编辑，需再返回离页才停止，详见菜单指南。

### IMU 故障后的菜单恢复

运行中 IMU 链路或姿态解算故障仍会立即停机。扶稳并保持车身静止，在
`Run Test → Balance` 连续确认两次：若需要恢复，这两次仅触发 IMU 重新初始化，
不会启动电机。菜单底部依次显示 `IMU INIT`、`IMU CALIBRATING`、
`IMU CONVERGING`；正常约 9 秒（6000 点静止标定约 6 秒，真实 1 kHz
姿态收敛约 3 秒），标定时检测到晃动最多重试三轮。

成功后恢复原菜单，仍为 `Balance: OFF`。松开所有按键，再重新确认两次
Balance 才启动。初始化、标定或收敛失败会显示原因，保持停车，可再次两次确认重试。
Run、Remote 和 Path 的二次确认入口也使用同一恢复流程。普通倾角/驱动故障不触发
IMU 重置；修改 `IMU Forward` 仍按原流程保存并重启。

恢复期间保留电机零输出/刹车、按键扫描和通信，暂停 Flash 写入；
航向重置会清除 Path 的旧航向锁并作废已武装的路径启动版本，已录制路线不删除。

### 同步上游 Yaw 控制更新

采用上游 `aec1699` 的 Yaw 反馈符号修正、±120°/s 目标角速度限幅、
Yaw Hold 航向偏移及共模转速回收，保留上述 IMU 恢复流程。
`Params → Yaw → Yaw Hold` 相对本次 Balance 启动航向设置偏移，默认0，目标按30°/s逼近；
共模回收默认开启，满足门控时会额外让角，因此不保证始终保持原航向。
旧 Flash 中负的 Yaw 增益加载时转正；配套上位机默认profile、示例工作区与生成脚本同步到96项参数。
算法及上游实测边界见 [Yaw设计说明](docs/design/Yaw符号修正与动量管理.md)。

## 摄像头显示与诊断

- 直接进入 **Image**，OFF或FAILED状态会请求启动；初始化失败时确认键重试。已READY时，确认键循环切换`BIN+L / BIN / GRAY`，上下键调整曝光。
- 停车进入 **Params → Camera → Diag 50/512** 可按逐飞例程配置采集原始图像。它绕过循迹/信标算法，不提供有效Run目标；50为请求配置帧率，实际帧率看F。
- 停车选择 **Normal Camera** 恢复正常处理、驱动默认配置帧率100及正常RAM曝光。诊断曝光修改不覆盖正常曝光参数，诊断模式不保存到Flash。
- 排查时结合RAW/LCD序号是否持续变化、原始灰度min/avg/max及EXP应答状态。LCD序号表示程序已调用绘制，不是屏幕硬件回读；判断含义详见上述诊断指南。

## 开发与验证

应用模块在 `code/`，入口/中断在 `user/`，第三方驱动在 `libraries/`，链接配置为 `Lcf_Tasking_Tricore_Tc.lsl`。ADS/TASKING 的 Build Project 是权威构建，`Debug/`、`Release/` 为生成目录，不能手改生成 makefile。上位机源码在 `tools/unicycle_station/`，应保持在 ADS 源码扫描排除项内。

推荐联调顺序：静止标定与轴向 → 架空电机/编码器 → 单轴 Test → 零点与 Balance → 里程标定 → 图像/IPM/元素 → 受约束的低速 Run。历史 PC 测试和发行包记录见 [验证记录](tools/unicycle_station/docs/validation.md)，不能代替本次目标构建或台架证据。
## 转弯饱和专项评估（2026-09-09）

当前压弯已改为持续转弯率方向项＋实际车速×转弯率前馈，参数、旧 Flash 兼容、验证与调节步骤见 [压弯控制说明](docs/design/压弯控制说明.md)。压弯与 Roll 合成目标硬上限仍为 10°。9 月 9 日的旧专项评估文档已移除，现行公式和验证步骤以压弯控制说明为准。

### 上位机路线设计与惯导执行

Embedded Station 0.6.0 新增“路线与惯导”页面，可生成直线、编辑航点、保存 JSON、上传下载车内路线、保存 Flash，并确认后启动车辆沿路线行驶。导航速度上限 1.50 m/s，默认 0.20 m/s；曲率与终点会提前限速。详见 [操作与协议](tools/unicycle_station/docs/path-navigation.md)。
