# 来源与版权

## CYT2BL3 双路无刷驱动修订

`tools/cyt2bl3_driver/code` 来自用户提供的逐飞 CYT2BL3 双驱开源工程；源码保留 Copyright (c) 2022 SEEKFREE / GPL-3.0-or-later 声明。修订串口解析与左右过载计数；配置和头文件保留作参考。许可证随目录提供。`tests/fixtures/cyt2bl3_stock` 保存原串口解析源码用于验证主控对未升级驱动的文本协议兼容性。此目录未接入TC264固件构建。

## MCARpreparation 姿态算法

- 来源：本地 `C:\Users\csz18\Desktop\mcar-mailun\MCARpreparation\project\code\attitude.c/.h`、`imu.c/.h`、`config.h` 中的姿态参数，以及 `tests/attitude_ekf_test.c`。
- 移植到 `code/attitude.c/.h`、`imu.c/.h`、`config.h`；算法保持原实现，采样适配 IMU660RB，时间基准改为 TC264 STM0。
- 保留源文件 GPL-3.0-or-later 说明；EKF 参考 Wang Hongxi 的 QuaternionEKF 设计，采用固定五维误差状态实现。姿态模块现仅保留 EKF，其他算法已移除。
- 姿态移植未接入原导航静止判定，因此不自动启用 Z 零偏跟踪。Wi-Fi 配置和传输的后续移植见下一节。

## MCARpreparation WiFi-SPI 传输

- 来源：本地 `C:\Users\csz18\Desktop\mcar-mailun\MCARpreparation\project\code\wifispi.c/.h` 和 `config.h` 的 WiFi 配置，包含移植时源工作区的当前修改。
- 移植至 `code/wifispi.c/.h`；保留 JustFloat、UDP 协议、订阅及滑块解析，网络配置集中至 `code/config.h`。
- 硬件与时间基准改用 Qcar 的 TC264 WiFi-SPI 驱动和 STM0；遥测及参数绑定替换为 Qcar 的 IMU、速度、级联输出和 PID。麦轮导航及远程路径控制不在本次移植范围。
- 不覆盖来源项目文件，不复制 RT1064 SDK、SPI/GPIO 引脚定义、可执行上位机或构建产物。协议兼容源上位机的通用 UDP/JustFloat 功能，需重选 Qcar 通道及参数。

## mcar-mailun 菜单

- 来源：本地 `C:\Users\csz18\Desktop\mcar-mailun\project\code\menu.c/.h` 与 `Mymenu.c/.h`。
- 通用节点层移植至 `code/menu_tree.c/.h`；Qcar 界面参考其树形导航与参数编辑方式实现。
- 节点池、循环链表及接口保留来源结构，动作和参数绑定改为 Qcar；未引入麦轮项目的地图和推箱控制依赖。

## 逐飞 TC264 库及示例

- 来源：本地 `../TC264_Library/Example/Coreboard_Demo/E13_dual_core_demo`。
- 本地库远端：https://gitee.com/seekfree/TC264_Library.git
- 本地库 Git 提交：`10cf2ae67d7d697ad29dd96d8c94e429a808ce00`。
- 保留 TC264 示例的启动、底层驱动、设备驱动、ADS 配置及 TC26B 链接脚本。
- 逐飞文件声明 Copyright (c) 2022 SEEKFREE，并采用 GPL-3.0-or-later；根目录 `LICENSE` 从本地库复制。Infineon 等第三方文件遵循各文件自己的声明。
- 修改涉及项目名、应用头文件接入、PIT/UART 回调、屏幕叠加显示、横屏配置及栈大小。

## monotroch 应用

- 仓库：https://github.com/feng0511/monotroch
- 提交：`bc24e89cac8d557327d03be9a24e940b9714adf7`。
- 复制 `code/` 下的 C/H 应用文件、`LQ_telecontrol.c/.h`、应用数据结构及自定义屏幕叠加函数。
- 修改涉及双核任务、UART 映射、助手接口、内存容量、边界检查、共享标志、初始化顺序和 Flash 版本标记。
- 没有复制 TC377 的 iLLD、启动程序、链接脚本或预编译设备配置库来代替 TC264 文件。
- 获取时仓库根目录未见独立 LICENSE 文件；应用文件的授权以原作者声明为准，本项目不为原作者代码重新指定许可证。

原仓库保存在工作目录的 `../.sources/monotroch`，可对照原始文件和提交。应用户要求，后续已修复编码混杂，并结合实现重写无法逐字恢复的中文注释；这些重写说明不代表原作者原文。非注释代码保持不变，修改前备份保存在 `../.sources/Qcar-comments-*.zip`。


## 2026-10-25 直立精简

上述来源记录为移植历史。本轮删除了视觉、轨迹、遥控和未使用算法，原直立状态机移至balance_control.c，通用PID独立为pid.c。EKF仍保留，未使用的Z零偏跟踪已删除；IPS200中应用追加的轮廓/信标叠加函数已移除。驱动可调接线与IRQ值移至config.h，原版权声明继续保留。

ADS工程和TC26B链接脚本恢复自本机此前编译过的逐飞工程；必需的zf_device_config.a同样来自该本机TC264库。主机回归编译器和临时验证产物位于仓库外，不作为固件依赖。
