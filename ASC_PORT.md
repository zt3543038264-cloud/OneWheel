# Qcar：参考仓库完整结构版本

本版在 port/asc-control-r10h 分支继续修正，基于 Kongmengwuwei/ASC_SummerTraining_Unicycle 提交 0eed51b8fd0f4e72ab92f737642ab1900f60cbd3。

此前只接入控制和A+内核的版本保留在Git历史中。本版采用参考仓库的完整应用分层、Menu/Display、参数管理、启动流程、单轴测试及上位机协议，不再使用旧Qcar平面目录、树形菜单或Mode2～10编号。

2026-11-09 补充：Roll 已改为增量 PI，当前实车默认值见 README；旧 Qcar 信标模块以独立 CPU1 模块接入当前 Beacon 菜单及 Run，未恢复旧控制器或旧编号模式。参数表现为 95 项，新增 15 项 Beacon；原始文件和哈希保留在 docs/reference/qcar-beacon，操作见 [信标识别与运行](docs/guides/信标识别与运行.md)。下述移植初期的 75 项/位置式 Roll 描述按当前源码及这两份修订为准。

## 目录与界面

    code/Common/                  参数、方向基准、PID
    code/HW_Drivers/IMU/           真实采集、静止标定、SPI截止
    code/HW_Drivers/Motor/         A/B飞轮、C地轮
    code/Estimation/Attitude/      A+ r10h
    code/Estimation/Vision/        参考视觉模块，默认上电不初始化摄像头
    code/Controller/               三轴串级及规划
    code/Menu/                     原仓库菜单和显示
    code/Protocols/                cfg/遥测/串口及WiFi-SPI
    user/                          原仓库双核入口与中断
    tools/unicycle_station/        与新固件匹配的参考上位机

主界面为 Params / Image / Run / Run Test / Path。CPU0启动和菜单都显式使用 IPS200_8X16_FONT。Menu/display.c及user/cpu0_main.c保留参考实现；Menu/menu.c的导航现已改为父子/兄弟链表，绘图函数、字体、颜色和布局保留。

日常调试入口：Params→Roll/Pitch/Yaw中的Rate/Angle/Speed单轴测试；整体原地平衡用Run Test→Balance。本版没有旧Mode6或Mode10入口。Run、Path等需要其自身条件；没有摄像头时不要用视觉Run代替Balance。

## 控制与姿态

完整保留参考控制结构：Roll回收→角度→位置式角速度；Pitch速度→角度→增量式角速度；Yaw航向→角速度。先保留Roll，再将yaw裁剪到剩余飞轮余量。

采样、A+、姿态发布和内环/角度环同在CPU0的1ms控制链；编码器/部分任务5ms，速度/回收20ms。原先分开定时器运行IMU和控制的桥接实现已删除。

A+保留原核心及单位矩阵：6000点静止采样，再等3000次成功更新。控制使用仅扣启动均值的陀螺仪；核心使用未重复扣偏的原始六轴物理量。SPI读事务100µs截止，时序/采集/姿态故障保持锁止，不能放宽时间门限来掩盖固定1ms核心的调用错误。

新界面使用参考仓库的IMU INIT FAILED、IMU CALIB INVALID、ATTITUDE NOT READY、ATTITUDE DIVERGED、IMU LINK LOST、BLDC LINK LOST等状态。旧F/IMU/ERR负数表不适用于此版本。

## 本车适配

- 保留TC264、SPI0 IMU、SPI2 IPS200、UART3 460800飞轮、TIM2 P33_7/P33_6地轮计数及P21_4/P21_5 PWM/DIR。
- 本车四键按原接线映射：上P20_7、下P20_6、确认P11_3、返回P11_2；界面交互采用参考菜单。
- 本车旧零点Roll=5.4°、Pitch=-0.2°经完整重力坐标旋转，得到参考Roll约-0.200891557°、Pitch约-5.399967004°，不是参考车的-0.6°/-3°。这只是保留现有测量，不代表重新验证了实际重心位置。
- 按参考坐标与混控解释方向参数。A/B差动和C轮/编码器的完整负反馈链仍需在本车确认；不能单独搬用旧界面的符号值。
- 地轮保留本车正/反730/1100 PWM摩擦补偿，补偿后再次限制最终输出。
- 飞轮保留本车文本SET-DUTY指令，接收文本与二进制转速；采用最新完整指令邮箱和UART3 TX中断，控制ISR不阻塞发送整行。已经开始的行必须发完，尚未开始的占空比可被新值替代，Stop以零指令替换待发值。保留可切回二进制的编译开关。
- 摄像头默认不自动初始化，Image菜单仍可显式请求启动；不会把CAM NOT INITIALIZED当作需要自动发车的条件。
- 工程名Qcar，固件名OneWheelUpright；构建目录使用实际ProjName，排除旧build/Debug/驱动示例等残留目录。旧A+ include路径已映射到新Estimation目录。

## 上位机、WiFi和保存

现在使用原仓库cfg/遥测协议，不再混用旧MCAR/JustFloat滑杆。桌面ASC_Unicycle_Station是对应的源码上位机。WiFi连接时要按模块实际IP及本地配置设置UDP端口，不能直接沿用上位机示例网络。

当前公共配置集中在 `code/config.h`，`STATION_WIFI_ENABLED=1` 选择 WiFi SPI。热点名、密码和电脑目标 IP 只填写不跟踪的 `code/config_private.h`，模板为 `code/config_private.example.h`。电脑接收端口 1349、模块本地端口 6666 只在公共配置定义。若使用 UART2，将公共开关设为 0，此时无需私有头文件。

参数使用参考的75项表和分组保存，新标识QCAF，避免误读之前OWB2/ASC1或另一辆参考车的记录。首次使用采用本车配置默认值，再停车保存。上位机Mock默认值同步本车配置，qcar-defaults.json明确标记为配置推导，不冒充实车回读；上游历史快照未改写。

## 验证与使用边界

当前测试149项通过，涵盖参考上位机、协议、实际C控制路径和本车驱动适配；A+实际核心的5组主机测试通过。ADS已验证公共UART和本机WiFi配置能够构建。见ASC_PORT_VALIDATION.md。

尚未烧录、没有验证本车LCD实拍、真实最坏ISR耗时、无支撑平衡或上电故障消失。字体/目录对齐不等于传感器故障已经解决。

打开ADS时先关闭旧Qcar工程，再重新打开并Clean，避免旧.cproject缓存覆盖新目录。烧录请使用新生成的OneWheelUpright.hex，不要使用Debug中残留的Qcar.hex。
## 2026-11-04 链表菜单

按本车需求，导航改为静态节点池中的Father / First_Son / Next_Brother / Last_Brother链表，目录节点拥有自己的光标和滚动位置。按键通过节点回调分派；页面编号仅用于现有绘图和实时信息样式，不再作为导航历史。

根目录五个入口、参数组、参数行、动作、Save/Back、Image、IPM与Path Map均建立真实父子关系。参数节点绑定现有参数描述，仍通过原有参数接口修改数值。静态池满时停止控制并显示MENU TREE CAPACITY ERROR，不使用malloc。

显示保留8×16字体、原颜色和坐标，实时数据和图像显示仍走原实现。上/下、确认、返回及原有启停保护保留。现有本地IMU诊断修改另行保留，不属于这次菜单改造。
## 参数编辑步长

确认进入参数编辑后，上/下增减，短按确认结束编辑；长按确认切换原步长的×0.01、×0.1、×1、×10、×100档位。页脚Step显示的是实际步长，不是倍率。持续按住只切换一次，松开后再次长按才切下一档。

每个节点独立保留档位，离开目录再回来仍保留，本次上电有效；重启恢复原步长，不修改75项参数表或Flash格式。整数跳过不足1的档位；方向和开关保持原有固定切换步长。小数显示精度随细调档位增加，Path速度也支持这些档位。Image快捷键保持原参考交互。

短按确认、返回及运行中返回停机的行为保留。切换档位本身不提交参数修改，参数值仍通过原有范围和运行状态检查。
