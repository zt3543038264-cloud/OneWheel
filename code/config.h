#ifndef CONFIG_H_
#define CONFIG_H_

// 项目公共配置的唯一入口。编辑编码：UTF-8。
// 本文件只包含宏，不包含硬件或参数类型，供驱动、中断和业务模块安全共用。
// 默认参数仅在 Flash 无有效记录时生效；运行参数仍由 g_param / 菜单管理。

// ===== 固定方向基准与编码器极性 =====
// 2026-09-21 已确认实车基准。菜单 +1=保持此基准，-1=相对反向。
// 这些符号属于硬件/控制适配，不能再次当作用户默认值写入参数。
// 与 PARAM_MAGIC 的 sign2 语义绑定；未来改基准须新增存储版本和迁移。
#define MOTOR_DIR_A_BASE      (1)
#define MOTOR_DIR_B_BASE      (1)
/* Raw encoder RPM polarity relative to a positive DRIVER command, not body Roll.
 * +1: positive driver duty reports positive raw RPM; -1: reports negative RPM.
 * Defaults preserve the old receive polarity. Verify actual CYT calibration
 * on the board. These are DEFAULTS for Motor -> Enc A/B when Flash has no key;
 * saved enc_dir_a/b override them independently of MOTOR_DIR. */
#ifndef FLY_ENCODER_POLARITY_A
#define FLY_ENCODER_POLARITY_A (1)
#endif
#ifndef FLY_ENCODER_POLARITY_B
#define FLY_ENCODER_POLARITY_B (1)
#endif
#if (FLY_ENCODER_POLARITY_A != 1 && FLY_ENCODER_POLARITY_A != -1) || \
    (FLY_ENCODER_POLARITY_B != 1 && FLY_ENCODER_POLARITY_B != -1)
#error "Flywheel encoder polarity must be +1 or -1"
#endif
#define MOTOR_DIR_C_BASE      (-1)
#define ENC_DIR_C_BASE        (-1)
#define STEER_DIR_BASE        (-1)
#define IMU_FORWARD_DIR_BASE  (-1)

// R_RATE_KD_BASE is retained ONLY for legacy Flash migration. The current
// Roll Rate controller is incremental PI and never uses a derivative gain.
#define R_RATE_KD_BASE        (-1.0f)

/* Yaw plant sign, measured on this car (2026-11-11).
 * Positive common-mode duty (A=B=+u) produces POSITIVE yaw acceleration here.
 * The former -1 assumed the opposite, so the rate loop was positive feedback:
 * with y_angle_kp=0 and y_rate_kp=+10 the body spun up by itself, y_rate_kp=-3
 * decayed 129 -> 7 deg/s, and u/gyro_z measured exactly -1.00.
 * control.c now uses (yaw_target - yaw) as the heading error, so both yaw
 * gains are POSITIVE user values. param_init() negates negative values that
 * older Flash records still hold, so a stored -60/-24 becomes +60/+24. */
#define Y_RATE_BASE           (1.0f)

// ===== 中断服务核与优先级 =====
// 中断服务核与优先级配置。
// INT_SERVICE 决定中断由哪个核响应。两个核共用 LCF_INTVEC0_START 这一张向量表，
// 所以 isr.c 里 IFX_INTERRUPT 的第二个参数一律是 0，改服务核只改本文件。
// 优先级必须全局唯一，跨核也不能重号，否则会落进同一个表项。
// CPU 服务优先级范围 1~255，DMA 服务优先级范围 0~47。

// CCU60_CH0：1ms 控制中断，运行在 CPU0
#define CCU6_0_CH0_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_0_CH0_ISR_PRIORITY 30

// CCU60_CH1、CCU61_CH0、CCU61_CH1 未使用，保留宏供 zf_driver_pit 编译
#define CCU6_0_CH1_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_0_CH1_ISR_PRIORITY 31
#define CCU6_1_CH0_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_1_CH0_ISR_PRIORITY 32
#define CCU6_1_CH1_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_1_CH1_ISR_PRIORITY 33

// ERU 通道 0/4、1/5 未使用，保留宏供 zf_driver_exti 编译
#define EXTI_CH0_CH4_INT_SERVICE IfxSrc_Tos_cpu0
#define EXTI_CH0_CH4_INT_PRIO  	40
#define EXTI_CH1_CH5_INT_SERVICE IfxSrc_Tos_cpu0
#define EXTI_CH1_CH5_INT_PRIO  	41

// ERU 通道 2：摄像头 PCLK，直接触发 DMA，不进 CPU，所以没有对应 ISR
#define EXTI_CH2_CH6_INT_SERVICE IfxSrc_Tos_dma
#define EXTI_CH2_CH6_INT_PRIO  	5

// ERU 通道 3：摄像头场同步，运行在 CPU1
#define EXTI_CH3_CH7_INT_SERVICE IfxSrc_Tos_cpu1
#define EXTI_CH3_CH7_INT_PRIO  	43

// DMA 通道 5：摄像头整场搬运完成，运行在 CPU1
#define	DMA_INT_SERVICE         IfxSrc_Tos_cpu1
#define DMA_DATA_CORE_ID        (1)                 // 预处理专用纯数字，DMA 状态放入 CPU1 DSRAM
#define DMA_INT_PRIO  	        60

// UART0 未使用，保留宏供 zf_driver_uart 编译。
// 工程不调 debug_init()，UART0 从不初始化，所以 isr.c 里没有对应的中断服务函数。
#define	UART0_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART0_TX_INT_PRIO       11
#define UART0_RX_INT_PRIO       10
#define UART0_ER_INT_PRIO       12

// UART1：摄像头配置串口，与摄像头其余中断同核，运行在 CPU1
#define	UART1_INT_SERVICE       IfxSrc_Tos_cpu1
#define UART1_TX_INT_PRIO       13
#define UART1_RX_INT_PRIO       14
#define UART1_ER_INT_PRIO       15

// UART2：无线转串口模块，调参与波形，运行在 CPU0
#define	UART2_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART2_TX_INT_PRIO       16
#define UART2_RX_INT_PRIO       17
#define UART2_ER_INT_PRIO       18

// UART3：CYT2BL3 双路无刷驱动，运行在 CPU0
#define	UART3_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART3_TX_INT_PRIO       19
#define UART3_RX_INT_PRIO       20
#define UART3_ER_INT_PRIO       21

// ===== 引脚、控制周期、PID、保护及功能默认值 =====
#ifndef QCAR_DRIVER_TEXT_DUTY
#define QCAR_DRIVER_TEXT_DUTY 1
#endif
#ifndef QCAR_CAMERA_AUTO_START
#define QCAR_CAMERA_AUTO_START 0
#endif
#define QCAR_GROUND_DEADZONE_POS 730.0f
#define QCAR_GROUND_DEADZONE_NEG 1100.0f

#define CTRL_PERIOD_MS          (1)            // 控制中断周期(ms)
#define CTRL_PIT_CH             (CCU60_CH0)    // 控制定时器通道

#define CTRL_DIV_ATT            (5)            // 编码器、Path、CPU1 反馈及压弯分频
#define CTRL_DIV_ANGLE          (1)            // 欧拉角发布与三个角度外环：1 kHz
#define ANGLE_PID_REFERENCE_MS  (5)            // 菜单/Flash 角度 PID 沿用原 5 ms 基准
#define ANGLE_PID_PERIOD_SCALE ((float)(CTRL_DIV_ANGLE * CTRL_PERIOD_MS) / (float)ANGLE_PID_REFERENCE_MS)
#define ATT_DT                  (0.001f)       // 姿态解算周期(s)
#define CTRL_DIV_KEY            (10)           // 按键扫描分频
#define CTRL_DIV_SPEED          (20)           // 速度环与飞轮回收环分频

#if (CTRL_DIV_ANGLE < 1) || (CTRL_DIV_ANGLE > CTRL_DIV_SPEED) || ((CTRL_DIV_SPEED % CTRL_DIV_ANGLE) != 0)
#error "Angle period must divide the 20 ms control window"
#endif

// W_Motor：动量轮 A/B 使用 CYT2BL3 双路无刷驱动，磁编码器 + FOC 由外置驱动板实现，UART 协议不变。
#define W_MOTOR_UART                    (UART_3)
#define W_MOTOR_UART_BAUDRATE           (460800)
#define W_MOTOR_UART_TX_PIN             (UART3_TX_P15_6)
#define W_MOTOR_UART_RX_PIN             (UART3_RX_P15_7)
#define W_MOTOR_DUTY_MAX                (10000)
#define W_MOTOR_LINK_TIMEOUT_MS         (60)
#define W_MOTOR_SPEED_REQUEST_MS        (200)

// Y_Motor：行进轮 C 使用 DRV8701E 通道 1，编码器为 P33.7 脉冲 + P33.6 方向。
#define Y_MOTOR_DIR_PIN                 (P21_4)
#define Y_MOTOR_PWM_PIN                 (ATOM0_CH3_P21_5)
#define Y_MOTOR_PWM_FREQUENCY_HZ        (17000)
#define Y_MOTOR_PWM_MAX_DUTY            (10000)
#define Y_MOTOR_FORWARD_DIR_LEVEL       (GPIO_LOW)
#define Y_MOTOR_ENCODER_INDEX           (TIM2_ENCODER)
#define Y_MOTOR_ENCODER_A_PIN           (TIM2_ENCODER_CH1_P33_7)
#define Y_MOTOR_ENCODER_B_PIN           (TIM2_ENCODER_CH2_P33_6)
#define Y_MOTOR_ENCODER_PERIOD_MS       (5)


// A/B motor directions select the local output frame. Raw encoder polarity is
// configured independently by Motor -> Enc A / Enc B (stored in Flash).
#define MOTOR_DIR_A_DEFAULT     (1)             // A轮输出与转速极性
#define MOTOR_DIR_B_DEFAULT     (1)             // B轮输出与转速极性
#define MOTOR_DIR_C_DEFAULT     (1)             // C轮输出极性
#define ENC_DIR_A_DEFAULT       (FLY_ENCODER_POLARITY_A)
#define ENC_DIR_B_DEFAULT       (FLY_ENCODER_POLARITY_B)
#define ENC_DIR_C_DEFAULT       (1)             // C轮脉冲/方向编码器计数极性
#define STEER_DIR_DEFAULT       (1)            // 转向极性，只允许 +1 或 -1

// 菜单按键
#define MENU_KEY_UP             (KEY_1)        // P20_6，上
#define MENU_KEY_DOWN           (KEY_2)        // P20_7，下
#define MENU_KEY_ENTER          (KEY_3)        // P11_2，确认
#define MENU_KEY_RETURN         (KEY_4)        // P11_3，返回

// 运行参数默认值：2026-09-21 实车 Schema 75 项读回并归一化，见 tools/unicycle_station/docs/defaults-20260921.json。
// 保留串口读回精度；有效 Flash 仍优先于编译默认值。
// 机械零点
#define IMU_FORWARD_DIR_DEFAULT (1) // +1 当前前向（原车尾）；-1 相对反向；保存重启
#define ROLL_ZERO_INIT_DEFAULT (-0.200891557f) // Qcar measured gravity, transformed
#define PITCH_ZERO_INIT_DEFAULT (-5.399967004f) // Qcar measured gravity, transformed

// 平衡串级 PID 参数
// Roll：飞轮回收环 -> 角度环 -> 角速度环。
#define R_RCY_KP_DEFAULT        (0.0001f)      // 2026-11-09 用户实车照片参数
#define R_RCY_KI_DEFAULT        (0.0f)         // 飞轮回收环 I
#define R_RCY_KD_DEFAULT        (0.0f)         // 飞轮回收环 D
#define R_RCY_IMAX              (50.0f)        // 飞轮回收环积分限幅
#define R_ANGLE_KP_DEFAULT      (30.0f)        // 横滚角度环 P
#define R_ANGLE_KI_DEFAULT      (0.0f)         // 横滚角度环 I
#define R_ANGLE_KD_DEFAULT      (20.0f)        // 横滚角度环 D（菜单5ms参考值）

// 角度环输出由角速度环统一限幅，积分只限制累计状态。
#define R_ANGLE_IMAX            (2000.0f)      // 横滚角度环积分限幅
#define R_RATE_KP_DEFAULT       (45.0f)        // 增量式PI角速度环 P
#define R_RATE_KI_DEFAULT       (0.05f)        // 增量式PI角速度环 I，每1ms一拍
#define R_RATE_KD_DEFAULT       (0.0f)        // 增量式PI，无D；旧Flash KD加载时归零
#define R_RATE_IMAX             (100.0f)       // 横滚角速度环积分限幅
// Roll 角速度反馈陷波：默认旁路(db=0)，只滤陀螺反馈，不改变前向通路
#define R_RATE_NOTCH_HZ_DEFAULT (45.0f)        // 照片菜单值(Hz)，dB=0时旁路
#define R_RATE_NOTCH_Q_DEFAULT  (3.0f)         // 品质因数，越大越窄
#define R_RATE_NOTCH_DB_DEFAULT (0.0f)         // 深度(dB)，0=严格旁路
#define R_RATE_NOTCH_HZ_MIN     (5.0f)
#define R_RATE_NOTCH_HZ_MAX     (200.0f)
#define R_RATE_NOTCH_Q_MIN      (0.5f)
#define R_RATE_NOTCH_Q_MAX      (20.0f)
#define R_RATE_NOTCH_DB_MIN     (-40.0f)
#define R_RATE_NOTCH_DB_MAX     (0.0f)

// Pitch：速度环 -> 角度环 -> 角速度环。
#define P_VEL_KP_DEFAULT        (0.06f)       // 俯仰速度环 P
#define P_VEL_KI_DEFAULT        (0.002f)       // 俯仰速度环 I
#define P_VEL_KD_DEFAULT        (0.0f)         // 俯仰速度环 D

#define P_VEL_LIMIT             (8.0f)         // 速度环输出的俯仰角目标限幅(°)
#define P_VEL_IMAX              (200.0f)       // 俯仰速度环累计误差限幅；I输出上限=abs(Ki)*200，当前默认约0.4°
#define P_ANGLE_KP_DEFAULT      (8.999999f)         // 俯仰角度环 P
#define P_ANGLE_KI_DEFAULT      (0.0f)         // 俯仰角度环 I
#define P_ANGLE_KD_DEFAULT      (0.0f)         // 俯仰角度环 D
#define P_ANGLE_IMAX            (50.0f)        // 俯仰角度环积分限幅
#define P_RATE_KP_DEFAULT       (20.0f)        // 俯仰角速度环 P
#define P_RATE_KI_DEFAULT       (0.6400001f)         // 俯仰角速度环 I
#define P_RATE_KD_DEFAULT       (0.0f)         // 俯仰角速度环 D
#define P_RATE_IMAX             (100.0f)       // 俯仰角速度环积分限幅

// Yaw：转向外环 -> 角速度内环。
#define Y_ANGLE_KP_DEFAULT      (18.00003f)        // 转向外环 P
#define Y_ANGLE_KI_DEFAULT      (0.0f)         // 转向外环 I
#define Y_ANGLE_KD_DEFAULT      (0.0f)         // 转向外环 D
#define Y_ANGLE_IMAX            (50.0f)        // 转向外环积分限幅
#define Y_RATE_KP_DEFAULT       (56.0f)        // 航向角速度内环 P
#define Y_RATE_KI_DEFAULT       (0.0f)         // 航向角速度内环 I
#define Y_RATE_KD_DEFAULT       (0.0f)         // 航向角速度内环 D
#define Y_RATE_IMAX             (100.0f)       // 航向角速度内环积分限幅


#define DIRECTION_PIXEL_KP_DEFAULT   (1.599999f)   // 横向像素误差到目标横摆角速度
#define DIRECTION_HEADING_KP_DEFAULT (0.9999999f)    // 赛道航向误差到目标横摆角速度
#define DIRECTION_CURVE_KFF_DEFAULT  (60.0f)    // 速度乘归一化曲率前馈
#define DIRECTION_BALANCE_KD_DEFAULT (0.0f)     // 方向偏差 D，报告初值为 0
// 双项压弯：转弯率前馈 + v*r/g；新名称避免旧 lean_roll_kp 的 Flash 值误用。
#define LEAN_TURN_KP_DEFAULT        (0.009999999f)    // 方向预压弯增益(s)
#define LEAN_TURN_KP_MAX            (0.10f)
#define LEAN_SPEED_KP_DEFAULT       (0.7f)     // v*r/g 物理项倍率
#define LEAN_SPEED_KP_MAX           (2.0f)
#define LEAN_SLEW_DPS_DEFAULT       (18.0f)    // 压弯变化速度(°/s)
#define LEAN_SLEW_DPS_MIN           (1.0f)
#define LEAN_SLEW_DPS_MAX           (60.0f)
#define LEAN_DT_S                   (0.005f)   // 压弯周期为 5 ms，独立于 CTRL_DIV_ANGLE 定义的角度环周期
#define LEAN_SPEED_FILTER_ALPHA     (0.10f)    // 5ms 拍，约45ms一阶时间常数
#define LEAN_SPEED_MIN_MPS          (0.05f)    // 低于此速度不主动压弯
#define LEAN_SPEED_FULL_MPS         (0.30f)    // 线性淡入到全权重
#define LEAN_RATE_DEAD_DPS          (2.0f)     // 连续转弯率死区
#define LEAN_GRAVITY_MPS2           (9.80665f)
#define LEAN_MAX_ANGLE_DEFAULT       (5.999995f)     // 压弯动态零点最大值(°)
#define DIRECTION_CAMERA_LIMIT       (15000.0f) // 加权和限幅，除以 345 后约 ±43.48 pixel
#define DIRECTION_CAMERA_WEIGHT_SUM  (345.0f)   // 60行方向权重总和，用于还原平均像素偏差
#define DIRECTION_ERROR_ALPHA        (0.25f)    // 方向误差低通的新值权重
#define DIRECTION_D_RATE_LIMIT       (200.0f)   // 滤波后误差变化率限幅(pixel/s)
#define DIRECTION_YAW_RATE_LIMIT     (120.0f)   // 正式循迹目标横摆角速度限幅(°/s)
#define DIRECTION_YAW_SLEW           (600.0f)   // 目标横摆角速度变化率(°/s^2)
#define DIRECTION_YAW_LEAD_LIMIT     (35.0f)    // 目标航向相对当前航向最大超前角(°)
#define DIRECTION_LEAN_LIMIT         (10.0f)    // 压弯角软件硬限幅(°)
#define ROLL_TARGET_LIMIT            (10.0f)    // 回收与压弯合成后的 Roll 目标限幅(°)
#define YAW_MOMENTUM_WARN_RPM        (2500.0f)  // 单项实测：共模RPM超过该值后温和降低转向与Run速度
#define YAW_MOMENTUM_HARD_RPM        (6000.0f)  // 共模RPM达到该值时进入最小权限，不依赖Flash软超速阈值
#define YAW_MOMENTUM_MIN_SCALE       (0.25f)    // 动量紧张时保留的最小转向/速度比例

// Yaw 共模动量回收。反作用轮只在转速变化时给车身力矩：P-only 航向环为了顶住静态
// 误差会持续给占空比，共模转速因此单向累积，最后撞到 FLY_SPEED_LIMIT 停机。
// 超出门限后让航向目标缓慢让角，让角本身产生反向转速变化，把飞轮带回 0。
// 代价是航向缓慢漂移；低于门限时完全停止，设 YAW_DESAT_ENABLE=0 可整体关闭。
#define YAW_DESAT_ENABLE             (1)
#define YAW_DESAT_RPM                (1200.0f)  // 超过该共模转速开始回收
#define YAW_DESAT_DPS                (3.0f)     // 让角速率(°/s)，越大回收越快、漂移越多
#define YAW_DESAT_LIMIT_DEG          (90.0f)    // 累计让角上限(°)
#define YAW_DESAT_MAX_ERR_DEG        (3.0f)     // 仅当航向误差小于此值(已稳住)才让角
#define YAW_DESAT_MAX_OUTPUT         (6000.0f)  // 仅当 Yaw 输出低于此值(未打满)才让角

// 角度环输出是角速度指令。不限幅时大误差会命令出几百 °/s，内环占空比打满、
// 失去阻尼，飞轮冲向高速后 Roll 也会失去权限。与 DIRECTION_YAW_RATE_LIMIT 同量级。
#define YAW_RATE_CMD_LIMIT           (120.0f)

// Balance 下 Yaw Hold 目标的变化速率(°/s)。直接给阶跃会让角度环饱和，
// 与大角度指令造成的超调是同一个问题。
#define YAW_HOLD_SLEW_DPS            (30.0f)
#define LEAN_DIR                     (1)        // +1=沿用实跑版本的压弯方向，-1=反向

#if (LEAN_DIR != 1) && (LEAN_DIR != -1)
#error "LEAN_DIR must be +1 or -1"
#endif

// 电机输出限幅与死区
// A/B 的起转死区由驱动内部处理，主控不做死区补偿。
#define FLYWHEEL_OUT_LIMIT      (10000)        // A/B 输出限幅
#define DRIVE_OUT_LIMIT         (8000)         // C轮输出限幅
#define DRIVE_DEAD_ZONE         (120)          // C轮死区补偿

// Test 与实跑采用相同输出限幅。
#define BAL_TEST_FLY_LIMIT      FLYWHEEL_OUT_LIMIT         // 飞轮测试输出限幅
#define BAL_TEST_DRIVE_LIMIT    DRIVE_OUT_LIMIT          // 行进轮测试输出限幅

// Motor 页架空点动测试参数。
#define JOG_DUTY_FLY_DEFAULT    (2500)          // A/B 点动占空比
#define JOG_DUTY_DRIVE_DEFAULT  (800)           // C 点动占空比，架空空载，验方向不需要转快

// 飞轮超速保护，0 表示关闭。
#define FLY_SPEED_LIMIT_DEFAULT (8000)         // A/B 转速上限(RPM)

// A/B 占空比每 1ms 的最大变化量，0 表示关闭斜坡。
#define FLY_SLEW_DEFAULT        (0)            // 实车恢复值；0 表示关闭斜坡

// 姿态保护阈值
#define ROLL_PROTECT_ANGLE_DEFAULT  (20.0f)    // 横滚误差阈值(°)，实车恢复值
#define PITCH_PROTECT_ANGLE_DEFAULT (50.0f)    // 俯仰误差阈值(°)，台架整定值

// 摄像头循迹使用整帧：第 0 行为远端，第 IMG_H-1 行为近端。
#define IMG_W                   MT9V03X_W      // 算法图像宽度(列)，188
#define IMG_H                   MT9V03X_H      // 算法图像高度(行)，120
#define IMG_COL_OFFSET          ((MT9V03X_W - IMG_W) / 2)   // 居中裁剪列偏移，整帧时为 0
#define IMG_ROW_OFFSET          (MT9V03X_H - IMG_H)         // 底部裁剪行偏移，整帧时为 0
#define IMG_MID_COL             (IMG_W / 2)    // 图像中心列
#if (IMG_W > MT9V03X_W) || (IMG_H > MT9V03X_H)
    #error "IMG_W/IMG_H 不能超过采图缓冲 MT9V03X_W/MT9V03X_H"
#endif

// 信标近处转弯：移植自 feng0511/monotroch bc24e89 track_turn_update。
// 单位为原始 188x120 图像的像素；只改 Run 视觉目标，不改直立 PID。
#define BEACON_TRACK_ROW_DEFAULT       (50)
#define BEACON_TURN_ROW_DEFAULT        (110)
#define BEACON_LINE_KP_DEFAULT         (0.8f)
#define BEACON_LINE_OFFSET_DEFAULT     (41.0f)
#define BEACON_SEARCH_ERROR_DEFAULT    (60.0f)
#define BEACON_SEARCH_FRAMES_DEFAULT   (20)

// 大津阈值保护
#define OTSU_TH_MIN             30             // 阈值下限
#define OTSU_TH_MAX             220            // 阈值上限
#define OTSU_CONTRAST_MIN       25             // 最小灰度跨度

// 逐行提边使用赛道内部种子和上一行边线约束搜索窗口。
#define EN_START_ROW_BOTTOM     (IMG_H - 3)    // 近端种子搜索下界
#define EN_START_ROW_TOP        (IMG_H - 20)   // 近端种子搜索上界
#define EN_MIN_ROAD_WIDE        25             // 种子行最小白区宽度(像素)
#define EN_ROW_TOP_LIMIT        2              // 提边终止行
#define EN_COL_MIN_LIMIT        2              // 扫描最左列，与 image_binarize() 涂黑的边框对齐
#define EN_COL_MAX_LIMIT        (IMG_W - 3)    // 扫描最右列
#define EN_SEED_RECOVER         4              // 种子落在黑区时允许左右拉回的最大列数
#define EN_SEED_STEP_MAX        8              // 种子一行最多跟随移动的列数
#define EN_MIN_RUN              6              // 白区窄于这个值就判前瞻到头
#define EN_EDGE_TRACK           12             // 边线搜索窗口，只在上一行边线 ± 这么多列内找
#define EN_BOTH_LOST_MAX        18             // 连续两侧都找不到可信边线的最大行数
#define EN_SLOPE_SPAN           3              // 算赛道斜率的中心差分半跨度(行)

#define ROAD_WIDE_NEAR_INIT     300            // 近端赛宽初值(像素)，算法行 IMG_H-1
#define ROAD_WIDE_HORIZON_ROW   (-16)          // 赛宽外推到 0 的那一行，可以是负数
#define ROAD_WIDE_MIN_RATIO     (0.60f)        // 半宽学习的可信下界
#define ROAD_WIDE_MAX_RATIO     (1.60f)        // 半宽学习的可信上界
#define ROAD_HALF_MIN           3              // 半宽下限(像素)，远端消隐处的兜底
#define TRACK_MIN_VALID_ROWS    12             // 最少有效中线行数

// 中线偏差由拟合曲线在可调前瞻行求值。
#define ERR_FRONT_ROW_DEFAULT   (67)             // 前瞻行(拟合线求值行)，从顶部起算的绝对行号
#define ERR_FIT_ROW_TOP         30             // 拟合区最远行号，再远的行畸变大不参与拟合

// 正式跑车(Run)
#define VISION_LINK_TIMEOUT_MS  100            // 视觉结果超时，停止前进但继续保持平衡
#define RUN_LOST_STOP_MS        600            // 连续丢线超过这么久就停车
#define RUN_STOP_SPEED_CNT      3              // 判"车已停住"的 20ms 编码器增量阈值
#define RUN_SPEED_MAX_MPS       (1.50f)         // 正式跑车绝对速度上限(m/s)

#define RUN_SPEED_STRAIGHT_DEFAULT     (1.199999f)
#define RUN_SPEED_CURVE_DEFAULT        (0.8000002f)
#define RUN_SPEED_CROSS_DEFAULT        (0.8000002f)
#define RUN_SPEED_RING_DEFAULT         (0.9999994f)
#define RUN_SPEED_RAMP_DEFAULT         (1.0f)
#define RUN_SPEED_LOST_DEFAULT         (0.2f)
#define RUN_ACCEL_MPS2_DEFAULT         (0.5f)
#define RUN_DECEL_MPS2_DEFAULT         (1.0f)
#define TRACK_CURVE_FULL_SCALE         (0.60f)
#define TRACK_QUALITY_MIN              (0.30f)
#define RUN_VALID_SPEED_FLOOR_MPS      (0.50f)  // 单项实测：有效普通赛道上，质量波动不得把速度压到近停
#define ZEBRA_STOP_OFFSET_M_DEFAULT    (0.15f)

// IPM 只变换边线点集，俯视坐标以赛道半宽为尺度。
typedef enum
{
    IPM_PICK_OK = 0,            // 四个角点都有，可以标定
    IPM_PICK_NO_TRACK,          // 这一帧循迹本身就无效
    IPM_PICK_FEW_ROWS,          // 双边都在的行数不够 IPM_FIT_MIN_ROWS
    IPM_PICK_SHORT_SPAN,        // 双边区行距不够 IPM_MIN_ROW_GAP
    IPM_PICK_NOT_STRAIGHT,      // 边线拟合残差超 IPM_FIT_TOL，弯道或车没摆正
    IPM_PICK_NARROW,            // 赛道太窄或远端不够远
} ipm_pick_t;

#define IPM_HALF_WIDTH          50             // 俯视平面里的赛道半宽，尺度基准
#define IPM_H0_DEFAULT          (2.532712f)
#define IPM_H1_DEFAULT          (-0.1793851f)
#define IPM_H2_DEFAULT          (-225.105f)
#define IPM_H3_DEFAULT          (-3.285259e-07f)
#define IPM_H4_DEFAULT          (3.455418f)
#define IPM_H5_DEFAULT          (-306.3803f)
#define IPM_H6_DEFAULT          (-3.525401e-10f)
#define IPM_H7_DEFAULT          (0.03943732f)
#define IPM_H8_DEFAULT          (1.0f)
// 标定由整段双边直线拟合生成四个角点，并排除补线点。
#define IPM_MIN_WIDTH           40             // 近端赛道最小像素宽度，比这窄说明没对准
#define IPM_FAR_MIN_WIDTH       18             // 远端采样行的最小像素宽度
#define IPM_FIT_MIN_ROWS        20             // 参与拟合的双边有效行数下限
#define IPM_FIT_TOL             (2.5f)         // 边线拟合平均残差上限(像素)，超了就不是直道
#define IPM_MIN_ROW_GAP         30             // 两个采样行的最小行距
#define IPM_CALIB_FRAMES        24             // 有效直道帧累计数量，坏帧跳过不清空累计

#define IPM_LEN_RATIO           (2.5f)
#define IPM_LEN_RATIO_MIN       (0.5f)         // 解出来超出这个范围就用兜底值
#define IPM_LEN_RATIO_MAX       (12.0f)

#define IPM_FOCAL_PIX           (130.0f)

// 元素使能默认值
#define ELEM_EN_ZEBRA_DEFAULT    (1)
#define ELEM_EN_CROSS_DEFAULT    (1)
#define ELEM_EN_RING_DEFAULT     (0)
#define ELEM_EN_RAMP_DEFAULT     (1)

// 元素超时参数采用 10ms 计数，与图像帧率无关。
#define RING_TIMEOUT_CNT        500            // 环岛单状态超时，5 秒
#define ELEM_GUARD_CNT          80             // 元素退出屏蔽，0.8 秒

// 斑马线仅统计双边有效行内的黑白条纹。
#define ZEBRA_JUMP_CNT          8              // 单行黑白跳变阈值
#define ZEBRA_EDGE_MARGIN       6              // 左右各让开这么多列，避开边线本身那一次跳变
#define ZEBRA_MIN_SPAN          40             // 赛道内部窄于这么多列就不数
#define ZEBRA_SCAN_ROWS         70             // 双边实测区里最多扫这么多行
#define ZEBRA_HIT_ROWS          3              // 扫描带里至少这么多行命中
#define ZEBRA_CONFIRM_FRAMES    3              // 连续确认帧数
#define ZEBRA_RELEASE_FRAMES    5              // 离开斑马线后的释放确认帧数
#define ZEBRA_RUN_MIN           2              // 有效黑条最小宽度(像素)
#define ZEBRA_RUN_MAX           12             // 有效黑条最大宽度(像素)
#define ZEBRA_BLACK_RUNS        4              // 单行至少包含的有效黑条数
#define ZEBRA_COVER_PERCENT     40             // 条纹横向覆盖赛道比例下限(%)

// 十字和环岛共用的边线角点参数。
#define CORNER_FLAT             5              // 平坦侧允许的逐行变化上限(列)
#define CORNER_JUMP             8              // 张开侧 2 行的最小跳变(列)
#define CORNER_JUMP2            15             // 张开侧 3、4 行的最小跳变(列)

// 十字补线优先连接上下角点，缺失下角点时延伸远端边线。
#define CROSS_LOST_CNT          15             // 丢线行数阈值
#define CROSS_CONFIRM_FRAMES    3              // 连续确认帧数
#define CROSS_RELEASE_FRAMES    5              // 双边恢复退出帧数
#define CROSS_TIMEOUT_10MS      150            // 十字超时，1.5 秒
#define CROSS_WIDE_OVER         18             // 中段赛宽余量
#define CROSS_WIDE_ROWS         4              // 中段超宽行数
#define CROSS_CORNER_GAP        12             // 上下角点最小行距，太近说明是噪点不是路口
#define CROSS_FIT_ROWS          15             // 单角点补线时参与拟合的行数(上角点往远端数)
#define CROSS_FIT_SKIP          5              // 紧挨上角点的这几行已经在拐，不进拟合
#define CROSS_EVIDENCE_ROWS     18             // 角点远侧检查外扩、丢线及远线的行数
#define CROSS_OUTWARD_MIN       10             // 角点远侧相对近侧向外扩张的最小列数
#define CROSS_FAR_LOST_ROWS     5              // 角点后迅速出画面所需的连续丢线行数
#define CROSS_FAR_LINE_ROWS     3              // 丢线后重新找到远线所需的连续实测行数

// 坡道由赛宽、前瞻和运动信息联合确认，按编码器里程退出。
#define RAMP_ROW_BOTTOM         70             // 没有实测双边区时的检测带近端行
#define RAMP_BAND_ROWS          35             // 检测带从双边实测区近端往远端数这么多行
#define RAMP_WIDE_OVER          14             // 超宽阈值(像素)
#define RAMP_WIDE_ROWS          8              // 带内需要超宽的行数
#define RAMP_STOP_ROW_MAX       95             // 120 行图像仍能看满远端时不判坡道
#define RAMP_CONFIRM_FRAMES     4              // 连续确认帧数
#define RAMP_SPEED_MIN_MPS      (0.03f)        // 低于该速度不进入坡道状态
#define RAMP_PITCH_MIN          (2.5f)         // 俯仰偏差佐证(°)
#define RAMP_PITCH_RATE_MIN     (8.0f)         // 坡脚处俯仰角速度佐证(°/s)
#define RAMP_DRIVE_OUT_MIN      (1200.0f)      // 行进轮控制输出佐证
#define RAMP_ENTRY_CNT_MIN      20             // 连续候选期间至少前进的编码器 counts
#define RAMP_DRIVE_FRAMES       2              // 连续候选期间大驱动输出的最少帧数
#define RAMP_EXIT_CNT           4000           // 进坡后按里程保持多少 counts 才退出
#define RAMP_TIMEOUT_10MS       500            // 坡道超时，5 秒

// 环岛状态机
#define RING_LOST_MIN           12             // 丢线计数下界
#define RING_LOST_MAX           50             // 丢线计数上界
#define RING_OPP_LOST           5              // 对侧丢线阈值
#define RING_VIEW               60             // 环岛最小有效前瞻行数
#define RING_CONFIRM_FRAMES     5              // 连续确认帧数
#define RING_ENTRY_NEAR_ROW     (IMG_H - 20)   // 候选时左右近端边线都必须可见
#define RING_ENTRY_FAR_ROW      (IMG_H / 4)    // 目标侧近端消失后只在远端重新可见
#define RING_STRAIGHT_RES       (3.0f)         // 对侧边线直线拟合平均残差上限(像素)
#define EDGE_RES_MIN_ROWS       25             // 算边线残差至少要这么多有效行
#define RING_S2_CNT_R_DEFAULT   (400)            // 状态2 右环编码器累计阈值
#define RING_S2_CNT_L_DEFAULT   (300)            // 状态2 左环编码器累计阈值
#define RING_ANGLE_DEFAULT      (340)            // 元素积分角阈值(°)
#define RING_S4_CNT             2000           // 状态4 计数阈值
#define RING_S5_CNT             1000           // 状态5 计数阈值

// 非正式 Run 的遥控与里程测试沿用原 counts/20ms 斜坡，不受 Run 参数影响。
#define MOTION_SPEED_UP_STEP_COUNT   (1.2f)
#define MOTION_SPEED_DOWN_STEP_COUNT (1.9f)
#define CAM_EXPOSURE_DEFAULT    (48)           // 摄像头曝光时间，实车可用值在 48 附近
#define CAM_DIAG_FPS            (50)           // optional Seekfree raw-image diagnostic
#define CAM_DIAG_EXPOSURE       (512)
#define VISION_FPS_WIN_MS       (500u)         // 帧率统计窗口(ms)，窗口越长读数越稳、跟随越慢

// C 轮里程标定
#define ODOM_COUNTS_PER_M_DEFAULT   (11695.0f)
#define ODOM_TEST_SPEED_DEFAULT     (0.05f)
#define ODOM_TEST_DISTANCE_M        (1.0f)
#define ODOM_TEST_SLOW_DISTANCE_M   (0.20f)
#define ODOM_TEST_MIN_SPEED_MPS     (0.05f)

// 无线 Run Test：speed:<相对航向角输入>,<速度输入>，两个输入范围均为 -90~90。
#define REMOTE_STEER_INPUT_LIMIT    (90.0f)
#define REMOTE_SPEED_INPUT_LIMIT    (90.0f)
#define REMOTE_SPEED_INPUT_DIVISOR  (40.0f)     // 速度输入除以 40 得到 m/s
#define REMOTE_SPEED_LIMIT_MPS      RUN_SPEED_MAX_MPS
#define REMOTE_CMD_TIMEOUT_MS       (1000u)

// ===== WiFi 公共配置 =====
// 1：WiFi SPI；0：原 UART2。电脑和车辆须连接同一局域网。
// 本机的热点名、密码和电脑 IP 仅填写 code/config_private.h。
#ifndef STATION_WIFI_ENABLED
#define STATION_WIFI_ENABLED 1
#endif
#define STATION_WIFI_PC_PORT "1349"       // 电脑上位机监听端口
#define STATION_WIFI_LOCAL_PORT "6666"    // 车载模块本地端口
#define STATION_WIFI_FRAME_MAX 512u       // 单帧 SPI 发送缓冲上限
#define STATION_WIFI_RX_MAX 32u           // 单次 SPI 接收缓冲上限
#if STATION_WIFI_ENABLED
#include <config_private.h>
#if !defined(STATION_WIFI_SSID) || !defined(STATION_WIFI_PASSWORD) || !defined(STATION_WIFI_PC_IP)
#error "Copy code/config_private.example.h to code/config_private.h and fill local WiFi settings"
#endif
#endif

#endif /* CONFIG_H_ */
