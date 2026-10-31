#ifndef ONEWHEEL_CONFIG_H
#define ONEWHEEL_CONFIG_H
/* Default TC264 wiring proposal; confirm against your actual PCB. */
#define QCAR_DRIVER_UART UART_3
#define QCAR_DRIVER_TX UART3_TX_P15_6
#define QCAR_DRIVER_RX UART3_RX_P15_7
#define QCAR_DRIVER_BAUD 460800
/* Text duty avoids stock CYT2BL3 binary/newline ambiguity. Set 0 only with
 * the corrected driver parser. Speed RX accepts both binary and text. */
#ifndef QCAR_DRIVER_TEXT_DUTY
#define QCAR_DRIVER_TEXT_DUTY 1
#endif
#if QCAR_DRIVER_TEXT_DUTY != 0 && QCAR_DRIVER_TEXT_DUTY != 1
#error QCAR_DRIVER_TEXT_DUTY must be 0 or 1
#endif
#define QCAR_DRIVER_SPEED_QUERY_MS 20u
#define QCAR_DRIVER_FEEDBACK_TIMEOUT_MS 100u
/* 左右飞轮最终输出方向：+1保持逻辑方向，-1反向；只允许+1或-1。
 * 两路独立，适用于Mode3/5/6/8和停车诊断；不改变PID Sign和原始转速。
 * 默认保持原有两路反向效果；修改后重新编译、烧录TC264。 */
#ifndef QCAR_FLYWHEEL_LEFT_SIGN
#define QCAR_FLYWHEEL_LEFT_SIGN  (-1)
#endif
#ifndef QCAR_FLYWHEEL_RIGHT_SIGN
#define QCAR_FLYWHEEL_RIGHT_SIGN (-1)
#endif
#if QCAR_FLYWHEEL_LEFT_SIGN != 1 && QCAR_FLYWHEEL_LEFT_SIGN != -1
#error QCAR_FLYWHEEL_LEFT_SIGN must be +1 or -1
#endif
#if QCAR_FLYWHEEL_RIGHT_SIGN != 1 && QCAR_FLYWHEEL_RIGHT_SIGN != -1
#error QCAR_FLYWHEEL_RIGHT_SIGN must be +1 or -1
#endif
#define QCAR_MOTOR_PWM ATOM0_CH3_P21_5
#define QCAR_MOTOR_DIR P21_4
#define QCAR_KEY1 P20_7
#define QCAR_KEY2 P20_6
#define QCAR_KEY3 P11_3
#define QCAR_EC11_KEY P11_2
/* 1: initialize the IMU and start the configured sampling timer at boot.
 * 0: leave the IMU off until Start / IMU preview. Use 0 to bisect a
 *    boot-time menu freeze between the IMU path and the rest. */
#ifndef QCAR_BOOT_IMU_INIT
#define QCAR_BOOT_IMU_INIT 1
#endif
/* 1: load saved parameters from DFlash at boot. 0: keep the compiled-in
 * defaults only. Setting 0 isolates the menu from the boot-time flash
 * read (Storage -> Load also does nothing while this is 0). */
#ifndef QCAR_BOOT_LOAD_FLASH
#define QCAR_BOOT_LOAD_FLASH 0
#endif
/* CPU0 owns WiFi-SPI UDP telemetry. */
#ifndef QCAR_WIFI_ENABLED
#define QCAR_WIFI_ENABLED 1
#endif
#if QCAR_WIFI_ENABLED != 0 && QCAR_WIFI_ENABLED != 1
#error QCAR_WIFI_ENABLED must be 0 or 1
#endif
#ifndef QCAR_WIFI_SSID
#define QCAR_WIFI_SSID "HDUASC"
#endif
#ifndef QCAR_WIFI_PASSWORD
#define QCAR_WIFI_PASSWORD "zyz520520"
#endif
#ifndef QCAR_WIFI_TARGET_IP
#define QCAR_WIFI_TARGET_IP "192.168.0.113"
#endif
#ifndef QCAR_WIFI_TARGET_PORT
#define QCAR_WIFI_TARGET_PORT "8081"
#endif
#ifndef QCAR_WIFI_LOCAL_PORT
#define QCAR_WIFI_LOCAL_PORT "5001"
#endif
#ifndef QCAR_WIFI_STARTUP_DELAY_MS
#define QCAR_WIFI_STARTUP_DELAY_MS 300u
#endif
#ifndef QCAR_WIFI_INIT_ATTEMPTS
#define QCAR_WIFI_INIT_ATTEMPTS 3u
#endif
#ifndef QCAR_WIFI_RETRY_DELAY_MS
#define QCAR_WIFI_RETRY_DELAY_MS 500u
#endif
#ifndef QCAR_WIFI_PERIOD_MS
#define QCAR_WIFI_PERIOD_MS 10u
#endif
#if QCAR_WIFI_PERIOD_MS < 2 || QCAR_WIFI_PERIOD_MS > 1000
#error QCAR_WIFI_PERIOD_MS must be between 2 and 1000
#endif
#if QCAR_WIFI_INIT_ATTEMPTS < 1
#error QCAR_WIFI_INIT_ATTEMPTS must be positive
#endif


/* zf_device_imu660rb.h：现用接线/速率 */
#define IMU660RB_USE_SOFT_IIC                       (0)                         // 默认使用硬件 SPI 方式驱动
#define IMU660RB_SOFT_IIC_DELAY                     (59 )                       // 软件 IIC 的时钟延时周期 数值越小 IIC 通信速率越快
#define IMU660RB_SCL_PIN                            (P20_11)                    // 软件 IIC SCL 引脚 连接 IMU660RB 的 SCL 引脚
#define IMU660RB_SDA_PIN                            (P20_14)                    // 软件 IIC SDA 引脚 连接 IMU660RB 的 SDA 引脚
#define IMU660RB_SPI_SPEED                          (10 * 1000 * 1000)          // 硬件 SPI 速率
#define IMU660RB_SPI                                (SPI_0           )          // 硬件 SPI 号
#define IMU660RB_SPC_PIN                            (SPI0_SCLK_P20_11)          // 硬件 SPI SCK 引脚
#define IMU660RB_SDI_PIN                            (SPI0_MOSI_P20_14)          // 硬件 SPI MOSI 引脚
#define IMU660RB_SDO_PIN                            (SPI0_MISO_P20_12)          // 硬件 SPI MISO 引脚
#define IMU660RB_CS_PIN                             (P20_13)                    // CS 片选引脚

/* zf_device_wifi_spi.h：现用接线/速率 */
#define WIFI_SPI_INDEX              (SPI_3             )        // 定义使用的SPI号
#define WIFI_SPI_SPEED              (10 * 1000 * 1000  )        // 硬件 SPI 速率
#define WIFI_SPI_SCK_PIN            (SPI3_SCLK_P22_3   )        // 定义SPI_SCK引脚
#define WIFI_SPI_MOSI_PIN           (SPI3_MOSI_P22_0   )        // 定义SPI_MOSI引脚
#define WIFI_SPI_MISO_PIN           (SPI3_MISO_P22_1   )        // 定义SPI_MISO引脚  IPS没有MISO引脚，但是这里任然需要定义，在spi的初始化时需要使用
#define WIFI_SPI_CS_PIN             (P22_2             )        // 定义SPI_CS引脚 采用软件CS引脚
#define WIFI_SPI_INT_PIN            (P15_8             )        // 定义握手引脚
#define WIFI_SPI_RST_PIN            (P23_1             )        // 定义复位引脚
#define WIFI_SPI_RECVIVE_FIFO_SIZE  (1024)                      // 接收FIFO大小
#define WIFI_SPI_READ_TRANSFER      (1)                         // 在调用wifi_spi_read_buffer 是否尝试发起SPI通讯来检测模块内是否有数据需要读取 1：发起SPI通讯 0：不发起SPI通讯，仅读取FIFO
#define WIFI_SPI_AUTO_CONNECT       (0)                         // 定义是否初始化时建立TCP或者UDP连接    0-不自动连接  1-自动连接TCP服务器  2-自动连接UDP

/* zf_device_ips200.h：现用接线/速率 */
#define IPS200_USE_SOFT_SPI             (0 )                                    // 默认使用硬件 SPI 方式驱动 建议使用硬件 SPI 方式驱动
#define IPS200_SOFT_SPI_DELAY           (0 )                                    // 软件 SPI 的时钟延时周期 数值越小 SPI 通信速率越快
#define IPS200_SCL_PIN                  (P15_3)                                 // 软件 SPI SCK 引脚
#define IPS200_SDA_PIN                  (P15_5)                                 // 软件 SPI MOSI 引脚
#define IPS200_SPI_SPEED                (60*1000*1000)                          // 硬件 SPI 速率
#define IPS200_SPI                      (SPI_2           )                      // 硬件 SPI 号
#define IPS200_SCL_PIN_SPI              (SPI2_SCLK_P15_3 )                      // 硬件 SPI SCK 引脚
#define IPS200_SDA_PIN_SPI              (SPI2_MOSI_P15_5 )                      // 硬件 SPI MOSI 引脚
#define IPS200_SDA_IN_PIN_SPI           (SPI2_MISO_P15_4 )                      // 硬件 SPI MISO 引脚  IPS没有MISO引脚，但是这里任然需要定义，在spi的初始化时需要使用
#define IPS200_RST_PIN_SPI              (P15_1)                                 // (SPI串口)液晶复位引脚定义
#define IPS200_DC_PIN_SPI               (P15_0)                                 // (SPI串口)液晶命令位引脚定义
#define IPS200_CS_PIN_SPI               (P15_2)                                 // (SPI串口)液晶片选引脚定义
#define IPS200_BLk_PIN_SPI              (P15_4)                                 // (SPI串口)液晶背光引脚定义

/* ===== IMU / EKF：保留现有算法参数和机体轴映射 ===== */
/* EKF noise densities (rad^2/s, rad^2/s^3), normalized gravity variance.
 * Gate uses normalized innovation squared, 3 DOF, 99 percent threshold.
 * No small-gyro deadband: preserve slow real yaw motion. */
#define EKF_GYRO_NOISE           0.0001f
#define EKF_BIAS_NOISE           0.00000001f
#define EKF_ACCEL_VARIANCE       0.0025f
#define EKF_CHI_SQUARE_LIMIT     11.345f
/* Single software preprocessing stage BEFORE fusion. No post-EKF LPF. */
#define IMU_GYRO_INPUT_TAU_S     0.002f
#define IMU_ACCEL_INPUT_TAU_S    0.004f
#define IMU_GYRO_SPIKE_DPS       300.0f
#define IMU_ACCEL_SPIKE_G        2.0f
#define EKF_REACQUIRE_STABLE_S   0.5f
#define EKF_REACQUIRE_GYRO_RADPS 0.03490659f
#define EKF_REACQUIRE_VIB_G      0.015f
#define EKF_REACQUIRE_NORM_G     0.03f
#define EKF_CORRECTION_MAX_RADPS 0.34906585f
#define EKF_BIAS_LIMIT_RADPS     0.08726646f
/* IMU quality: seconds, g and rad/s. Independent of the control-rate LPF. */
#define EKF_QUALITY_TAU_S         1.0f
#define EKF_VIBRATION_TAU_S       0.05f
#define EKF_ACCEL_SOFT_ERROR_G    0.04f
#define EKF_ACCEL_SOFT_VIB_G      0.05f
#define EKF_NIS_SOFT_LIMIT        3.0f
#define EKF_R_SCALE_MAX           16.0f
#define EKF_BIAS_LEARN_MAX_RADPS  0.17453293f
#define EKF_BIAS_SLEW_RADPS2      0.01f
#define EKF_GYRO_STEP_MAX_RADPS   0.34906585f
#define IMU_PREDICT_LIMIT_S       0.5f
#define IMU_POOR_ACCEPT_RATIO     0.5f
#define IMU_POOR_ACCEPT_LIMIT_S   1.0f
#define IMU_POOR_R_SCALE          8.0f
#define IMU_RAW_CLIP_COUNTS       32000

#define ACC_NORM_MIN             0.85f
#define ACC_NORM_MAX             1.15f
#define MAX_SAMPLE_DT            0.020f
/* 超过 MAX_SAMPLE_DT 但在本值以内：只丢这一帧，不锁止（总线偶发毛刺）。 */
#define MAX_SKIP_DT              0.050f

#define CALIBRATION_SAMPLES      500u /* 500Hz: keep a 1 second stillness window. */
#define IMU_CTRL1_XL_CONFIG      0x7Eu /* 833Hz accelerometer, +/-8g. */
#define IMU_CTRL2_G_CONFIG       0x7Cu /* 833Hz gyroscope, +/-2000dps. */
#define IMU_CTRL3_C_CONFIG       0x44u /* BDU + address increment, NOT a filter. */
#define IMU_CTRL4_C_CONFIG       0x02u /* Enable gyro LPF1. */
#define IMU_CTRL6_C_CONFIG       0x02u /* FTYPE=010: 140Hz at 833Hz ODR. */
#define IMU_CTRL7_G_CONFIG       0x00u /* Gyro high performance, HPF disabled. */
#define IMU_CTRL8_XL_CONFIG      0x00u /* No acceleration high pass/user offset. */
#define CAL_GYRO_LIMIT_DPS       10.0f
#define CAL_GYRO_SPAN_DPS        6.0f
#define CAL_ACCEL_SPAN_G         0.12f
/* Reject sustained rotation even if its peak-to-peak span is small. */
#define CAL_GYRO_MEAN_MAX_DPS    3.5f
#define IMU_TIMEOUT_MS           100u

/* Observed mounting: side tilt was old pitch, fore/aft was old roll.
 * Body X/Y/Z = sensor Z/-Y/X. New X=old Y, new Y=-old X, new Z=old Z.
 * Both accel and gyro use this right-handed mapping. Motor sign is separate. */
#define IMU_BODY_X_AXIS 2
#define IMU_BODY_Y_AXIS 1
#define IMU_BODY_Z_AXIS 0
#define IMU_BODY_X_SIGN 1.0f
#define IMU_BODY_Y_SIGN -1.0f
#define IMU_BODY_Z_SIGN 1.0f
#define DEG_TO_RAD 0.01745329251994329577f
#define RAD_TO_DEG 57.29577951308232088f

/* ===== 直立参数：只在此处改编译默认值；菜单/上位机改的是 RAM =====
 * displayed Roll = flywheel = imu_pitch_deg / gyro.y
 * displayed Pitch = ground wheel = imu_roll_deg / gyro.x
 * 零点采用本车静止扶稳读数；不要套用旧版原始量零点 -100 / 190。
 */
#define BALANCE_ZERO_X_DEG           4.7f
#define BALANCE_ZERO_Y_DEG           -1.0f
#define FLY_RATE_KP                 14.4f
#define FLY_RATE_KI                  0.2f
#define FLY_RATE_KD                  0.0f
#define FLY_ANGLE_KP                75.0f
#define FLY_ANGLE_KI                 0.0f
#define FLY_ANGLE_KD                 0.3f
#define FLY_VEL_KP                   0.05f
#define FLY_VEL_KD                   0.002f
#define FLY_VEL_D_LPF_TAU_S          0.030f
#define FLY_VEL_ANGLE_LIMIT_DEG     15.0f
#define FLY_RATE_LIMIT_DPS         160.0f
#define FLY_PWM_LIMIT             8000.0f
#define FLY_OUTPUT_SIGN              -1.0f
#define GROUND_RATE_KP              2.0f
#define GROUND_RATE_KI               0.0f
#define GROUND_RATE_KD               0.0f
#define GROUND_ANGLE_KP             10.0f
#define GROUND_ANGLE_KI              0.0f
#define GROUND_ANGLE_KD              0.0f
#define GROUND_RATE_LIMIT_DPS      100.0f
#define GROUND_PWM_LIMIT          6000.0f
#define GROUND_OUTPUT_SIGN           1.0f
/* Mode2 speed outer loop: rightward travel has +counts, right tilt lowers
 * displayed Pitch. Negative PI output therefore adds a positive target tilt.
 * These four defaults/menu RAM values are not stored in the existing Flash. */
#define WHEEL_SPEED_KP               0.001f
#define WHEEL_SPEED_KI               0.0f
#define WHEEL_SPEED_TILT_LIMIT_DEG    1.0f
#define WHEEL_SPEED_I_LIMIT_DEG       0.5f
#define WHEEL_SPEED_TILT_SIGN        -1.0f
#define WHEEL_SPEED_GAIN_MAX          0.1f
#define WHEEL_SPEED_TILT_MAX_DEG      5.0f
#define WHEEL_SPEED_ENCODER_TIMEOUT_MS 60u
/* Position P -> limited target speed, in encoder counts (4096/wheel rev). */
#define WHEEL_POSITION_KP              0.5f
#define WHEEL_POSITION_SPEED_LIMIT_CPS 300.0f
#define WHEEL_POSITION_GAIN_MAX        5.0f
#define WHEEL_POSITION_SPEED_MAX_CPS 2000.0f
#define WHEEL_POSITION_PERIOD_MS      100u
#define WHEEL_POSITION_DEADBAND_COUNT   8u
/* Allowed tuning ranges, separate from the active/default limits above. */
#define BALANCE_RATE_LIMIT_MAX_DPS  200.0f
#define BALANCE_PWM_LIMIT_MAX      8000.0f
/* Ground-wheel static friction: the geared travel wheel needs roughly this
 * much duty before it breaks away, so the closed-loop ground output offsets a
 * nonzero command in its own direction (feed-forward). The offset follows the
 * startup ramp, so Start still ramps smoothly. 0 disables the compensation;
 * Mode8 raw PWM and the menu PWM test never use it. */
/* Independent nonnegative PWM offsets, selected after output direction mapping.
 * 0 disables that direction's compensation. WiFi edits RAM, not these defaults. */
#ifndef GROUND_DEADZONE_POS_PWM
#define GROUND_DEADZONE_POS_PWM    2000.0f
#endif
#ifndef GROUND_DEADZONE_NEG_PWM
#define GROUND_DEADZONE_NEG_PWM    3500.0f
#endif
/* Negative or zero is treated as "no feed-forward" by the shaping helper;
 * the TASKING preprocessor cannot compare float literals in #if. */
/* PID gains use seconds: integral += Ki*error*dt, derivative = Kd*de/dt.
 * Defaults remain P-only. 0=position form, 1=incremental form. */
#define BALANCE_ANGLE_PID_INCREMENTAL 0
#define BALANCE_RATE_PID_INCREMENTAL  0
#define BALANCE_ANGLE_I_LIMIT_DPS     0.0f
#define BALANCE_RATE_I_LIMIT_PWM      0.0f
/* Control uses EKF angles and the SAME prepared gyro used by EKF prediction.
 * Legacy zero constants only support old external test/tools; no post-filter code. */
#define FLY_RATE_LPF_TAU_S            0.0f
#define BALANCE_ANGLE_LPF_TAU_S       0.0f
/* Flywheel angle-error -> PWM, Modes 3/5 only. PWM per degree; 0 disables.
 * Compile-time setting, independent of Flash/RAM PID gains. */
#ifndef FLY_ANGLE_FF_KP
#define FLY_ANGLE_FF_KP             300.0f
#endif
#define BALANCE_TILT_LIMIT_DEG       30.0f
/* Tolerate brief publication gaps; fault at 50ms without a new fused sample. */
#define BALANCE_IMU_STALE_MS         50u
#define BALANCE_STARTUP_RAMP_MS    1000u
#define APP_DEFAULT_MODE               5u /* 2 dual + ground speed; 3/4/5 angle; 6/7 rate-only; 8 PWM debug */
/* Mode8 signed raw duty commands; 10000 means 100% duty. Starts at zero.
 * No PID, IMU readiness or tilt interlock in this bench-test mode. */
#define DEBUG_PWM_LIMIT             4000.0f
#define APP_CONTROL_TICK_US         1000u
#define APP_CONTROL_DIVIDER            2u
#define APP_IMU_PERIOD_US           2000u
#define APP_KEY_PERIOD_MS              1u
#define APP_POLL_PERIOD_MS            10u
#define APP_DISPLAY_DIVIDER            5u
#define APP_WIFI_ANNOUNCE_MS        1000u
#define APP_TELEMETRY_PROFILE_MS      20u
#define MOTOR_PWM_HZ               17000u
/* Actual drive encoder wiring: count P33.7, direction P33.6. */
#define MOTOR_ENCODER_TIM           TIM2_ENCODER
#define MOTOR_ENCODER_COUNT_PIN     TIM2_ENCODER_CH1_P33_7
#define MOTOR_ENCODER_DIR_PIN       TIM2_ENCODER_CH2_P33_6
/* Rising-edge counting (1x): 1024 pulses/encoder rev, 4 encoder revs/wheel rev. */
#define WHEEL_ENCODER_PULSES_PER_REV          1024u
#define WHEEL_ENCODER_REVS_PER_WHEEL_REV         4u
#define WHEEL_ENCODER_COUNTS_PER_WHEEL_REV \
    (WHEEL_ENCODER_PULSES_PER_REV * WHEEL_ENCODER_REVS_PER_WHEEL_REV)
/* Ground-wheel observation only; sign affects counts, never PWM. */
#ifndef WHEEL_ENCODER_SIGN
#define WHEEL_ENCODER_SIGN             1
#endif
#define WHEEL_ENCODER_READ_MS          2u
#define WHEEL_ENCODER_SPEED_MS        20u
#define WHEEL_ENCODER_MAX_GAP_MS      10u
#define WHEEL_ENCODER_POSITION_LIMIT 16777215L
#if WHEEL_ENCODER_SIGN != 1 && WHEEL_ENCODER_SIGN != -1
#error WHEEL_ENCODER_SIGN must be +1 or -1
#endif
#if WHEEL_ENCODER_READ_MS < 1 || WHEEL_ENCODER_READ_MS > WHEEL_ENCODER_MAX_GAP_MS || WHEEL_ENCODER_MAX_GAP_MS > WHEEL_ENCODER_SPEED_MS || WHEEL_ENCODER_SPEED_MS > 1000
#error Invalid wheel encoder sampling periods
#endif
#define APP_KEY_DEBOUNCE_MS            10u
#define APP_KEY_LONG_PRESS_MS        1000u
#define APP_MENU_CAPACITY             64u
#define APP_MENU_STEPS {0.01f,0.1f,1.0f,10.0f,100.0f}

#if APP_DEFAULT_MODE < 2 || APP_DEFAULT_MODE > 8
#error APP_DEFAULT_MODE must be 2..8
#endif
#if APP_CONTROL_TICK_US * APP_CONTROL_DIVIDER != APP_IMU_PERIOD_US
#error Control and IMU update periods must match
#endif
#if APP_IMU_PERIOD_US < 1000u || APP_IMU_PERIOD_US % 1000u != 0u
#error Control timeout and startup timers require whole milliseconds
#endif

/* Interrupt routing/priorities (all unique). */
#define CCU6_0_CH0_INT_SERVICE	IfxSrc_Tos_cpu0	    // 定义CCU6_0 PIT通道0中断服务类型，即中断是由谁响应处理 IfxSrc_Tos_cpu0 IfxSrc_Tos_cpu1 IfxSrc_Tos_dma 不可设置为其他值
#define CCU6_0_CH0_ISR_PRIORITY 30	                // 定义CCU6_0 PIT通道0中断优先级 优先级范围1-255 越大优先级越高 与平时使用的单片机不一样
#define CCU6_0_CH1_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_0_CH1_ISR_PRIORITY 31
#define CCU6_1_CH0_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_1_CH0_ISR_PRIORITY 32
#define CCU6_1_CH1_INT_SERVICE	IfxSrc_Tos_cpu0
#define CCU6_1_CH1_ISR_PRIORITY 33
#define EXTI_CH0_CH4_INT_SERVICE IfxSrc_Tos_cpu0	// 定义ERU通道0和通道4中断服务类型，即中断是由谁响应处理 IfxSrc_Tos_cpu0 IfxSrc_Tos_cpu1 IfxSrc_Tos_dma  不可设置为其他值
#define EXTI_CH0_CH4_INT_PRIO  	40	                // 定义ERU通道0和通道4中断优先级 优先级范围1-255 越大优先级越高 与平时使用的单片机不一样
#define EXTI_CH1_CH5_INT_SERVICE IfxSrc_Tos_cpu0	// 定义ERU通道1和通道5中断服务类型，同上
#define EXTI_CH1_CH5_INT_PRIO  	41	                // 定义ERU通道1和通道5中断优先级 同上
#define EXTI_CH2_CH6_INT_SERVICE IfxSrc_Tos_dma	    // 定义ERU通道2和通道6中断服务类型，同上
#define EXTI_CH2_CH6_INT_PRIO  	5	                // 定义ERU通道2和通道6中断优先级 可设置范围为0-47
#define EXTI_CH3_CH7_INT_SERVICE IfxSrc_Tos_cpu0	// 定义ERU通道3和通道7中断服务类型，同上
#define EXTI_CH3_CH7_INT_PRIO  	43	                // 定义ERU通道3和通道7中断优先级 同上
#define	DMA_INT_SERVICE         IfxSrc_Tos_cpu0	    // ERU触发DMA中断服务类型，即中断是由谁响应处理 IfxSrc_Tos_cpu0 IfxSrc_Tos_cpu1 IfxSrc_Tos_dma  不可设置为其他值
#define DMA_INT_PRIO  	        60	                // ERU触发DMA中断优先级 优先级范围1-255 越大优先级越高 与平时使用的单片机不一样
#define	UART0_INT_SERVICE       IfxSrc_Tos_cpu0	    // 定义串口0中断服务类型，即中断是由谁响应处理 IfxSrc_Tos_cpu0 IfxSrc_Tos_cpu1 IfxSrc_Tos_dma  不可设置为其他值
#define UART0_TX_INT_PRIO       10	                // 定义串口0发送中断优先级 优先级范围1-255 越大优先级越高 与平时使用的单片机不一样
#define UART0_RX_INT_PRIO       11	                // 定义串口0接收中断优先级 优先级范围1-255 越大优先级越高 与平时使用的单片机不一样
#define UART0_ER_INT_PRIO       12	                // 定义串口0错误中断优先级 优先级范围1-255 越大优先级越高 与平时使用的单片机不一样
#define	UART1_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART1_TX_INT_PRIO       13
#define UART1_RX_INT_PRIO       14
#define UART1_ER_INT_PRIO       15
#define	UART2_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART2_TX_INT_PRIO       16
#define UART2_RX_INT_PRIO       17
#define UART2_ER_INT_PRIO       18
#define	UART3_INT_SERVICE       IfxSrc_Tos_cpu0
#define UART3_TX_INT_PRIO       19
#define UART3_RX_INT_PRIO       20
#define UART3_ER_INT_PRIO       21
#endif /* ONEWHEEL_CONFIG_H */
