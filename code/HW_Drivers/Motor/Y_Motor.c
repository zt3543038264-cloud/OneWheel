#include "HW_Drivers/Motor/Y_Motor.h"
#include <stddef.h>
#include "Common/board_config.h"
#include <limits.h>
#if C_BENCH_SIGNAL_OUTPUT_ALLOWED
#include "IfxStm.h"
#endif

#define Y_MOTOR_SPEED_WINDOW_SAMPLES  (CTRL_DIV_SPEED / Y_MOTOR_ENCODER_PERIOD_MS)

#if ((CTRL_DIV_SPEED % Y_MOTOR_ENCODER_PERIOD_MS) != 0)
    #error "CTRL_DIV_SPEED 必须是 Y_MOTOR_ENCODER_PERIOD_MS 的整数倍"
#endif

#if (DRIVE_OUT_LIMIT > Y_MOTOR_PWM_MAX_DUTY)
    #error "DRIVE_OUT_LIMIT 不能超过行进轮 PWM 硬件量程"
#endif

#pragma section all "cpu0_dsram"
static volatile int16 y_motor_count_5ms;        // 最近一次 5ms 采样的计数
static volatile int16 y_motor_count_20ms;       // 最近一个完整 20ms 窗口的计数
static volatile int32 y_motor_total_count;      // 自上次清零以来的累计计数
static int32 y_motor_window_count;              // 当前 20ms 窗口的累加值
static uint8 y_motor_window_samples;            // 当前 20ms 窗口已累加的 5ms 采样数
static volatile uint32 y_motor_sample_id_20ms;
#if C_BENCH_SIGNAL_OUTPUT_ALLOWED
static int32 y_motor_bench_applied;
static uint32 y_motor_bench_ticks_per_ms;
static uint64 y_motor_bench_zero_at;
#endif
static uint8 y_motor_dir_level;                 // 当前 DIR 电平，换向时才重写
#pragma section all restore

#if C_BENCH_SIGNAL_OUTPUT_ALLOWED
static uint64 y_motor_bench_now(void)
{
    uint32 hi, lo, again;
    do {
        hi = IfxStm_getOffset32Timer(&MODULE_STM0);
        lo = IfxStm_getLower(&MODULE_STM0);
        again = IfxStm_getOffset32Timer(&MODULE_STM0);
    } while (hi != again);
    return ((uint64)hi << 32) | lo;
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将行进轮控制量限制到软件安全范围
// 参数说明     duty            待限制控制量
// 返回参数     int32           限幅后的控制量
// 使用示例     duty = Y_Motor_LimitDuty(duty);
//-------------------------------------------------------------------------------------------------------------------
static int32 Y_Motor_LimitDuty(int32 duty)
{
    return func_limit_ab(duty, -DRIVE_OUT_LIMIT, DRIVE_OUT_LIMIT);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化行进轮 C 的方向、PWM 和脉冲方向编码器
// 参数说明     void
// 返回参数     void
// 使用示例     Y_Motor_Init();
//-------------------------------------------------------------------------------------------------------------------
void Y_Motor_Init(void)
{
    y_motor_sample_id_20ms = 0;
    y_motor_count_5ms = 0;
    y_motor_count_20ms = 0;
    y_motor_total_count = 0;
    y_motor_window_count = 0;
    y_motor_window_samples = 0;
    y_motor_dir_level = (uint8)Y_MOTOR_FORWARD_DIR_LEVEL;
#if C_BENCH_SIGNAL_OUTPUT_ALLOWED
    {
        float frequency = IfxStm_getFrequency(&MODULE_STM0);
        y_motor_bench_ticks_per_ms =
            (frequency >= 1000.0f && frequency <= 1000000000.0f) ?
                (uint32)(frequency / 1000.0f) : 0u;
    }
    y_motor_bench_applied = 0;
    y_motor_bench_zero_at = y_motor_bench_now();
#endif

    gpio_init(Y_MOTOR_DIR_PIN,
              GPO,
              Y_MOTOR_FORWARD_DIR_LEVEL,
              GPO_PUSH_PULL);
    pwm_init(Y_MOTOR_PWM_PIN, Y_MOTOR_PWM_FREQUENCY_HZ, 0);

    encoder_dir_init(Y_MOTOR_ENCODER_INDEX,
                     Y_MOTOR_ENCODER_A_PIN,
                     Y_MOTOR_ENCODER_B_PIN);
    encoder_clear_count(Y_MOTOR_ENCODER_INDEX);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置行进轮 C 的有符号占空比并执行方向、限幅和死区补偿
// 参数说明     duty            行进轮控制量，符号决定转向
// 返回参数     void
// 使用示例     Y_Motor_SetDuty(g_motor_c);
//-------------------------------------------------------------------------------------------------------------------
void Y_Motor_SetDuty(int32 duty)
{
    uint8 level;

    if (!VEHICLE_MOTOR_OUTPUT_ALLOWED) duty = 0;
    duty = Y_Motor_LimitDuty(duty);
    duty *= MOTOR_DIR_C;
    if (duty > 0)
        duty += (int32)QCAR_GROUND_DEADZONE_POS;        // 补掉起转死区，输出直接从 0 跳到 DRIVE_DEAD_ZONE
    else if (duty < 0)
        duty -= (int32)QCAR_GROUND_DEADZONE_NEG;
    duty = Y_Motor_LimitDuty(duty);     // 死区补偿后仍不得越过 C 轮控制安全限幅

    if (duty == 0)
    {
        pwm_set_duty(Y_MOTOR_PWM_PIN, 0);
        return;                         // 保持当前 DIR 电平，下次同向时不用再翻
    }

    level = (uint8)((duty > 0) ? Y_MOTOR_FORWARD_DIR_LEVEL
                               : ((Y_MOTOR_FORWARD_DIR_LEVEL == GPIO_HIGH) ? GPIO_LOW : GPIO_HIGH));
    if (level != y_motor_dir_level)
    {
        pwm_set_duty(Y_MOTOR_PWM_PIN, 0);   // 只在换向这一拍卸载，避免 H 桥直通
        gpio_set_level(Y_MOTOR_DIR_PIN, level);
        y_motor_dir_level = level;
    }

    pwm_set_duty(Y_MOTOR_PWM_PIN, (uint32)((duty > 0) ? duty : -duty));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将行进轮 C 的 PWM 占空比清零
// 参数说明     void
// 返回参数     void
// 使用示例     Y_Motor_Stop();
//-------------------------------------------------------------------------------------------------------------------
void Y_Motor_Stop(void)
{
    pwm_set_duty(Y_MOTOR_PWM_PIN, 0);
#if C_BENCH_SIGNAL_OUTPUT_ALLOWED
    if (y_motor_bench_applied != 0) y_motor_bench_zero_at = y_motor_bench_now();
    y_motor_bench_applied = 0;
#endif
}

/* Raw signed diagnostic PWM, no MOTOR_DIR_C and NO legacy deadzone. The
 * controller owns its session, lease and sensor checks. This sink independently
 * enforces the build gate and absolute raw range before any abs conversion.
 * A sign change holds PWM zero for >=2 real STM ms before changing DIR. This
 * is software break-before-make, NOT verified gate dead time or hardware stop. */
int32 Y_Motor_BenchSetRawDuty(int32 duty)
{
#if C_BENCH_SIGNAL_OUTPUT_ALLOWED
    uint8 level;
    uint64 now;
    duty = func_limit_ab(duty, -C_BENCH_RAW_DUTY_MAX, C_BENCH_RAW_DUTY_MAX);
    if (!y_motor_bench_ticks_per_ms || !duty) { Y_Motor_Stop(); return 0; }
    now = y_motor_bench_now();
    level = (uint8)((duty > 0) ? Y_MOTOR_FORWARD_DIR_LEVEL :
        ((Y_MOTOR_FORWARD_DIR_LEVEL == GPIO_HIGH) ? GPIO_LOW : GPIO_HIGH));
    if (level != y_motor_dir_level) {
        if (y_motor_bench_applied != 0) { Y_Motor_Stop(); return 0; }
        if (now - y_motor_bench_zero_at <
            (uint64)y_motor_bench_ticks_per_ms * C_BENCH_DIRECTION_ZERO_MS) {
            pwm_set_duty(Y_MOTOR_PWM_PIN, 0); return 0;
        }
        gpio_set_level(Y_MOTOR_DIR_PIN, level);
        y_motor_dir_level = level;
    }
    pwm_set_duty(Y_MOTOR_PWM_PIN, (uint32)((duty > 0) ? duty : -duty));
    y_motor_bench_applied = duty;
    return duty;
#else
    (void)duty;
    Y_Motor_Stop();
    return 0;
#endif
}

uint32 Y_Motor_GetEncoderSampleId20ms(void)
{
    uint32 irq = interrupt_global_disable();
    uint32 id = y_motor_sample_id_20ms;
    interrupt_global_enable(irq);
    return id;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取并清零硬件编码器，更新 5ms 计数、20ms 速度和累计里程
// 参数说明     void
// 返回参数     void
// 使用示例     Y_Motor_EncoderUpdate5ms();
//-------------------------------------------------------------------------------------------------------------------
void Y_Motor_EncoderUpdate5ms(void)
{
    /* Widen before the direction product: -INT16_MIN must not wrap negative. */
    int32 raw_count = (int32)(int16)encoder_get_count(Y_MOTOR_ENCODER_INDEX);
    int32 count = raw_count * ((ENC_DIR_C < 0) ? -1 : 1);

    encoder_clear_count(Y_MOTOR_ENCODER_INDEX);
    y_motor_count_5ms = (int16)func_limit_ab(count, -32768, 32767);
    /* Long bench runs cannot invoke signed-overflow undefined behaviour. */
    if (count > 0 && y_motor_total_count > INT32_MAX - count)
        y_motor_total_count = INT32_MAX;
    else if (count < 0 && y_motor_total_count < INT32_MIN - count)
        y_motor_total_count = INT32_MIN;
    else y_motor_total_count += count;
    y_motor_window_count += (int32)count;
    y_motor_window_samples++;

    if (y_motor_window_samples >= Y_MOTOR_SPEED_WINDOW_SAMPLES)
    {
        y_motor_count_20ms =
            (int16)func_limit_ab(y_motor_window_count, -32768, 32767);
        y_motor_sample_id_20ms++; /* unsigned sequence; not an encoder-wire health flag */
        y_motor_window_count = 0;
        y_motor_window_samples = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除行进轮硬件计数、周期速度与累计里程
// 参数说明     void
// 返回参数     void
// 使用示例     Y_Motor_EncoderClear();
//-------------------------------------------------------------------------------------------------------------------
void Y_Motor_EncoderClear(void)
{
    uint32 interrupt_state = interrupt_global_disable();

    encoder_clear_count(Y_MOTOR_ENCODER_INDEX);
    y_motor_sample_id_20ms = 0;
    y_motor_count_5ms = 0;
    y_motor_count_20ms = 0;
    y_motor_total_count = 0;
    y_motor_window_count = 0;
    y_motor_window_samples = 0;
    interrupt_global_enable(interrupt_state);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取行进轮最近完整 20ms 窗口的编码器增量
// 参数说明     void
// 返回参数     int16           行进轮速度(counts/20ms)
// 使用示例     int16 speed = Y_Motor_GetSpeed20ms();
//-------------------------------------------------------------------------------------------------------------------
int16 Y_Motor_GetSpeed20ms(void)
{
    uint32 interrupt_state = interrupt_global_disable();
    int16 speed = y_motor_count_20ms;

    interrupt_global_enable(interrupt_state);
    return speed;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取行进轮自上次清零以来的累计编码器计数
// 参数说明     void
// 返回参数     int32           行进轮累计计数
// 使用示例     int32 total = Y_Motor_GetTotalCount();
//-------------------------------------------------------------------------------------------------------------------
int32 Y_Motor_GetTotalCount(void)
{
    uint32 interrupt_state = interrupt_global_disable();
    int32 total = y_motor_total_count;

    interrupt_global_enable(interrupt_state);
    return total;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 C 轮 20ms 编码器增量换算为实际线速度
// 参数说明     count_20ms      C 轮编码器增量(counts/20ms)
// 返回参数     float           实际线速度(m/s)
// 使用示例     float speed = Y_Motor_Count20msToMps(Y_Motor_GetSpeed20ms());
//-------------------------------------------------------------------------------------------------------------------
float Y_Motor_Count20msToMps(float count_20ms)
{
    if (ODOM_COUNTS_PER_M <= 0.0f) return 0.0f;
    return count_20ms * 1000.0f / (ODOM_COUNTS_PER_M * (float)CTRL_DIV_SPEED);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将目标线速度换算为 Pitch 速度环使用的 20ms 编码器目标
// 参数说明     speed_mps       目标线速度(m/s)
// 返回参数     float           编码器速度目标(counts/20ms)
// 使用示例     float target = Y_Motor_MpsToCount20ms(0.20f);
//-------------------------------------------------------------------------------------------------------------------
float Y_Motor_MpsToCount20ms(float speed_mps)
{
    return speed_mps * ODOM_COUNTS_PER_M * (float)CTRL_DIV_SPEED / 1000.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 C 轮累计编码器计数换算为实际行进距离
// 参数说明     count           C 轮累计编码器计数
// 返回参数     float           有符号行进距离(m)
// 使用示例     float distance = Y_Motor_CountToMeter(Y_Motor_GetTotalCount());
//-------------------------------------------------------------------------------------------------------------------
float Y_Motor_CountToMeter(int32 count)
{
    if (ODOM_COUNTS_PER_M <= 0.0f) return 0.0f;
    return (float)count / ODOM_COUNTS_PER_M;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 C 轮当前实际线速度
// 参数说明     void
// 返回参数     float           实际线速度(m/s)
// 使用示例     float speed = Y_Motor_GetSpeedMps();
//-------------------------------------------------------------------------------------------------------------------
float Y_Motor_GetSpeedMps(void)
{
    return Y_Motor_Count20msToMps((float)Y_Motor_GetSpeed20ms());
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 C 轮自上次清零以来的实际行进距离
// 参数说明     void
// 返回参数     float           有符号行进距离(m)
// 使用示例     float distance = Y_Motor_GetDistanceMeter();
//-------------------------------------------------------------------------------------------------------------------
float Y_Motor_GetDistanceMeter(void)
{
    return Y_Motor_CountToMeter(Y_Motor_GetTotalCount());
}
