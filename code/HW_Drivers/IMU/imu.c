#include "HW_Drivers/IMU/imu.h"
#include "Common/board_config.h"
#include "Common/param.h"
#include <math.h>
#if defined(__TASKING__)
#pragma section all "cpu0_dsram"
#endif

imu_data_t imu;                         // 车体系加速度(g)与角速度(°/s)

static int s_forward_dir = IMU_FORWARD_DIR_BASE * IMU_FORWARD_DIR_DEFAULT; // 本次启动实际使用的坐标

static float gyro_bias_x, gyro_bias_y, gyro_bias_z;     // 静止标定得到的陀螺零偏

static float yaw_angle;                 // 转向外环用的连续航向角，永不清零
static float element_angle;             // 元素状态机使用的独立连续转角

static float static_acc_x = 0.0f;       // 标定期间的重力方向，供姿态冷启动对齐
static float static_acc_y = 0.0f;
static float static_acc_z = 1.0f;
static volatile imu_calib_state_t calib_state = IMU_CALIB_NONE;  // 静止标定结果

static volatile uint8 link_lost;        // 链路失效标志，由显式重初始化清除

#define IMU_SETTLE_MS           (150)   // 改完 ODR 后等数字滤波器稳定的时间(ms)

#define IMU_CALIB_SAMPLES       (6000)   // 单次静止标定采样点数
#define IMU_CALIB_RETRY         (3)     // 检测到晃动后的重试次数
#define IMU_CALIB_MOVE_DPS      (3.0f)  // 标定期间陀螺峰峰值超过它判定为动过
#define IMU_CALIB_MOVE_G        (0.15f) // 标定期间加计峰峰值超过它判定为动过

/* Recovery-only workspace, CPU0-owned. Use globally addressed spare DSPR1
 * data RAM; CPU0's fixed-stack layout has insufficient contiguous slack.
 * No control/estimator state or stack reserve is moved. */
#if defined(__TASKING__)
#pragma section farbss "cpu1_dsram"
#endif
static struct {
    float gs[3], as[3], gmin[3], gmax[3], amin[3], amax[3];
    uint16 samples;
    uint8 attempt;
    volatile uint8 busy;
} s_cal;
#if defined(__TASKING__)
#pragma section farbss restore
#endif

static const char *s_imu_fault = "IMU LINK LOST";
static void imu_latch_fault(const char *reason)
{
    if (!link_lost) s_imu_fault = reason;
    link_lost = 1;
}
const char *imu_fault_text(void) { return s_imu_fault; }

#include "imu_spi.inc"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把传感器轴换到车体轴，加速度与陀螺必须用同一套交换和同一套符号
// 参数说明     sx/sy/sz 传感器轴输入；bx/by/bz 车体轴输出
// 返回参数     void
// 使用示例     imu_sensor_to_body(sx, sy, sz, &bx, &by, &bz);
//-------------------------------------------------------------------------------------------------------------------
static void imu_sensor_to_body(float sx, float sy, float sz,
                               float *bx, float *by, float *bz)
{
    *bx = (float)s_forward_dir * sy;       // 车体 X：左右倾，对应 Roll
    *by = (float)s_forward_dir * sz;       // 车体 Y：前后倾，对应 Pitch
    *bz = sx;       // 车体 Z：航向，对应 Yaw
}

#if (IMU660RB_USE_SOFT_IIC == 0)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写入 IMU660RB 寄存器
// 参数说明     reg 寄存器地址；data 写入数据
// 返回参数     void
// 使用示例     imu_write_reg(IMU660RB_CTRL1_XL, value);
//-------------------------------------------------------------------------------------------------------------------
static void imu_write_reg(uint8 reg, uint8 data)
{
    IMU660RB_CS(0);
    spi_write_8bit_register(IMU660RB_SPI, (uint8)(reg | IMU660RB_SPI_W), data);
    IMU660RB_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 IMU660RB 寄存器
// 参数说明     reg 寄存器地址
// 返回参数     uint8           寄存器数据
// 使用示例     uint8 value = imu_read_reg(IMU660RB_CTRL1_XL);
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu_read_reg(uint8 reg)
{
    uint8 data=0xFFu;
    if (!r10h_bus_read(reg,&data,1u)) imu_latch_fault("IMU SPI READ FAIL");
    return data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     配置 IMU660RB 采样率和数字滤波器
// 参数说明     void
// 返回参数     uint8           0 表示配置成功，1 表示回读失败
// 使用示例     uint8 state = imu_tune_sensor();
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu_tune_sensor(void)
{
    /* Keep the full vendor initialization (BDU, auto-increment, filters).
     * Match the accepted reference: both ODRs 1667 Hz, +/-8g, +/-2000 dps. */
    imu_write_reg(IMU660RB_CTRL1_XL, 0x8Cu);
    imu_write_reg(IMU660RB_CTRL2_G, 0x8Cu);
    imu660rb_transition_factor[0] = 4098.0f;
    imu660rb_transition_factor[1] = 14.3f;
    system_delay_ms(5);
    return (uint8)(imu_read_reg(IMU660RB_WHO_AM_I) != 0x6Bu ||
        imu_read_reg(IMU660RB_CTRL1_XL) != 0x8Cu ||
        imu_read_reg(IMU660RB_CTRL2_G) != 0x8Cu ||
        (imu_read_reg(IMU660RB_CTRL3_C) & 0x44u) != 0x44u);
}

#else
#error "IMU660RB 必须用硬件 SPI: 软件 IIC 下库不暴露 IIC 句柄, ODR 只能停在 52/208Hz, 1kHz 控制环会一直读到重复样本"
#endif

#include "imu_r10h.inc"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取车体系三轴角速度
// 参数说明     bx 角速度 X 输出；by 角速度 Y 输出；bz 角速度 Z 输出
// 返回参数     void
// 使用示例     imu_read_gyro_body(&gx, &gy, &gz);
//-------------------------------------------------------------------------------------------------------------------
static void imu_read_gyro_body(float *bx, float *by, float *bz)
{
    (void)r10h_read_body(); /* One complete six-axis burst per calibration/control sample. */
    *bx=r10h_raw_body.gyro_x; *by=r10h_raw_body.gyro_y; *bz=r10h_raw_body.gyro_z;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取车体系三轴加速度
// 参数说明     bx 加速度 X 输出；by 加速度 Y 输出；bz 加速度 Z 输出
// 返回参数     void
// 使用示例     imu_read_acc_body(&ax, &ay, &az);
//-------------------------------------------------------------------------------------------------------------------
static void imu_read_acc_body(float *bx, float *by, float *bz)
{
    /* Same burst as gyro, no second bus transaction / mixed sample. */
    *bx=r10h_raw_body.acc_x; *by=r10h_raw_body.acc_y; *bz=r10h_raw_body.acc_z;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 IMU660RB 并等待滤波器稳定
// 参数说明     void
// 返回参数     uint8           0 表示成功，非 0 表示失败
// 使用示例     uint8 state = imu_init();
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_init(void)
{
    uint8 state;

    s_forward_dir = (IMU_FORWARD_DIR < 0) ? -1 : 1;
    link_lost     = 0;
    s_imu_fault = "IMU LINK LOST";
    r10h_sample_valid = r10h_identity_ticks = 0;
    calib_state = IMU_CALIB_NONE;
    s_cal.busy = 0;
    yaw_angle     = 0.0f;
    element_angle = 0.0f;

#ifndef R10H_HOST_TEST
    r10h_bus_timeout_ticks=(uint32)(IfxStm_getFrequency(&MODULE_STM0)*0.0001f);
#endif
    state = imu660rb_init();          
    if (state != 0)
        return state;

    if (imu_tune_sensor() != 0)
        return 1;

    
    system_delay_ms(IMU_SETTLE_MS);
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     静止标定陀螺仪零偏和重力方向
// 参数说明     void
// 返回参数     void
// 使用示例     imu_calibrate();
//-------------------------------------------------------------------------------------------------------------------
static void imu_calibrate_reset_window(void)
{
    uint8 k;
    s_cal.samples = 0;
    for (k = 0; k < 3u; ++k) {
        s_cal.gs[k] = s_cal.as[k] = 0.0f;
        s_cal.gmin[k] = s_cal.amin[k] = 1e30f;
        s_cal.gmax[k] = s_cal.amax[k] = -1e30f;
    }
}

void imu_calibrate_begin(void)
{
    s_cal.busy = 0;
    calib_state = IMU_CALIB_NONE;
    s_cal.attempt = 0;
    imu_calibrate_reset_window();
    s_cal.busy = (uint8)!link_lost;
}

uint8 imu_calibrate_busy(void) { return s_cal.busy; }

void imu_calibrate_tick(void)
{
    float g[3], a[3], norm, inv;
    uint8 k, moved = 0;
    if (!s_cal.busy) return;
    imu_read_gyro_body(&g[0], &g[1], &g[2]);
    imu_read_acc_body(&a[0], &a[1], &a[2]);
    if (link_lost) { s_cal.busy = 0; return; }
    for (k = 0; k < 3u; ++k) {
        s_cal.gs[k] += g[k]; s_cal.as[k] += a[k];
        if (g[k] < s_cal.gmin[k]) s_cal.gmin[k] = g[k];
        if (g[k] > s_cal.gmax[k]) s_cal.gmax[k] = g[k];
        if (a[k] < s_cal.amin[k]) s_cal.amin[k] = a[k];
        if (a[k] > s_cal.amax[k]) s_cal.amax[k] = a[k];
    }
    if (++s_cal.samples < IMU_CALIB_SAMPLES) return;
    inv = 1.0f / (float)IMU_CALIB_SAMPLES;
    for (k = 0; k < 3u; ++k)
        if (s_cal.gmax[k]-s_cal.gmin[k] > IMU_CALIB_MOVE_DPS ||
            s_cal.amax[k]-s_cal.amin[k] > IMU_CALIB_MOVE_G) moved = 1;
    norm = sqrtf(s_cal.as[0]*s_cal.as[0] + s_cal.as[1]*s_cal.as[1] +
                 s_cal.as[2]*s_cal.as[2]) * inv;
    if (norm < 0.75f || norm > 1.25f) moved = 1;
    if (!moved) {
        /* Only commit a complete, stationary window. */
        gyro_bias_x = s_cal.gs[0]*inv; gyro_bias_y = s_cal.gs[1]*inv;
        gyro_bias_z = s_cal.gs[2]*inv;
        static_acc_x = s_cal.as[0]*inv; static_acc_y = s_cal.as[1]*inv;
        static_acc_z = s_cal.as[2]*inv;
        calib_state = IMU_CALIB_OK;
        s_cal.busy = 0;
    } else {
        calib_state = IMU_CALIB_MOVED;
        if (++s_cal.attempt >= IMU_CALIB_RETRY) s_cal.busy = 0;
        else imu_calibrate_reset_window();
    }
}

void imu_calibrate(void)
{
    imu_calibrate_begin();
    while (imu_calibrate_busy()) {
        imu_calibrate_tick();
        system_delay_ms(1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 IMU 标定状态
// 参数说明     void
// 返回参数     imu_calib_state_t 当前标定状态
// 使用示例     imu_calib_state_t state = imu_calib_state();
//-------------------------------------------------------------------------------------------------------------------
imu_calib_state_t imu_calib_state(void)
{
    return calib_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取标定期间采集的车体系重力方向
// 参数说明     ax 加速度 X 输出；ay 加速度 Y 输出；az 加速度 Z 输出
// 返回参数     uint8           1 表示数据有效，0 表示尚未标定
// 使用示例     uint8 valid = imu_get_static_acc(&ax, &ay, &az);
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_get_static_acc(float *ax, float *ay, float *az)
{
    *ax = static_acc_x;
    *ay = static_acc_y;
    *az = static_acc_z;
    return (uint8)(calib_state == IMU_CALIB_OK && !link_lost);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     每 1 ms 采集完整六轴，并更新车体系角速度
// 参数说明     void
// 返回参数     void
// 使用示例     imu_update_gyro();
//-------------------------------------------------------------------------------------------------------------------
void imu_update_gyro(void)
{
    float bx, by, bz;
    imu_read_gyro_body(&bx, &by, &bz);
    if (link_lost) return;
    imu_read_acc_body(&imu.acc_x, &imu.acc_y, &imu.acc_z);
    /* Controllers retain bias-corrected, unfiltered 1 ms body rates.
     * A+ gets the separate raw sample and removes startup bias internally. */
    imu.gyro_x=bx-gyro_bias_x; imu.gyro_y=by-gyro_bias_y; imu.gyro_z=bz-gyro_bias_z;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查 IMU660RB 通信链路状态
// 参数说明     void
// 返回参数     uint8           1 表示链路失效，0 表示正常
// 使用示例     uint8 lost = imu_link_lost();
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_forward_pending(void)
{
    return (uint8)(s_forward_dir != IMU_FORWARD_DIR);
}

uint8 imu_link_lost(void)
{
    return link_lost;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发布姿态解算得到的航向角和本周期增量
// 参数说明     yaw_continuous 连续航向角；yaw_delta 本周期航向增量
// 返回参数     void
// 使用示例     imu_publish_attitude_yaw(att.yaw, yaw_delta);
//-------------------------------------------------------------------------------------------------------------------
void imu_publish_attitude_yaw(float yaw_continuous, float yaw_delta)
{
    yaw_angle = yaw_continuous;
    element_angle += yaw_delta;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取连续航向角
// 参数说明     void
// 返回参数     float           连续航向角，单位 deg
// 使用示例     float yaw = imu_get_angle_yaw();
//-------------------------------------------------------------------------------------------------------------------
float imu_get_angle_yaw(void)
{
    return yaw_angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取元素处理使用的独立累计转角
// 参数说明     void
// 返回参数     float           元素累计转角，单位 deg
// 使用示例     float angle = imu_get_angle_element();
//-------------------------------------------------------------------------------------------------------------------
float imu_get_angle_element(void)
{
    return element_angle;
}

#if defined(__TASKING__)
#pragma section all restore
#endif
