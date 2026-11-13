/* IMU660RA 2ms pipeline: sample, calibrate, fuse and publish.
 * TC264 STM0 clock, +/-8g and +/-2000dps. */
#include "imu.h"
#include "attitude.h"
#include "config.h"
#include "IfxStm.h"
#include "attitude_algorithm.h"
#include "zf_common_interrupt.h"
#include "zf_device_imu660ra.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"
#include <math.h>
#include <string.h>
#define IMU_STATUS_REG 0x03u /* BMI270 STATUS. */
#define IMU_DATA_READY_MASK 0xC0u /* drdy_acc (bit 7) + drdy_gyr (bit 6). */
static float g_stm_hz;

volatile int32 imu_attitude_status=IMU_ATTITUDE_INIT_FAILED;
volatile float imu_roll_deg, imu_pitch_deg, imu_yaw_deg;
volatile float imu_accel_norm_g, imu_calibration_percent;
volatile float imu_accel_g[3], imu_gyro_dps[3];
volatile float imu_gyro_bias_z_dps;
volatile float imu_sample_dt_ms, imu_solve_us, imu_solve_max_us;
volatile int32 imu_accel_used=-1;
volatile float imu_ekf_nis=-1.0f, imu_ekf_bias_xy_dps[2];
volatile uint32 imu_sample_sequence;
volatile uint32 imu_spi_timeouts,imu_not_ready_samples,imu_last_status_reg,imu_bad_bursts;
volatile int32 imu_quality,imu_reject_reason,imu_bias_learning;
volatile float imu_reject_ms,imu_accept_ratio,imu_accel_vibration_g;
volatile float imu_gyro_step_dps,imu_r_scale,imu_bias_slew_dps2,imu_recovery_percent;
volatile uint32 imu_reject_count,imu_gyro_clips,imu_accel_clips,imu_recovery_count;
volatile float imu_pre_gyro_dps[3],imu_pre_accel_g[3],imu_pre_norm_g;
volatile int32 imu_reacquiring;
volatile float imu_gyro_z_integral_deg;
/* Stable telemetry protocol value: 3 = EKF. */
const uint8 imu_attitude_method=3u;
static Ekf6 g_attitude;
static ImuCalibration g_calibration;
static uint32 g_last_sample_ticks, g_last_success_ticks;
static uint8 g_have_sample, g_calibrated;
static volatile uint8 g_update_enabled, g_recalibration_requested;
static imu_navigation_sample_t g_navigation_sample;
static uint32 g_navigation_generation;
static uint8 g_sample_gyro_clip;
bool imu_get_navigation_sample(imu_navigation_sample_t *out)
{
    if (out==NULL || imu_attitude_status!=IMU_ATTITUDE_RUNNING ||
        g_navigation_sample.sequence==0u) return false;
    *out=g_navigation_sample;
    return true;
}

static void imu_publish_navigation(const float gyro_dps[3],const float accel_g[3],float dt)
{
    unsigned i;
    g_navigation_sample.roll_deg=imu_roll_deg;
    g_navigation_sample.pitch_deg=imu_pitch_deg;
    g_navigation_sample.yaw_deg=imu_yaw_deg;
    g_navigation_sample.dt_s=dt;
    g_navigation_sample.generation=g_navigation_generation;
    for(i=0u;i<3u;++i) {
        g_navigation_sample.accel_g[i]=accel_g[i];
        g_navigation_sample.gyro_dps[i]=(g_attitude.input_ready?g_attitude.gyro_lpf[i]*RAD_TO_DEG:gyro_dps[i]-g_calibration.bias_dps[i]);
        if(i<2u) g_navigation_sample.gyro_dps[i]-=g_attitude.bias_xy[i]*RAD_TO_DEG;
    }
    ++g_navigation_sample.sequence;
    if(g_navigation_sample.sequence==0u) ++g_navigation_sample.sequence;
    imu_sample_sequence=g_navigation_sample.sequence;
    attitude_publish_state(g_attitude.q, g_navigation_sample.gyro_dps, g_attitude.accel_lpf);
}

/* uint32 无符号做差处理单次计数回绕；保留原 DWT 时间基准。 */
static uint32 imu_ticks(void) { return IfxStm_getLower(&MODULE_STM0); }
static float imu_seconds(uint32 elapsed) { return (float)elapsed/g_stm_hz; }
static uint32 imu_ms_ticks(uint32 ms) { return (uint32)((g_stm_hz/1000.0f)*(float)ms); }
/* Publish only completed solves; max lasts until IMU reinitialization. */
static void imu_publish_filter_diagnostics(uint32 started)
{
    imu_solve_us=imu_seconds((uint32)(imu_ticks()-started))*1000000.0f;
    if(imu_solve_us>imu_solve_max_us) imu_solve_max_us=imu_solve_us;
    imu_accel_used=g_attitude.accel_used;
    imu_ekf_nis=g_attitude.innovation_nis;
    imu_ekf_bias_xy_dps[0]=g_attitude.bias_xy[0]*RAD_TO_DEG;
    imu_ekf_bias_xy_dps[1]=g_attitude.bias_xy[1]*RAD_TO_DEG;
    imu_quality=g_attitude.quality;
    imu_reject_reason=g_attitude.reject_reason;
    imu_reject_ms=g_attitude.reject_age_s*1000.0f;
    imu_accept_ratio=g_attitude.accept_ratio;
    imu_reject_count=g_attitude.reject_count;
    imu_accel_vibration_g=sqrtf(g_attitude.accel_vibration_sq);
    imu_gyro_step_dps=sqrtf(g_attitude.gyro_step_sq)*RAD_TO_DEG;
    imu_r_scale=g_attitude.observation_r_scale;
    imu_bias_learning=g_attitude.bias_learning;
    imu_bias_slew_dps2=g_attitude.bias_slew_dps2;
    for(unsigned i=0;i<3;++i) {
        imu_pre_gyro_dps[i]=g_attitude.gyro_lpf[i]*RAD_TO_DEG;
        imu_pre_accel_g[i]=g_attitude.accel_lpf[i];
    }
    imu_pre_norm_g=g_attitude.input_accel_norm;
    imu_reacquiring=g_attitude.reacquiring;
}
static void imu_reg_write(uint8 reg,uint8 value)
{
    uint8 tx[2]={reg|IMU660RA_SPI_W,value};
    IMU660RA_CS(0);
    if(spi_transfer_8bit_timeout(IMU660RA_SPI,tx,NULL,2,1))++imu_spi_timeouts;
    IMU660RA_CS(1);system_delay_ms(1);
}
static uint8 imu_reg_read(uint8 reg)
{
    uint8 tx[3]={reg|IMU660RA_SPI_R,0,0},rx[3];
    uint8 failed;
    IMU660RA_CS(0);
    failed=spi_transfer_8bit_timeout(IMU660RA_SPI,tx,rx,3,1);
    IMU660RA_CS(1);
    if(failed){++imu_spi_timeouts;return 0xff;}
    return rx[2];
}
/* RA (BMI270): STATUS bits 7/6; accel then gyro at 0x0C..0x17.
 * A read clocks address + one dummy byte + data. One burst owns SPI0. */
static uint8 imu_read_sample(float gyro_dps[3],float accel_g[3])
{
    uint8 data[12],i;
    uint8 tx[14]={0},rx[14];
    float sensor_g[3],sensor_a[3];
    uint8 status=imu_reg_read(IMU_STATUS_REG);
    uint8 accel_clip=0;
    g_sample_gyro_clip=0;
    /* 单次读到 0xFF/无 ready 位多为总线毛刺，同一 tick 内重试一次。 */
    if (status==0xFFu || (status&IMU_DATA_READY_MASK)!=IMU_DATA_READY_MASK) status=imu_reg_read(IMU_STATUS_REG);
    imu_last_status_reg=status;
    if (status==0xFFu || (status&IMU_DATA_READY_MASK)!=IMU_DATA_READY_MASK) {++imu_not_ready_samples;return 0u;}
    tx[0]=IMU660RA_ACC_ADDRESS|IMU660RA_SPI_R;
    IMU660RA_CS(0);
    i=spi_transfer_8bit_timeout(IMU660RA_SPI,tx,rx,14,1);
    IMU660RA_CS(1);
    if(i){++imu_spi_timeouts;return 0u;}
    memcpy(data,rx+2,12);
    /* 12 字节完全相同（全 0x00 或全 0xFF）是总线掉线，不能当样本交给 EKF。 */
    for (i=1u;i<12u;++i) if (data[i]!=data[0]) break;
    if (i==12u) {++imu_bad_bursts;return 0u;}
    imu660ra_acc_x=(int16)((uint16)data[0]|((uint16)data[1]<<8));
    imu660ra_acc_y=(int16)((uint16)data[2]|((uint16)data[3]<<8));
    imu660ra_acc_z=(int16)((uint16)data[4]|((uint16)data[5]<<8));
    imu660ra_gyro_x=(int16)((uint16)data[6]|((uint16)data[7]<<8));
    imu660ra_gyro_y=(int16)((uint16)data[8]|((uint16)data[9]<<8));
    imu660ra_gyro_z=(int16)((uint16)data[10]|((uint16)data[11]<<8));
    for(i=0u;i<3u;++i) {
        int16 raw_a=(int16)((uint16)data[2u*i]|((uint16)data[2u*i+1u]<<8));
        int16 raw_g=(int16)((uint16)data[6u+2u*i]|((uint16)data[7u+2u*i]<<8));
        if(raw_g>=IMU_RAW_CLIP_COUNTS || raw_g<=-IMU_RAW_CLIP_COUNTS) g_sample_gyro_clip=1;
        if(raw_a>=IMU_RAW_CLIP_COUNTS || raw_a<=-IMU_RAW_CLIP_COUNTS) accel_clip=1;
        sensor_g[i]=imu660ra_gyro_transition(raw_g);
        sensor_a[i]=imu660ra_acc_transition(raw_a);
    }
    gyro_dps[0]=sensor_g[IMU_BODY_X_AXIS]*IMU_BODY_X_SIGN;
    gyro_dps[1]=sensor_g[IMU_BODY_Y_AXIS]*IMU_BODY_Y_SIGN;
    gyro_dps[2]=sensor_g[IMU_BODY_Z_AXIS]*IMU_BODY_Z_SIGN;
    accel_g[0]=sensor_a[IMU_BODY_X_AXIS]*IMU_BODY_X_SIGN;
    accel_g[1]=sensor_a[IMU_BODY_Y_AXIS]*IMU_BODY_Y_SIGN;
    accel_g[2]=sensor_a[IMU_BODY_Z_AXIS]*IMU_BODY_Z_SIGN;
    if(g_sample_gyro_clip) ++imu_gyro_clips;
    if(accel_clip) ++imu_accel_clips;
    return 1u;
}
static uint8 imu_hardware_init(void)
{
    if (imu660ra_init()!=0u) return 0u;
    /* The vendor RA driver loads the BMI270 configuration image and sets
     * +/-8g, +/-2000dps conversion factors. Override its 50/200Hz defaults
     * with 800Hz normal-bandwidth performance sampling for the 500Hz solver. */
    imu_reg_write(IMU660RA_PWR_CONF,IMU_RA_PWR_CONF_CONFIG);
    imu_reg_write(IMU660RA_ACC_CONF,IMU_RA_ACC_CONF_CONFIG);
    imu_reg_write(IMU660RA_GYR_CONF,IMU_RA_GYR_CONF_CONFIG);
    imu_reg_write(IMU660RA_PWR_CTRL,IMU_RA_PWR_CTRL_CONFIG);
    return (imu_reg_read(IMU660RA_CHIP_ID)==0x24u &&
            imu_reg_read(IMU660RA_PWR_CONF)==IMU_RA_PWR_CONF_CONFIG &&
            imu_reg_read(IMU660RA_PWR_CTRL)==IMU_RA_PWR_CTRL_CONFIG &&
            imu_reg_read(IMU660RA_ACC_CONF)==IMU_RA_ACC_CONF_CONFIG &&
            imu_reg_read(IMU660RA_GYR_CONF)==IMU_RA_GYR_CONF_CONFIG &&
            imu_reg_read(IMU660RA_ACC_RANGE)==0x02u &&
            imu_reg_read(IMU660RA_GYR_RANGE)==0x00u)?1u:0u;
}
static void imu_reset_pipeline(void)
{
    ++g_navigation_generation;
    memset(&g_navigation_sample,0,sizeof(g_navigation_sample));
    memset(&g_attitude,0,sizeof(g_attitude));
    calibration_reset(&g_calibration);
    g_have_sample=0u; g_calibrated=0u;
    imu_roll_deg=0.0f; imu_pitch_deg=0.0f; imu_yaw_deg=0.0f;
    imu_accel_norm_g=0.0f; imu_calibration_percent=0.0f;
    imu_gyro_bias_z_dps=0.0f;
    imu_sample_dt_ms=imu_solve_us=imu_solve_max_us=0.0f;
    imu_accel_used=-1; imu_ekf_nis=-1.0f;
    imu_ekf_bias_xy_dps[0]=imu_ekf_bias_xy_dps[1]=0.0f;
    imu_sample_sequence=0u;
    imu_gyro_z_integral_deg=0.0f;
    imu_recovery_percent=0; imu_recovery_count=0;
    imu_quality=imu_reject_reason=imu_bias_learning=0;
    imu_reject_ms=imu_accel_vibration_g=imu_gyro_step_dps=imu_bias_slew_dps2=0;
    imu_accept_ratio=imu_r_scale=1;
    imu_reject_count=0;
    for(unsigned i=0;i<3;++i) { imu_pre_gyro_dps[i]=0;imu_pre_accel_g[i]=0; }
    imu_pre_norm_g=0;imu_reacquiring=0;
    attitude_reset_state();
    g_last_sample_ticks=imu_ticks();
    g_last_success_ticks=g_last_sample_ticks;
}
void new_imu_init(void)
{
    g_stm_hz=IfxStm_getFrequency(&MODULE_STM0);
    g_update_enabled=0u;
    imu_reset_pipeline();
    if (imu_hardware_init()) {
        imu_attitude_status=IMU_ATTITUDE_CALIBRATING;
        g_update_enabled=1u;
    } else imu_attitude_status=IMU_ATTITUDE_INIT_FAILED;
}
void imu_update(void)
{
    float gyro_dps[3],accel_g[3],gyro_rad_s[3],euler[3],dt;
    uint32 now, solve_started;
    uint8 i;
    /* BAD_DT/FILTER_ERROR 保留原锁止行为；需重新标定恢复。 */
    if (!g_update_enabled || imu_attitude_status==IMU_ATTITUDE_BAD_DT ||
        imu_attitude_status==IMU_ATTITUDE_FILTER_ERROR) return;
    now=imu_ticks();
    if (!imu_read_sample(gyro_dps,accel_g)) {
        if ((uint32)(now-g_last_success_ticks)>imu_ms_ticks(IMU_TIMEOUT_MS))
            imu_attitude_status=IMU_ATTITUDE_TIMEOUT;
        return;
    }
    g_last_success_ticks=now;
    dt=imu_seconds((uint32)(now-g_last_sample_ticks));
    g_last_sample_ticks=now;
    for (i=0u;i<3u;++i) {
        imu_gyro_dps[i]=gyro_dps[i]; imu_accel_g[i]=accel_g[i];
    }
    imu_accel_norm_g=sqrtf(accel_g[0]*accel_g[0]+accel_g[1]*accel_g[1]+accel_g[2]*accel_g[2]);
    /* A clipped gyro cannot bound the integration error; never integrate it.
     * Keep raw telemetry and drop only the clipped integration sample. */
    if(g_sample_gyro_clip) {
        if(!g_calibrated) {
            calibration_reset(&g_calibration); imu_calibration_percent=0;
            imu_attitude_status=IMU_ATTITUDE_CALIBRATING; return;
        }
        imu_quality=EKF_QUALITY_UNTRUSTED;
        imu_reject_reason=EKF_REJECT_CLIP; imu_accel_used=0;
        return; /* Drop this sample; no persistent quality interlock. */
    }
    /* 第一帧只建立采样时间基准。 */
    if (!g_have_sample) { g_have_sample=1u; return; }
    imu_sample_dt_ms=dt*1000.0f;
    if (dt<=0.0f || dt>MAX_SAMPLE_DT) {
        /* 短暂空档只丢这一帧（时间基准上面已重建）；真正卡死才锁止。 */
        if (dt>0.0f && dt<=MAX_SKIP_DT) return;
        imu_attitude_status=IMU_ATTITUDE_BAD_DT; return;
    }
    if (!g_calibrated) {
        if (calibration_push(&g_calibration,gyro_dps,accel_g)) {
            g_calibrated=(uint8)ekf6_init(&g_attitude,g_calibration.initial_accel);
            if (!g_calibrated) { imu_attitude_status=IMU_ATTITUDE_FILTER_ERROR; return; }
            for(i=0;i<3u;++i) {
                g_attitude.gyro_lpf[i]=g_attitude.gyro_reference[i]=(gyro_dps[i]-g_calibration.bias_dps[i])*DEG_TO_RAD;
                g_attitude.previous_accel[i]=g_calibration.initial_accel[i];
            }
            g_attitude.input_ready=1;

            ekf6_euler(&g_attitude,euler);
            imu_roll_deg=euler[0]; imu_pitch_deg=euler[1]; imu_yaw_deg=euler[2];
            imu_gyro_bias_z_dps=g_calibration.bias_dps[2];
            imu_attitude_status=IMU_ATTITUDE_RUNNING;
            imu_publish_navigation(gyro_dps,accel_g,dt);
        }
        imu_calibration_percent=100.0f*(float)g_calibration.count/(float)CALIBRATION_SAMPLES;
        return;
    }
    for (i=0u;i<3u;++i)
        gyro_rad_s[i]=(gyro_dps[i]-g_calibration.bias_dps[i])*DEG_TO_RAD;
    solve_started=imu_ticks();
    if (!ekf6_update(&g_attitude,gyro_rad_s,accel_g,dt)) {
        imu_attitude_status=IMU_ATTITUDE_FILTER_ERROR; return;
    }
    ekf6_euler(&g_attitude,euler);
    imu_publish_filter_diagnostics(solve_started);
    imu_gyro_z_integral_deg += gyro_rad_s[2]*RAD_TO_DEG*dt;
    imu_roll_deg=euler[0]; imu_pitch_deg=euler[1]; imu_yaw_deg=euler[2];
    imu_accel_norm_g=g_attitude.accel_norm;
    imu_attitude_status=IMU_ATTITUDE_RUNNING;
    imu_publish_navigation(gyro_dps,accel_g,dt);
}
void imu_request_recalibration(void) { g_recalibration_requested=1u; }
/* Called with control timers stopped. Never reuse a faulted/partial calibration.
 * Gate motors until the first new fused sample is published. */
void imu_prepare_start(void)
{
    uint32 primask = interrupt_global_disable();
    uint8 reusable = g_calibrated && !g_recalibration_requested &&
                     imu_attitude_status == IMU_ATTITUDE_RUNNING;
    if (reusable) {
        g_last_sample_ticks = imu_ticks();
        g_last_success_ticks = g_last_sample_ticks;
        g_have_sample = 0u;
        imu_attitude_status = IMU_ATTITUDE_CALIBRATING;
    } else g_recalibration_requested = 1u;
    interrupt_global_enable(primask);
    imu_service();
}
void imu_recover_after_stall(void)
{
    uint32 primask;
    if (imu_attitude_status!=IMU_ATTITUDE_BAD_DT &&
        imu_attitude_status!=IMU_ATTITUDE_TIMEOUT) return;
    primask=interrupt_global_disable();
    g_last_sample_ticks=imu_ticks();
    g_last_success_ticks=g_last_sample_ticks;
    g_have_sample=0u;                 /* 下一帧只重建时间基准 */
    imu_attitude_status=g_calibrated?IMU_ATTITUDE_RUNNING:IMU_ATTITUDE_CALIBRATING;
    interrupt_global_enable(primask);
}
/* 主循环服务；只在短临界区更新状态，硬件初始化在开中断时执行。 */
void imu_service(void)
{
    uint32 primask;
    uint8 need_hardware_init;
    if (!g_recalibration_requested) return;
    primask=interrupt_global_disable();
    g_recalibration_requested=0u; g_update_enabled=0u;
    need_hardware_init=(imu_attitude_status==IMU_ATTITUDE_INIT_FAILED)?1u:0u;
    interrupt_global_enable(primask);
    if (need_hardware_init && !imu_hardware_init()) {
        imu_attitude_status=IMU_ATTITUDE_INIT_FAILED; return;
    }
    primask=interrupt_global_disable();
    imu_reset_pipeline();
    imu_attitude_status=IMU_ATTITUDE_CALIBRATING;
    g_update_enabled=1u;
    interrupt_global_enable(primask);
}
