/* Master API shim around the unchanged new-branch 500 Hz RA/EKF pipeline.
 * Included source units keep existing project build file lists valid. */
#include "HW_Drivers/IMU/imu.h"
#include "Common/board_config.h"
#include "Common/param.h"
#include <math.h>
#include <string.h>
#include "Estimation/Attitude/new_ekf/attitude.inc"
#include "Estimation/Attitude/new_ekf/attitude_algorithm.inc"
#include "Estimation/Attitude/new_ekf/imu.inc"

imu_data_t imu;
static uint8 divider, fresh;
/* Application-only, reboot-latched publication watchdog. The imported
 * sensor/filter sources below are unchanged, including their recovery APIs. */
#define MASTER_IMU_STALE_MS 25u
#define MASTER_IMU_CALIBRATION_MS 10000u
static uint64 published_ticks, stale_ticks;
static uint8 publication_valid;
typedef enum {
    MASTER_IMU_FAULT_NONE=0, MASTER_IMU_FAULT_PIPELINE,
    MASTER_IMU_FAULT_STALE, MASTER_IMU_FAULT_CALIBRATION,
    MASTER_IMU_FAULT_RESTART
} master_imu_fault_t;
static volatile master_imu_fault_t master_fault;
static int32 fault_pipeline_status;
static uint32 published_sequence, published_generation;
static float element_angle, published_yaw;
static imu_calib_state_t startup_state;
static imu_data_t startup_mean;

/* Control/display aliases only: roll<-new pitch/Y; pitch<-new roll/X.
 * These six fields are NOT a new right-handed estimator coordinate system.
 * The estimator itself retains body = sensor Y/-Z/-X unchanged. */
static void master_sample(float ax,float ay,float az,float gx,float gy,float gz,imu_data_t *out)
{
    out->acc_x=ay; out->acc_y=ax; out->acc_z=az;
    out->gyro_x=gy; out->gyro_y=gx; out->gyro_z=gz;
}
/* TIM6/TIM0/TIM6 avoids the shared CAP latch, which another core or an ISR
 * reading TIM0 can overwrite. Use the full counter, not a wrapping 32-bit age. */
static uint64 master_imu_ticks(void)
{
    uint32 upper, lower, again;
    do {
        upper=IfxStm_getOffset32Timer(&MODULE_STM0);
        lower=IfxStm_getLower(&MODULE_STM0);
        again=IfxStm_getOffset32Timer(&MODULE_STM0);
    } while (upper!=again);
    return ((uint64)upper<<32)|lower;
}
static void master_latch_fault(master_imu_fault_t reason)
{
    if (master_fault==MASTER_IMU_FAULT_NONE) {
        fault_pipeline_status=imu_attitude_status;
        master_fault=reason;
    }
    fresh=0;
}
/* CPU0 foreground may also query status. Protect the 64-bit publication time
 * and latch against the CPU0 sampling ISR; never clear the latch here. */
static uint8 master_check_fault(void)
{
    uint32 mask=interrupt_global_disable();
    if (master_fault==MASTER_IMU_FAULT_NONE) {
        if (imu_attitude_status<0) master_latch_fault(MASTER_IMU_FAULT_PIPELINE);
        else if (startup_state==IMU_CALIB_OK) {
            if (!publication_valid || master_imu_ticks()-published_ticks>=stale_ticks)
                master_latch_fault(MASTER_IMU_FAULT_STALE);
            else if (imu_attitude_status!=IMU_ATTITUDE_RUNNING ||
                     g_navigation_sample.generation!=published_generation)
                master_latch_fault(MASTER_IMU_FAULT_RESTART);
        }
    }
    interrupt_global_enable(mask);
    return (uint8)(master_fault!=MASTER_IMU_FAULT_NONE);
}
static void publish_legacy_sample(uint64 sampled_at)
{
    imu_navigation_sample_t sample;
    fresh=0;
    if (master_fault!=MASTER_IMU_FAULT_NONE || startup_state!=IMU_CALIB_OK ||
        !imu_get_navigation_sample(&sample) ||
        (sample.sequence==published_sequence && sample.generation==published_generation)) return;
    if (master_imu_ticks()-sampled_at>=stale_ticks) {
        master_latch_fault(MASTER_IMU_FAULT_STALE); return;
    }
    published_sequence=sample.sequence; published_generation=sample.generation;
    published_ticks=sampled_at; publication_valid=1; fresh=1;
    master_sample(sample.accel_g[0],sample.accel_g[1],sample.accel_g[2],
                  sample.gyro_dps[0],sample.gyro_dps[1],sample.gyro_dps[2],&imu);
}
/* Boot only, with sampling/control timers stopped and motors disabled. No
 * production runtime path calls this function or clears master_fault. */
uint8 imu_init(void)
{
    divider=fresh=publication_valid=0; published_ticks=stale_ticks=0;
    published_sequence=published_generation=0; element_angle=published_yaw=0;
    startup_state=IMU_CALIB_NONE; master_fault=MASTER_IMU_FAULT_NONE;
    fault_pipeline_status=IMU_ATTITUDE_INIT_FAILED;
    memset(&imu,0,sizeof(imu)); memset(&startup_mean,0,sizeof(startup_mean));
    new_imu_init();
    stale_ticks=(uint64)imu_ms_ticks(MASTER_IMU_STALE_MS);
    if (imu_attitude_status<0) master_latch_fault(MASTER_IMU_FAULT_PIPELINE);
    return (uint8)(imu_attitude_status==IMU_ATTITUDE_INIT_FAILED);
}
/* Boot-only, timers stopped: original 500-sample calibration at 2ms.
 * A failed/late calibration is terminal for this boot, even if the vehicle
 * becomes still later. No imported recovery/service call clears this latch. */
void imu_calibrate(void)
{
    uint64 started, sampled_at, calibration_ticks;
    if (master_check_fault()) return;
    startup_state=IMU_CALIB_NONE; fresh=publication_valid=0;
    imu_request_recalibration(); imu_service(); published_sequence=published_generation=0;
    started=master_imu_ticks();
    calibration_ticks=(uint64)imu_ms_ticks(MASTER_IMU_CALIBRATION_MS);
    for (;;) {
        system_delay_ms(2); sampled_at=master_imu_ticks(); imu_update();
        if (master_imu_ticks()-started>=calibration_ticks) {
            startup_state=IMU_CALIB_MOVED;
            master_latch_fault(MASTER_IMU_FAULT_CALIBRATION); return;
        }
        if (master_check_fault()) return;
        if (imu_attitude_status==IMU_ATTITUDE_RUNNING) {
            startup_state=IMU_CALIB_OK;
            master_sample(g_calibration.initial_accel[0],g_calibration.initial_accel[1],g_calibration.initial_accel[2],
                          g_calibration.bias_dps[0],g_calibration.bias_dps[1],g_calibration.bias_dps[2],&startup_mean);
            publish_legacy_sample(sampled_at); divider=0; return;
        }
    }
}
void imu_update_gyro(void)
{
    uint64 sampled_at;
    fresh=0;
    /* Check before polling: a newly returned frame cannot erase an outage. */
    if (master_check_fault()) return;
    if (++divider<2u) return;
    divider=0; sampled_at=master_imu_ticks(); imu_update();
    /* Also catch SPI/solve overruns before accepting a new publication. */
    if (master_check_fault()) return;
    publish_legacy_sample(sampled_at);
}
uint8 imu_get_raw_sample(imu_data_t *out)
{
    if (!out || master_check_fault() || startup_state!=IMU_CALIB_OK ||
        !publication_valid || !fresh || imu_attitude_status!=IMU_ATTITUDE_RUNNING) return 0;
    master_sample(imu_accel_g[0],imu_accel_g[1],imu_accel_g[2],imu_gyro_dps[0],imu_gyro_dps[1],imu_gyro_dps[2],out);
    return 1;
}
float imu_sample_dt_s(void) { return imu_sample_dt_ms*0.001f; }
uint32 imu_get_sample_sequence(void) { return imu_sample_sequence; }
uint8 imu_link_lost(void) { return master_check_fault(); }
const char *imu_fault_text(void)
{
    (void)master_check_fault();
    switch (master_fault) {
    case MASTER_IMU_FAULT_STALE: return "IMU STALE / REBOOT";
    case MASTER_IMU_FAULT_CALIBRATION: return "CALIB TIMEOUT / REBOOT";
    case MASTER_IMU_FAULT_RESTART: return "IMU RESTART / REBOOT";
    case MASTER_IMU_FAULT_PIPELINE:
        switch (fault_pipeline_status) {
        case IMU_ATTITUDE_INIT_FAILED:return "RA INIT FAIL / REBOOT";
        case IMU_ATTITUDE_TIMEOUT:return "RA TIMEOUT / REBOOT";
        case IMU_ATTITUDE_BAD_DT:return "RA BAD DT / REBOOT";
        case IMU_ATTITUDE_FILTER_ERROR:return "EKF ERROR / REBOOT";
        default:return "IMU ERROR / REBOOT";
        }
    default:
        return startup_state==IMU_CALIB_OK ? "RA EKF RUNNING" : "RA CALIBRATING";
    }
}
uint8 imu_forward_pending(void) { return 0; } /* Mount mapping is new-branch compile-time fixed. */
imu_calib_state_t imu_calib_state(void) { return startup_state; }
uint8 imu_get_startup_mean(imu_data_t *out) { if(!out || startup_state!=IMU_CALIB_OK || imu_link_lost())return 0;*out=startup_mean;return 1; }
uint8 imu_get_static_acc(float *x,float *y,float *z) { if(!x||!y||!z)return 0;*x=startup_mean.acc_x;*y=startup_mean.acc_y;*z=startup_mean.acc_z;return (uint8)(startup_state==IMU_CALIB_OK && !imu_link_lost()); }
float imu_get_angle_yaw(void) { return published_yaw; }
float imu_get_angle_element(void) { return element_angle; }
void imu_publish_attitude_yaw(float yaw,float delta) { published_yaw=yaw;element_angle+=delta; }
