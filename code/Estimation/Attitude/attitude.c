/* Master presentation/control aliases; the new-branch EKF owns all solving. */
#include "Estimation/Attitude/attitude.h"
#include "HW_Drivers/IMU/imu.h"
#include "Estimation/Attitude/new_ekf/imu.h"
#include "Estimation/Attitude/new_ekf/attitude_algorithm.h"
#include <string.h>
attitude_t att;
att_diag_t att_diag;
static uint32 last_sequence, last_generation;
void attitude_init(void)
{
    memset(&att,0,sizeof(att)); memset(&att_diag,0,sizeof(att_diag));
    last_sequence=last_generation=0;
    imu_publish_attitude_yaw(0,0);
    attitude_update_rate(); attitude_update();
}
uint8 attitude_converged(void) { return (uint8)(imu_calib_state()==IMU_CALIB_OK && imu_attitude_status==IMU_ATTITUDE_RUNNING && !imu_link_lost()); }
uint8 attitude_diverged(void) { return imu_link_lost(); }
void attitude_update_rate(void)
{
    imu_data_t raw;
    att_diag.sample_fresh=imu_get_raw_sample(&raw);
    att_diag.sample_sequence=imu_get_sample_sequence();
    if (imu_link_lost()) { att.roll_rate=att.pitch_rate=att.yaw_rate=0;return; }
    att.roll_rate=imu.gyro_x; att.pitch_rate=imu.gyro_y; att.yaw_rate=imu.gyro_z;
    att_diag.period_us=imu_sample_dt_ms*1000.0f;
    att_diag.update_us=imu_solve_us; att_diag.max_update_us=imu_solve_max_us;
    att_diag.acc_norm=imu_accel_norm_g;
    att_diag.acc_valid=(uint8)(imu_accel_used==1);
    att_diag.bias_z=imu_gyro_bias_z_dps*0.01745329251994329577f;
    att_diag.is_static=(uint8)(imu_bias_learning!=0);
    att_diag.acc_weight=0; /* EKF has NIS/R-scale, not the r10h correction weight. */
}
void attitude_tick1ms(void) { } /* Solving already ran inside every second IMU poll. */
void attitude_update(void)
{
    float yaw;
    imu_navigation_sample_t sample;
    if (imu_link_lost() || !imu_get_navigation_sample(&sample) ||
        (last_sequence==sample.sequence && last_generation==sample.generation)) return;
    if (last_generation!=sample.generation) { att.yaw=0; imu_publish_attitude_yaw(0,0); }
    last_sequence=sample.sequence; last_generation=sample.generation;
    att.roll=imu_pitch_deg; att.pitch=imu_roll_deg;
    yaw=attitude_state.True_yaw;
    imu_publish_attitude_yaw(yaw,yaw-att.yaw);
    att.yaw=yaw; att.yaw_wrapped=imu_yaw_deg;
}
