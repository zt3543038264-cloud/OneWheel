/* Compatibility view for Qcar control, backed by imu.c / Ekf6. */
#ifndef _ATTITUDE_ALGORITHM_H_
#define _ATTITUDE_ALGORITHM_H_
#include "zf_common_headfile.h"
#include "imu.h"
#define RAD2DEG 57.29577951308232088f
#define DEG2RAD 0.01745329251994329577f
struct ATTITUDE_STATE_t {
    /* EKF output: gyro: bias-corrected body deg/s, acc: body m/s^2; angles: degrees. */
    struct { float x,y,z; } gyro, acc;
    float q0,q1,q2,q3,rMat[3][3];
    float pitch,roll,yaw,True_yaw;
};
extern struct ATTITUDE_STATE_t attitude_state;
void attitude_reset_state(void);
void attitude_publish_state(const float q[4],const float gyro_dps[3],const float accel_g[3]);
#endif
