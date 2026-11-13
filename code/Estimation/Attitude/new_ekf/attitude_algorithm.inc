/* Qcar compatibility adapter. Algorithm and calibration ported from
 * mcar-mailun/MCARpreparation, GPL-3.0-or-later; see THIRD_PARTY.md. */
#include "attitude_algorithm.h"
#include <string.h>
struct ATTITUDE_STATE_t attitude_state;
static float last_yaw;
static uint8 have_yaw;
void attitude_reset_state(void)
{
    memset(&attitude_state,0,sizeof(attitude_state));
    attitude_state.q0=1.0f;
    last_yaw=0; have_yaw=0;
}
void attitude_publish_state(const float q[4],const float g[3],const float a[3])
{
    float w=q[0],x=q[1],y=q[2],z=q[3],delta;
    attitude_state.q0=w; attitude_state.q1=x; attitude_state.q2=y; attitude_state.q3=z;
    attitude_state.gyro.x=g[0]; attitude_state.gyro.y=g[1]; attitude_state.gyro.z=g[2];
    attitude_state.acc.x=a[0]*9.80665f; attitude_state.acc.y=a[1]*9.80665f; attitude_state.acc.z=a[2]*9.80665f;
    attitude_state.rMat[0][0]=1-2*(y*y+z*z); attitude_state.rMat[0][1]=2*(x*y-w*z); attitude_state.rMat[0][2]=2*(x*z+w*y);
    attitude_state.rMat[1][0]=2*(x*y+w*z); attitude_state.rMat[1][1]=1-2*(x*x+z*z); attitude_state.rMat[1][2]=2*(y*z-w*x);
    attitude_state.rMat[2][0]=2*(x*z-w*y); attitude_state.rMat[2][1]=2*(y*z+w*x); attitude_state.rMat[2][2]=1-2*(x*x+y*y);
    attitude_state.roll=imu_roll_deg; attitude_state.pitch=imu_pitch_deg; attitude_state.yaw=imu_yaw_deg;
    if(have_yaw) {
        delta=imu_yaw_deg-last_yaw;
        if(delta>180) delta-=360;
        if(delta< -180) delta+=360;
        attitude_state.True_yaw+=delta;
    } else { attitude_state.True_yaw=imu_yaw_deg; have_yaw=1; }
    last_yaw=imu_yaw_deg;
}
