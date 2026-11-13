#include "improved_ahrs.h"
#include <math.h>
#include <string.h>

static float clamp(float x, float lo, float hi)
{ return x < lo ? lo : (x > hi ? hi : x); }
static float wrap(float x)
{ while (x > 180.0f) x -= 360.0f; while (x < -180.0f) x += 360.0f; return x; }
static void output(improved_ahrs_t *s)
{
    const float *q = s->q;
    float yaw = atan2f(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]*q[2]+q[3]*q[3]))*AHRS_RAD_TO_DEG;
    s->yaw += wrap(yaw-s->prev_yaw);
    s->prev_yaw = yaw;
    s->out.roll = atan2f(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]*q[1]+q[2]*q[2]))*AHRS_RAD_TO_DEG;
    s->out.pitch = asinf(clamp(2*(q[0]*q[2]-q[1]*q[3]), -1, 1))*AHRS_RAD_TO_DEG;
    s->out.yaw = s->yaw;
    s->out.bias_z_dps = s->bias[2]*AHRS_RAD_TO_DEG;
}
void improved_init(improved_ahrs_t *s, const ahrs_sample_t *a)
{
    float roll = atan2f(a->ay, a->az);
    float pitch = atan2f(-a->ax, sqrtf(a->ay*a->ay+a->az*a->az));
    float cr=cosf(roll/2), sr=sinf(roll/2), cp=cosf(pitch/2), sp=sinf(pitch/2);
    memset(s, 0, sizeof(*s));
    s->q[0]=cr*cp; s->q[1]=sr*cp; s->q[2]=cr*sp; s->q[3]=-sr*sp;
    s->prev_acc[0]=a->ax; s->prev_acc[1]=a->ay; s->prev_acc[2]=a->az;
    output(s);
}
void improved_update(improved_ahrs_t *s, const ahrs_sample_t *a, float dt)
{
    float an=sqrtf(a->ax*a->ax+a->ay*a->ay+a->az*a->az);
    float g[3]={a->gx*AHRS_DEG_TO_RAD,a->gy*AHRS_DEG_TO_RAD,a->gz*AHRS_DEG_TO_RAD};
    float q0=s->q[0],q1=s->q[1],q2=s->q[2],q3=s->q[3];
    float v[3]={2*(q1*q3-q0*q2),2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2)};
    float av[3]={a->ax,a->ay,a->az}, e[3]={0,0,0};
    float rate=sqrtf(a->gx*a->gx+a->gy*a->gy+a->gz*a->gz);
    float change=0, w=0, theta[3], dq[4], t2, norm;
    unsigned int i;
    float kp=1.5f;
    for(i=0;i<3;i++) { float d=av[i]-s->prev_acc[i]; change+=d*d; s->prev_acc[i]=av[i]; }
    /* Shared window bias estimator in candidate compare_core owns bias.
     * Never learn gyro bias from individual noisy samples. */
    (void)rate; (void)change;
    s->out.is_static=0;
    /* Magnitude trust: never discard good gravity just because rotation is fast.
     * Translation with near-1g magnitude still requires other motion information. */
    if(an > 0.30f && an < 3.0f) {
        w=clamp(1-fabsf(an-1)/0.25f,0,1);
        for(i=0;i<3;i++) av[i]/=an;
        e[0]=av[1]*v[2]-av[2]*v[1]; e[1]=av[2]*v[0]-av[0]*v[2]; e[2]=av[0]*v[1]-av[1]*v[0];
    }
    s->out.acc_weight=w;
#ifdef C_MOTION_KP
    /* Lower gravity pull during fast motion limits acceleration-induced tilt
     * transients; keep full static/slow convergence without discarding gravity. */
    kp=1.5f-0.9f*clamp((rate-2.0f)/18.0f,0,1);
#endif
    /* No gyro dead zone: preserve real slow rotations. */
    for(i=0;i<3;i++) theta[i]=(g[i]-s->bias[i]+kp*w*e[i])*0.5f*dt;
    t2=theta[0]*theta[0]+theta[1]*theta[1]+theta[2]*theta[2];
    if(t2<0.001f) { dq[0]=1-t2/2; for(i=0;i<3;i++) dq[i+1]=theta[i]*(1-t2/6); }
    else { float t=sqrtf(t2), k=sinf(t)/t; dq[0]=cosf(t); for(i=0;i<3;i++) dq[i+1]=theta[i]*k; }
    s->q[0]=q0*dq[0]-q1*dq[1]-q2*dq[2]-q3*dq[3];
    s->q[1]=q0*dq[1]+q1*dq[0]+q2*dq[3]-q3*dq[2];
    s->q[2]=q0*dq[2]-q1*dq[3]+q2*dq[0]+q3*dq[1];
    s->q[3]=q0*dq[3]+q1*dq[2]-q2*dq[1]+q3*dq[0];
    norm=sqrtf(s->q[0]*s->q[0]+s->q[1]*s->q[1]+s->q[2]*s->q[2]+s->q[3]*s->q[3]);
    if(!(norm>1e-6f) || norm>2) { s->out.fault=1; s->q[0]=q0; s->q[1]=q1; s->q[2]=q2; s->q[3]=q3; return; }
    for(i=0;i<4;i++) s->q[i]/=norm;
    output(s);
}
