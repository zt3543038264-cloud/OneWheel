/* EKF attitude estimator and calibration, ported from MCARpreparation.
 * Original implementation: GPL-3.0-or-later. See THIRD_PARTY.md. */
#include "attitude.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

/* 避免快速浮点优化将 NaN/Inf 比较优化掉；memcpy 避免别名访问。 */
static int finite_float(float value)
{
    uint32_t bits;
    typedef char float_size_check[(sizeof(float)==sizeof(uint32_t)) ? 1 : -1];
    (void)sizeof(float_size_check);
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}
static float attitude_clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}
void calibration_reset(ImuCalibration *c)
{
    memset(c, 0, sizeof(*c));
}
/* 当前500个连续合格样本，500Hz下约1秒；不能识别所有匀速运动。 */
int calibration_push(ImuCalibration *c, const float g[3], const float a[3])
{
    unsigned i;
    float n=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (!finite_float(n) || n < 0.9f || n > 1.1f) {
        calibration_reset(c); return 0;
    }
    for (i=0; i<3; ++i) {
        if (!finite_float(g[i]) || fabsf(g[i]) > CAL_GYRO_LIMIT_DPS) {
            calibration_reset(c); return 0;
        }
        if (!c->count) {
            c->min_g[i]=c->max_g[i]=g[i]; c->min_a[i]=c->max_a[i]=a[i];
        }
        if (g[i]<c->min_g[i]) c->min_g[i]=g[i];
        if (g[i]>c->max_g[i]) c->max_g[i]=g[i];
        if (a[i]<c->min_a[i]) c->min_a[i]=a[i];
        if (a[i]>c->max_a[i]) c->max_a[i]=a[i];
        /* 峰峰值超限：丢弃整个静止窗口。 */
        if (c->max_g[i]-c->min_g[i]>CAL_GYRO_SPAN_DPS ||
            c->max_a[i]-c->min_a[i]>CAL_ACCEL_SPAN_G) {
            calibration_reset(c); return 0;
        }
    }
    for (i=0; i<3; ++i) { c->sum_g[i]+=g[i]; c->sum_a[i]+=a[i]; }
    if (++c->count < CALIBRATION_SAMPLES) return 0;
    /* Validate all axes before publishing any new bias estimate. */
    for (i=0; i<3; ++i) {
        if (fabsf(c->sum_g[i]/c->count) > CAL_GYRO_MEAN_MAX_DPS) {
            calibration_reset(c); return 0;
        }
    }
    for (i=0; i<3; ++i) {
        c->bias_dps[i]=c->sum_g[i]/c->count;
        c->initial_accel[i]=c->sum_a[i]/c->count;
    }
    return 1;
}
int new_attitude_init(float q[4], float *accel_norm, const float a[3])
{
    float norm, roll, pitch, cr, sr, cp, sp;
    if (!finite_float(a[0]) || !finite_float(a[1]) || !finite_float(a[2])) return 0;
    norm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (!finite_float(norm) || norm < ACC_NORM_MIN || norm > ACC_NORM_MAX) return 0;
    roll=atan2f(a[1],a[2]);
    pitch=atan2f(-a[0],sqrtf(a[1]*a[1]+a[2]*a[2]));
    cr=cosf(roll*0.5f); sr=sinf(roll*0.5f);
    cp=cosf(pitch*0.5f); sp=sinf(pitch*0.5f);
    q[0]=cr*cp; q[1]=sr*cp; q[2]=cr*sp; q[3]=-sr*sp;
    *accel_norm=norm;
    return 1;
}
/* R=Rz(yaw)*Ry(pitch)*Rx(roll)，pitch 在 +/-90 度处存在万向锁。 */
void attitude_euler(const float q[4], float e[3])
{
    float w=q[0],x=q[1],y=q[2],z=q[3];
    e[0]=atan2f(2.0f*(w*x+y*z),1.0f-2.0f*(x*x+y*y))*RAD_TO_DEG;
    e[1]=asinf(attitude_clamp(2.0f*(w*y-z*x),-1.0f,1.0f))*RAD_TO_DEG;
    e[2]=atan2f(2.0f*(w*z+x*y),1.0f-2.0f*(y*y+z*z))*RAD_TO_DEG;
}
void attitude_cube_euler(const float q[4], float e[3])
{
    float w=q[0],x=q[1],y=q[2],z=q[3];
    float r00=1.0f-2.0f*(y*y+z*z),r02=2.0f*(x*z+w*y);
    float r10=2.0f*(x*y+w*z),r11=1.0f-2.0f*(x*x+z*z);
    float r12=2.0f*(y*z-w*x),r20=2.0f*(x*z-w*y);
    float r22=1.0f-2.0f*(x*x+y*y);
    float cx=sqrtf(r10*r10+r11*r11);
    e[0]=atan2f(-r12,cx)*RAD_TO_DEG;
    if (cx>1e-5f) {
        e[1]=atan2f(r02,r22)*RAD_TO_DEG;
        e[2]=atan2f(r10,r11)*RAD_TO_DEG;
    } else {
        e[1]=atan2f(-r20,r00)*RAD_TO_DEG;
        e[2]=0;
    }
}
/* Quaternion error-state EKF, adapted from the QuaternionEKF design in
 * MCARpreparation (Wang Hongxi design): gravity observation, bias estimate,
 * acceleration LPF and innovation gating. Uses a minimal local error instead
 * of four unconstrained quaternion errors. Units: rad/s, g, seconds.
 * The fifth error state is by; bz is deliberately not inferred from gravity.
 */
static void ekf_skew(const float v[3], float m[3][3])
{
    m[0][0]=0; m[0][1]=-v[2]; m[0][2]=v[1];
    m[1][0]=v[2]; m[1][1]=0; m[1][2]=-v[0];
    m[2][0]=-v[1]; m[2][1]=v[0]; m[2][2]=0;
}
static int ekf_rotate(float q[4], const float angle[3])
{
    float n=sqrtf(angle[0]*angle[0]+angle[1]*angle[1]+angle[2]*angle[2]);
    float k=n>1e-8f?sinf(0.5f*n)/n:0.5f;
    float c=cosf(0.5f*n), x=angle[0]*k,y=angle[1]*k,z=angle[2]*k;
    float w=q[0],qx=q[1],qy=q[2],qz=q[3];
    unsigned i;
    q[0]=w*c-qx*x-qy*y-qz*z;
    q[1]=w*x+qx*c+qy*z-qz*y;
    q[2]=w*y-qx*z+qy*c+qz*x;
    q[3]=w*z+qx*y-qy*x+qz*c;
    n=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (!finite_float(n) || n<1e-6f) return 0;
    for(i=0;i<4;++i) q[i]/=n;
    return 1;
}
/* Carry acceleration history into the current body frame using gyro motion.
 * Rotate by dq^-1: a gravity vector expressed in the rotating sensor frame. */
static int ekf_transport_accel(float v[3], const float angle[3])
{
    float dq[4]={1,0,0,0},x,y,z,w,out[3];
    if(!ekf_rotate(dq,angle)) return 0;
    w=dq[0]; x=dq[1]; y=dq[2]; z=dq[3];
    out[0]=(1-2*(y*y+z*z))*v[0]+2*(x*y+w*z)*v[1]+2*(x*z-w*y)*v[2];
    out[1]=2*(x*y-w*z)*v[0]+(1-2*(x*x+z*z))*v[1]+2*(y*z+w*x)*v[2];
    out[2]=2*(x*z+w*y)*v[0]+2*(y*z-w*x)*v[1]+(1-2*(x*x+y*y))*v[2];
    memcpy(v,out,sizeof(out)); return 1;
}
static float ekf_yaw_rad(const float q[4])
{
    return atan2f(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
}
/* Gravity does not observe heading. Preserve the gyro-predicted Euler yaw
 * exactly after the finite tilt injection (projected gain is first order). */
static void ekf_preserve_yaw(float q[4], float predicted_yaw)
{
    float d=predicted_yaw-ekf_yaw_rad(q),c=cosf(.5f*d),s=sinf(.5f*d);
    float w=q[0],x=q[1],y=q[2],z=q[3];
    q[0]=c*w-s*z; q[1]=c*x-s*y; q[2]=c*y+s*x; q[3]=c*z+s*w;
}
/* P <- F P F^T; works in-place with fixed workspace. */
static void ekf_cov_transform(float p[5][5], const float f[5][5])
{
    float t[5][5]={{0}},out[5][5]={{0}};
    unsigned i,j,k;
    for(i=0;i<5;++i) for(j=0;j<5;++j) for(k=0;k<5;++k)
        t[i][j]+=f[i][k]*p[k][j];
    for(i=0;i<5;++i) for(j=0;j<5;++j) for(k=0;k<5;++k)
        out[i][j]+=t[i][k]*f[j][k];
    memcpy(p,out,sizeof(out));
}
static int ekf_inverse3(const float s[3][3],float inv[3][3])
{
    float aug[3][6]; unsigned i,j,k;
    for(i=0;i<3;++i) for(j=0;j<6;++j)
        aug[i][j]=j<3?s[i][j]:(j-3==i?1.0f:0.0f);
    for(i=0;i<3;++i) {
        unsigned pivot=i;
        for(j=i+1;j<3;++j) if(fabsf(aug[j][i])>fabsf(aug[pivot][i])) pivot=j;
        if(!finite_float(aug[pivot][i]) || fabsf(aug[pivot][i])<1e-12f) return 0;
        for(k=0;k<6;++k) { float v=aug[i][k]; aug[i][k]=aug[pivot][k]; aug[pivot][k]=v; }
        { float d=aug[i][i]; for(k=0;k<6;++k) aug[i][k]/=d; }
        for(j=0;j<3;++j) if(j!=i) {
            float d=aug[j][i]; for(k=0;k<6;++k) aug[j][k]-=d*aug[i][k];
        }
    }
    for(i=0;i<3;++i) for(j=0;j<3;++j) inv[i][j]=aug[i][j+3];
    return 1;
}
int ekf6_init(Ekf6 *s,const float a[3])
{
    Ekf6 next; unsigned i;
    memset(&next,0,sizeof(next));
    next.innovation_nis=-1.0f;
    next.accept_ratio=1.0f; next.observation_r_scale=1.0f;
    if(!new_attitude_init(next.q,&next.accel_norm,a)) return 0;
    for(i=0;i<3;++i) { next.accel_lpf[i]=a[i]; next.covariance[i][i]=0.01f; }
    for(i=3;i<5;++i) next.covariance[i][i]=0.0001f;
    *s=next; return 1;
}
int ekf6_update(Ekf6 *s,const float g[3],const float a[3],float dt)
{
    Ekf6 next=*s;
    float f[5][5]={{0}},skew[3][3],angle[3],omega[3],h[3],res[3];
    float hmat[3][5]={{0}},pht[5][3]={{0}},smat[3][3]={{0}},inv[3][3];
    float gain[5][3]={{0}},dx[5]={0},norm,alpha,predicted_yaw,yaw_jac[3];
    float residual_sq=0.0f,gyro_step_sq=0.0f,gyro_norm_sq=0.0f,input_gyro_norm_sq=0.0f,r_variance;
    unsigned i,j,k;
    if(!finite_float(dt) || dt<=0 || dt>MAX_SAMPLE_DT) return 0;
    for(i=0;i<3;++i) if(!finite_float(g[i]) || !finite_float(a[i])) return 0;
    norm=0;
    for(i=0;i<4;++i) { if(!finite_float(s->q[i])) return 0; norm+=s->q[i]*s->q[i]; }
    if(!finite_float(norm) || norm<1e-12f) return 0;
    for(i=0;i<2;++i) if(!finite_float(s->bias_xy[i])) return 0;
    for(i=0;i<5;++i) for(j=0;j<5;++j) if(!finite_float(s->covariance[i][j])) return 0;
    norm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if(!finite_float(norm)) return 0;
    next.accel_norm=norm; next.accel_used=0; next.innovation_nis=-1.0f;
    next.reject_reason=EKF_REJECT_NONE; next.bias_learning=0; next.bias_slew_dps2=0;
    next.observation_r_scale=1.0f;
    /* One causal preprocessing stage. A lone extreme excursion is held for
     * one frame; a persistent large motion is admitted on the second frame. */
    if(!next.input_ready) {
        for(i=0;i<3;++i) { next.gyro_lpf[i]=next.gyro_reference[i]=g[i]; next.previous_accel[i]=a[i]; }
        next.input_ready=1;
    }
    alpha=dt/(IMU_GYRO_INPUT_TAU_S+dt);
    for(i=0;i<3;++i) {
        float candidate=g[i]; unsigned bit=1u<<i;
        if(fabsf(candidate-next.gyro_reference[i])>IMU_GYRO_SPIKE_DPS*DEG_TO_RAD && !(next.gyro_hold_mask&bit)) {
            candidate=next.gyro_lpf[i];next.gyro_hold_mask|=bit;
        } else { next.gyro_hold_mask&=~bit;next.gyro_reference[i]=candidate; }
        next.gyro_lpf[i]+=alpha*(candidate-next.gyro_lpf[i]);
        input_gyro_norm_sq+=next.gyro_lpf[i]*next.gyro_lpf[i];
        omega[i]=next.gyro_lpf[i]-(i<2?next.bias_xy[i]:0); angle[i]=omega[i]*dt;
    }
    if(!ekf_rotate(next.q,angle) || !ekf_transport_accel(next.accel_lpf,angle)) return 0;
    alpha=dt/(EKF_VIBRATION_TAU_S+dt);
    for(i=0;i<3;++i) {
        float d=a[i]-next.accel_lpf[i],dg=g[i]-next.previous_gyro[i];
        residual_sq+=d*d; gyro_step_sq+=dg*dg; gyro_norm_sq+=omega[i]*omega[i];
        next.previous_gyro[i]=g[i];
    }
    next.accel_vibration_sq+=alpha*(residual_sq-next.accel_vibration_sq);
    {
        float dot=0,n2=0;
        for(i=0;i<3;++i) { dot+=(a[i]-next.accel_lpf[i])*next.accel_lpf[i];n2+=next.accel_lpf[i]*next.accel_lpf[i]; }
        /* Longitudinal vibration changes magnitude, not gravity direction.
         * Do not throw away reliable tilt direction solely for vertical shaking. */
        float transverse=fmaxf(0.0f,residual_sq-dot*dot/fmaxf(n2,1e-6f));
        next.gravity_vibration_sq+=alpha*(transverse-next.gravity_vibration_sq);
    }
    if(next.have_gyro) next.gyro_step_sq+=alpha*(gyro_step_sq-next.gyro_step_sq);
    next.have_gyro=1;
    /* Keep ALL finite non-clipped acceleration samples for vector smoothing.
     * Evaluate gravity usefulness AFTER smoothing, not on individual vibration peaks. */
    alpha=dt/(IMU_ACCEL_INPUT_TAU_S+dt);
    for(i=0;i<3;++i) {
        float candidate=a[i];unsigned bit=1u<<i;
        if(fabsf(candidate-next.previous_accel[i])>IMU_ACCEL_SPIKE_G && !(next.accel_hold_mask&bit)) {
            candidate=next.previous_accel[i];next.accel_hold_mask|=bit;
        } else { next.accel_hold_mask&=~bit;next.previous_accel[i]=candidate; }
        next.accel_lpf[i]+=alpha*(candidate-next.accel_lpf[i]);
    }
    norm=sqrtf(next.accel_lpf[0]*next.accel_lpf[0]+next.accel_lpf[1]*next.accel_lpf[1]+next.accel_lpf[2]*next.accel_lpf[2]);
    next.input_accel_norm=norm;
    next.reacquiring=0;
    if(fabsf(norm-1.0f)<=EKF_REACQUIRE_NORM_G &&
       next.accel_vibration_sq<EKF_REACQUIRE_VIB_G*EKF_REACQUIRE_VIB_G &&
       input_gyro_norm_sq<EKF_REACQUIRE_GYRO_RADPS*EKF_REACQUIRE_GYRO_RADPS)
        next.gravity_stable_s=fminf(2.0f,next.gravity_stable_s+dt);
    else next.gravity_stable_s=0;
    predicted_yaw=ekf_yaw_rad(next.q);
    ekf_skew(omega,skew);
    for(i=0;i<5;++i) f[i][i]=1;
    for(i=0;i<3;++i) for(j=0;j<3;++j) f[i][j]-=skew[i][j]*dt;
    f[0][3]=-dt; f[1][4]=-dt;
    ekf_cov_transform(next.covariance,f);
    for(i=0;i<5;++i) next.covariance[i][i]+=(i<3?EKF_GYRO_NOISE:EKF_BIAS_NOISE)*dt;
    if(norm<ACC_NORM_MIN || norm>ACC_NORM_MAX) { next.reject_reason=EKF_REJECT_NORM; goto commit; }
    if(norm<1e-6f || !finite_float(norm)) { next.reject_reason=EKF_REJECT_GEOMETRY; goto commit; }
    {
        float w=next.q[0],x=next.q[1],y=next.q[2],z=next.q[3];
        float sinp=2*(w*y-z*x),cp2=fmaxf(0.0f,1-sinp*sinp);
        h[0]=2*(x*z-w*y); h[1]=2*(w*x+y*z); h[2]=w*w-x*x-y*y+z*z;
        /* Euler heading becomes singular near vertical: gyro prediction only. */
        if(cp2<0.01f) { next.reject_reason=EKF_REJECT_GEOMETRY; goto commit; }
        yaw_jac[0]=0; yaw_jac[1]=h[1]/cp2; yaw_jac[2]=h[2]/cp2;
    }
    ekf_skew(h,skew);
    for(i=0;i<3;++i) {
        res[i]=next.accel_lpf[i]/norm-h[i];
        for(j=0;j<3;++j) hmat[i][j]=skew[i][j];
    }
    for(i=0;i<5;++i) for(j=0;j<3;++j) for(k=0;k<5;++k)
        pht[i][j]+=next.covariance[i][k]*hmat[j][k];
    for(i=0;i<3;++i) for(j=0;j<3;++j) {
        for(k=0;k<5;++k) smat[i][j]+=hmat[i][k]*pht[k][j];
        if(i==j) smat[i][j]+=EKF_ACCEL_VARIANCE;
    }
    if(!ekf_inverse3(smat,inv)) return 0;
    next.innovation_nis=0.0f;
    for(i=0;i<3;++i) for(j=0;j<3;++j) next.innovation_nis+=res[i]*inv[i][j]*res[j];
    if(!finite_float(next.innovation_nis)) return 0;
    /* Gate using BASE R, so increasing R cannot hide a bad observation. */
    if(next.innovation_nis>EKF_CHI_SQUARE_LIMIT) {
        if(next.gravity_stable_s<EKF_REACQUIRE_STABLE_S) { next.reject_reason=EKF_REJECT_NIS; goto commit; }
        next.reacquiring=1; /* Stable gravity can gradually recover an erroneous prediction. */
    }
    {
        float n=fabsf(next.input_accel_norm-1.0f)/EKF_ACCEL_SOFT_ERROR_G;
        float v=next.gravity_vibration_sq/(EKF_ACCEL_SOFT_VIB_G*EKF_ACCEL_SOFT_VIB_G);
        float nis=fmaxf(0.0f,next.innovation_nis/EKF_NIS_SOFT_LIMIT-1.0f);
        next.observation_r_scale=attitude_clamp(1.0f+n*n+v+nis,1.0f,EKF_R_SCALE_MAX);
    }
    r_variance=EKF_ACCEL_VARIANCE*next.observation_r_scale;
    for(i=0;i<3;++i) smat[i][i]+=r_variance-EKF_ACCEL_VARIANCE;
    if(!ekf_inverse3(smat,inv)) return 0;
    for(i=0;i<5;++i) for(j=0;j<3;++j) for(k=0;k<3;++k) gain[i][j]+=pht[i][k]*inv[k][j];
    /* Project the angle gain onto zero Euler-yaw correction. Do this BEFORE
     * Joseph covariance update; do not pretend a gravity observation reduced
     * uncertainty along the unobserved heading direction. yaw_jac dot h = 1. */
    for(j=0;j<3;++j) {
        float yaw_gain=0;
        for(i=0;i<3;++i) yaw_gain+=yaw_jac[i]*gain[i][j];
        for(i=0;i<3;++i) gain[i][j]-=h[i]*yaw_gain;
    }
    {
        float correction[3]={0},length=0,scale;
        for(i=0;i<3;++i) { for(j=0;j<3;++j) correction[i]+=gain[i][j]*res[j];length+=correction[i]*correction[i]; }
        length=sqrtf(length);scale=length>EKF_CORRECTION_MAX_RADPS*dt?EKF_CORRECTION_MAX_RADPS*dt/length:1.0f;
        for(i=0;i<3;++i) for(j=0;j<3;++j) gain[i][j]*=scale;
    }
    next.bias_learning=(!next.reacquiring && fabsf(next.input_accel_norm-1.0f)<=EKF_ACCEL_SOFT_ERROR_G &&
        next.accel_vibration_sq<=EKF_ACCEL_SOFT_VIB_G*EKF_ACCEL_SOFT_VIB_G &&
        next.gyro_step_sq<=EKF_GYRO_STEP_MAX_RADPS*EKF_GYRO_STEP_MAX_RADPS &&
        gyro_norm_sq<=EKF_BIAS_LEARN_MAX_RADPS*EKF_BIAS_LEARN_MAX_RADPS &&
        next.innovation_nis<=EKF_NIS_SOFT_LIMIT);
    /* Constrain gain BEFORE state AND Joseph covariance update. Frozen bias
     * rows are zero (Schmidt-style); do not shrink bias variance as if learned. */
    for(i=3;i<5;++i) {
        float requested=0,limited,scale;
        for(j=0;j<3;++j) requested+=gain[i][j]*res[j];
        limited=next.bias_learning?attitude_clamp(requested,-EKF_BIAS_SLEW_RADPS2*dt,EKF_BIAS_SLEW_RADPS2*dt):0;
        limited=attitude_clamp(next.bias_xy[i-3]+limited,-EKF_BIAS_LIMIT_RADPS,EKF_BIAS_LIMIT_RADPS)-next.bias_xy[i-3];
        scale=requested!=0.0f?limited/requested:(next.bias_learning?1.0f:0.0f);
        for(j=0;j<3;++j) gain[i][j]*=scale;
        next.bias_slew_dps2=fmaxf(next.bias_slew_dps2,fabsf(limited)*RAD_TO_DEG/dt);
    }
    for(i=0;i<5;++i) for(j=0;j<3;++j) dx[i]+=gain[i][j]*res[j];
    /* Joseph form retains symmetry/positive variance with finite precision. */
    memset(f,0,sizeof(f));
    for(i=0;i<5;++i) for(j=0;j<5;++j) {
        f[i][j]=(i==j?1.0f:0.0f);
        for(k=0;k<3;++k) f[i][j]-=gain[i][k]*hmat[k][j];
    }
    ekf_cov_transform(next.covariance,f);
    for(i=0;i<5;++i) for(j=0;j<5;++j) for(k=0;k<3;++k)
        next.covariance[i][j]+=r_variance*gain[i][k]*gain[j][k];
    if(!ekf_rotate(next.q,dx)) return 0;
    ekf_preserve_yaw(next.q,predicted_yaw);
    for(i=0;i<2;++i) next.bias_xy[i]=attitude_clamp(next.bias_xy[i]+dx[i+3],-EKF_BIAS_LIMIT_RADPS,EKF_BIAS_LIMIT_RADPS);
    /* Reset local error coordinates after quaternion injection. */
    ekf_skew(dx,skew); memset(f,0,sizeof(f));
    for(i=0;i<5;++i) f[i][i]=1;
    for(i=0;i<3;++i) for(j=0;j<3;++j) f[i][j]-=0.5f*skew[i][j];
    ekf_cov_transform(next.covariance,f);
    next.accel_used=1;
commit:
    next.reject_age_s=next.accel_used?0.0f:fminf(60.0f,next.reject_age_s+dt);
    if(!next.accel_used && next.reject_count<0xffffffffu) ++next.reject_count;
    next.accept_ratio+=(dt/(EKF_QUALITY_TAU_S+dt))*((next.accel_used?1.0f:0.0f)-next.accept_ratio);
    next.poor_accept_s=(next.accept_ratio<IMU_POOR_ACCEPT_RATIO ||
        next.observation_r_scale>=IMU_POOR_R_SCALE ||
        next.gyro_step_sq>EKF_GYRO_STEP_MAX_RADPS*EKF_GYRO_STEP_MAX_RADPS)?
        fminf(60.0f,next.poor_accept_s+dt):fmaxf(0.0f,next.poor_accept_s-2.0f*dt);
    next.quality=(next.reject_age_s>=IMU_PREDICT_LIMIT_S || next.poor_accept_s>=IMU_POOR_ACCEPT_LIMIT_S)?
        EKF_QUALITY_UNTRUSTED:(!next.accel_used?EKF_QUALITY_PREDICT:
        (next.observation_r_scale>2.0f || next.accept_ratio<0.7f?EKF_QUALITY_WEAK:EKF_QUALITY_GOOD));
    for(i=0;i<5;++i) for(j=i;j<5;++j) {
        float v=0.5f*(next.covariance[i][j]+next.covariance[j][i]);
        if(!finite_float(v) || (i==j && v<0)) return 0;
        next.covariance[i][j]=next.covariance[j][i]=v;
    }
    *s=next; return 1;
}
void ekf6_euler(const Ekf6 *s,float e[3]) { attitude_euler(s->q,e); }
void ekf6_cube_euler(const Ekf6 *s,float e[3]) { attitude_cube_euler(s->q,e); }
