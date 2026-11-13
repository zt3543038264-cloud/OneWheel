#include "compare_core.h"
#include "MahonyAhrs.h"
#include "IMU_Filtter.h"
#include "unicycle_ahrs.h"
#include "improved_ahrs.h"
#include <string.h>
#include <math.h>
compare_result_t compare_result;
static MahonyAhrs_t state_a;
static improved_ahrs_t state_c;
float compare_bias_diag[21];
/* r2 uses 250ms windows. Final uses 1s mean/variance, three-window lock
 * and 100ms heading veto. Variance permits zero-mean motor vibration.
 * Six-axis yaw below the chosen threshold remains unobservable; do not
 * treat these bench thresholds as universal stationary detection. */
static float sums[6],squares[6],window_acc[3],bias[3],previous_acc[3];
static unsigned window_count,stable_windows,has_previous;
static float filtered_rate_sums[3];
static uint8_t stationary;
#ifdef C_MOTION_KP
static float fast_heading,fast_q[4],window_q[4];
static unsigned fast_count;
/* Only the stationary classifier is smoothed. The exported A/C rotation
 * remains on the existing 1 kHz filter/integration path. */
static float quiet_q[4],quiet_before[4],quiet_tilt,quiet_fast_q[4],quiet_fast_heading,quiet_fast_tilt,classifier_acc_variance;
static void quiet_rotation_step(void) {
    float dot=0,n=0;unsigned j;
    for(j=0;j<4;j++)dot+=quiet_q[j]*state_c.q[j];
    for(j=0;j<4;j++) {
        float target=dot<0 ? -state_c.q[j] : state_c.q[j];
        quiet_q[j]+=(classifier_acc_variance>0.03f?0.00025f:0.002f)*(target-quiet_q[j]);n+=quiet_q[j]*quiet_q[j];
    }
    n=1.0f/sqrtf(n);for(j=0;j<4;j++)quiet_q[j]*=n;
}
static float net_tilt(const float *before,const float *after,float dt) {
    float a[3]={2*(before[1]*before[3]-before[0]*before[2]),
                2*(before[0]*before[1]+before[2]*before[3]),
                1-2*(before[1]*before[1]+before[2]*before[2])};
    float b[3]={2*(after[1]*after[3]-after[0]*after[2]),
                2*(after[0]*after[1]+after[2]*after[3]),
                1-2*(after[1]*after[1]+after[2]*after[2])};
    float d=0;for(unsigned j=0;j<3;j++){float x=b[j]-a[j];d+=x*x;}
    return 2*asinf(fminf(1.0f,0.5f*sqrtf(d)))*AHRS_RAD_TO_DEG/dt;
}
static float heading_bias(const float *q) {
    return bias[0]*2*(q[1]*q[3]-q[0]*q[2])+bias[1]*2*(q[0]*q[1]+q[2]*q[3])+bias[2]*(1-2*(q[1]*q[1]+q[2]*q[2]));
}
/* Net quaternion rotation excludes closed vibration/coning loops. Euler
 * derivatives and average raw gyro can create false heading bias here. */
static float net_heading(const float *before,const float *after,float dt) {
    float d[3],g[3],n,k,dot=0;
    d[0]=before[0]*after[1]-before[1]*after[0]-before[2]*after[3]+before[3]*after[2];
    d[1]=before[0]*after[2]+before[1]*after[3]-before[2]*after[0]-before[3]*after[1];
    d[2]=before[0]*after[3]-before[1]*after[2]+before[2]*after[1]-before[3]*after[0];
    n=sqrtf(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
    k=n>0.000001f ? 2*asinf(n>1?1:n)/n : 2;
    if(before[0]*after[0]+before[1]*after[1]+before[2]*after[2]+before[3]*after[3]<0)k=-k;
    g[0]=2*(before[1]*before[3]-before[0]*before[2]);
    g[1]=2*(before[0]*before[1]+before[2]*before[3]);
    g[2]=1-2*(before[1]*before[1]+before[2]*before[2]);
    for(unsigned j=0;j<3;j++)dot+=d[j]*g[j];
    return dot*k*AHRS_RAD_TO_DEG/dt;
}
#define BIAS_WINDOW_COUNT (1000u)
#define BIAS_WINDOW_FLOAT (1000.0f)
#define BIAS_STATIC_WINDOWS (3u)
#define BIAS_LEARN_ALPHA (0.16666667f)
#define BIAS_MEAN_LIMIT_SQ (1.0f)
#define BIAS_GYRO_VAR_LIMIT (14400.0f)
#define BIAS_ACC_VAR_LIMIT (4.0f)
#define BIAS_ACC_STEP_LIMIT (0.000001f)
#else
#define BIAS_WINDOW_COUNT (250u)
#define BIAS_WINDOW_FLOAT (250.0f)
#define BIAS_STATIC_WINDOWS (8u)
#define BIAS_LEARN_ALPHA (0.04761905f)
#define BIAS_MEAN_LIMIT_SQ (0.0064f)
#define BIAS_GYRO_VAR_LIMIT (16.0f)
#define BIAS_ACC_VAR_LIMIT (0.0064f)
#define BIAS_ACC_STEP_LIMIT (0.000009f)
#endif
static void window_bias(const ahrs_sample_t *a) {
    float v[6]={a->gx,a->gy,a->gz,a->ax,a->ay,a->az};
    float filtered_mean[3],m[6],var[6],rate2=0,gv=0,av=0,norm,step=0;unsigned j;
#ifdef C_MOTION_KP
    quiet_rotation_step();
    if(++fast_count>=100u) {
        /* Restore the learned heading component for motion detection.
         * Otherwise an old pose bias can veto learning at a new held pose,
         * or a wrongly learned yaw can masquerade as zero-rate motion. */
        fast_heading=net_heading(fast_q,state_c.q,0.1f)+heading_bias(fast_q);
        quiet_fast_heading=net_heading(quiet_fast_q,quiet_q,0.1f);
        quiet_fast_tilt=net_tilt(quiet_fast_q,quiet_q,0.1f);
        memcpy(quiet_fast_q,quiet_q,sizeof(quiet_fast_q));
        memcpy(fast_q,state_c.q,sizeof(fast_q));fast_count=0;
    }
    if(fabsf(fast_heading)>0.06f || fabsf(quiet_fast_heading)>0.06f || (classifier_acc_variance<0.03f && quiet_fast_tilt>=0.06f))stationary=0;
#endif
    /* Previous 1 kHz low-pass output is already bias corrected. A full
     * second mean retains DC slow rotation, but attenuates incomplete
     * high-frequency vibration cycles at the window boundary. */
    filtered_rate_sums[0]+=g_imufilter_1000hz.gyrox;
    filtered_rate_sums[1]+=g_imufilter_1000hz.gyroy;
    filtered_rate_sums[2]+=g_imufilter_1000hz.gyroz;
    for(j=0;j<6;j++) { sums[j]+=v[j];squares[j]+=v[j]*v[j]; }
    if(++window_count<BIAS_WINDOW_COUNT)return;
    for(j=0;j<6;j++) {
        m[j]=sums[j]/BIAS_WINDOW_FLOAT;var[j]=squares[j]/BIAS_WINDOW_FLOAT-m[j]*m[j];
        if(var[j]<0)var[j]=0;
        sums[j]=squares[j]=0;
        if(j<3) {
            filtered_mean[j]=filtered_rate_sums[j]/BIAS_WINDOW_FLOAT;
            filtered_rate_sums[j]=0;rate2+=filtered_mean[j]*filtered_mean[j];gv+=var[j];
        }
        else { av+=var[j];float d=0.2f*(m[j]-previous_acc[j-3]);step+=d*d;window_acc[j-3]=m[j]; }
    }
    classifier_acc_variance=av;
    norm=sqrtf(m[3]*m[3]+m[4]*m[4]+m[5]*m[5]);
    /* A physical slow tilt moves gravity even when heading is unchanged.
     * Detect it explicitly before relaxing vibration variance/mean steps. */
    float heading=net_heading(quiet_before,quiet_q,1.0f);
    float uncorrected_heading=heading+heading_bias(quiet_before);
    quiet_tilt=net_tilt(quiet_before,quiet_q,1.0f);
    memcpy(quiet_before,quiet_q,sizeof(quiet_before));
    if(has_previous && rate2<0.005625f && gv<BIAS_GYRO_VAR_LIMIT && av<BIAS_ACC_VAR_LIMIT &&
       fabsf(norm-1)<0.08f && step<(av>0.03f?0.000009f:BIAS_ACC_STEP_LIMIT)
#ifdef C_MOTION_KP
       && fabsf(uncorrected_heading)<0.06f && fabsf(heading)<0.06f && fabsf(quiet_fast_heading)<0.06f && (av>=0.03f || quiet_tilt<0.06f) && (av>=0.03f || quiet_fast_tilt<0.06f) && (av>=0.03f || fabsf(fast_heading)<0.06f)
#endif
       )stable_windows++;
    else stable_windows=0;
#ifdef C_MOTION_KP
    if(fabsf(uncorrected_heading)>0.06f || fabsf(heading)>0.06f || fabsf(quiet_fast_heading)>0.06f || (classifier_acc_variance<0.03f && quiet_tilt>=0.06f) || (classifier_acc_variance<0.03f && quiet_fast_tilt>=0.06f))stable_windows=0;
#endif
    stationary=(uint8_t)(stable_windows>=BIAS_STATIC_WINDOWS);
    if(stationary) {
#ifdef C_MOTION_KP
        float grav[3]={m[3]/norm,m[4]/norm,m[5]/norm},measured_heading=0,old_heading=0;
        for(j=0;j<3;j++) { measured_heading+=(filtered_mean[j]+bias[j])*grav[j];old_heading+=bias[j]*grav[j]; }
        for(j=0;j<3;j++) {
            float tangent_error=filtered_mean[j]+bias[j]-measured_heading*grav[j]-(bias[j]-old_heading*grav[j]);
            /* Quiet heading recovers faster than the noisy raw tangent mean.
             * Strong vibration uses a damped gain with the 4 s classifier. */
            bias[j]+=(av>0.03f?0.08f:BIAS_LEARN_ALPHA)*tangent_error
                    +(av>0.03f?0.08f:0.5f)*heading*grav[j];
            if(bias[j]>2)bias[j]=2;
            if(bias[j]<-2)bias[j]=-2;
        }
#else
        for(j=0;j<3;j++)bias[j]+=BIAS_LEARN_ALPHA*(m[j]-bias[j]);
#endif
    }
#ifdef C_MOTION_KP
    for(j=0;j<6;j++) { compare_bias_diag[j]=m[j];compare_bias_diag[j+6]=var[j]; }
    compare_bias_diag[12]=sqrtf(rate2);compare_bias_diag[13]=sqrtf(step);
    compare_bias_diag[14]=fast_heading;compare_bias_diag[15]=(float)stable_windows;
    for(j=0;j<3;j++)compare_bias_diag[16+j]=bias[j];
    compare_bias_diag[19]=heading;compare_bias_diag[20]=(float)stationary;
#endif
    /* A clearly rotating window invalidates the previous held-pose mean.
     * The first subsequent quiet window establishes a new reference, without
     * learning bias. Three further valid windows are still required. Slow
     * motion keeps its history and all heading/tilt/rate vetoes apply. */
    if(rate2>4.0f) {
        /* Actual rotation must not be left in the vibration smoother. It
         * would otherwise forbid bias learning through successive holds.
         * Reset the classifier only; exported A/C states are untouched.
         * This window remains moving and three clean windows after a new
         * acceleration reference are still required before bias learning. */
        has_previous=0;stable_windows=0;stationary=0;
        memcpy(quiet_q,state_c.q,sizeof(quiet_q));
        memcpy(quiet_before,state_c.q,sizeof(quiet_before));
        memcpy(quiet_fast_q,state_c.q,sizeof(quiet_fast_q));
        quiet_tilt=quiet_fast_tilt=quiet_fast_heading=0;
    }
    else {
        for(j=0;j<3;j++) {
            if(has_previous)previous_acc[j]+=0.2f*(m[j+3]-previous_acc[j]);
            else previous_acc[j]=m[j+3];
        }
        has_previous=1;
    }
    window_count=0;
}


static float gyro_sum[3], yaw_previous, yaw_continuous;
static unsigned int gyro_count;
static uint32_t (*read_clock)(void);
static float clock_scale;
static uint32_t stamp(void) { return read_clock ? read_clock() : 0u; }
static float duration(uint32_t start) { return (float)(stamp()-start)*clock_scale; }
void compare_core_set_clock(uint32_t (*clock_ticks)(void), float us_per_tick)
{ read_clock=clock_ticks; clock_scale=us_per_tick; }
static uint8_t finite_sample(const ahrs_sample_t *s)
{
    return (uint8_t)(isfinite(s->gx)&&isfinite(s->gy)&&isfinite(s->gz)&&
        isfinite(s->ax)&&isfinite(s->ay)&&isfinite(s->az)&&
        fabsf(s->gx)<10000&&fabsf(s->gy)<10000&&fabsf(s->gz)<10000&&
        fabsf(s->ax)<100&&fabsf(s->ay)<100&&fabsf(s->az)<100&&
        s->ax*s->ax+s->ay*s->ay+s->az*s->az>1e-12f);
}
static float wrap(float x)
{ while(x>180) x-=360; while(x < -180) x+=360; return x; }
void compare_core_init(const ahrs_sample_t *s)
{
    memset(&compare_result,0,sizeof(compare_result));
    memset(filtered_rate_sums,0,sizeof(filtered_rate_sums));
    memset(sums,0,sizeof(sums));memset(squares,0,sizeof(squares));memset(bias,0,sizeof(bias));
    window_count=stable_windows=has_previous=0;stationary=0;
#ifdef C_MOTION_KP
    fast_heading=0;fast_count=0;
    memset(compare_bias_diag,0,sizeof(compare_bias_diag));
#endif
    window_acc[0]=s->ax;window_acc[1]=s->ay;window_acc[2]=s->az;
    memset(gyro_sum,0,sizeof(gyro_sum)); gyro_count=0;
    yaw_previous=yaw_continuous=0;
    MahonyAhrs_Init(&state_a); IMUFilter_Init(); uni_init(s); improved_init(&state_c,s);
#ifdef C_MOTION_KP
    memcpy(fast_q,state_c.q,sizeof(fast_q));memcpy(window_q,state_c.q,sizeof(window_q));
    memcpy(quiet_q,state_c.q,sizeof(quiet_q));memcpy(quiet_before,state_c.q,sizeof(quiet_before));memcpy(quiet_fast_q,state_c.q,sizeof(quiet_fast_q));quiet_tilt=quiet_fast_tilt=quiet_fast_heading=classifier_acc_variance=0;
    previous_acc[0]=s->ax;previous_acc[1]=s->ay;previous_acc[2]=s->az;
#endif
    state_a.q0=state_c.q[0];state_a.q1=state_c.q[1];state_a.q2=state_c.q[2];state_a.q3=state_c.q[3];
    yaw_previous=MahonyAhrs_GetEulerDegrees(&state_a).yaw;
    yaw_continuous=yaw_previous;
    compare_result.b.roll=uni_att.roll; compare_result.b.pitch=uni_att.pitch;
    compare_result.c=state_c.out;
}
uint8_t compare_core_step(const ahrs_sample_t *s)
{
    ahrs_sample_t filtered,corrected,scaled;
    (void)scaled;
    MahonyAhrs_Euler_t ea;
    uint32_t start;
    if(!finite_sample(s)) {
        compare_result.invalid_samples++;
        memset(gyro_sum,0,sizeof(gyro_sum)); gyro_count=0;
        return 0;
    }
    compare_result.sample=*s;
#ifdef GYRO_CAL_MATRIX
    {
        static const float matrix[9]=GYRO_CAL_MATRIX;
        scaled=*s;
        scaled.gx=matrix[0]*s->gx+matrix[1]*s->gy+matrix[2]*s->gz;
        scaled.gy=matrix[3]*s->gx+matrix[4]*s->gy+matrix[5]*s->gz;
        scaled.gz=matrix[6]*s->gx+matrix[7]*s->gy+matrix[8]*s->gz;
        s=&scaled;
    }
#endif
    window_bias(s);corrected=*s;
    corrected.gx-=bias[0];corrected.gy-=bias[1];corrected.gz-=bias[2];
    s=&corrected;
    start=stamp();
    IMUFilter_Update(s->gx,s->gy,s->gz,s->ax,s->ay,s->az);
    compare_result.filter_us=duration(start);
    filtered.gx=g_imufilter_1000hz.gyrox; filtered.gy=g_imufilter_1000hz.gyroy; filtered.gz=g_imufilter_1000hz.gyroz;
    filtered.ax=g_imufilter_1000hz.accx; filtered.ay=g_imufilter_1000hz.accy; filtered.az=g_imufilter_1000hz.accz;
    /* A expects specific force with the opposite sign to the shared +1g input. */
    start=stamp();
    MahonyAhrs_Update(&state_a,filtered.gx,filtered.gy,filtered.gz,-filtered.ax,-filtered.ay,-filtered.az,0.001f);
    ea=MahonyAhrs_GetEulerDegrees(&state_a);
    yaw_continuous+=wrap(ea.yaw-yaw_previous); yaw_previous=ea.yaw;
    compare_result.a.roll=ea.roll; compare_result.a.pitch=ea.pitch; compare_result.a.yaw=yaw_continuous;
    compare_result.a.acc_weight=state_a.acc_weight_final;
    compare_result.a.bias_z_dps=state_a.gyro_bias_z_static*AHRS_RAD_TO_DEG;
    compare_result.a.is_static=state_a.is_static;
    compare_result.a_us=duration(start);
    gyro_sum[0]+=s->gx; gyro_sum[1]+=s->gy; gyro_sum[2]+=s->gz;
    if(++gyro_count==5) {
        ahrs_sample_t avg=*s;
        avg.gx=gyro_sum[0]/5; avg.gy=gyro_sum[1]/5; avg.gz=gyro_sum[2]/5;
        start=stamp();
        uni_update(&avg);
        compare_result.b.roll=uni_att.roll; compare_result.b.pitch=uni_att.pitch; compare_result.b.yaw=uni_att.yaw;
        compare_result.b.acc_weight=uni_diag.acc_valid ? 1.0f-fabsf(uni_diag.acc_norm-1.0f)*0.5f : 0.0f;
        compare_result.b.bias_z_dps=uni_diag.bias_z*AHRS_RAD_TO_DEG;
        compare_result.b.fault=uni_diverged();
        compare_result.b_us=duration(start);
        memset(gyro_sum,0,sizeof(gyro_sum)); gyro_count=0;
    }
    start=stamp();
    improved_update(&state_c,&filtered,0.001f); compare_result.c=state_c.out;
    compare_result.c_us=duration(start);
    compare_result.a.bias_z_dps=compare_result.b.bias_z_dps=compare_result.c.bias_z_dps=bias[2];
    compare_result.a.is_static=compare_result.b.is_static=compare_result.c.is_static=stationary;
    compare_result.samples++;
    return 1;
}
