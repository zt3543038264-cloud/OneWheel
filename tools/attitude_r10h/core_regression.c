#include "compare_core.h"
#include <stdio.h>
#include <math.h>
extern compare_result_t v2_compare_result;
void v2_compare_core_init(const ahrs_sample_t *);
unsigned char v2_compare_core_step(const ahrs_sample_t *);
int main(void) {
    ahrs_sample_t s={0,0,0,0,0,1};
    for(int trial=0;trial<3;trial++) {
        float rate=trial==0?0.1f:trial==1?0.5f:90.0f;
        v2_compare_core_init(&s);
        for(int i=0;i<4000;i++)v2_compare_core_step(&s);
        s.gz=rate;
        for(int i=0;i<10000;i++)v2_compare_core_step(&s);
        float out[3]={v2_compare_result.a.yaw,v2_compare_result.b.yaw,v2_compare_result.c.yaw};
        for(int j=0;j<3;j++)if(fabsf(out[j]-10*rate)>(rate<1?0.05f:0.8f))return 10+trial*3+j;
        if(v2_compare_result.c.is_static)return 20;
        printf("preserve %.3f dps: %.4f %.4f %.4f deg\n",rate,out[0],out[1],out[2]);s.gz=0;
    }
    s.ay=0.5f;s.az=0.8660254f;v2_compare_core_init(&s);
    v2_compare_core_step(&s);
    if(fabsf(v2_compare_result.a.roll-30)>0.01f ||fabsf(v2_compare_result.b.roll-30)>0.01f||fabsf(v2_compare_result.c.roll-30)>0.01f)return 30;
    s.ay=0;s.az=1;v2_compare_core_init(&s);
    for(int i=0;i<30000;i++) {
        s.gz=0.04f+0.3f*sinf(i*0.31f);s.gx=2*sinf(i*0.41f);
        v2_compare_core_step(&s);
    }
    if(!v2_compare_result.c.is_static ||fabsf(v2_compare_result.c.bias_z_dps-0.04f)>0.008f)return 40;
    printf("vibration window static: bias %.6f dps; A/B/C yaw %.4f %.4f %.4f\n",v2_compare_result.c.bias_z_dps,v2_compare_result.a.yaw,v2_compare_result.b.yaw,v2_compare_result.c.yaw);
    s=(ahrs_sample_t){0,0,0,0,0,1};v2_compare_core_init(&s);
    for(int i=0;i<4000;i++)v2_compare_core_step(&s);
    unsigned mistaken=0;
    for(int i=0;i<10000;i++) {
        float angle=(i+1)*0.0001f*0.0174532925199433f;
        s.gx=0.1f;s.gy=0.0f;s.gz=0.0f;
        s.ay=sinf(angle);s.az=cosf(angle);v2_compare_core_step(&s);
        if(i>2000 && v2_compare_result.c.is_static)mistaken++;
    }
    printf("physical slow pitch: A/B/C %.5f %.5f %.5f deg; mistaken static %u\n",v2_compare_result.a.roll,v2_compare_result.b.roll,v2_compare_result.c.roll,mistaken);
    if(mistaken || fabsf(v2_compare_result.a.roll-1)>0.05f ||fabsf(v2_compare_result.c.roll-1)>0.05f)return 60;
    s.ax=NAN;if(v2_compare_core_step(&s)!=0||v2_compare_result.invalid_samples!=1)return 50;
    puts("PASS: slow yaw, fast yaw, tilted gravity initialization, noisy static bias and invalid samples");return 0;
}
