
#include "compare_core.h"
#include <math.h>
#include <stdio.h>
extern compare_result_t v2_compare_result;
void v2_compare_core_init(const ahrs_sample_t *);
unsigned char v2_compare_core_step(const ahrs_sample_t *);
int main(void) {
    ahrs_sample_t s={0,0,0,0,0,1};
    for(int trial=0;trial<3;trial++) {
        float yaw=trial==0?0.0f:trial==1?0.1f:0.5f;
        v2_compare_core_init(&s);float before=0;
        for(int i=0;i<60000;i++) {
            float t=i*.001f;
            s.gx=5*sinf(2*3.14159265359f*237*t);
            s.gy=3*sinf(2*3.14159265359f*281*t);
            s.gz=yaw+(trial==0?0.04f:0)+.3f*sinf(2*3.14159265359f*303*t);
            s.ax=.6f*sinf(2*3.14159265359f*237*t);
            s.ay=.6f*sinf(2*3.14159265359f*281*t);
            s.az=1+.3f*sinf(2*3.14159265359f*303*t);
            v2_compare_core_step(&s);
            if(i==39999)before=v2_compare_result.a.yaw;
        }
        printf("strong vibration yaw %.3f dps: final %.5f, last20s %.5f, bias %.6f, static %u\n",yaw,v2_compare_result.a.yaw,v2_compare_result.a.yaw-before,v2_compare_result.a.bias_z_dps,v2_compare_result.a.is_static);
        if(trial==0 && (fabsf(v2_compare_result.a.yaw-before)>.12f ||fabsf(v2_compare_result.a.bias_z_dps-.04f)>.012f))return 10;
        if(trial>0 && fabsf(v2_compare_result.a.yaw-60*yaw)>.3f)return 20+trial;
        s=(ahrs_sample_t){0,0,0,0,0,1};
    }
    return 0;
}
