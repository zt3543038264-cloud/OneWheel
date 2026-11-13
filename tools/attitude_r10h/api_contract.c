#include "r10h_attitude.h"
#include <math.h>
#include <stdio.h>
int main(void) {
    ahrs_sample_t mean={.2f,-.1f,.3f,0,0,1}, bad=mean;
    ahrs_output_t before,after;
    if(r10h_update(&mean)||r10h_get_output(&before))return 1;
    if(r10h_init(NULL))return 2;
    bad.az=0;if(r10h_init(&bad))return 3;
    bad=mean;bad.gx=NAN;if(r10h_init(&bad))return 4;
    if(!r10h_init(&mean)||r10h_get_output(&before))return 5;
    if(r10h_update(NULL))return 6;
    if(!r10h_update(&mean)||!r10h_get_output(&before))return 7;
    bad=mean;bad.gx=INFINITY;
    if(r10h_update(&bad)||!r10h_get_output(&after))return 8;
    if(before.roll!=after.roll||before.pitch!=after.pitch||before.yaw!=after.yaw)return 9;
    if(!r10h_init(&mean)||r10h_get_output(&after))return 10;
    puts("PASS: init/update/output availability, null, invalid mean and stale output");
    return 0;
}
