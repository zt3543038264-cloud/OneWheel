#if defined(__TASKING__)
#pragma section all "cpu0_dsram"
#endif
#include "../include/r10h_attitude.h"
#include "../core/candidate/compare_core.h"
#include <math.h>
extern compare_result_t v2_compare_result;
void v2_compare_core_init(const ahrs_sample_t *s);
uint8_t v2_compare_core_step(const ahrs_sample_t *s);
void v2_compare_core_set_clock(uint32_t (*clock_ticks)(void), float us_per_tick);
static float startup_bias[3];
static uint8_t initialized, has_output;
static uint8_t valid(const ahrs_sample_t *s) {
    return (uint8_t)(s && isfinite(s->gx) && isfinite(s->gy) && isfinite(s->gz) &&
        isfinite(s->ax) && isfinite(s->ay) && isfinite(s->az) &&
        fabsf(s->gx)<10000 && fabsf(s->gy)<10000 && fabsf(s->gz)<10000 &&
        fabsf(s->ax)<100 && fabsf(s->ay)<100 && fabsf(s->az)<100 &&
        s->ax*s->ax+s->ay*s->ay+s->az*s->az>1e-12f);
}
uint8_t r10h_init(const ahrs_sample_t *mean) {
    ahrs_sample_t initial;
    initialized=has_output=0;
    if(!valid(mean)) return 0;
    startup_bias[0]=mean->gx; startup_bias[1]=mean->gy; startup_bias[2]=mean->gz;
    initial=*mean; initial.gx=initial.gy=initial.gz=0;
    v2_compare_core_init(&initial);
    initialized=1; return 1;
}
uint8_t r10h_update(const ahrs_sample_t *raw) {
    ahrs_sample_t input;
    uint8_t accepted;
    if(!initialized || !raw) return 0;
    input=*raw;
    input.gx-=startup_bias[0]; input.gy-=startup_bias[1]; input.gz-=startup_bias[2];
    accepted=v2_compare_core_step(&input);
    if(accepted) has_output=1;
    return accepted;
}
uint8_t r10h_get_output(ahrs_output_t *out) {
    if(!initialized || !has_output || !out) return 0;
    *out=v2_compare_result.a;
    return 1;
}
void r10h_set_clock(uint32_t (*clock_ticks)(void), float us_per_tick) {
    v2_compare_core_set_clock(clock_ticks,us_per_tick);
}

#if defined(__TASKING__)
#pragma section all restore
#endif
