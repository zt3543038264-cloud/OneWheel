#ifndef COMPARE_CORE_H
#define COMPARE_CORE_H
#include "ahrs_types.h"
/* A: original CYT 1kHz + its filter. B: original Unicycle 200Hz.
 * C: experimental improved 1kHz + A's filter. Singleton, CPU0 only. */
typedef struct {
    ahrs_output_t a, b, c;
    ahrs_sample_t sample;
    uint32_t samples, invalid_samples;
    float filter_us, a_us, b_us, c_us;
} compare_result_t;
extern compare_result_t compare_result;
void compare_core_init(const ahrs_sample_t *static_acc);
uint8_t compare_core_step(const ahrs_sample_t *sample);
void compare_core_set_clock(uint32_t (*clock_ticks)(void), float us_per_tick);
#endif
