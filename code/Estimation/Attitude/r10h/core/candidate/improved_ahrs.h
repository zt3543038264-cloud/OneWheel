#ifndef IMPROVED_AHRS_H
#define IMPROVED_AHRS_H
#include "ahrs_types.h"
typedef struct {
    float q[4], bias[3], prev_yaw, yaw, prev_acc[3];
    uint32_t static_count;
    ahrs_output_t out;
} improved_ahrs_t;
void improved_init(improved_ahrs_t *s, const ahrs_sample_t *sample);
void improved_update(improved_ahrs_t *s, const ahrs_sample_t *sample, float dt);
#endif
