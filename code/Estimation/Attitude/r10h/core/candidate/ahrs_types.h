#ifndef AHRS_TYPES_H
#define AHRS_TYPES_H
#include <stdint.h>
/* Body axes are right handed. Acceleration is +Z = +1g when level. */
typedef struct { float gx, gy, gz, ax, ay, az; } ahrs_sample_t;
typedef struct {
    float roll, pitch, yaw;
    float acc_weight, bias_z_dps;
    uint8_t is_static, fault;
} ahrs_output_t;
#define AHRS_DEG_TO_RAD (0.017453292519943f)
#define AHRS_RAD_TO_DEG (57.29577951308232f)
#endif
