#ifndef UNICYCLE_AHRS_H
#define UNICYCLE_AHRS_H
#include "ahrs_types.h"
#define ATT_DT (0.005f)
typedef struct {
    float roll, pitch, yaw, yaw_wrapped;
    float roll_rate, pitch_rate, yaw_rate;
} uni_attitude_t;
typedef struct { float acc_norm, bias_z; uint8_t acc_valid; } uni_diag_t;
extern uni_attitude_t uni_att;
extern uni_diag_t uni_diag;
static inline float constrain_float(float x, float lo, float hi)
{ return x < lo ? lo : (x > hi ? hi : x); }
void uni_init(const ahrs_sample_t *initial);
void uni_update(const ahrs_sample_t *sample);
void uni_update_rate(void);
uint8_t uni_converged(void);
uint8_t uni_diverged(void);
#endif
