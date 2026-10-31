#ifndef WHEEL_HOLD_H
#define WHEEL_HOLD_H
#include <stdint.h>
#include "wheel_encoder.h"

/* Modes 2/10: position P -> speed PI -> ground tilt. No actuator writes.
 * Parameters are degrees per (count/s), degrees per count, and degrees.
 * The six runtime parameters are not part of the current Flash record. */
extern volatile uint8_t wheel_hold_enabled, wheel_feedback_valid;
extern volatile int32_t wheel_hold_state; /* 0 off, 1 waiting, 2 active, -1 encoder, -2 config */
extern volatile uint32_t wheel_feedback_age_ms;
extern volatile float wheel_target_cps, wheel_speed_error, wheel_tilt_offset, wheel_speed_integral;
extern float wheel_speed_kp, wheel_speed_ki, wheel_tilt_limit, wheel_i_limit;
extern volatile int32_t wheel_ref_count,wheel_pos_error;
extern float wheel_pos_kp,wheel_speed_limit;
void wheel_hold_defaults(void);
void wheel_hold_reset(void);
void wheel_hold_suspend(void); /* Clear PI/output, retain last diagnostic/fault. */
/* 1 ready, 0 waiting for first window, -7 bad encoder, -8 stale, -9 config,
 * -10 discontinuous position/epoch. Origin survives inhibition until Stop/Start.
 * Call only when IMU/control gates pass. Same sequence never integrates twice. */
int wheel_hold_update(const wheel_encoder_sample_t *sample, int freeze_integral);
#endif
