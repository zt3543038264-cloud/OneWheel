/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef DRIVER_SAFETY_H
#define DRIVER_SAFETY_H
#include <stdint.h>
#include <stddef.h>
#define DRIVER_SAFE_B_DUTY_MAX 500
#define DRIVER_SAFE_CAPABILITY 1u /* right-only, max 500, max 200 ms */
#define DRIVER_SAFE_COMMAND_MS 100u
#define DRIVER_SAFE_ARM_WINDOW_MS 5000u
#define DRIVER_SAFE_PULSE_MS 200u
#define DRIVER_SAFE_MAIN_MS 100u
#define DRIVER_SAFE_CONTROL_MS 10u
#define DRIVER_SAFE_FRAME_BYTES 96u
/* Values are part of the version 1 wire contract. */
enum { DRIVER_SAFE_DISARMED, DRIVER_SAFE_LOCAL_READY, DRIVER_SAFE_ZEROED,
       DRIVER_SAFE_ARMED, DRIVER_SAFE_LATCHED };
enum { DRIVER_SAFE_NONE, DRIVER_SAFE_BOOT, DRIVER_SAFE_MANUAL,
       DRIVER_SAFE_COMMAND_TIMEOUT, DRIVER_SAFE_ARM_WINDOW_EXPIRED,
       DRIVER_SAFE_PULSE_LIMIT, DRIVER_SAFE_CALIBRATION_INVALID,
       DRIVER_SAFE_MOTOR_FAULT, DRIVER_SAFE_SEQUENCE_EXHAUSTED,
       DRIVER_SAFE_MAIN_TIMEOUT, DRIVER_SAFE_CONTROL_TIMEOUT };
void driver_safety_init(void);
void driver_safety_tick_1ms(void);
void driver_safety_main_alive(void);
void driver_safety_control_alive(unsigned side);
/* Physical level sampled at 10 ms; boot-held button cannot grant permission. */
void driver_safety_button_sample(int released);
void driver_safety_disarm(unsigned reason);
int driver_safety_output_allowed(unsigned side);
void driver_safety_service(void);
void motor_driver_receive_byte(uint8_t byte);
/* Hardware adapter: GPIO-low gates, not merely a zero torque request. */
void motor_output_inhibit(void);
extern uint8_t motor_calibration_valid;
#endif
