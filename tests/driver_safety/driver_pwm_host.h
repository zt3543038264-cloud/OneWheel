#ifndef DRIVER_PWM_HOST_H
#define DRIVER_PWM_HOST_H
#include "driver_safety_host.h"
typedef struct {uint32_t u32Register;} test_reg;
typedef struct {test_reg unCC0,unCC1,unCC0_BUFF,unCC1_BUFF,unTR_CMD,unCTRL;} stc_TCPWM_GRP_CNT_t;
typedef struct {unsigned level[6],mux[6],mode[6];} stc_GPIO_PRT_t;
extern stc_TCPWM_GRP_CNT_t test_pwm[6];
extern stc_GPIO_PRT_t test_gpio[2];
#define PWM_PRIOD_LOAD 4000u
#define HSIOM_SEL_GPIO 0u
#define CY_GPIO_DM_STRONG 6u
static inline void Cy_GPIO_Clr(volatile stc_GPIO_PRT_t *p,uint32_t n){p->level[n]=0;}
static inline void Cy_GPIO_SetHSIOM(volatile stc_GPIO_PRT_t *p,uint32_t n,unsigned m){p->mux[n]=m;}
static inline void Cy_GPIO_SetDrivemode(volatile stc_GPIO_PRT_t *p,uint32_t n,unsigned m){p->mode[n]=m;}
#define MOTOR_LEFT_A_PHASE_TCPWM_TIMER (&test_pwm[0])
#define MOTOR_LEFT_A_PHASE_PORT_H (&test_gpio[0])
#define MOTOR_LEFT_A_PHASE_PIN_H 0u
#define MOTOR_LEFT_A_PIN_HSIOM_H 8u
#define MOTOR_LEFT_A_PHASE_PORT_L (&test_gpio[0])
#define MOTOR_LEFT_A_PHASE_PIN_L 1u
#define MOTOR_LEFT_A_PIN_HSIOM_L 8u
#define MOTOR_LEFT_B_PHASE_TCPWM_TIMER (&test_pwm[1])
#define MOTOR_LEFT_B_PHASE_PORT_H (&test_gpio[0])
#define MOTOR_LEFT_B_PHASE_PIN_H 2u
#define MOTOR_LEFT_B_PIN_HSIOM_H 8u
#define MOTOR_LEFT_B_PHASE_PORT_L (&test_gpio[0])
#define MOTOR_LEFT_B_PHASE_PIN_L 3u
#define MOTOR_LEFT_B_PIN_HSIOM_L 8u
#define MOTOR_LEFT_C_PHASE_TCPWM_TIMER (&test_pwm[2])
#define MOTOR_LEFT_C_PHASE_PORT_H (&test_gpio[0])
#define MOTOR_LEFT_C_PHASE_PIN_H 4u
#define MOTOR_LEFT_C_PIN_HSIOM_H 8u
#define MOTOR_LEFT_C_PHASE_PORT_L (&test_gpio[0])
#define MOTOR_LEFT_C_PHASE_PIN_L 5u
#define MOTOR_LEFT_C_PIN_HSIOM_L 8u
#define MOTOR_RIGHT_A_PHASE_TCPWM_TIMER (&test_pwm[3])
#define MOTOR_RIGHT_A_PHASE_PORT_H (&test_gpio[1])
#define MOTOR_RIGHT_A_PHASE_PIN_H 0u
#define MOTOR_RIGHT_A_PIN_HSIOM_H 8u
#define MOTOR_RIGHT_A_PHASE_PORT_L (&test_gpio[1])
#define MOTOR_RIGHT_A_PHASE_PIN_L 1u
#define MOTOR_RIGHT_A_PIN_HSIOM_L 8u
#define MOTOR_RIGHT_B_PHASE_TCPWM_TIMER (&test_pwm[4])
#define MOTOR_RIGHT_B_PHASE_PORT_H (&test_gpio[1])
#define MOTOR_RIGHT_B_PHASE_PIN_H 2u
#define MOTOR_RIGHT_B_PIN_HSIOM_H 8u
#define MOTOR_RIGHT_B_PHASE_PORT_L (&test_gpio[1])
#define MOTOR_RIGHT_B_PHASE_PIN_L 3u
#define MOTOR_RIGHT_B_PIN_HSIOM_L 8u
#define MOTOR_RIGHT_C_PHASE_TCPWM_TIMER (&test_pwm[5])
#define MOTOR_RIGHT_C_PHASE_PORT_H (&test_gpio[1])
#define MOTOR_RIGHT_C_PHASE_PIN_H 4u
#define MOTOR_RIGHT_C_PIN_HSIOM_H 8u
#define MOTOR_RIGHT_C_PHASE_PORT_L (&test_gpio[1])
#define MOTOR_RIGHT_C_PHASE_PIN_L 5u
#define MOTOR_RIGHT_C_PIN_HSIOM_L 8u
void motor_left_duty_set(uint16,uint16,uint16);
void motor_right_duty_set(uint16,uint16,uint16);
void motor_left_channel_set(uint8,uint8,uint8);
void motor_right_channel_set(uint8,uint8,uint8);
#endif
