#ifndef DRIVER_SAFETY_HOST_H
#define DRIVER_SAFETY_HOST_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
typedef uint8_t uint8; typedef int8_t int8; typedef uint16_t uint16;
typedef int16_t int16; typedef uint32_t uint32; typedef int32_t int32;
#define UART_2 2
#define UART2_RX_P07_0 0
#define UART2_TX_P07_1 1
#define CPUIntIdx7_IRQn 7
#define PROTECT_MODE 1
#define ENCODER_ERROR 1
#define BATTERY_ERROR 2
#define func_limit_ab(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
typedef struct { float motor_duty; unsigned motor_protect_state,encoder_state,lose_control_protect_count; } test_motor;
typedef struct { unsigned protect_flag,battery_state; } test_battery;
extern test_motor motor_left,motor_right;
extern test_battery battery_value;
extern uint32_t test_irq;
static inline uint32_t __get_PRIMASK(void) {return test_irq;}
static inline void __disable_irq(void) {test_irq=1;}
static inline void __set_PRIMASK(uint32_t n) {test_irq=n;}
static inline void NVIC_SetPriority(int irq,unsigned priority) {(void)irq;(void)priority;}
static inline void uart_init(int a,int b,int c,int d) {(void)a;(void)b;(void)c;(void)d;}
static inline void uart_rx_interrupt(int a,int b) {(void)a;(void)b;}
int uart_query_byte(int uart,uint8_t *byte);
void uart_write_buffer(int uart,const uint8_t *data,uint32_t n);
#endif
