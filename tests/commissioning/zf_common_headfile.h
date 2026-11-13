#ifndef COMMISSION_HOST_H
#define COMMISSION_HOST_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define func_abs(x) ((x)<0?-(x):(x))
typedef uint8_t uint8; typedef uint16_t uint16; typedef uint32_t uint32; typedef uint64_t uint64;
typedef int8_t int8; typedef int16_t int16; typedef int32_t int32;
#define MT9V03X_W 188
#define MT9V03X_H 120
#define CCU60_CH0 0
#define func_limit(x,limit) ((x)<-(limit)?-(limit):((x)>(limit)?(limit):(x)))
#define func_limit_ab(x,lo,hi) ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))
uint32 interrupt_global_disable(void); void interrupt_global_enable(uint32);
void key_init(int); void key_scanner(void); void pit_ms_init(int,int);
#endif
