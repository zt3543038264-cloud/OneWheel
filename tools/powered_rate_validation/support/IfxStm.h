#ifndef COMMISSION_STM_H
#define COMMISSION_STM_H
#include "zf_common_headfile.h"
extern int host_stm;
#define MODULE_STM0 host_stm
uint32 IfxStm_getLower(int *); uint32 IfxStm_getOffset32Timer(int *);
float IfxStm_getFrequency(int *);
#endif
