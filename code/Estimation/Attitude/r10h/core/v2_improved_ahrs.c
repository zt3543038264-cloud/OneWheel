#if defined(__TASKING__)
#pragma section all "cpu0_dsram"
#endif
#include "v2_prefix.h"
#include "v3_calibration.h"
#define R10_NORMAL_STARTUP_GAIN (1)
#include "candidate/improved_ahrs.inc"

#if defined(__TASKING__)
#pragma section all restore
#endif
