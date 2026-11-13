#include <stdint.h>
extern uint32_t host_clock, host_clock_cost;
#define MODULE_STM0 host_clock
static inline uint32_t IfxStm_getLower(const uint32_t *p) { uint32_t n=*p; host_clock+=host_clock_cost; return n; }
static inline float IfxStm_getFrequency(const uint32_t *p) { (void)p; return 1000000.0f; }
