#include "zf_common_headfile.h"

void core1_main(void)
{
    disable_Watchdog();
    interrupt_global_enable(0);
    cpu_wait_event_ready();
    for (;;) {}
}
