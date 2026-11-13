/* Exercise the actual target wait/deadline code against QSPI registers. */
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned fault_count;
static void imu_latch_fault(const char *reason) { assert(!strcmp(reason,"IMU SPI TIMEOUT")); ++fault_count; }
typedef uint8_t uint8;typedef uint32_t uint32;
#define IMU660RB_SPI 0
#define IMU660RB_SPI_R 128
static unsigned selected;
#define IMU660RB_CS(x) (selected=!(x))
#include "IfxQspi.h"
#include "../../code/HW_Drivers/IMU/imu_spi.inc"
uint32_t host_clock,host_clock_cost;
Ifx_QSPI host_qspi;
Ifx_QSPI_BACON bacon[4];
static unsigned transfers,fail_transfer,fail_end,last_transfer;
Ifx_QSPI *IfxQspi_getAddress(IfxQspi_Index n) { (void)n;return &host_qspi; }
void IfxQspi_clearAllEventFlags(Ifx_QSPI *q) { q->STATUS.B.PT1F=0; }
void IfxQspi_writeBasicConfigurationBeginStream(Ifx_QSPI *q,uint32_t v) { (void)q;(void)v;last_transfer=0; }
void IfxQspi_writeBasicConfigurationEndStream(Ifx_QSPI *q,uint32_t v) { (void)q;(void)v;last_transfer=1; }
void IfxQspi_writeTransmitFifo(Ifx_QSPI *q,uint32_t v) {
    (void)v;++transfers;q->STATUS.B.RXFIFOLEVEL=transfers==fail_transfer?0:1;
    if(last_transfer && !fail_end)q->STATUS.B.PT1F=1;
}
uint32_t IfxQspi_readReceiveFifo(Ifx_QSPI *q) { q->STATUS.B.RXFIFOLEVEL=0;return 0x35; }
static void reset(void) { host_qspi.STATUS.B.RXFIFOLEVEL=host_qspi.STATUS.B.PT1F=0;host_clock=0;host_clock_cost=5;transfers=fail_transfer=fail_end=last_transfer=0;r10h_bus_timeout_ticks=100; }
int main(void) {
    uint8 bytes[12];
    reset();assert(r10h_bus_read(0x22,bytes,12));assert(!selected && transfers==13 && bytes[11]==0x35);
    for(unsigned fail=1;fail<=13;fail++) {
        reset();fail_transfer=fail;assert(!r10h_bus_read(0x22,bytes,12));assert(!selected && host_clock<=110);assert(fault_count);
    }
    reset();fail_end=1;assert(!r10h_bus_read(0x22,bytes,12));assert(!selected && host_clock<=110);assert(fault_count);
    reset();host_clock=UINT32_MAX-50u;fail_transfer=1;assert(!r10h_bus_read(0x22,bytes,12));assert(!selected);
    reset();assert(!r10h_bus_read(0x22,bytes,0));
    puts("PASS: target SPI RX/deadline/end waits, 12-byte burst, CS release, timer wrap and invalid length");return 0;
}
