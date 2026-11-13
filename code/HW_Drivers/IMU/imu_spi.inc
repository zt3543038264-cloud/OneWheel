/* IMU-only bounded SPI receive path, using the initialized vendor BACON.
 * Common SeekFree SPI APIs remain unchanged. Every wait shares one 100 us
 * transaction deadline; failure releases CS and latches IMU loss upstream. */
#ifdef R10H_HOST_TEST
extern uint8 r10h_host_bus_read(uint8 reg,uint8 *data,uint8 len);
static uint8 r10h_bus_read(uint8 reg,uint8 *data,uint8 len)
{ return r10h_host_bus_read(reg,data,len); }
#else
#include "IfxQspi.h"
#include "IfxStm.h"
extern Ifx_QSPI_BACON bacon[4];
static uint32 r10h_bus_timeout_ticks;
static uint8 r10h_bus_read(uint8 reg,uint8 *data,uint8 len)
{
    Ifx_QSPI *bus=IfxQspi_getAddress((IfxQspi_Index)IMU660RB_SPI);
    uint32 start=IfxStm_getLower(&MODULE_STM0);
    uint8 j;
    if (!len || !r10h_bus_timeout_ticks) return 0;
    for (j=0;j<4u && bus->STATUS.B.RXFIFOLEVEL;++j) (void)IfxQspi_readReceiveFifo(bus);
    IfxQspi_clearAllEventFlags(bus);
    IMU660RB_CS(0);
    IfxQspi_writeBasicConfigurationBeginStream(bus,bacon[IMU660RB_SPI].U);
    IfxQspi_writeTransmitFifo(bus,(uint8)(reg | IMU660RB_SPI_R));
    while (!bus->STATUS.B.RXFIFOLEVEL)
        if ((uint32)(IfxStm_getLower(&MODULE_STM0)-start)>=r10h_bus_timeout_ticks) goto timeout;
    (void)IfxQspi_readReceiveFifo(bus);
    for (j=0;j<len;++j)
    {
        if (j+1u==len) IfxQspi_writeBasicConfigurationEndStream(bus,bacon[IMU660RB_SPI].U);
        IfxQspi_writeTransmitFifo(bus,0);
        while (!bus->STATUS.B.RXFIFOLEVEL)
            if ((uint32)(IfxStm_getLower(&MODULE_STM0)-start)>=r10h_bus_timeout_ticks) goto timeout;
        data[j]=(uint8)IfxQspi_readReceiveFifo(bus);
    }
    while (!bus->STATUS.B.PT1F)
        if ((uint32)(IfxStm_getLower(&MODULE_STM0)-start)>=r10h_bus_timeout_ticks) goto timeout;
    IfxQspi_clearAllEventFlags(bus);
    IMU660RB_CS(1);
    return 1;
timeout:
    imu_latch_fault("IMU SPI TIMEOUT");
    IMU660RB_CS(1);
    IfxQspi_clearAllEventFlags(bus);
    return 0; /* No bus reset or retry while the vehicle can be moving. */
}
#endif
