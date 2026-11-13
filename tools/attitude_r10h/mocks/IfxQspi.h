#ifndef HOST_QSPI_H
#define HOST_QSPI_H
#include <stdint.h>
typedef struct { uint32_t U; } Ifx_QSPI_BACON;
typedef int IfxQspi_Index;
typedef struct { struct { struct { unsigned RXFIFOLEVEL,PT1F; } B; } STATUS; } Ifx_QSPI;
extern Ifx_QSPI host_qspi;
Ifx_QSPI *IfxQspi_getAddress(IfxQspi_Index);
void IfxQspi_clearAllEventFlags(Ifx_QSPI *);
void IfxQspi_writeBasicConfigurationBeginStream(Ifx_QSPI *,uint32_t);
void IfxQspi_writeBasicConfigurationEndStream(Ifx_QSPI *,uint32_t);
void IfxQspi_writeTransmitFifo(Ifx_QSPI *,uint32_t);
uint32_t IfxQspi_readReceiveFifo(Ifx_QSPI *);
#endif
