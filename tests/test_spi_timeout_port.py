"""Exercise the imported new-branch SPI helper without target memory/hardware."""
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]

def test_imported_runtime_spi_deadline(tmp_path):
    src=(ROOT/'libraries/zf_driver/zf_driver_spi.c').read_bytes().decode('latin1')
    a=src.index('uint8 spi_transfer_8bit_timeout(');b=src.index('\n}',a)+2
    stub=r'''
#include <stdint.h>
#include <assert.h>
typedef uint8_t uint8; typedef uint32_t uint32; typedef int spi_index_enum; typedef int IfxQspi_Index;
typedef struct { struct {struct {uint32 RXFIFOLEVEL,PT1F;}B;} STATUS;} Ifx_QSPI;
static Ifx_QSPI module;static struct{uint32 U;}bacon[1];static int MODULE_STM0;
static uint32 ticks,step,rx,clears;
static Ifx_QSPI *IfxQspi_getAddress(int n){assert(n==0);return &module;}
static uint32 IfxStm_getLower(void *p){(void)p;ticks+=step;return ticks;}
static float IfxStm_getFrequency(void *p){(void)p;return 1000000.0f;}
static void spi_clear_fifo(volatile Ifx_QSPI *p){(void)p;++clears;}
static void IfxQspi_clearAllEventFlags(volatile Ifx_QSPI *p){(void)p;}
static void IfxQspi_writeBasicConfigurationBeginStream(volatile Ifx_QSPI *p,uint32 v){(void)p;(void)v;}
static void IfxQspi_writeBasicConfigurationEndStream(volatile Ifx_QSPI *p,uint32 v){(void)p;(void)v;}
static void IfxQspi_writeTransmitFifo(volatile Ifx_QSPI *p,uint8 v){(void)p;(void)v;}
static uint32 IfxQspi_readReceiveFifo(volatile Ifx_QSPI *p){(void)p;return ++rx;}
'''
    test=r'''
int main(void){
 uint8 in[4]={1,2,3,4},out[4]={0};
 module.STATUS.B.RXFIFOLEVEL=1;module.STATUS.B.PT1F=1;step=10;
 assert(spi_transfer_8bit_timeout(0,in,out,4,1)==0);assert(out[0]==1&&out[3]==4);
 assert(spi_transfer_8bit_timeout(0,in,out,0,1)==0);
 assert(spi_transfer_8bit_timeout(0,in,out,4,0)==1);
 assert(spi_transfer_8bit_timeout(0,in,out,4,1001)==1);
 module.STATUS.B.RXFIFOLEVEL=0;step=250;ticks=0xfffffff0u;
 assert(spi_transfer_8bit_timeout(0,in,out,4,1)==1);
 module.STATUS.B.RXFIFOLEVEL=1;module.STATUS.B.PT1F=0;
 assert(spi_transfer_8bit_timeout(0,in,out,4,1)==1);assert(clears>=5);
 return 0;
}
'''
    p=tmp_path/'spi.c';p.write_text(stub+src[a:b]+test)
    exe=tmp_path/'spi'
    x=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',str(p),'-o',str(exe)],capture_output=True,text=True)
    assert x.returncode==0,x.stderr
    x=subprocess.run([str(exe)],capture_output=True,text=True,timeout=5)
    assert x.returncode==0,x.stderr
