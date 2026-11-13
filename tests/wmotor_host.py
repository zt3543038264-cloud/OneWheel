"""Hardware boundary for linked, real W_Motor.c host tests (not hardware proof)."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
HEADER = r'''#ifndef HOST_HW
#define HOST_HW
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
typedef uint8_t uint8; typedef uint16_t uint16; typedef uint32_t uint32; typedef uint64_t uint64;
typedef int16_t int16; typedef int32_t int32; typedef uint8_t boolean;
typedef struct {int dummy;} Ifx_SRC_SRCR;
extern struct host_uart {void *asclin;} uart3_handle;
extern int MODULE_STM0;
uint32 IfxStm_getOffset32Timer(void*); uint32 IfxStm_getLower(void*); float IfxStm_getFrequency(void*);
#define UART_3 3
#define UART3_TX_P15_6 6
#define UART3_RX_P15_7 7
#define func_limit_ab(x,l,h) ((x)<(l)?(l):((x)>(h)?(h):(x)))
uint32 interrupt_global_disable(void); void interrupt_global_enable(uint32);
void uart_init(int,uint32,int,int); void uart_rx_interrupt(int,int); uint8 uart_query_byte(int,uint8*);
int IfxAsclin_getTxFifoFillLevel(void*); void IfxAsclin_writeTxData(void*,uint8);
volatile Ifx_SRC_SRCR *IfxAsclin_getSrcPointerTx(void*); void IfxAsclin_enableTxFifoFillLevelFlag(void*,boolean);
void IfxSrc_enable(volatile Ifx_SRC_SRCR*); void IfxSrc_disable(volatile Ifx_SRC_SRCR*); void IfxAsclin_clearTxFifoFillLevelFlag(void*);
#endif
'''
HARDWARE = r'''#include "HW_Drivers/Motor/W_Motor.c"
#include <stdlib.h>
param_t g_param;
struct host_uart uart3_handle;
int MODULE_STM0;
static uint64 ticks;
static float clock_hz=1000000.0f;
static Ifx_SRC_SRCR src;
static int irq,tx_enabled;
static uint8 fifo[16],rx[512];
static unsigned fill,rn,ri,written;
static char wire[65536];
static unsigned wn;
uint32 IfxStm_getOffset32Timer(void *p){(void)p;return (uint32)(ticks>>32);}
uint32 IfxStm_getLower(void *p){(void)p;return (uint32)ticks;}
float IfxStm_getFrequency(void *p){(void)p;return clock_hz;}
uint32 interrupt_global_disable(void){int old=irq;irq=1;return old;}
void interrupt_global_enable(uint32 x){irq=(int)x;}
void uart_init(int n,uint32 baud,int tx,int r){assert(n==3&&baud==460800);(void)tx;(void)r;uart3_handle.asclin=&src;}
void uart_rx_interrupt(int n,int en){assert(n==3&&en);}
uint8 uart_query_byte(int n,uint8 *v){assert(n==3);if(ri==rn)return 0;*v=rx[ri++];return 1;}
int IfxAsclin_getTxFifoFillLevel(void *p){(void)p;return (int)fill;}
void IfxAsclin_writeTxData(void *p,uint8 v){(void)p;assert(fill<16&&irq);fifo[fill++]=v;++written;}
volatile Ifx_SRC_SRCR *IfxAsclin_getSrcPointerTx(void *p){(void)p;return &src;}
void IfxAsclin_enableTxFifoFillLevelFlag(void *p,boolean b){(void)p;(void)b;}
void IfxSrc_enable(volatile Ifx_SRC_SRCR *p){assert(p==&src);tx_enabled=1;}
void IfxSrc_disable(volatile Ifx_SRC_SRCR *p){assert(p==&src);tx_enabled=0;}
void IfxAsclin_clearTxFifoFillLevelFlag(void *p){(void)p;}
static void drain(void){for(unsigned i=0;i<10000;i++){
 if(fill){wire[wn++]=(char)fifo[0];memmove(fifo,fifo+1,--fill);assert(wn<sizeof(wire)-1);}
 else if(tx_enabled)W_Motor_TxHandler();else{wire[wn]=0;return;}
 }assert(!"TX did not finish");}
static void feed(const uint8 *b,unsigned n){assert(n<sizeof(rx));memcpy(rx,b,n);rn=n;ri=0;while(ri<rn)W_Motor_RxHandler();}
static void line(const char *s){feed((const uint8*)s,(unsigned)strlen(s));}
static void status(unsigned state,uint32 session,uint32 seq,unsigned reason){
 char b[90];snprintf(b,sizeof(b),"SAFE-STATUS,1,%u,%lu,%lu,%u,1\r\n",state,(unsigned long)session,(unsigned long)seq,reason);line(b);
}
static void advance(unsigned ms){ticks+=(uint64)ms*1000u;}
static void clearwire(void){drain();wn=0;wire[0]=0;}
static w_motor_bench_status_t snapshot(void){w_motor_bench_status_t s;W_Motor_BenchReadStatus(&s);return s;}
static void boot(void){
 fill=rn=ri=wn=written=0;irq=tx_enabled=0;ticks=0;clock_hz=1000000.0f;
 g_param.motor_dir_a=g_param.motor_dir_b=1;g_param.fly_slew=0;
 W_Motor_Init();assert(written<=16);drain();
}
'''

def compile_run(tmp_path, main, extra=()):
    (tmp_path / 'zf_common_headfile.h').write_text(HEADER)
    (tmp_path / 'IfxStm.h').write_text('#include \"zf_common_headfile.h\"\n')
    (tmp_path / 'test.c').write_text(HARDWARE + main)
    exe = tmp_path / 'driver.exe'
    command = ['gcc', '-std=c99', '-Wall', '-Wextra', '-Wno-unknown-pragmas',
               '-Wno-unused-function', '-Werror', *extra,
               '-I'+str(tmp_path), '-I'+str(ROOT/'code'), str(tmp_path/'test.c'), '-o', str(exe)]
    r = subprocess.run(command, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    r = subprocess.run([str(exe)], capture_output=True, text=True, timeout=10)
    assert r.returncode == 0, r.stdout + r.stderr
    return r.stdout
