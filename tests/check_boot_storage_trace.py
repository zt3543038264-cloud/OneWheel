"""Compile actual application startup and verify trace does not reorder startup."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
source=(root/'code/app.c').read_text().replace('#include "zf_common_headfile.h"','').replace('#include "app.h"','')
prefix=r'''
#include <assert.h>
#include <stdio.h>
typedef unsigned char uint8;typedef unsigned uint32;
#define IPS200_PORTAIT 0
#define RGB565_WHITE 1
#define RGB565_BLACK 0
#define IPS200_TYPE_SPI 0
#define IPS200_6X8_FONT 0
static int calls[64],count;
static void record(int n){calls[count++]=n;}
static void clock_init(void){record(1);}
static void debug_init(void){record(2);}
static void ips200_set_dir(int n){(void)n;record(3);}
static void ips200_set_color(int n,int m){(void)n;(void)m;record(4);}
static void ips200_init(int n){(void)n;record(5);}
static void ips200_set_font(int n){(void)n;record(6);}
static void ips200_show_string(int x,int y,const char *s){(void)x;(void)y;(void)s;record(7);}
static void menu_init(void){record(8);}
static void cpu_init_notify(void){record(9);}
static void interrupt_global_enable(int n){assert(n==0);record(10);}
static void menu_hardware_init(void){record(11);}
static void menu_poll(void){record(12);}
static void storage_boot_mark(uint32 stage,const char *label){assert(label);record(stage);}
'''
main=r'''
int main(void){
 const int expected[]={1,2,3,4,5,6,7,8,100,101,9,102,10,103,11,118,119,12,120,12};
 app_init();app_poll();app_poll();assert(count==sizeof(expected)/sizeof(expected[0]));
 for(unsigned i=0;i<sizeof(expected)/sizeof(expected[0]);++i)assert(calls[i]==expected[i]);
 puts("Boot trace: actual app startup order and one-time poll breadcrumbs PASS");
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-boot-trace-') as d:
 p=Path(d);c=p/'test.c';exe=p/'test';c.write_text(prefix+source+main)
 subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',str(c),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
