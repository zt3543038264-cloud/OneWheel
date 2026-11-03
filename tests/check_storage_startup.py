"""Compile actual startup and verify safe initialization without diagnostics."""
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
'''
main=r'''
int main(void){
 const int expected[]={1,2,3,4,5,6,7,8,9,10,11,12,12};
 app_init();app_poll();app_poll();assert(count==sizeof(expected)/sizeof(expected[0]));
 for(unsigned i=0;i<sizeof(expected)/sizeof(expected[0]);++i)assert(calls[i]==expected[i]);
 puts("Startup: actual app order, foreground polling and no trace hooks PASS");
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-boot-trace-') as d:
 p=Path(d);c=p/'test.c';exe=p/'test';c.write_text(prefix+source+main)
 subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',str(c),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)

menu=(root/'code/menu.c').read_text()
start=menu.index('void menu_hardware_init(void)')
end=menu.index('static void menu_auto_wifi_service',start)
hardware=menu[start:end]
hardware_prefix=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "config.h"
typedef unsigned uint32;
#define CCU60_CH1 1
static int stops,wheels,driver_inits,motor_zeros,driver_zeros,imu_inits,timers;
static bool driver_ready,control_timers_ready;
static uint32 auto_wifi_per_ms;
static const char *message;
static int MODULE_STM0;
static void stop_car(void){++stops;}
static void ensure_wheel(void){++wheels;}
static void motor_m_ctrl(int n){assert(n==0);++motor_zeros;}
static void small_driver_uart_init(void){++driver_inits;}
static void small_driver_set_duty(int l,int r){assert(!l && !r);++driver_zeros;}
static int ensure_imu(void){++imu_inits;return 1;}
static void pit_init(int timer,unsigned period){assert(timer==CCU60_CH1 && period==APP_IMU_PERIOD_US);++timers;}
static float IfxStm_getFrequency(int *stm){assert(stm==&MODULE_STM0);return 30000000;}
'''
hardware_main=r'''
int main(void){
 menu_hardware_init();menu_hardware_init();
 assert(stops==2 && wheels==2 && driver_inits==1 && motor_zeros==2 && driver_zeros==2);
 assert(imu_inits==2 && timers==2 && control_timers_ready && auto_wifi_per_ms==30000 && message);
 puts("Startup: actual hardware init keeps outputs zero, only IMU timer, no automatic Start PASS");
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-safe-startup-') as d:
 p=Path(d);c=p/'test.c';exe=p/'test';c.write_text(hardware_prefix+hardware+hardware_main)
 subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(root/'code'),str(c),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
# Ensure diagnostic entry points and screen text are absent from production.
for name in ('app.c','menu.c','flash.c','flash.h'):
 text=(root/'code'/name).read_text()
 for forbidden in ('storage_diag_', 'storage_boot_mark', 'storage_probe_read', 'storage_check_record', 'Probe read', 'Check record', 'SAVE:', 'BOOT:'):
  assert forbidden not in text,(name,forbidden)
