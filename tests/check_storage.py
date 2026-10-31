"""Compact Flash serialization, power-loss rejection and readback verification."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
source=(root/'code/flash.c').read_text('utf-8')
payload=source[source.index('#define BALANCE_STORAGE_MAGIC'):]
prefix=r'''
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include "config.h"
typedef uint32_t uint32;
static struct {float balance_angle_x,balance_angle_y;} mono_params;
static float roll_rate_test_kp,roll_rate_test_limit,roll_rate_test_sign,pitch_balance_kp,pitch_balance_rate_limit;
static float ground_rate_test_kp,ground_rate_test_limit,ground_rate_test_sign,ground_angle_kp,ground_angle_rate_limit;
static uint32 control_startup_ramp_ms;
static uint32 memory[16];static unsigned writes,erases,last_page;static int dropped=-1,irq;
#define my_flash_read(sector,page,type) memory[page]
static uint32 interrupt_global_disable(void){assert(!irq);irq=1;return 0;}
static void interrupt_global_enable(uint32 n){assert(!n && irq);irq=0;}
static void eeprom_erase_sector(uint32 n){assert(!n && !irq);memset(memory,0xff,sizeof(memory));++erases;}
static void eeprom_page_program(uint32 n,uint32 p,uint32 *v){assert(!n && p<15 && !irq);if((int)p!=dropped)memory[p]=*v;++writes;last_page=p;}
static void IfxFlash_resetToRead(int n){assert(!n);}
static void balance_control_defaults(void){
 mono_params.balance_angle_x=BALANCE_ZERO_X_DEG;mono_params.balance_angle_y=BALANCE_ZERO_Y_DEG;
 roll_rate_test_kp=FLY_RATE_KP;roll_rate_test_limit=FLY_PWM_LIMIT;roll_rate_test_sign=FLY_OUTPUT_SIGN;
 pitch_balance_kp=FLY_ANGLE_KP;pitch_balance_rate_limit=FLY_RATE_LIMIT_DPS;
 ground_rate_test_kp=GROUND_RATE_KP;ground_rate_test_limit=GROUND_PWM_LIMIT;ground_rate_test_sign=GROUND_OUTPUT_SIGN;
 ground_angle_kp=GROUND_ANGLE_KP;ground_angle_rate_limit=GROUND_RATE_LIMIT_DPS;control_startup_ramp_ms=BALANCE_STARTUP_RAMP_MS;
}
'''
main=r'''
static void repair_checksum(void){uint32 crc=BALANCE_STORAGE_MAGIC;for(unsigned i=1;i<=13;++i)crc=checksum_word(crc,memory[i]);memory[14]=crc;}
int main(void){
 get_flash();assert(ground_rate_test_kp==GROUND_RATE_KP);
 ground_rate_test_kp=42;mono_params.balance_angle_y=1.5f;control_startup_ramp_ms=600;
 assert(write_flash() && writes==15 && last_page==0 && erases==1 && !irq);
 ground_rate_test_kp=1;get_flash();assert(ground_rate_test_kp==42 && mono_params.balance_angle_y==1.5f && control_startup_ramp_ms==600);
 memory[6]^=1;get_flash();assert(ground_rate_test_kp==GROUND_RATE_KP);
 ground_rate_test_sign=0;unsigned old=erases;assert(!write_flash() && erases==old);
 balance_control_defaults();dropped=6;assert(!write_flash());get_flash();assert(ground_rate_test_kp==GROUND_RATE_KP);
 dropped=0;ground_rate_test_kp=45;assert(!write_flash());get_flash();assert(ground_rate_test_kp==GROUND_RATE_KP);
 dropped=-1;assert(write_flash());float bad=501;memcpy(&memory[6],&bad,4);repair_checksum();get_flash();assert(ground_rate_test_kp==GROUND_RATE_KP);
 bad=NAN;memcpy(&memory[6],&bad,4);repair_checksum();get_flash();assert(ground_rate_test_kp==GROUND_RATE_KP);
 assert(write_flash());memory[13]=5001;repair_checksum();get_flash();assert(control_startup_ramp_ms==BALANCE_STARTUP_RAMP_MS);
 balance_control_defaults();assert(pitch_balance_rate_limit==FLY_RATE_LIMIT_DPS && roll_rate_test_limit==8000);
 assert(roll_rate_test_kp==FLY_RATE_KP && pitch_balance_kp==FLY_ANGLE_KP && ground_rate_test_kp==GROUND_RATE_KP && mono_params.balance_angle_y==BALANCE_ZERO_Y_DEG && mono_params.balance_angle_x==BALANCE_ZERO_X_DEG && ground_rate_test_limit==6000);
 pitch_balance_rate_limit=ground_angle_rate_limit=200;roll_rate_test_limit=ground_rate_test_limit=8000;
 assert(write_flash());balance_control_defaults();get_flash();
 assert(pitch_balance_rate_limit==200 && ground_angle_rate_limit==200 && roll_rate_test_limit==8000 && ground_rate_test_limit==8000);
 old=erases;pitch_balance_rate_limit=200.1f;assert(!write_flash() && erases==old);
 pitch_balance_rate_limit=200;ground_rate_test_limit=8000.1f;assert(!write_flash() && erases==old);
 puts("Flash: roundtrip, marker-last, checksum, partial writes, finite/range guards and readback PASS");return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-flash-') as d:
    p=Path(d);test=p/'flash_test.c';test.write_text(prefix+payload+main,encoding='utf-8');exe=p/'flash.exe'
    subprocess.run(['gcc','-std=c99','-DQCAR_BOOT_LOAD_FLASH=1','-I'+str(root/'code'),str(test),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
