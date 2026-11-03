"""Compile the actual low-level DFlash helpers against an asynchronous controller."""
from pathlib import Path
import subprocess, tempfile
root = Path(__file__).resolve().parents[1]
src = (root / 'code/flash.c').read_text()
src = src[src.index('/* The iLLD'):src.index('#define BALANCE_STORAGE_MAGIC')]
src = src.replace('#include "IfxStm.h"', '')
prefix = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
typedef uint32_t uint32; typedef uint16_t uint16;
#define QCAR_EEPROM_SECTOR_NUM 12u
#define QCAR_EEPROM_SECTOR_PAGE_NUM 1024u
#define IFXFLASH_DFLASH_PAGE_LENGTH 8u
static int MODULE_STM0;
#define __dsync() ((void)0)
static struct {uint32 U;} FLASH0_FSR;
static struct {uint32 start;} IfxFlash_dFlashTableEepLog[12]={{0xaf000000}};
static unsigned ticks, irq, unlocked, pending, mode, resets, loads, programs, clears;
/* mode: 1 busy forever, 2 page never ready, 3 protection, 4 entry failure,
 * 5 program verify error, 6 sequence error, 7 operation error, 8 erase verify */
static uint32 IfxStm_getLower(void *unused) {
 (void)unused; ticks++;
 if(pending && --pending==0) FLASH0_FSR.U=1024;
 return ticks;
}
static float IfxStm_getFrequency(void *unused){(void)unused;return 1000;}
static uint32 interrupt_global_disable(void){unsigned old=irq;irq=1;return old;}
static void interrupt_global_enable(uint32 old){assert(irq && !unlocked);irq=old;}
static uint16 IfxScuWdt_getSafetyWatchdogPassword(void){assert(irq);return 42;}
static void IfxScuWdt_clearSafetyEndinit(uint16 p){assert(irq && p==42);unlocked=1;}
static void IfxScuWdt_setSafetyEndinit(uint16 p){assert(irq && p==42 && unlocked);unlocked=0;}
static void IfxFlash_resetToRead(unsigned n){assert(!n && !(FLASH0_FSR.U&2));FLASH0_FSR.U=0;pending=0;resets++;}
static void IfxFlash_clearStatus(unsigned n){assert(!n);FLASH0_FSR.U=0;clears++;}
static void IfxFlash_eraseSector(uint32 addr){assert(irq && unlocked && addr==0xaf000000);FLASH0_FSR.U=mode==1?2:mode==3?8192:mode==8?(1u<<26):0;}
static unsigned IfxFlash_enterPageMode(uint32 addr){assert(irq && !unlocked && addr>=0xaf000000);if(mode==4)return 1;if(mode!=2)pending=3;return 0;}
static void IfxFlash_loadPage(uint32 addr,uint32 lo,uint32 hi){(void)addr;(void)lo;assert(irq && !unlocked && !hi && FLASH0_FSR.U==1024);loads++;}
static void IfxFlash_writePage(uint32 addr){(void)addr;assert(irq && unlocked);programs++;FLASH0_FSR.U=mode==1?2:mode==5?(1u<<25):mode==6?(1u<<12):mode==7?(1u<<11):0;}
'''
main = r'''
static void reset(unsigned m){mode=m;FLASH0_FSR.U=0;ticks=irq=unlocked=pending=resets=loads=programs=clears=0;}
int main(void){
 uint32 word=123;
 reset(0);assert(eeprom_erase_sector(0));assert(eeprom_page_program(0,1,&word));assert(loads==1 && programs==1 && ticks>=3 && !irq && !unlocked);
 reset(1);assert(!eeprom_erase_sector(0) && ticks<=1002 && !irq && !unlocked);assert(resets==1);assert(!eeprom_page_program(0,1,&word));assert(!programs);
 reset(1);assert(!eeprom_page_program(0,1,&word) && programs==1 && !irq && !unlocked && !resets);
 reset(2);assert(!eeprom_page_program(0,1,&word) && ticks<20 && !loads && !irq);
 reset(3);assert(!eeprom_erase_sector(0) && !irq && !unlocked);
 reset(4);assert(!eeprom_page_program(0,1,&word) && !loads && !irq);
 for(unsigned m=5;m<=7;m++){reset(m);assert(!eeprom_page_program(0,1,&word) && programs==1 && !irq && !unlocked);}
 reset(8);assert(!eeprom_erase_sector(0));
 reset(0);assert(!eeprom_erase_sector(12));assert(!eeprom_page_program(12,0,&word));assert(!eeprom_page_program(0,1024,&word));assert(!eeprom_page_program(0,0,0));assert(!loads && !programs);
 reset(0);ticks=UINT32_MAX-2;assert(eeprom_page_program(0,1,&word));
 reset(0);irq=1;assert(eeprom_erase_sector(0) && irq);assert(eeprom_page_program(0,1,&word) && irq);
 puts("DFlash: asynchronous page readiness, bounded waits, command errors, IRQ/ENDINIT restoration PASS");
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-flash-hw-') as d:
    p=Path(d);c=p/'test.c';exe=p/'test';c.write_text(prefix+src+main)
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',str(c),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)
