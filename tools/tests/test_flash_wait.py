"""Compile driver implementation with host registers; never access device memory."""
from pathlib import Path
import re, subprocess, tempfile
root=Path(__file__).resolve().parents[2]
s=(root/'libraries/zf_driver/zf_driver_flash.c').read_bytes().decode('latin1')
s=s[s.index('/* CPU0 foreground'):]
s=re.sub(r'^#include .*$', '', s, flags=re.M)
stub=r'''
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
typedef uint32_t uint32;typedef uint16_t uint16;typedef uint8_t uint8;
typedef union {uint32 uint32_type;float float_type;} flash_data_union;
typedef enum {FLASH_STATUS_OK,FLASH_STATUS_ARGUMENT,FLASH_STATUS_BUSY,FLASH_STATUS_TIMEOUT,FLASH_STATUS_HARDWARE,FLASH_STATUS_VERIFY,FLASH_STATUS_DISABLED} flash_status_t;
#define EEPROM_PAGE_LENGTH 1024
#define EEPROM_PAGE_NUM 12
#define EEPROM_PAGE_SIZE 8192
#define FLASH_DATA_SIZE 8
static struct {uint32 U;} FLASH0_FSR;
static int MODULE_STM0;
static struct {uint32 start;} IfxFlash_dFlashTableEepLog[12];
static uint32 tick, advance, sync_error;
static uint32 IfxStm_getLower(void *p){(void)p;tick+=advance;return tick;}
static float IfxStm_getFrequency(void *p){(void)p;return 1000000;}
static void __dsync(void){FLASH0_FSR.U |= sync_error;}
static uint32 interrupt_global_disable(void){return 1;}
static void interrupt_global_enable(uint32 x){(void)x;}
static void IfxFlash_resetToRead(uint32 x){(void)x;}
static void IfxFlash_clearStatus(uint32 x){(void)x;}
static uint16 IfxScuWdt_getSafetyWatchdogPassword(void){return 0;}
static void IfxScuWdt_clearSafetyEndinit(uint16 x){(void)x;}
static void IfxScuWdt_setSafetyEndinit(uint16 x){(void)x;}
static void IfxFlash_eraseSector(uint32 x){(void)x;}
static uint32 IfxFlash_enterPageMode(uint32 x){(void)x;return 0;}
static void IfxFlash_loadPage(uint32 x,uint32 y,uint32 z){(void)x;(void)y;(void)z;}
static void IfxFlash_writePage(uint32 x){(void)x;}
'''
test=r'''
int main(void){
#if STORAGE_FLASH_VALIDATED
 FLASH0_FSR.U=0;assert(wait_status(BUSY_MASK,0,10));
 FLASH0_FSR.U=PAGE_MASK;assert(wait_status(PAGE_MASK,PAGE_MASK,10));
 FLASH0_FSR.U=BUSY_MASK;advance=1000;assert(!wait_status(BUSY_MASK,0,10));assert(s_fault&&s_status==FLASH_STATUS_TIMEOUT);
 FLASH0_FSR.U=0;assert(!ready(0,11)); /* timeout cannot be cleared by a subsequent read */
 s_fault=0;advance=0;assert(!wait_status(PAGE_MASK,PAGE_MASK,10)); /* stalled STM still bounded */
 {const unsigned bits[]={11,12,13,18,19,20,21,22,25,26};
  for(unsigned i=0;i<sizeof(bits)/sizeof(bits[0]);i++) {
   s_fault=0;FLASH0_FSR.U=1u<<bits[i];
   assert(!ready(0,11) && s_fault && s_status==FLASH_STATUS_HARDWARE);
   s_fault=0;assert(!wait_status(BUSY_MASK,0,10));assert(s_fault && s_status==FLASH_STATUS_HARDWARE);
   s_fault=0;FLASH0_FSR.U=0;sync_error=1u<<bits[i];
   assert(!wait_status(BUSY_MASK,0,10));assert(s_fault && s_status==FLASH_STATUS_HARDWARE);
   sync_error=0;
  }
 }
 s_fault=0;FLASH0_FSR.U=0;assert(!ready(1,0)&&!ready(0,12));
#else
 assert(!ready(0,11));assert(s_status==FLASH_STATUS_DISABLED);
 flash_erase_page(0,11);assert(s_status==FLASH_STATUS_DISABLED);
#endif
 puts("PASS bounded driver waits, timeout latch, errors, bounds or disabled gate");return 0;
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);(p/'test.c').write_text(stub+s+test)
 for flag in (0,1):
  subprocess.run(['gcc','-std=c99','-O2','-Wall','-Wextra','-Werror','-Wno-int-to-pointer-cast',f'-DSTORAGE_FLASH_VALIDATED={flag}',str(p/'test.c'),'-o',str(p/'test')],check=True)
  subprocess.run([str(p/'test')],check=True)
