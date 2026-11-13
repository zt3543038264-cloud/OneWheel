"""Real parameter/path/Flash sources against an 8 KiB-per-sector host model.

No device connection. GCC non-PIE keeps mock addresses in the TC26B uint32 range.
The register model cannot prove silicon ECC/trap, ENDINIT, or interrupt timing.
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HEADER = r'''
#ifndef FLASH_HOST_H
#define FLASH_HOST_H
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>
typedef uint32_t uint32; typedef int32_t int32; typedef uint16_t uint16;
typedef uint8_t uint8;
typedef union {uint32 uint32_type; int32 int32_type; float float_type;} flash_data_union;
typedef enum {FLASH_STATUS_OK,FLASH_STATUS_ARGUMENT,FLASH_STATUS_BUSY,
 FLASH_STATUS_TIMEOUT,FLASH_STATUS_HARDWARE,FLASH_STATUS_VERIFY,FLASH_STATUS_DISABLED} flash_status_t;
#define EEPROM_PAGE_LENGTH 1024u
#define EEPROM_PAGE_NUM 12u
#define EEPROM_PAGE_SIZE 8192u
#define FLASH_DATA_SIZE 8u
extern flash_data_union flash_union_buffer[EEPROM_PAGE_LENGTH];
extern float g_roll_zero,g_pitch_zero;
flash_status_t flash_last_status(void);
uint8 flash_check(uint32,uint32);
void flash_read_page(uint32,uint32,uint32 *,uint16);
void flash_read_page_to_buffer(uint32,uint32);
void flash_erase_page(uint32,uint32);
void flash_write_page(uint32,uint32,const uint32 *,uint16);
uint8 flash_write_page_from_buffer(uint32,uint32);
void flash_buffer_clear(void);
int control_path_storage_ready(void);
extern uint32 mock_flash[12][2048];
extern unsigned mock_erases,mock_writes,mock_syncs,mock_ecc_after_sync;
extern uint32 mock_ecc_bit, mock_erase_ecc, mock_write_ecc;
extern struct mock_fsr {uint32 U;} FLASH0_FSR;
void mock_reset(void);
#endif
'''
DRIVER_STUB = r'''
#include "zf_common_headfile.h"
#include "Common/storage_profile.h"
uint32 mock_flash[12][2048];
unsigned mock_erases,mock_writes,mock_syncs,mock_ecc_after_sync;
uint32 mock_ecc_bit,mock_erase_ecc,mock_write_ecc;
struct mock_fsr FLASH0_FSR;
static int MODULE_STM0;
static struct {uint32 start;} IfxFlash_dFlashTableEepLog[12];
static uint32 tick, loaded_low,loaded_high;
static uint32 IfxStm_getLower(void *p){(void)p;return ++tick;}
static float IfxStm_getFrequency(void *p){(void)p;return 1000000;}
static void __dsync(void){++mock_syncs;if(mock_ecc_after_sync && !--mock_ecc_after_sync)FLASH0_FSR.U|=mock_ecc_bit;}
static uint32 interrupt_global_disable(void){return 1;}
static void interrupt_global_enable(uint32 x){(void)x;}
static void IfxFlash_resetToRead(uint32 x){(void)x;}
static void IfxFlash_clearStatus(uint32 x){(void)x;FLASH0_FSR.U=0;}
static uint16 IfxScuWdt_getSafetyWatchdogPassword(void){return 0;}
static void IfxScuWdt_clearSafetyEndinit(uint16 x){(void)x;}
static void IfxScuWdt_setSafetyEndinit(uint16 x){(void)x;}
static void IfxFlash_eraseSector(uint32 x){++mock_erases;memset((void *)(uintptr_t)x,0,8192);FLASH0_FSR.U=0;if(mock_erase_ecc){mock_ecc_after_sync=2;mock_ecc_bit=mock_erase_ecc;}}
static uint32 IfxFlash_enterPageMode(uint32 x){(void)x;FLASH0_FSR.U=1u<<10;return 0;}
static void IfxFlash_loadPage(uint32 x,uint32 y,uint32 z){(void)x;loaded_low=y;loaded_high=z;}
static void IfxFlash_writePage(uint32 x){uint32 *p=(uint32 *)(uintptr_t)x;++mock_writes;p[0]=loaded_low;p[1]=loaded_high;FLASH0_FSR.U=0;if(mock_write_ecc){mock_ecc_after_sync=2;mock_ecc_bit=mock_write_ecc;}}
'''
DRIVER_RESET = r'''
void mock_reset(void) {
 unsigned i;assert((uintptr_t)&mock_flash[11][2047]<=UINT32_MAX);
 memset(mock_flash,0,sizeof(mock_flash));
 for(i=0;i<12;i++)IfxFlash_dFlashTableEepLog[i].start=(uint32)(uintptr_t)mock_flash[i];
 FLASH0_FSR.U=0;s_fault=0;s_status=FLASH_STATUS_OK;tick=0;
 mock_erases=mock_writes=mock_syncs=mock_ecc_after_sync=0;
 mock_ecc_bit=mock_erase_ecc=mock_write_ecc=0;
}
'''
HARNESS = r'''
#include "zf_common_headfile.h"
#include "Common/param.c"
#include "Controller/Planner/path_store.h"
float g_roll_zero,g_pitch_zero;
int control_path_storage_ready(void){return 1;}
#if STORAGE_FLASH_VALIDATED
static uint32 snapshot[12][2048];
static void unchanged(unsigned erases,unsigned writes) {
 assert(mock_erases==erases && mock_writes==writes);
 assert(!memcmp(snapshot,mock_flash,sizeof(snapshot)));
}
static void test_ecc(void) {
 unsigned i,j;
 uint32 out[1024],value=0x12345678u;
 const unsigned bits[]={11,12,13,18,19,20,21,22,25,26};
 const unsigned reads[]={1,2,1999,2000,2047,2048};
 for(i=0;i<sizeof(bits)/sizeof(bits[0]);i++) {
  mock_reset();FLASH0_FSR.U=1u<<bits[i];
  assert(flash_check(0,11));assert(flash_last_status()==FLASH_STATUS_HARDWARE);
  assert(mock_syncs==0);FLASH0_FSR.U=0;
  flash_erase_page(0,11);flash_write_page(0,11,&value,1);
  assert(mock_erases==0 && mock_writes==0); /* hardware error is reboot-latched */
  for(j=0;j<sizeof(reads)/sizeof(reads[0]);j++) {
   mock_reset();memset(out,0xa5,sizeof(out));
   mock_ecc_bit=1u<<bits[i];mock_ecc_after_sync=reads[j];
   flash_read_page(0,11,out,1024);assert(flash_last_status()==FLASH_STATUS_HARDWARE);
   assert(out[(reads[j]-1)/2]==0xa5a5a5a5u); /* failing word is never published */
   FLASH0_FSR.U=0;flash_erase_page(0,11);assert(!mock_erases);
  }
 }
 /* The occupied fast path and final blank word must both check post-read ECC. */
 for(i=0;i<2;i++) {
  mock_reset();mock_flash[11][0]=i?0:1;
  mock_ecc_bit=1u<<21;mock_ecc_after_sync=i?2048:1;
  assert(flash_check(0,11));assert(flash_last_status()==FLASH_STATUS_HARDWARE);
 }
 mock_reset();mock_erase_ecc=1u<<21;flash_erase_page(0,11);
 assert(flash_last_status()==FLASH_STATUS_HARDWARE); /* not overwritten by VERIFY */
 mock_reset();mock_write_ecc=1u<<21;flash_write_page(0,11,&value,1);
 assert(mock_writes==1 && flash_last_status()==FLASH_STATUS_HARDWARE);
 mock_reset();mock_flash[11][2047]=value;
 memcpy(snapshot,mock_flash,sizeof(snapshot));flash_write_page(0,11,&value,1);
 assert(flash_last_status()==FLASH_STATUS_VERIFY);unchanged(0,0);
 puts("PASS real Flash: pre/post-read ECC, both halves, boundary reads, latch, verify status, no implicit erase");
}
static void test_param(void) {
 unsigned i,half,erases,writes;
 uint32 prefix;
 mock_reset();param_init();assert(param_save());prefix=3+2*mock_flash[11][2];
 /* Every low tail and every high half, not only the original slot-1000 repro. */
 for(half=0;half<2;half++) for(i=half?0:prefix;i<1024;i++) {
  mock_reset();param_init();assert(param_save());
  mock_flash[11][2*i+half]=0xdeadbeefu;
  memcpy(snapshot,mock_flash,sizeof(snapshot));erases=mock_erases;writes=mock_writes;
  assert(!param_save());unchanged(erases,writes);
  assert(!param_erase());unchanged(erases,writes);
  param_init();assert(param_storage_status()!=PARAM_STORAGE_OK);unchanged(erases,writes);
 }
 mock_reset();param_init();g_param.r_rate_kp=123;assert(param_save());
 param_init();assert(g_param.r_rate_kp==123 && param_storage_status()==PARAM_STORAGE_OK);
 assert(param_erase());assert(!flash_check(0,11));
 mock_reset();param_init();assert(param_save());
 memcpy(snapshot,mock_flash,sizeof(snapshot));erases=mock_erases;writes=mock_writes;
 /* header check consumes one sync; inject during later full-sector validation */
 mock_ecc_bit=1u<<21;mock_ecc_after_sync=2001;
 assert(!param_save());assert(param_storage_status()==PARAM_STORAGE_IO);unchanged(erases,writes);
 assert(!param_erase());unchanged(erases,writes);
 puts("PASS real parameter layer + driver: all low tails/high halves preserved, ECC blocks erase, valid ORA1 roundtrip");
}
static void route(float end) {
 path_point_t points[2]={{0,0,0},{0,end*.1f,end*.1f}};
 path_nav_init();assert(path_nav_import(points,2));
}
static void test_path(void) {
 unsigned page,i,half,erases,writes;
 /* Both destination slots, every tail low word and every high half. */
 for(page=9;page<=10;page++) for(half=0;half<2;half++) for(i=half?0:11;i<1024;i++) {
  mock_reset();route(1);assert(path_store_save());
  route(2);assert(path_store_save());
  if(page==10){route(3);assert(path_store_save());}
  mock_flash[page][2*i+half]=0xfeedbeefu;
  memcpy(snapshot,mock_flash,sizeof(snapshot));erases=mock_erases;writes=mock_writes;
  route(4);assert(!path_store_save());unchanged(erases,writes);
  assert(path_store_status()!=PATH_STORAGE_OK);
 }
 mock_reset();route(1);assert(path_store_save());route(2);assert(path_store_save());
 route(3);assert(path_store_save());path_store_init();
 {path_status_t s;path_nav_status(&s);assert(s.count==2 && fabsf(s.length_m-.3f)<.0001f);}
 path_nav_clear();assert(path_store_save());path_store_init();
 {path_status_t s;path_nav_status(&s);assert(s.count==0);}
 mock_reset();route(1);assert(path_store_save());route(2);assert(path_store_save());
 memcpy(snapshot,mock_flash,sizeof(snapshot));erases=mock_erases;writes=mock_writes;
 mock_ecc_bit=1u<<21;mock_ecc_after_sync=2001;
 route(3);assert(!path_store_save());assert(path_store_status()==PATH_STORAGE_IO);unchanged(erases,writes);
 puts("PASS real path layer + driver: both targets/all tails/high halves preserved, ECC blocks erase, ORP1 and tombstone roundtrip");
}
#endif
int main(void) {
 mock_reset();
#if STORAGE_FLASH_VALIDATED
 test_ecc();test_param();test_path();
#else
 {uint32 value=1,out=123;
 param_init();assert(!param_save()&&!param_erase());path_store_init();assert(!path_store_save()&&!path_store_load());
 flash_read_page(0,11,&out,1);assert(out==123);assert(flash_check(0,11));
 flash_erase_page(0,11);flash_write_page(0,11,&value,1);
 assert(flash_last_status()==FLASH_STATUS_DISABLED && !mock_syncs && !mock_erases && !mock_writes);}
 puts("PASS default-disabled real parameter/path/driver integration: zero memory or hardware I/O");
#endif
 return 0;
}
'''

def run():
    # Tie the mocked status positions to the bundled silicon register contract.
    register_bits = (ROOT/'libraries/infineon_libraries/Infra/Sfr/TC26B/_Reg/IfxFlash_bf.h').read_text()
    for name, bit in {'OPER':11,'SQER':12,'PROER':13,'DFSBER':18,'DFDBER':19,
                      'DFTBER':20,'DFMBER':21,'SRIADDERR':22,'PVER':25,'EVER':26}.items():
        assert re.search(r'#define IFX_FLASH_FSR_'+name+r'_OFF \('+str(bit)+r'u\)', register_bits), name
    driver = (ROOT/'libraries/zf_driver/zf_driver_flash.c').read_bytes().decode('latin1')
    driver = re.sub(r'^#include .*$', '', driver[driver.index('/* CPU0 foreground'):], flags=re.M)
    with tempfile.TemporaryDirectory(prefix='flash-preservation-') as td:
        p=Path(td);(p/'Controller').mkdir()
        (p/'zf_common_headfile.h').write_text(HEADER)
        (p/'Controller/control.h').write_text('#include "zf_common_headfile.h"\n')
        (p/'path_store_stubs.h').write_text('#include "zf_common_headfile.h"\n')
        (p/'driver.c').write_text(DRIVER_STUB+driver+DRIVER_RESET)
        (p/'test.c').write_text(HARNESS)
        for enabled in (0,1):
            exe=p/f'test{enabled}'
            flags = ['-DSTORAGE_FLASH_VALIDATED=1','-DSTORAGE_BOOT_LOAD=1'] if enabled else []
            subprocess.run(['gcc','-std=c99','-O2','-fno-pie','-no-pie','-Wall','-Wextra','-Werror',
                '-Wno-int-to-pointer-cast','-Wno-unknown-pragmas',*flags,
                '-DPATH_HOST_TEST','-DPATH_STORE_TEST',
                '-I'+str(p),'-I'+str(ROOT/'code'),str(p/'driver.c'),str(p/'test.c'),
                str(ROOT/'code/Controller/Planner/path_nav.c'),str(ROOT/'code/Controller/Planner/path_store.c'),
                '-lm','-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True,timeout=60)

if __name__=='__main__':
    run()
