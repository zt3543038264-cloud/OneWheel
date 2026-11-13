"""Host-compile actual parameter implementation. No device access."""
from pathlib import Path
import subprocess, tempfile
ROOT = Path(__file__).resolve().parents[2]
STUB = r'''
#ifndef STUB_H
#define STUB_H
#include <stdint.h>
#include <string.h>
#include <math.h>
typedef uint32_t uint32; typedef int32_t int32; typedef uint16_t uint16;
typedef uint8_t uint8;
typedef union {uint32 uint32_type; int32 int32_type; float float_type;} flash_data_union;
#define EEPROM_PAGE_LENGTH 1024
#define FLASH_STATUS_OK 0
extern flash_data_union flash_union_buffer[1024];
extern float g_roll_zero,g_pitch_zero;
int flash_last_status(void); uint8 flash_check(uint32,uint32);
void flash_erase_page(uint32,uint32); void flash_read_page_to_buffer(uint32,uint32);
uint8 flash_write_page_from_buffer(uint32,uint32); void flash_buffer_clear(void);
#endif
'''
HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include "Common/param.c"
param_t g_param; volatile uint32 g_param_revision;
float g_roll_zero,g_pitch_zero;
flash_data_union flash_union_buffer[1024], disk[1024];
static int io_error, fail_write, reads, writes;
int flash_last_status(void) {return io_error;}
uint8 flash_check(uint32 s,uint32 p) {unsigned i; assert(s==0 && p==11); reads++; for(i=0;i<1024;i++)if(disk[i].uint32_type)return 1;return 0;}
void flash_read_page_to_buffer(uint32 s,uint32 p){assert(s==0&&p==11);reads++;memcpy(flash_union_buffer,disk,sizeof disk);}
void flash_erase_page(uint32 s,uint32 p){assert(s==0&&p==11);writes++;if(!io_error)memset(disk,0,sizeof disk);}
uint8 flash_write_page_from_buffer(uint32 s,uint32 p){assert(s==0&&p==11);writes++;if(io_error)return 1;if(!fail_write)memcpy(disk,flash_union_buffer,sizeof disk);return 0;}
void flash_buffer_clear(void){memset(flash_union_buffer,0,sizeof flash_union_buffer);}
int main(void){
 param_init();
#if !STORAGE_FLASH_VALIDATED
 assert(reads==0 && writes==0 && param_storage_status()==PARAM_STORAGE_DISABLED);
 assert(!param_save() && !param_erase()); assert(reads==0 && writes==0);
#else
 assert(param_save()); g_param.r_rate_kp=123; param_init(); assert(g_param.r_rate_kp!=123);
 {int n=writes; disk[0].uint32_type=PARAM_MAGIC_LEGACY; param_init(); assert(param_storage_status()==PARAM_STORAGE_INCOMPATIBLE); assert(!param_save()&&!param_erase()&&writes==n);}
 {int n=writes; disk[0].uint32_type=PARAM_MAGIC_MASTER; param_init(); assert(param_storage_status()==PARAM_STORAGE_INCOMPATIBLE); assert(!param_save()&&!param_erase()&&writes==n);}
 memset(disk,0,sizeof disk); assert(param_save()); disk[4].uint32_type^=1;
 {int n=writes; assert(!param_save()&&!param_erase()&&writes==n); assert(param_storage_status()==PARAM_STORAGE_CORRUPT);}
 memset(disk,0,sizeof disk); assert(param_save());
 disk[2].uint32_type=0x12345678; {unsigned n=2+2*disk[1].uint32_type;disk[n].uint32_type=param_crc32_words((uint32*)disk,n);}
 {int n=writes; assert(!param_save()&&!param_erase()&&writes==n); assert(param_storage_status()==PARAM_STORAGE_INCOMPATIBLE);}
 memset(disk,0,sizeof disk); assert(param_save()); fail_write=1;g_param.r_rate_kp+=1;assert(!param_save());fail_write=0;
 io_error=1;{int n=writes;assert(!param_save()&&!param_erase()&&writes==n);}
#endif
 puts("PASS parameter storage compatibility, preservation, readback/error or default no-I/O gate");
 return 0;
}
'''
def run():
 with tempfile.TemporaryDirectory() as td:
  p=Path(td);(p/'Controller').mkdir();(p/'zf_common_headfile.h').write_text(STUB)
  (p/'Controller/control.h').write_text('#include "zf_common_headfile.h"\n')
  (p/'test.c').write_text(HARNESS)
  for enabled in (0,1):
   exe=p/f'test{enabled}'
   subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',f'-DSTORAGE_FLASH_VALIDATED={enabled}',f'-DSTORAGE_BOOT_LOAD={enabled}','-I',str(p),'-I',str(ROOT/'code'),str(p/'test.c'),'-lm','-o',str(exe)],check=True)
   subprocess.run([str(exe)],check=True)
if __name__=='__main__':run()
