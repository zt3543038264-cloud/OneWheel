"""Compile the actual flash_read dispatch branch with isolated transport stubs."""
from pathlib import Path
import tempfile,subprocess
ROOT=Path(__file__).resolve().parents[2]
s=(ROOT/'code/Protocols/vofa.c').read_text();start=s.index('    if (strcmp(fields[0], "flash_read")');end=s.index('    else if (strcmp(fields[0], "hello")',start);branch=s[start:end]
source=r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t uint8;typedef uint16_t uint16;typedef uint32_t uint32;
#define VOFA_CMD_FORMAT 0
static int stopped=1,pending,reads;static uint16 s_save_seq,s_schema_seq;static char reply[256];
static int cfg_stopped(void){return stopped;}static int control_ipm_pending(void){return pending;}
static int param_flash_read_words(uint16 offset,uint32*out,uint16 count){unsigned i;reads++;for(i=0;i<count;i++)out[i]=offset+i;return 1;}
static int cfg_reply(uint16 seq,const char*state,const char*msg){assert(seq==1);snprintf(reply,sizeof(reply),"%s,%s",state,msg);return 1;}
static int command(char*offset){char *fields[]={"flash_read","1",offset};uint8 count=3;uint16 seq=1;char *p;char payload[144];
'''+branch+r'''
return 1;}
int main(void){
assert(command("0"));assert(strstr(reply,"ok,flash_read,0,00000000"));
assert(command("1016"));assert(strstr(reply,"000003FF"));
{int n=reads;command("1017");command("-1");command("999999999999999999");command("");command("1");assert(reads==n);}
stopped=0;{int n=reads;command("0");assert(reads==n&&strstr(reply,"BUSY"));}stopped=1;
pending=1;{int n=reads;command("0");assert(reads==n);}pending=0;
s_save_seq=1;{int n=reads;command("0");assert(reads==n);}s_save_seq=0;
s_schema_seq=1;{int n=reads;command("0");assert(reads==n);}
puts("PASS actual read-only export branch, bounds and stopped/pending guards");return 0;}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);(p/'test.c').write_text(source);subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',str(p/'test.c'),'-o',str(p/'test')],check=True);subprocess.run([str(p/'test')],check=True)
