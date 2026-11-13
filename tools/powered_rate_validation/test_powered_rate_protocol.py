"""Real latest commissioning parser -> real control.c -> mocked motor boundary.
Does not emulate UART transport, keep-log storage, or physical hardware.
"""
from pathlib import Path
import os
import subprocess
import re
ROOT=Path(os.environ.get("ONEWHEEL_PROJECT",str(Path(__file__).resolve().parents[2])))
SUPPORT=Path(__file__).resolve().parent/"support"

def function(text,name):
    match=re.search(r'^static[^\n]*\b'+name+r'\([^\n]*\)\s*\{',text,re.M)
    assert match,name
    end=text.index('{',match.start())+1;depth=1
    while depth:
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[match.start():end]

def test_latest_parser_actual_controller(tmp_path):
    control=(SUPPORT/'control_host.c').read_text().split('int main(void){')[0]
    parser=(ROOT/'code/Protocols/vofa_commission.inc').read_text()
    source=control+r'''
#include <stdlib.h>
#include <errno.h>
static char response[512];
static unsigned log_fields,log_result;
static uint32 log_session,log_sequence;
static void keep_log_fields(uint32 s,uint32 q){log_fields++;log_session=s;log_sequence=q;}
static void keep_log_result(uint8 result){log_result=result;}
static uint8 cfg_reply(uint16 seq,const char *status,const char *payload){
 int n=snprintf(response,sizeof(response),"rsp:%u,%s,%s\n",(unsigned)seq,status,payload);
 assert(n>0&&n<(int)sizeof(response));return 1;
}
static uint8 cmd_parse_float_strict(const char *s,float *out){
 char *end;float v;if(!s||!*s)return 0;errno=0;v=strtof(s,&end);
 if(*end||errno||!isfinite(v))return 0;*out=v;return 1;
}
'''
    for name in ('commission_u32','commission_c_reply','commission_c_blocked','commission_c_cfg'):
        source+=function(parser,name)+'\n'
    source+=r'''
static void command(char *wire){
 char text[192],*fields[12];uint8 count=0;char *part;
 assert(strlen(wire)<sizeof(text));strcpy(text,wire);
 for(part=strtok(text,",");part;part=strtok(NULL,",")){assert(count<12);fields[count++]=part;}
 assert(count>=2);response[0]=0;commission_c_cfg(fields,count,42);
}
static void command_start(const char *mode,float target,float kp,float ki,int duration,int limit,uint32 session){
 char wire[180];snprintf(wire,sizeof(wire),"cbench_start,42,%lu,%s,%.9g,%.9g,%.9g,%d,%d",(unsigned long)session,mode,(double)target,(double)kp,(double)ki,duration,limit);command(wire);
}
int main(void){
 uint32 session;control_c_bench_diag_t d;char wire[180];
 cb_ready();command("cbench_status,42");assert(strstr(response,"rsp:42,ok,cbench_status,1,0,"));
 for(int mode=4;mode<=5;mode++){
  cb_ready();session=cb_status().session;att.pitch_rate=1;
  command_start(mode==4?"powered_rate_pos":"powered_rate_neg",0,20,0,1000,200,session);
  assert(strstr(response,"rsp:42,ok,cbench_start,1,1,"));
  cb_ticks(12);d=cb_status();assert(d.session==session+1&&d.mode==mode);
  assert((mode==4&&d.applied==-20)||(mode==5&&d.applied==20));
  snprintf(wire,sizeof(wire),"cbench_keep,42,%lu,1",(unsigned long)d.session);command(wire);
  assert(strstr(response,"ok,cbench_keep")&&log_fields&&log_result==1&&log_session==d.session&&log_sequence==1);
  command(wire);assert(strstr(response,"err,CBENCH_BLOCKED")&&log_result==2);
  command("cbench_stop,42");assert(strstr(response,"ok,cbench_stop"));cb_quiet();
  command_start("powered_rate_pos",0,20,0,1000,200,session);
  assert(strstr(response,"err,CBENCH_BLOCKED")&&!control_c_bench_running());
 }
 cb_ready();session=cb_status().session;
 command_start("powered_rate_pos",1,20,0,1000,200,session);assert(strstr(response,"err,"));
 command_start("powered_rate_pos",0,20,1,1000,200,session);assert(strstr(response,"err,"));
 command_start("powered_rate_pos",0,20.1f,0,1000,200,session);assert(strstr(response,"err,"));
 command_start("powered_rate_pos",0,20,0,1001,200,session);assert(strstr(response,"err,"));
 command_start("powered_rate_pos",0,20,0,1000,201,session);assert(strstr(response,"err,"));
 command_start("signal_raw",1,0,0,100,100,session);assert(strstr(response,"err,CBENCH_BLOCKED"));
 command_start("signal_rate",0,20,0,100,100,session);assert(strstr(response,"err,CBENCH_BLOCKED"));
 command_start("powered_rate",0,20,0,100,100,session);assert(strstr(response,"err,BAD_MODE"));
 command("cbench_start,42,0,powered_rate_pos,nan,20,0,100,100");assert(strstr(response,"err,BAD_CBENCH"));
 command("cbench_start,42,0,powered_rate_pos,0,inf,0,100,100");assert(strstr(response,"err,BAD_CBENCH"));
 command("cbench_start,42,0,powered_rate_pos,0,20,0,100,100,extra");assert(strstr(response,"err,BAD_FORMAT"));
 command("cbench_keep,42,0,0");assert(strstr(response,"err,BAD_CBENCH")&&log_session==0&&log_sequence==0);
 assert(!control_c_bench_running());cb_quiet();
 assert(control_preview_start(TUNE_AXIS_PITCH,TUNE_RING_RATE));
 command_start("powered_rate_pos",0,20,0,100,100,cb_status().session);
 assert(strstr(response,"err,CBENCH_BLOCKED"));control_stop();cb_quiet();
 puts("PASS latest commissioning parser and real controller integrated; keep-log hooks preserved");return 0;
}
'''
    c=tmp_path/'protocol_control.c';c.write_text(source);exe=tmp_path/'protocol_control'
    cmd=['gcc','-std=c99','-O2','-ffunction-sections','-fdata-sections',
         '-fsanitize=undefined','-fno-sanitize-recover=all','-DHOST_C_BENCH_TEST=1',
         '-I'+str(SUPPORT),'-I'+str(ROOT/'code'),str(c),'-Wl,--gc-sections','-lm','-o',str(exe)]
    r=subprocess.run(cmd,capture_output=True,text=True);assert r.returncode==0,r.stderr
    r=subprocess.run([str(exe)],capture_output=True,text=True);assert r.returncode==0,r.stdout+r.stderr
    assert 'PASS latest commissioning parser' in r.stdout
