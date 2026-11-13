"""Execute actual camera boot/manual command paths, without LCD/camera hardware."""
from pathlib import Path
import re,subprocess
import pytest
ROOT=Path(__file__).resolve().parents[1]
def function(text,signature):
 a=text.index(signature);b=text.index('{',a);depth=1;i=b+1
 while depth:
  depth+=(text[i]=='{')-(text[i]=='}');i+=1
 return text[a:i]+'\n'
@pytest.mark.parametrize('auto',[0,1])
def test_boot_policy_does_not_disable_manual_retry(tmp_path,auto):
 s=(ROOT/'code/Estimation/Vision/vision_core.c').read_text(encoding='utf-8')
 source=r'''#include <stdint.h>
#include <assert.h>
#include <string.h>
typedef uint8_t uint8;typedef uint16_t uint16;
typedef struct {uint16 cam_exposure;} vision_feedback_t;
typedef enum {VISION_CMD_SET_EXPOSURE=1,VISION_CMD_RESTART_CAMERA,VISION_CMD_CALIB_IPM} vision_command_t;
enum {VISION_CORE_OFF,VISION_CORE_STARTING,VISION_CORE_READY,VISION_CORE_FAILED,VISION_DISPLAY_BIN_LINE=0};
static vision_feedback_t s_feedback_cache;
static int s_display_frame,s_frame_seq,s_ipm_calib_seq,s_ipm_collecting,s_ipm_collect_count,s_ipm_applied[8],s_command_seq,s_camera_exposure,s_heartbeat,s_display_request,s_display_ready,s_display_mode,s_core_state;
static float s_ipm_point_sum[4],s_ipm_near_sum,s_ipm_far_sum,s_ipm_ratio_sum;
static struct {int seq;} s_command_box;
static int calls,failed=1,pending,exposures;
static void vision_feedback_defaults(vision_feedback_t *x){x->cam_exposure=512;}
static int vision_feedback_read(vision_feedback_t *x){(void)x;return 0;}
static void image_init(void){}static void element_init(void){}
static int mt9v03x_init(void){calls++;return failed;}
static int mt9v03x_set_exposure_time(int v){assert(v>0);exposures++;return 0;}
static int vision_command_read(vision_command_t *c,uint16 *v){if(!pending)return 0;*c=(vision_command_t)pending;*v=300;pending=0;return 1;}
'''
 source+=function(s,'static uint8 vision_camera_start(void)')+function(s,'static void vision_command_service(void)')+function(s,'void vision_core_init(void)')
 source+=r'''
int main(void){
 vision_core_init();assert(calls==QCAR_CAMERA_AUTO_START);
 assert(s_core_state==(QCAR_CAMERA_AUTO_START?VISION_CORE_FAILED:VISION_CORE_OFF));
 pending=VISION_CMD_RESTART_CAMERA;vision_command_service();assert(calls==QCAR_CAMERA_AUTO_START+1&&s_core_state==VISION_CORE_FAILED);
 failed=0;pending=VISION_CMD_RESTART_CAMERA;vision_command_service();assert(calls==QCAR_CAMERA_AUTO_START+2&&s_core_state==VISION_CORE_READY&&exposures==1);
 pending=VISION_CMD_SET_EXPOSURE;vision_command_service();assert(s_camera_exposure==300&&exposures==2);
 pending=VISION_CMD_CALIB_IPM;vision_command_service();assert(s_ipm_collecting==1&&s_ipm_collect_count==0);
 return 0;}
'''
 p=tmp_path/'boot.c';p.write_text(source);exe=tmp_path/'boot.exe'
 r=subprocess.run(['gcc','-std=c99','-Wall','-Wextra',f'-DQCAR_CAMERA_AUTO_START={auto}',str(p),'-o',str(exe)],capture_output=True,text=True);assert r.returncode==0,r.stderr
 r=subprocess.run([str(exe)],capture_output=True,text=True,timeout=5);assert r.returncode==0,r.stdout+r.stderr
@pytest.mark.parametrize('logging',[0,1])
def test_cpu1_camera_failure_cannot_clear_cpu0_lcd_by_default(tmp_path,logging):
 header=(ROOT/'libraries/zf_device/zf_device_mt9v03x.h').read_bytes().decode('utf-8',errors='replace')
 guard=re.search(r'#ifndef MT9V03X_INIT_LOG_ENABLE.*?#endif',header,re.S).group()
 src=(ROOT/'libraries/zf_device/zf_device_mt9v03x.c').read_bytes().decode('utf-8',errors='replace')
 body=function(src,'uint8 mt9v03x_init (void)')
 p=tmp_path/'camera.c';p.write_text(guard+'\n#define zf_log(x,msg) CAMERA_LCD_ERROR(msg)\n'+body,encoding='utf-8')
 r=subprocess.run(['gcc','-E','-P',f'-DMT9V03X_INIT_LOG_ENABLE={logging}',str(p)],capture_output=True,text=True);assert r.returncode==0,r.stderr
 assert r.stdout.count('CAMERA_LCD_ERROR(')==(2 if logging else 0)
