"""Production camera IPC, profile, raw capture, display ownership and pixels.

Peripheral calls are mocked; camera frames and LCD SPI transfers are not hardware
proof. The production algorithms/functions are compiled rather than reimplemented.
"""
import re
import subprocess

from test_lean_limits import ROOT, _compile_run, _function


def function(text, signature):
    start = text.index(signature)
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end] + '\n'


def camera_source():
    source = r'''
#define MT9V03X_W 188
#define MT9V03X_H 120
#define MT9V03X_FPS_DEF 100
#define QCAR_CAMERA_AUTO_START 0
#include "Estimation/Vision/vision_core.h"
#include "Estimation/Vision/image.h"
#include "Estimation/Vision/perspective.h"
#include "Menu/display.h"
#include <assert.h>
#include <math.h>
param_t g_param;
float g_vision_fps=50;
uint8 g_vision_ipm_ok;
uint16 g_vision_age_ms;
uint32 g_control_uptime_ms;
Image my_image;elem_action_t g_elem_action;island_t g_island;
float g_mid_error,g_track_lateral,g_track_heading,g_track_curvature,g_track_quality,g_direction_camera;
uint8 g_direction_valid;int g_err_front_row;
static uint8 img_gray[120][188],mt9v03x_image[120][188],mt9v03x_finish_flag;
static uint32 mt9v03x_vsync_count,mt9v03x_dma_count,mt9v03x_busy_drop_count;
static unsigned starts,exposures,edges,beacons,sets,init_failed,exposure_failed,last_fps,last_exposure;
static int ipm_ok;
static uint8 mt9v03x_init_profile(uint16 fps,uint16 exposure){starts++;last_fps=fps;last_exposure=exposure;return (uint8)init_failed;}
static uint8 mt9v03x_set_exposure_time(uint16 exposure){exposures++;last_exposure=exposure;return (uint8)exposure_failed;}
void image_init(void){memset(&my_image,0,sizeof(my_image));}
void element_init(void){memset(&g_elem_action,0,sizeof(g_elem_action));}
uint8 ipm_ready(void){return (uint8)ipm_ok;}
void ipm_clear(void){ipm_ok=0;}
uint8 ipm_load(const float h[9]){(void)h;return (uint8)ipm_ok;}
void ipm_store(float h[9]){memset(h,0,9*sizeof(float));}
uint8 ipm_calibrate(float a,float b,float c,float d,float e,float f,float g){(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g;return 0;}
ipm_pick_t image_ipm_pick_points(float p[4],int *rn,int *rf,float *ratio){memset(p,0,4*sizeof(float));*rn=*rf=0;*ratio=0;return IPM_PICK_NO_TRACK;}
void image_process_edges(uint8 learning){(void)learning;edges++;memset(my_image.image_two_value,255,sizeof(my_image.image_two_value));}
void image_process_finish(void){g_direction_valid=1;g_direction_camera=0;}
void element_set_motion(const element_motion_t *m){(void)m;}
void element_process(void){}
void beacon_process(const uint8_t *g,uint8_t (*b)[188],const beacon_config_t *c,beacon_result_t *r){
 memset(r,0,sizeof(*r));if(!g||!b||!c||!c->mode)return;beacons++;memset(b,255,120*188);r->valid=1;r->cx=80;r->cy=70;
}
void beacon_reset(void){}
/* Detector/control math is covered by test_beacon_follow; camera tests retain
 * a deterministic valid target without linking the large legacy classifier. */
void beacon_follow_update(const beacon_config_t *c,uint8_t running,beacon_result_t *r){
 (void)running;r->steer_valid=r->valid;r->follow_mid_x=c->mid_x;
 r->error_px=r->cx-c->mid_x;
}
const uint8 *image_gray_data(void){return &img_gray[0][0];}
uint8 param_set_by_name(const char *n,float v){assert(!strcmp(n,"cam_exposure"));sets++;g_param.cam_exposure=(int)v;return 1;}
'''
    image=(ROOT/'code/Estimation/Vision/image.c').read_text(encoding='utf-8')
    source += function(image, 'uint8 image_grab(void)')
    source += function(image, 'void image_copy_gray(uint8 *destination)')
    source += '#include "Estimation/Beacon/beacon_config.c"\n'
    core=(ROOT/'code/Estimation/Vision/vision_core.c').read_text(encoding='utf-8')
    source += re.sub(r'^#include[^\n]*\n', '', core, flags=re.M)
    source += r'''
#define IPS200_TYPE_SPI 1
#define IPS200_PORTAIT 0
#define IPS200_CROSSWISE_180 1
#define IPS200_8X16_FONT 0
#define IPS200_CS(x) ((void)(x))
#define zf_assert(x) assert(x)
#define RGB565_BLACK 0
#define RGB565_WHITE 65535
#define RGB565_RED 63488
#define RGB565_YELLOW 65504
#define RGB565_GREEN 2016
#define RGB565_BLUE 31
static int ips200_display_type=1,ips200_width_max=320,ips200_height_max=240;
static uint16 panel[240][320];static unsigned region_x,region_y,region_w,draws,points,lines;
static char text_rows[15][41];
static void ips200_set_font(int x){(void)x;}
static void ips200_set_color(int a,int b){(void)a;(void)b;}
static void ips200_show_string(int x,int y,const char *t){assert(x>=0&&y>=0&&y<=224);assert(strlen(t)<=40);snprintf(text_rows[y/16],41,"%s",t);}
static void ips200_set_dir(int d){(void)d;}
static void ips200_init(int t){(void)t;}
static void ips200_clear(void){memset(panel,0,sizeof(panel));}
static void ips200_draw_point(int x,int y,int c){assert(x<320&&y<240);panel[y][x]=(uint16)c;points++;}
static void ips200_draw_line(int x,int y,int xx,int yy,int c){assert(x<320&&y<240&&xx<320&&yy<240);(void)c;lines++;}
static void ips200_set_region(uint16 x,uint16 y,uint16 xx,uint16 yy){assert(xx<320&&yy<240);region_x=x;region_y=y;region_w=xx-x+1;draws++;}
static void ips200_write_16bit_data_array(const uint16 *a,uint32 n){assert(n==region_w);memcpy(&panel[region_y++][region_x],a,n*2);}
'''
    driver=(ROOT/'libraries/zf_device/zf_device_ips200.c').read_bytes().decode('utf-8',errors='replace')
    source += function(driver,'void ips200_show_gray_image (')
    display=(ROOT/'code/Menu/display.c').read_text(encoding='utf-8')
    source += re.sub(r'^#include[^\n]*\n', '', display, flags=re.M)
    source += r'''
static void feedback(uint16 exposure){
 vision_feedback_t f=s_feedback_cache;
 f.input_seq++;f.cam_exposure=exposure;vision_feedback_publish(&f);
}
static void frame(int value){
 memset(mt9v03x_image,value,sizeof(mt9v03x_image));mt9v03x_finish_flag=1;
 mt9v03x_vsync_count++;mt9v03x_dma_count++;g_control_uptime_ms+=20;vision_core_run();
}
static void boot(void){
 g_param.cam_exposure=48;vision_core_init();display_init();
 assert(vision_core_state()==VISION_CORE_OFF && !starts && !exposures);
}
'''
    return source


def test_fifo_capacity_wrap_and_start_plus_exposure_are_ordered(tmp_path):
    _compile_run(tmp_path,camera_source()+r'''
int main(void){
 boot();vision_command_t c;uint16 v;
 assert(vision_command_request(VISION_CMD_RESTART_CAMERA,0));
 assert(vision_command_request(VISION_CMD_SET_EXPOSURE,200));
 vision_core_run();assert(starts==1 && last_fps==100 && exposures==1);
 vision_core_run();assert(last_exposure==200 && exposures==2);
 assert(!vision_command_read(&c,&v));
 assert(!vision_command_request(VISION_CMD_NONE,0));assert(!vision_command_request(VISION_CMD_SET_EXPOSURE,3));
 assert(!vision_command_request(VISION_CMD_SET_PROFILE,2));
 for(unsigned i=0;i<8;i++)assert(vision_command_request(VISION_CMD_SET_EXPOSURE,(uint16)(100+i)));
 assert(!vision_command_request(VISION_CMD_RESTART_CAMERA,0));
 for(unsigned i=0;i<8;i++){assert(vision_command_read(&c,&v));assert(c==VISION_CMD_SET_EXPOSURE && v==100+i);}
 s_command_box.head=s_command_box.tail=0xfffffffcU;
 for(unsigned i=0;i<8;i++)assert(vision_command_request(VISION_CMD_SET_EXPOSURE,(uint16)(200+i)));
 assert(!vision_command_request(VISION_CMD_RESTART_CAMERA,0));
 for(unsigned i=0;i<8;i++){assert(vision_command_read(&c,&v));assert(v==200+i);}
 assert(s_command_box.head==s_command_box.tail);
 return 0;
}
''')


def test_no_frame_exposure_error_retry_and_latest_normal_edits(tmp_path):
    _compile_run(tmp_path,camera_source()+r'''
int main(void){
 boot();vision_camera_status_t st;
 assert(vision_command_request(VISION_CMD_RESTART_CAMERA,0));vision_core_run();assert(exposures==1);
 exposure_failed=1;feedback(180);vision_core_run();
 assert(vision_camera_status_read(&st) && st.exposure_state==VISION_EXPOSURE_FAILED);
 assert(st.exposure_requested==180 && st.exposure_applied==48 && !st.gray_valid);
 for(int i=0;i<50;i++)vision_core_run();assert(exposures==2); /* no retry storm */
 exposure_failed=0;assert(vision_command_request(VISION_CMD_SET_EXPOSURE,180));vision_core_run();
 assert(vision_camera_status_read(&st) && st.exposure_applied==180 && st.exposure_state==VISION_EXPOSURE_OK);
 /* Parameter edits coalesce even when no frame or queue slot is available. */
 for(int i=0;i<8;i++)assert(vision_command_request(VISION_CMD_CALIB_IPM,0));
 for(int i=200;i<230;i++)display_set_exposure((uint16)i);
 assert(g_param.cam_exposure==229 && !s_exposure_enqueue_failed);
 feedback((uint16)g_param.cam_exposure);vision_core_run();assert(last_exposure==229);
 return 0;
}
''')


def test_diag_profile_raw_path_limits_and_normal_restore(tmp_path):
    _compile_run(tmp_path,camera_source()+r'''
int main(void){
 boot();vision_camera_status_t st;vision_result_t r;
 assert(vision_command_request(VISION_CMD_SET_PROFILE,1));assert(!vision_camera_run_allowed());vision_core_run();
 assert(last_fps==50 && last_exposure==512 && g_param.cam_exposure==48);
 frame(180);assert(!edges&&!beacons);assert(vision_result_read(&r) && !r.track_valid && !r.run_active && r.speed_limit_mps==0);
 assert(!vision_camera_run_allowed());
 assert(vision_camera_status_read(&st) && st.gray_min==180 && st.gray_mean==180 && st.gray_max==180);
 display_image_page_enter();display_track_view(); /* first frame snapshot requested below */
 frame(180);display_track_view();assert(strstr(text_rows[1],"RAW50"));
 display_set_exposure(600);vision_core_run();assert(last_exposure==600 && g_param.cam_exposure==48);
 assert(vision_command_request(VISION_CMD_CALIB_IPM,0));vision_core_run();assert(!s_ipm_collecting);
 assert(vision_command_request(VISION_CMD_SET_PROFILE,0));vision_core_run();
 assert(last_fps==100 && last_exposure==48);frame(100);assert(edges && vision_camera_run_allowed());
 assert(vision_command_request(VISION_CMD_RESTART_CAMERA,0));assert(!vision_camera_run_allowed());
 return 0;
}
''')


def test_gray_pixels_snapshots_restart_marker_and_missing_snapshot_text(tmp_path):
    _compile_run(tmp_path,camera_source()+r'''
int main(void){
 boot();assert(vision_command_request(VISION_CMD_RESTART_CAMERA,0));vision_core_run();
 display_image_page_enter();s_mode=DISP_MODE_GRAY;vision_display_request(VISION_DISPLAY_GRAY);
 for(int y=0;y<120;y++)for(int x=0;x<188;x++)mt9v03x_image[y][x]=(uint8)(32+(x+y)%224);
 mt9v03x_finish_flag=1;vision_core_run();assert(!memcmp(img_gray,mt9v03x_image,sizeof(img_gray)));
 display_track_view();assert(draws==1 && s_lcd_frame_seq==1);
 for(int y=0;y<107;y++)for(int x=0;x<240;x++){
  uint8 v=img_gray[y*120/107][x*188/240];uint16 expected=((v>>3)<<11)|((v>>2)<<5)|(v>>3);
  assert(panel[74+y][40+x]==expected);
 }
 /* An old-generation snapshot is never drawn after a restart. */
 frame(200);assert(s_display_ready);unsigned before=draws;
 assert(vision_command_request(VISION_CMD_RESTART_CAMERA,0));vision_core_run();
 display_track_view();assert(draws==before && !s_display_ready);
 frame(100);display_track_view();assert(draws==before+1);
 /* Beacon point at row 70 is visible with search_stop_line=0. */
 s_mode=DISP_MODE_BIN_LINE;vision_display_request(VISION_DISPLAY_BIN_LINE);
 s_feedback_cache.beacon.mode=1;frame(100);before=points+lines;display_track_view();assert(points+lines>before);
 /* Empty capture remains black and is explicitly identified, not masked. */
 s_mode=DISP_MODE_GRAY;vision_display_request(VISION_DISPLAY_GRAY);frame(0);display_track_view();
 assert(strstr(text_rows[12],"ALL ZERO"));
 /* Result frames may advance while display snapshots are withheld. */
 s_view_has_frame=0;s_display_ready=0;image_page_draw_header();assert(strstr(text_rows[12],"WAIT DISPLAY"));
 return 0;
}
''')


def test_profile_initialization_restores_driver_defaults_on_success_and_failure(tmp_path):
    driver=(ROOT/'libraries/zf_device/zf_device_mt9v03x.c').read_text(encoding='utf-8')
    _compile_run(tmp_path,r'''
#include "zf_common_headfile.h"
#include <assert.h>
#define MT9V03X_FPS 3
#define MT9V03X_EXP_TIME 2
static int16 mt9v03x_set_confing_buffer[10][2];static uint8 failed;
static uint8 mt9v03x_init(void){assert(mt9v03x_set_confing_buffer[3][1]==50 && mt9v03x_set_confing_buffer[2][1]==512);return failed;}
'''+function(driver,'uint8 mt9v03x_init_profile(')+r'''
int main(void){
 mt9v03x_set_confing_buffer[3][1]=100;mt9v03x_set_confing_buffer[2][1]=48;
 for(failed=0;failed<=1;failed++){
  assert(mt9v03x_init_profile(50,512)==failed);
  assert(mt9v03x_set_confing_buffer[3][1]==100 && mt9v03x_set_confing_buffer[2][1]==48);
 }
 assert(mt9v03x_init_profile(0,512));assert(mt9v03x_init_profile(50,3));return 0;
}
''')


def test_manual_start_and_profile_controls_use_stopped_gate(tmp_path):
    control=(ROOT/'code/Controller/control.c').read_text(encoding='utf-8')
    source=camera_source()+r'''
#define START_STOP 0
#define MOTOR_JOG_NONE 0
static int start_flag,test,jog,path;
static uint8 g_cam_ok;
static uint8 control_test_running(void){return (uint8)test;}
static uint8 control_jog_running(void){return (uint8)jog;}
static uint8 path_nav_busy(void){return (uint8)path;}
'''+function(control,'uint8 control_camera_debug_start(void)')+function(control,'uint8 control_camera_profile_request(uint8 diagnostic)')+r'''
int main(void){
 boot();assert(!control_camera_debug_start());assert(s_command_box.head==1);
 vision_core_run();assert(starts==1 && control_camera_debug_start() && g_cam_ok);
 assert(s_command_box.head==s_command_box.tail); /* READY does not reinitialize */
 s_core_state=VISION_CORE_FAILED;assert(!control_camera_debug_start());vision_core_run();assert(starts==2);
 start_flag=1;assert(!control_camera_profile_request(1));start_flag=0;
 test=1;assert(!control_camera_profile_request(1));test=0;
 jog=1;assert(!control_camera_profile_request(1));jog=0;
 path=1;assert(!control_camera_profile_request(1));path=0;
 assert(!control_camera_profile_request(2));assert(control_camera_profile_request(1));vision_core_run();assert(last_fps==50);
 return 0;
}
'''
    _compile_run(tmp_path,source)


def test_preboot_request_and_frame_stats_rounding_and_mode_switch(tmp_path):
    _compile_run(tmp_path,camera_source()+r'''
int main(void){
 vision_camera_status_t st;
 assert(vision_command_request(VISION_CMD_RESTART_CAMERA,0));boot();vision_core_run();assert(starts==1);
 display_image_page_enter();frame(255);display_track_view();
 assert(vision_camera_status_read(&st) && st.gray_min==255 && st.gray_mean==255 && st.gray_max==255);
 unsigned n=draws;frame(123);display_next_mode();display_track_view();assert(draws==n && !s_display_ready);
 frame(123);display_track_view();assert(draws==n+1);
 assert(vision_camera_status_read(&st) && st.gray_mean==123);
 /* A busy status copy is rejected; the reader does not claim it is coherent. */
 s_camera_box.seq|=1u;assert(!vision_camera_status_read(&st));s_camera_box.seq++;
 assert(!vision_camera_status_read(0));
 return 0;
}
''')


def test_initialize_failure_is_visible_and_explicit_retry_recovers(tmp_path):
    _compile_run(tmp_path,camera_source()+r'''
int main(void){
 boot();vision_camera_status_t st;init_failed=1;
 assert(vision_command_request(VISION_CMD_RESTART_CAMERA,0));vision_core_run();
 assert(vision_core_state()==VISION_CORE_FAILED && !exposures && !vision_camera_run_allowed());
 assert(vision_camera_status_read(&st) && !st.gray_valid && st.exposure_applied==0 && st.exposure_state==VISION_EXPOSURE_UNKNOWN);
 init_failed=0;assert(vision_command_request(VISION_CMD_RESTART_CAMERA,0));vision_core_run();
 assert(vision_core_state()==VISION_CORE_READY && starts==2 && exposures==1);
 frame(80);assert(vision_camera_run_allowed());return 0;
}
''')


def test_actual_menu_hierarchy_and_profile_actions_fit_pool(tmp_path,monkeypatch):
    # Reuse the existing behavioral navigation contract with the optional
    # in-memory compiler. The actual menu builder and actual tree pool run.
    monkeypatch.syspath_prepend(str(ROOT/'tests'))
    import test_linked_menu
    original_run=subprocess.run

    def run(args,**kwargs):
        if args[0]=='gcc':
            text=(tmp_path/'tree.c').read_text(encoding='utf-8')
            tree=(ROOT/'code/Menu/menu_tree.c').read_text(encoding='utf-8').replace('#include "menu_tree.h"','#include "Menu/menu_tree.h"')
            source='#define MT9V03X_W 188\n#define MT9V03X_H 120\n'+text
            source+='\n#include <math.h>\nstatic float roundf(float x){if(x==0)return x;return x<0?ceilf(x-.5f):floorf(x+.5f);}\n'+tree
            work=tmp_path/'inmemory';work.mkdir()
            _compile_run(work,source)
            return subprocess.CompletedProcess(args,0,'','')
        if args[0]==str(tmp_path/'tree.exe'):
            return subprocess.CompletedProcess(args,0,'','')
        return original_run(args,**kwargs)

    monkeypatch.setattr(subprocess,'run',run)
    test_linked_menu.test_actual_hierarchy_navigation_callbacks_and_exhaustion(tmp_path)
