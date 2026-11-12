"""Run restored Qcar algorithms and current Run gates; no camera/vehicle simulation."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest
from test_lean_limits import ROOT, _compile_run, _function
from test_imu_forward import param_host_source


def run_beacon(tmp_path, source):
    runner = os.environ.get('ONEWHEEL_HOST_TCC')
    gcc = shutil.which('gcc')
    if not runner and not gcc:
        pytest.skip('Set ONEWHEEL_HOST_TCC or install GCC for native beacon tests')
    src = tmp_path / 'beacon_test.c'
    src.write_text(source, encoding='utf-8')
    include = ROOT/'tools/unicycle_station/tests/firmware_host'
    args = [runner or gcc]
    if runner:
        args += ['-I'+str(include/'tcc')]
    args += ['-I'+str(include), '-I'+str(ROOT/'code')]
    args += [str(p) for p in sorted((ROOT/'code/Estimation/Beacon').glob('*.c'))]
    if runner:
        args += ['-run', str(src)]
    else:
        exe = tmp_path/'beacon_test.exe'
        args += [str(src), '-std=c99', '-lm', '-o', str(exe)]
    result = subprocess.run(args, capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout+result.stderr
    if not runner:
        result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stdout+result.stderr


HEADER = r'''
#include "Estimation/Beacon/beacon_platform.h"
#include "Estimation/Beacon/beacon_core.h"
#include <assert.h>
static struct { unsigned before; uint8_t p[120][188]; unsigned after; } gray,binary;
static void guards(void){assert(gray.before==123 && gray.after==456 && binary.before==123 && binary.after==456);}
static void patch(int x,int y){for(int j=y;j<y+20;j++)for(int i=x;i<x+24;i++)gray.p[j][i]=((i+j)%2)?128:255;}
'''


def test_detector_three_modes_target_loss_and_buffer_edges(tmp_path):
    run_beacon(tmp_path, HEADER+r'''
int main(void){
 beacon_config_t c;beacon_result_t r;
 gray.before=binary.before=123;gray.after=binary.after=456;
 beacon_defaults(&c);assert(c.mode==0);
 for(int mode=1;mode<=3;mode++){
  c.mode=mode;memset(gray.p,0,sizeof(gray.p));
  beacon_process(&gray.p[0][0],binary.p,&c,&r);assert(!r.valid);
  patch(70,65);
  for(int frame=0;frame<45;frame++)beacon_process(&gray.p[0][0],binary.p,&c,&r);
  assert(r.valid && r.count==1 && fabsf(r.cx-81.5f)<.01f && fabsf(r.cy-74.5f)<.01f);
  assert(r.distance_m>0 && r.distance_m<100);guards();
  memset(gray.p,0,sizeof(gray.p));
  for(int frame=0;frame<45;frame++){
   beacon_process(&gray.p[0][0],binary.p,&c,&r);
   assert(!r.valid && r.count==0 && r.next==0 && r.route_distance_m==0);
  }
 }
 c.mode=1;patch(12,60);patch(78,70);patch(140,85);
 beacon_process(&gray.p[0][0],binary.p,&c,&r);
 assert(r.valid && r.count==3 && r.next>=1 && r.next<=3);
 unsigned seen=0;for(int i=0;i<3;i++){assert(r.order[i]>=1 && r.order[i]<=3);seen|=1u<<r.order[i];}assert(seen==14);
 c.mode=0;beacon_process(&gray.p[0][0],binary.p,&c,&r);assert(!r.valid);
 c.mode=3;beacon_process(&gray.p[0][0],binary.p,&c,&r);assert(r.valid);
 beacon_process(0,binary.p,&c,&r);assert(!r.valid && r.cx==0);
 beacon_process(&gray.p[0][0],0,&c,&r);assert(!r.valid);
 beacon_process(&gray.p[0][0],binary.p,0,&r);assert(!r.valid);
 beacon_process(0,0,0,0);
 memset(gray.p,255,sizeof(gray.p));c.mode=1;
 beacon_process(&gray.p[0][0],binary.p,&c,&r);assert(!r.valid);guards();
 /* Same-mode camera restarts must discard the old temporal tracker too. */
 imageinfo.time_num=12;beacon_reset();memset(gray.p,0,sizeof(gray.p));
 beacon_process(&gray.p[0][0],binary.p,&c,&r);assert(imageinfo.time_num==0 && !r.valid);
 return 0;
}
''')


def test_rolling_labels_match_full_image_connectivity_and_gray_statistics(tmp_path):
    # A late bridge merges two older labels; spaced pixels exercise R=1..4.
    pixels = {(x,y) for x in range(10,21) for y in range(30,33)}
    pixels |= {(x,y) for x in range(30,41) for y in range(30,33)}
    pixels |= {(x,33) for x in range(10,41)}
    pixels |= {(70,45),(72,47),(75,50),(79,54),(79,58)}
    pixels |= {(x,y) for x in range(185,188) for y in range(116,120)}
    source = HEADER+'int main(void){\n'
    for x,y in sorted(pixels):
        source += f'gray.p[{y}][{x}]={(x*7+y*3)%256};binary.p[{y}][{x}]=255;\n'
    source += 'ContourInfo out[MAX_LABELS];\n'
    for radius in range(1,5):
        pending=set(pixels);groups=[]
        while pending:
            first=pending.pop();group={first};queue=[first]
            while queue:
                x,y=queue.pop()
                near={p for p in pending if max(abs(p[0]-x),abs(p[1]-y))<=radius}
                pending-=near;group|=near;queue.extend(near)
            groups.append(group)
        source += f'memset(out,0,sizeof(out));assert(label_components_change(binary.p,gray.p,out,{radius})=={len(groups)});\n'
        for group in groups:
            n=len(group);cx=sum(x for x,y in group)/n;cy=sum(y for x,y in group)/n
            values=[(x*7+y*3)%256 for x,y in group]
            avg=sum(values)/n;std=(sum(v*v for v in values)/n-avg*avg)**.5
            source += f'{{int found=0;for(int i=0;i<{len(groups)};i++)if(fabsf(out[i].cx-{cx:.9f}f)<.001f && fabsf(out[i].cy-{cy:.9f}f)<.001f){{found++;assert(out[i].area=={n});assert(fabsf(out[i].ave_gray-{avg:.9f}f)<.001f);assert(fabsf(out[i].Std_Varaince-{std:.9f}f)<.005f);assert(out[i].mingray=={min(values)} && out[i].max_gray=={max(values)});}}assert(found==1);}}\n'
    run_beacon(tmp_path,source+'return 0;}\n')


def test_planner_zero_to_three_targets_and_route_cost(tmp_path):
    run_beacon(tmp_path, HEADER+r'''
int main(void){
 ImageInfo im;Turn_info turn;memset(&im,0,sizeof(im));memset(&turn,0,sizeof(turn));
 turn.weight_dist=1;mono_beacon.dynamic_mid_line=93;
 for(int n=1;n<=3;n++){
  im.numContours=n;
  for(int i=0;i<n;i++){im.contour_filter[i].distance=(float)(i+1);im.contour_filter[i].angle=90;im.contour_filter[i].cx=40+30*i;im.contour_filter[i].cy=70;}
  tragectory_planning_update(&im,&turn);
  assert(turn.next_beacon==1 && turn.cx==40 && turn.cy==70);
  assert(fabsf(best_combined_show.total_dist-n)<.001f);
  unsigned seen=0;for(int i=0;i<n;i++){assert(best_combined_show.order[i]>=1 && best_combined_show.order[i]<=n);seen|=1u<<best_combined_show.order[i];}assert(seen==((1u<<(n+1))-2));
 }
 im.numContours=0;tragectory_planning_update(&im,&turn);
 assert(turn.next_beacon==0 && best_combined_show.total_dist==0);
 return 0;
}
''')


def test_component_capacity_and_invalid_camera_coordinates(tmp_path):
    run_beacon(tmp_path,HEADER+r'''
int main(void){
 struct {unsigned before;ContourInfo data[MAX_LABELS];unsigned after;} out;
 memset(&out,0,sizeof(out));out.before=123;out.after=456;
 for(int y=30;y<100;y+=10)for(int x=10;x<180;x+=10){binary.p[y][x]=255;gray.p[y][x]=150;}
 assert(label_components_change(binary.p,gray.p,out.data,1)==MAX_LABELS);
 assert(out.before==123 && out.after==456);
 ContourInfo c;float xyz[3];memset(&c,0,sizeof(c));
 pixel2world(188,70,xyz,&c);assert(c.distance==100);
 pixel2world(70,120,xyz,&c);assert(c.distance==100);
 return 0;
}
''')


def test_run_requires_fresh_matching_source_but_beacon_not_lane_ipm(tmp_path):
    code=(ROOT/'code/Controller/control.c').read_text(encoding='utf-8')
    source=r'''
#define MT9V03X_W 188
#define MT9V03X_H 120
#define START_STOP 0
#include "config.h"
#include "Common/param.h"
#include "Estimation/Vision/vision_core.h"
#include <assert.h>
param_t g_param;
uint8 start_flag=START_STOP,g_track_valid=1,g_vision_ipm_ok=0,g_vision_source_mode=0;
static uint8 s_run_active,s_run_stop,s_direction_last_mode,s_zebra_stop_latched,s_run_vision_armed,g_vision_stop_request;
static int s_test_status,s_run_lost_ms,s_zebra_stop_start_count,g_vofa_mode,g_target_distance;
static float g_run_speed_target_mps,s_direction_offset,s_direction_last_error,s_direction_yaw_rate_target,s_direction_yaw_rate_cmd,s_direction_hold_yaw,g_yaw_target;
static int state=VISION_CORE_READY,lost=0,allowed=1,starts=0;
#define CTRL_TEST_STATUS_INVALID 99
#define RUN_STOP_NONE 0
#define VOFA_RUN 1
vision_core_state_t vision_core_state(void){return (vision_core_state_t)state;}
static uint8 vision_link_lost(void){return (uint8)lost;}
uint8 vision_camera_run_allowed(void){return 1;}
uint8 control_balance_start(void){starts++;return (uint8)allowed;}
static float imu_get_angle_yaw(void){return 8;}
'''+_function(code,'control_run_start')+r'''
int main(void){
 assert(!control_run_start() && starts==0);
 g_param.beacon.mode=1;assert(!control_run_start() && starts==0);
 g_vision_source_mode=1;lost=1;assert(!control_run_start() && starts==0);
 lost=0;g_track_valid=0;assert(!control_run_start() && starts==0);
 g_track_valid=1;state=VISION_CORE_FAILED;assert(!control_run_start() && starts==0);
 state=VISION_CORE_READY;allowed=0;assert(!control_run_start() && starts==1);
 allowed=1;assert(control_run_start() && starts==2 && s_run_active && !s_run_vision_armed && g_target_distance==0);
 return 0;
}
'''
    _compile_run(tmp_path,source)


def test_beacon_group_flash_roundtrip_preserves_existing_roll(tmp_path):
    _compile_run(tmp_path,param_host_source()+r'''
int main(void){
 const char *names[]={"beacon_mode","beacon_threshold","beacon_speed_mps"};
 param_init();assert(g_param.beacon.mode==0 && g_param.r_rate_kp==45 && g_param.r_rate_ki==.05f);
 g_param.r_rate_kp=42;g_param.enc_dir_a=-1;assert(param_save());
 assert(param_set_by_name("beacon_mode",3));assert(param_set_by_name("beacon_threshold",90));
 assert(param_set_by_name("beacon_speed_mps",.12f));assert(param_save_names(names,3));
 param_init();assert(g_param.beacon.mode==3 && g_param.beacon.threshold==90 && g_param.beacon.speed_mps==.12f);
 assert(g_param.r_rate_kp==42 && g_param.enc_dir_a==-1);
 return 0;
}
''')


def test_cpu1_publication_mode_switch_and_target_loss(tmp_path):
    code=(ROOT/'code/Estimation/Vision/vision_core.c').read_text(encoding='utf-8')
    source=HEADER+r'''
#undef left
#undef right
#define MT9V03X_W 188
#define MT9V03X_H 120
#include "Estimation/Vision/vision_core.h"
#include "Estimation/Vision/image.h"
Image my_image;elem_action_t g_elem_action;island_t g_island;
float g_mid_error,g_track_lateral,g_track_heading,g_track_curvature,g_track_quality,g_direction_camera;
uint8 g_direction_valid;int g_err_front_row;
static vision_feedback_t s_feedback_cache,incoming;
static vision_result_t published;
static uint32 s_heartbeat,s_frame_seq,s_ipm_param_revision;
static uint16 s_camera_exposure,s_feedback_exposure;
static uint8 s_exposure_pending;
static vision_camera_status_t s_camera_status;
static void vision_camera_status_publish(void){}
static void vision_exposure_apply(void){}
static void vision_capture_status(void){s_frame_seq++;}
static uint8 s_display_mode,s_ipm_calib_seq;
static float s_ipm_applied[9];
static int s_core_state=VISION_CORE_READY,have_frame=1,publications,edges;
static void vision_command_service(void){}
uint8 image_grab(void){return (uint8)have_frame;}
static uint8 vision_feedback_read(vision_feedback_t *f){*f=incoming;return 1;}
static void mt9v03x_set_exposure_time(uint16 exposure){(void)exposure;}
static int ipm_load(float *h){(void)h;return 0;}
static void ipm_clear(void){}
static int ipm_ready(void){return 0;}
static void ipm_store(float *h){memset(h,0,9*sizeof(float));}
const uint8 *image_gray_data(void){return &gray.p[0][0];}
static void vision_display_publish(vision_display_mode_t m){(void)m;}
static void vision_result_publish(const vision_result_t *r){published=*r;publications++;}
void image_process_edges(uint8 learning){(void)learning;edges++;my_image.Track_Valid=0;}
static void vision_ipm_collect(void){}
void element_set_motion(const element_motion_t *m){(void)m;}
void element_process(void){memset(&g_elem_action,0,sizeof(g_elem_action));}
void image_process_finish(void){g_direction_valid=0;g_direction_camera=0;}
'''+_function(code,'vision_core_run')+r'''
int main(void){
 beacon_defaults(&incoming.beacon);incoming.beacon.mode=1;incoming.input_seq=23;incoming.run_active=1;
 patch(70,65);vision_core_run();
 assert(publications==1 && !edges && published.source_mode==1 && published.frame_seq==1);
 assert(published.beacon.valid && published.track_valid && published.run_active && published.input_seq==23);
 assert(published.direction_camera>0 && published.speed_limit_mps==incoming.beacon.speed_mps);
 have_frame=0;vision_core_run();assert(publications==1);
 have_frame=1;memset(gray.p,0,sizeof(gray.p));vision_core_run();
 assert(publications==2 && !published.track_valid && !published.beacon.valid && published.speed_limit_mps==0 && published.direction_camera==0);
 assert(published.track_mode==TRACK_MODE_HOLD);
 incoming.beacon.mode=0;incoming.input_seq++;vision_core_run();
 assert(publications==3 && edges==1 && published.source_mode==0 && !published.beacon.valid);
 incoming.beacon.mode=3;incoming.input_seq++;patch(70,65);vision_core_run();
 assert(published.source_mode==3 && published.beacon.valid && edges==1);
 return 0;
}
'''
    run_beacon(tmp_path,source)


def test_historical_reference_files_are_unchanged():
    directory=ROOT/'docs/reference/qcar-beacon'
    manifest=json.loads((directory/'manifest.json').read_text(encoding='utf-8'))
    for name,digest in manifest['files'].items():
        assert hashlib.sha256((directory/(name+'.in')).read_bytes()).hexdigest()==digest
