"""Production beacon pursuit and Run gates; no actuator or vehicle simulation."""
from test_beacon_restore import HEADER, run_beacon
from test_imu_forward import param_host_source
from test_lean_limits import ROOT, _compile_run, _function


FOLLOW = HEADER + r'''
static beacon_config_t c;
static beacon_result_t r;
static void target(float x,float y,int track,int running){
 memset(&r,0,sizeof(r));r.valid=1;r.cx=x;r.cy=y;r.track=track;
 beacon_follow_update(&c,(uint8_t)running,&r);
}
static void missing(int running){memset(&r,0,sizeof(r));beacon_follow_update(&c,(uint8_t)running,&r);}
static void close_to(float a,float b){assert(fabsf(a-b)<.01f);}
static void init(void){beacon_defaults(&c);c.mode=1;beacon_reset();}
'''


def test_real_follow_far_near_turn_search_reacquire_both_sides(tmp_path):
    run_beacon(tmp_path, FOLLOW + r'''
int main(void){
 init();target(93,40,0,1);
 assert(r.valid && r.steer_valid && !r.searching && r.follow_state==BEACON_FOLLOW_TRACK);
 close_to(r.follow_mid_x,93);close_to(r.error_px,0);
 target(93,75,0,1);close_to(r.follow_mid_x,112);close_to(r.error_px,-19);
 assert(r.follow_state==BEACON_FOLLOW_NEAR && !r.turn);
 target(93,110,0,1);assert(r.turn==left);close_to(r.follow_mid_x,140);
 missing(1);assert(!r.valid && r.searching && r.steer_valid && r.turn==left);
 assert(r.cx==0 && r.cy==0 && r.missed_frames==1);close_to(r.error_px,-60);
 target(110,40,0,1);assert(r.valid && !r.searching && !r.missed_frames);
 close_to(r.error_px,17);
 /* A fresh multi-target planner result can request the opposite approach. */
 target(93,40,right,1);assert(r.track==right_right);
 target(93,75,0,1);close_to(r.follow_mid_x,74);close_to(r.error_px,19);
 target(93,110,0,1);assert(r.turn==right);close_to(r.follow_mid_x,46);
 missing(1);assert(r.searching && r.turn==right);close_to(r.error_px,60);
 return 0;
}
''')


def test_search_requires_this_run_history_and_camera_mode_resets(tmp_path):
    run_beacon(tmp_path, FOLLOW + r'''
int main(void){
 init();for(int i=0;i<100;i++){missing(1);assert(!r.searching && !r.steer_valid);}
 target(120,40,0,1);
 for(int i=1;i<20;i++){missing(1);assert(!r.searching && !r.steer_valid && r.missed_frames==i);}
 missing(1);assert(r.searching && r.turn==left && r.missed_frames==20);
 missing(0);assert(!r.searching && !r.steer_valid && !r.missed_frames);
 target(70,115,0,0);missing(0);assert(!r.searching); /* preview never commands a search */
 missing(1);assert(!r.searching && !r.missed_frames); /* discard preview on Start */
 target(70,40,0,1);for(int i=0;i<20;i++)missing(1);
 assert(r.searching && r.turn==right);
 c.mode=2;missing(1);assert(!r.searching && !r.missed_frames);
 target(70,115,0,1);beacon_reset();missing(1);assert(!r.searching);
 c.mode=0;missing(1);assert(!r.searching && !r.steer_valid);
 return 0;
}
''')


def test_follow_configuration_guards_bounds_and_disable_search(tmp_path):
    run_beacon(tmp_path, FOLLOW + r'''
int main(void){
 init();c.track_row=c.turn_row;target(93,75,0,1);assert(!r.steer_valid);
 init();c.line_kp=NAN;target(93,75,0,1);assert(!r.steer_valid);
 init();c.mid_x=187;c.line_kp=2;c.line_offset=0;target(93,119,0,1);
 close_to(r.follow_mid_x,187);
 c.search_error_px=0;missing(1);assert(!r.searching && !r.steer_valid);
 init();target(93,40,0,1);target(93,40,right,1);
 c.mid_x=0;c.line_kp=2;c.line_offset=0;target(93,119,0,1);close_to(r.follow_mid_x,0);
 return 0;
}
''')


def test_follow_flash_roundtrip_rejects_inverted_rows_preserves_roll(tmp_path):
    _compile_run(tmp_path, param_host_source() + r'''
int main(void){
 const char *names[]={"beacon_track_row","beacon_turn_row","beacon_line_kp",
 "beacon_line_offset","beacon_search_error","beacon_search_frames"};
 param_init();g_param.r_rate_kp=42;assert(param_save());
 assert(g_param.beacon.track_row==50 && g_param.beacon.turn_row==110);
 assert(!param_set_by_name("beacon_track_row",110));
 assert(!param_set_by_name("beacon_turn_row",50));
 assert(param_set_by_name("beacon_track_row",55));assert(param_set_by_name("beacon_turn_row",108));
 assert(param_set_by_name("beacon_line_kp",.7f));assert(param_set_by_name("beacon_line_offset",38));
 assert(param_set_by_name("beacon_search_error",45));assert(param_set_by_name("beacon_search_frames",12));
 assert(param_save_names(names,6));param_init();
 assert(g_param.r_rate_kp==42 && g_param.beacon.track_row==55 && g_param.beacon.turn_row==108);
 assert(g_param.beacon.line_kp==.7f && g_param.beacon.line_offset==38);
 assert(g_param.beacon.search_error_px==45 && g_param.beacon.search_frames==12);
 return 0;
}
''')


def test_search_controller_timeout_freshness_speed_cap_and_steering(tmp_path):
    control = (ROOT/'code/Controller/control.c').read_text(encoding='utf-8')
    source = r'''
#define MT9V03X_W 188
#define MT9V03X_H 120
#include "config.h"
#include "Common/param.h"
#include "Common/pid.c"
#include "Estimation/Vision/vision_core.h"
#include <assert.h>
#include <string.h>
#include <math.h>
param_t g_param;
beacon_result_t g_beacon_result;
#define RUN_STOP_VISION 1
#define RUN_STOP_ZEBRA 2
#define RUN_STOP_LOST 3
typedef uint8 run_stop_t;
static uint8 s_run_active=1,s_run_vision_armed=1,g_track_valid=0,g_vision_source_mode=1;
static uint8 s_run_stop,g_vision_stop_request,s_zebra_stop_latched,s_direction_last_mode;
static uint8 g_vision_track_mode=TRACK_MODE_MIDDLE;
static uint16 s_run_lost_ms,g_vision_age_ms;
static int32 s_zebra_stop_start_count;
static int g_target_distance;
static float g_run_speed_target_mps,s_speed_ramp,s_direction_yaw_rate_target,s_direction_yaw_rate_cmd;
static float s_direction_offset,s_direction_last_error,s_direction_hold_yaw,g_yaw_target;
static float g_lean_offset,s_lean_speed_mps,g_vision_direction_camera;
static float g_vision_heading_error,g_vision_curvature,g_yaw_momentum_scale=1;
static pid_t p_vel_pid;
static uint8 vision_link_lost(void){return g_vision_age_ms>=VISION_LINK_TIMEOUT_MS;}
static int32 Y_Motor_GetTotalCount(void){return 0;}
static float Y_Motor_CountToMeter(int32 count){return (float)count;}
static int Y_Motor_GetSpeed20ms(void){return 0;}
static float Y_Motor_GetSpeedMps(void){return 0;}
static float imu_get_angle_yaw(void){return 0;}
static int speed_target_from_mps(float v){return (int)(v*10000.0f+.5f);}
static int func_abs(int v){return v<0?-v:v;}
static int ctrl_is_finite(float v){return v==v && v<=3e38f && v>=-3e38f;}
'''
    for name in ['run_speed_limit','run_hold_balance','beacon_search_active',
                 'run_target_update','direction_balance_Control','direction_control_update',
                 'direction_yaw_target_update']:
        source += _function(control, name)+'\n'
    source += r'''
int main(void){
 g_param.run_speed_straight=.1f;g_param.run_speed_lost=.2f;
 g_param.beacon.mode=1;g_param.beacon.speed_mps=.08f;g_param.beacon.yaw_limit_dps=10;
 g_param.steer_dir=1;g_param.direction_pixel_kp=1;
 g_beacon_result.searching=g_beacon_result.steer_valid=1;
 g_vision_direction_camera=60*DIRECTION_CAMERA_WEIGHT_SUM;
 assert(beacon_search_active());run_target_update();assert(g_target_distance==800);
 for(int i=0;i<100;i++)direction_control_update();
 assert(s_direction_yaw_rate_target<0 && s_direction_yaw_rate_target>=-10);
 direction_yaw_target_update();assert(g_yaw_target<0);
 /* Waiting for enough misses: no blind forward motion and no steering. */
 g_beacon_result.searching=0;run_target_update();assert(g_target_distance==0);
 direction_control_update();assert(s_direction_yaw_rate_target==0);
 /* A source mismatch, unarmed Run, or stopped Run must reject search. */
 g_beacon_result.searching=1;g_vision_source_mode=2;assert(!beacon_search_active());
 g_vision_source_mode=1;s_run_vision_armed=0;assert(!beacon_search_active());
 s_run_vision_armed=1;s_run_active=0;assert(!beacon_search_active());s_run_active=1;
 /* Even after permission has been lost, cached search mode cannot steer. */
 g_vision_age_ms=VISION_LINK_TIMEOUT_MS;assert(!beacon_search_active());
 run_target_update();assert(!s_run_active && g_target_distance==0 && s_run_stop==RUN_STOP_VISION);
 s_run_active=s_run_vision_armed=1;g_vision_age_ms=0;s_run_lost_ms=RUN_LOST_STOP_MS-1;
 run_target_update();assert(!s_run_active && g_target_distance==0 && s_run_stop==RUN_STOP_LOST);
 /* Lane lost-track behavior is unchanged. */
 s_run_active=1;s_run_lost_ms=0;g_param.beacon.mode=0;
 run_target_update();assert(g_target_distance==1000);
 return 0;
}
'''
    _compile_run(tmp_path, source)
