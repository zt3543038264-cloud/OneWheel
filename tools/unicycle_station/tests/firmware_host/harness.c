/* Host contract test only. Never link or flash this file to the TC264. */
#include "zf_common_headfile.h"
#define PATH_HOST_TEST 1
#include "../../../../code/Controller/Planner/path_nav.h"
uint8 control_path_start(float speed);
uint8 control_path_storage_ready(void);
void control_path_status(path_status_t *out);
uint8 control_path_point(uint16 index,path_point_t *out);
int path_nav_busy(void);
uint8 control_path_active(void);
#include "../../../../code/Protocols/vofa.c"
#include <assert.h>

param_t g_param;
volatile uint32 g_param_revision;
start_state_t start_flag;
attitude_t att;
att_diag_t att_diag;
uint8 g_imu_ok=1, g_cam_ok=1;
volatile uint32 g_control_uptime_ms;
uint8 g_track_valid=1,g_vision_active_elem,g_vision_island_state,g_vision_track_mode;
volatile uint16 g_vision_age_ms;
volatile uint32 g_vision_frame_seq=1;
float g_vision_curvature,g_vision_quality=1;
float Y_Motor_GetSpeedMps(void) { return 0.0f; }
static uint8 mock_test, mock_jog, mock_run, mock_remote;
static int mock_path_busy;
static uint8 mock_path_active;
int path_nav_busy(void) { return mock_path_busy; }
uint8 control_path_active(void) { return mock_path_active; }
static path_status_t mock_path;
static path_point_t mock_points[256];
static uint32 mock_path_revision=1;
uint32_t path_nav_revision(void) {return mock_path_revision;}
void control_path_status(path_status_t *out) {*out=mock_path;}
uint8 control_path_point(uint16 index,path_point_t *out) {if(index>=mock_path.count)return 0;*out=mock_points[index];return 1;}
int path_nav_import(const path_point_t *p,uint16_t n) {
    if(n<2 || p[0].x!=0 || p[0].y!=0)return 0;
    memcpy(mock_points,p,n*sizeof(*p));mock_path.count=n;mock_path.valid=1;mock_path.length_m=p[n-1].s;mock_path_revision++;return 1;
}
void path_nav_modified(void) {mock_path.dirty=1;}
uint8 control_path_start(float speed) {if(speed<.05f || speed>1.5f)return 0;mock_path_active=1;start_flag=START_BALANCE;return 1;}
uint8 control_path_storage_ready(void) {return start_flag==START_STOP;}
int path_store_save(void) {return 1;}
int path_store_load(void) {return 1;}

static int save_count, stop_count;
static int wheel_speed;
const param_desc_t g_param_table[] = {
    {"r_rate_kp", &g_param.r_rate_kp, 1, -2000,2000},
    {"r_angle_kp", &g_param.r_angle_kp, 1,-2000,2000},
    {"motor_dir_a", &g_param.motor_dir_a, 0,-1,1},
    {"cam_exposure", &g_param.cam_exposure, 0,4,1600},
    {"run_speed_straight", &g_param.run_speed_straight,1,0,1.5f},
    {"lean_speed_kp", &g_param.lean_speed_kp,1,0,2},
    {"direction_rate_kd", &g_param.direction_balance_kd,1,0,1},
    {"imu_forward_dir", &g_param.imu_forward_dir,0,-1,1}
};
uint16 param_count(void) { return sizeof(g_param_table)/sizeof(g_param_table[0]); }
const param_desc_t *param_find(const char *name) {
    uint16 i; for(i=0;i<param_count();i++) if(strcmp(name,g_param_table[i].name)==0) return &g_param_table[i]; return NULL;
}
uint8 param_get_by_name(const char *name,float *value) {
    const param_desc_t *d=param_find(name); if(!d)return 0;
    *value=d->is_float?*(float*)d->ptr:(float)*(int*)d->ptr; return 1;
}
void param_sync_zero(void) {}
void param_set_imu_forward_dir(int direction) { g_param.imu_forward_dir=direction; }
uint8 imu_forward_pending(void) { return g_param.imu_forward_dir!=1; }
uint8 param_save(void) {save_count++;return 1;}
uint8 param_save_names(const char *const *names,uint16 count) {(void)names;(void)count;save_count++;return 1;}
uint8 control_test_running(void) {return mock_test;}
motor_jog_t control_jog_running(void) {return (motor_jog_t)mock_jog;}
uint8 control_run_running(void) {return mock_run;}
uint8 control_remote_running(void) {return mock_remote;}
void control_stop(void) {start_flag=START_STOP;mock_test=mock_jog=mock_run=mock_remote=0;stop_count++;}
run_stop_t control_run_stop_reason(void) {return RUN_STOP_NONE;}
control_test_status_t control_test_last_status(void) {return CTRL_TEST_STATUS_OK;}
uint8 control_ipm_pending(void) {return 0;}
void control_remote_status(float *steer,float *speed,uint16 *age,uint8 *seen) {*steer=10;*speed=.15f;*age=25;*seen=1;}
uint8 control_remote_command(float a,float b) {(void)a;(void)b;return mock_remote;}
void control_run_diag_snapshot(volatile control_run_diag_t *out) {memset((void *)out,0,sizeof(*out));out->uptime_ms=g_control_uptime_ms;out->roll=12.5f;out->state_flags=mock_run ? 1 : 2;}
imu_calib_state_t imu_calib_state(void) {return IMU_CALIB_OK;}
uint8 imu_link_lost(void) {return 0;}
float g_pwm_roll;
uint8 imu_get_raw_sample(imu_data_t *out) { memset(out,0,sizeof(*out)); out->acc_z=1.0f; out->gyro_x=2.0f; return 1; }
uint8 W_Motor_LinkLost(void) {return 0;}
int16 W_Motor_GetSpeed1(void) {return (int16)wheel_speed;}
int16 W_Motor_GetSpeed2(void) {return (int16)wheel_speed;}
int16 Y_Motor_GetSpeed20ms(void) {return (int16)wheel_speed;}
uint8 attitude_converged(void) {return 1;}
uint8 attitude_diverged(void) {return 0;}

/* Commissioning parser fixtures: production controller/transport are tested separately. */
static control_preview_diag_t preview;
static control_timing_diag_t timing;
static w_motor_bench_status_t bench;
static unsigned preview_starts, bench_starts, bench_stops;
/* Parser fixtures only: real C controller behavior has a separate host test. */
static control_c_bench_diag_t cbench;
static unsigned cbench_starts,cbench_stops,cbench_keeps;
uint32 control_c_bench_start(control_c_bench_mode_t mode,float target,float kp,float ki,
                            uint16 duration,uint16 limit,uint32 expected) {
 if(cbench.active || expected!=cbench.session || cbench.session==0xffffffffu) return 0;
 if((mode==C_BENCH_SIGNAL_RAW || mode==C_BENCH_SIGNAL_RATE) && !cbench.signal_allowed) return 0;
 ++cbench_starts;++cbench.session;cbench.active=1;cbench.mode=mode;cbench.state=C_BENCH_RUNNING;
 cbench.target=target;cbench.kp=kp;cbench.ki=ki;cbench.duration_ms=duration;cbench.limit=limit;
 cbench.heartbeat_sequence=0;cbench.reason=C_BENCH_REASON_OK;return cbench.session;
}
uint8 control_c_bench_heartbeat(uint32 session,uint32 sequence) {
 if(!cbench.active || session!=cbench.session || sequence<=cbench.heartbeat_sequence) return 0;
 ++cbench_keeps;cbench.heartbeat_sequence=sequence;return 1;
}
void control_c_bench_stop(void){++cbench_stops;cbench.active=0;cbench.applied=0;cbench.state=C_BENCH_STOPPED;}
uint8 control_c_bench_running(void){return cbench.active;}
void control_c_bench_status(control_c_bench_diag_t *out){*out=cbench;}
uint8 control_preview_start(tune_axis_t axis,tune_ring_t ring) {
 if(bench.busy||preview.active)return 0;
 ++preview_starts;preview.active=1;preview.axis=axis;preview.ring=ring;return 1;
}
void control_preview_stop(void){preview.active=0;}
uint8 control_preview_running(void){return preview.active;}
void control_preview_status(control_preview_diag_t *out){*out=preview;}
void control_timing_status(control_timing_diag_t *out){*out=timing;}
uint8 control_bench_b_start(int16 duty,uint16 duration,uint32 session){
 if(preview.active||bench.busy||!session||session!=bench.session)return 0;
 ++bench_starts;bench.busy=1;bench.right_duty=duty;bench.elapsed_ms=duration;return 1;
}
void control_bench_b_stop(void){++bench_stops;bench.busy=0;bench.right_duty=0;}
uint8 control_bench_b_running(void){return bench.busy;}
void W_Motor_BenchReadStatus(w_motor_bench_status_t *out){*out=bench;}

static char response[4096];
static const char *drain(void) {
    unsigned n=0;while(s_tx_tail!=s_tx_head && n<sizeof(response)-1)response[n++]=(char)s_tx_buf[(s_tx_tail++)&VOFA_TX_MASK];
    response[n]=0;return response;
}
static void feed(const char *text) {
    while(*text) {s_rx_buf[s_rx_head&VOFA_RX_MASK]=(uint8)*text++;s_rx_head++;}
    vofa_cmd_poll();
}
int main(void) {
    vofa_init();start_flag=START_STOP;g_param.imu_forward_dir=1;
    feed("cfg:preview_start,100,roll,angle\n");assert(strstr(drain(),"ok,preview_start,1,ZERO_OUTPUT"));assert(preview_starts==1);
    feed("cfg:preview_start,101,yaw,rate\n");assert(strstr(drain(),"err,BAD_AXIS"));assert(preview_starts==1);
    feed("cfg:preview_start,102,pitch,vel\n");assert(strstr(drain(),"err,BAD_RING"));assert(preview_starts==1);
    feed("cfg:preview_status,103\n");assert(strstr(drain(),"ok,preview_status,1,1"));
    feed("cfg:preview_stop,104\n");assert(strstr(drain(),"ok,preview_stop"));assert(!preview.active);
    feed("cfg:timing,105\n");assert(strstr(drain(),"ok,timing,1,"));
    bench.session=4294967295u;bench.ready=1;
    feed("cfg:bench_status,106\n");assert(strstr(drain(),"4294967295"));
    feed("cfg:bench_start,107,100,100,B\n");assert(strstr(drain(),"SESSION_REQUIRED"));
    feed("cfg:bench_start,108,100,100,B,4294967296\n");assert(strstr(drain(),"BAD_BENCH"));
    feed("cfg:bench_start,109,501,100,B,4294967295\n");assert(strstr(drain(),"BAD_BENCH"));
    feed("cfg:bench_start,110,100,151,B,4294967295\n");assert(strstr(drain(),"BAD_BENCH"));
    feed("cfg:bench_start,111,100,100,A,4294967295\n");assert(strstr(drain(),"BAD_BENCH"));
    feed("cfg:bench_start,112,100,100,B,0\n");assert(strstr(drain(),"BAD_BENCH"));
    feed("cfg:bench_start,113,100,100,B,2\n");assert(strstr(drain(),"BLOCKED"));assert(bench_starts==0);
    feed("cfg:bench_start,114,-500,150,B,4294967295\n");assert(strstr(drain(),"ok,bench_start,1,REQUESTED"));assert(bench_starts==1&&bench.right_duty==-500);
    assert(cfg_status());assert(strstr(drain(),"stat:0,0,0,0,2,"));
    feed("cfg:set,116,r_rate_kp,33\n");assert(strstr(drain(),"RUNNING_LOCKED"));
    feed("cfg:save,117,all\n");assert(strstr(drain(),"SAVE_BLOCKED"));
    feed("cfg:bench_stop,115\n");assert(strstr(drain(),"ok,bench_stop"));assert(bench_stops==1&&!bench.busy);
    feed("cfg:hello,1\r\n");assert(strstr(drain(),"rsp:1,ok,hello,1,"));
    feed("cfg:remote,21\n");assert(strstr(drain(),"rsp:21,ok,remote,1,0,10,0.15,25,1"));
    mock_remote=1;feed("cfg:remote,22\n");assert(strstr(drain(),"remote,1,1,"));mock_remote=0;
    feed("cfg:remote,23,start\n");assert(strstr(drain(),"err,"));assert(!mock_remote);
    feed("cfg:set,2,r_rate_kp,9999\r\n");assert(g_param.r_rate_kp==2000);assert(strstr(drain(),"2000,CLAMPED"));
    feed("cfg:set,3,r_rate_kp,nan\n");assert(g_param.r_rate_kp==2000);assert(strstr(drain(),"INVALID_VALUE"));
    feed("cfg:set,4,cam_exposure,4.5\n");assert(strstr(drain(),"INVALID_VALUE"));
    feed("cfg:set,5,motor_dir_a,0\n");assert(strstr(drain(),"OUT_OF_RANGE"));
    /* Fixed RA EKF frame: reject changes, acknowledge identical values without
     * touching direction, mechanical zeros, or the parameter revision. */
    {
        uint32 revision = g_param_revision;
        g_param.roll_zero_init = 1.25f;
        g_param.pitch_zero_init = -4.5f;
        feed("cfg:set,50,imu_forward_dir,0\n");assert(strstr(drain(),"OUT_OF_RANGE"));
        feed("cfg:set,51,imu_forward_dir,-1\n");assert(strstr(drain(),"rsp:51,err,FIXED_FRAME,"));
        mock_jog=1;
        feed("cfg:set,52,imu_forward_dir,-1\n");assert(strstr(drain(),"rsp:52,err,FIXED_FRAME,"));
        mock_jog=0;
        start_flag=START_BALANCE;
        feed("cfg:set,53,imu_forward_dir,1\n");assert(strstr(drain(),"rsp:53,ok,set,imu_forward_dir,1,FIXED_FRAME"));
        start_flag=START_STOP;
        feed("cfg:set,54,imu_forward_dir,1\n");assert(strstr(drain(),"rsp:54,ok,set,imu_forward_dir,1,FIXED_FRAME"));
        assert(g_param.imu_forward_dir==1);
        assert(g_param.roll_zero_init==1.25f && g_param.pitch_zero_init==-4.5f);
        assert(g_param_revision==revision);
    }
    mock_jog=1;feed("cfg:set,6,r_rate_kp,3\n");assert(strstr(drain(),"RUNNING_LOCKED"));mock_jog=0;
    mock_test=1;g_tune_axis=TUNE_AXIS_ROLL;g_tune_ring=TUNE_RING_RATE;
    feed("cfg:set,7,r_angle_kp,4\n");assert(strstr(drain(),"RUNNING_LOCKED"));
    feed("cfg:set,8,r_rate_kp,12\n");assert(g_param.r_rate_kp==12);assert(strstr(drain(),"APPLIED"));
    start_flag=START_BALANCE;feed("cfg:set,9,motor_dir_a,-1\n");assert(strstr(drain(),"UNSAFE_PARAM"));
    feed("cfg:save,10,all\n");assert(strstr(drain(),"SAVE_BLOCKED"));
    feed("stop\n");assert(stop_count==1 && start_flag==START_STOP);drain();
    feed("cfg:save,11,all\n");assert(s_save_seq==11);feed("stop\n");assert(!s_save_seq && save_count==0);drain();
    feed("cfg:set,12,r_rate_kp,1");assert(g_param.r_rate_kp==12);
    s_rx_idle_ms=1000;vofa_cmd_poll();feed("cfg:set,13,r_rate_kp,2\n");assert(g_param.r_rate_kp==12);drain();
    feed("cfg:set,14,r_rate_kp,3\n");assert(g_param.r_rate_kp==3);drain();
    feed("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxstop\n");assert(stop_count==2);drain();
    s_tx_head=VOFA_TX_SIZE-2;s_tx_tail=0;assert(!tx_push((const uint8*)"abc",3));assert(s_tx_head==VOFA_TX_SIZE-2);s_tx_tail=s_tx_head;
    feed("cfg:schema,15\n");for(unsigned i=0;i<9;i++){g_control_uptime_ms+=31;cfg_poll();}assert(strstr(drain(),"rsp:15,ok,schema,8"));
    g_control_uptime_ms+=600;cfg_poll();drain();
    feed("cfg:save,16,all\n");s_rx_idle_ms=101;cfg_poll();assert(save_count==1);assert(strstr(drain(),"save,all,VERIFIED"));
    /* Diagnostic state capture continues with ordinary VOFA output disabled. */
    task_reset();s_station_att=1;g_vofa_mode=VOFA_OFF;
    g_control_uptime_ms=1000;g_vision_active_elem=ELEM_NONE;g_vision_age_ms=0;
    g_vision_curvature=0;vofa_snapshot();assert(s_task.state==1 && s_task_head==1);
    g_vision_curvature=.04f;g_control_uptime_ms=1010;task_capture();assert(s_task.state==1);
    g_control_uptime_ms=1160;task_capture();assert(s_task.state==2);
    g_vision_curvature=.025f;g_control_uptime_ms=1200;task_capture();assert(s_task.state==2);
    g_vision_active_elem=ELEM_CROSS;g_control_uptime_ms=1201;task_capture();assert(s_task.state==4);
    task_poll();assert(strstr(drain(),"taskevt:1,1000,0,1,"));
    g_vision_active_elem=ELEM_RING_LEFT;g_vision_island_state=1;g_control_uptime_ms=1400;task_capture();
    g_vision_island_state=2;g_control_uptime_ms++;task_capture();assert(s_task.phase==2);
    g_vision_age_ms=VISION_LINK_TIMEOUT_MS+1;g_control_uptime_ms++;task_capture();assert(s_task.state==10);
    for(unsigned i=0;i<40;i++){g_vision_age_ms=(i&1)?0:VISION_LINK_TIMEOUT_MS+1;g_control_uptime_ms++;task_capture();}
    assert(s_task_dropped>0 && s_task_head-s_task_tail==TASK_EVENT_CAPACITY);
    g_control_uptime_ms+=200;task_poll();assert(strstr(drain(),"task:"));
    assert(save_count==1 && stop_count==2);
    /* Handshake enables diagnostic frames in Balance/Remote and single-axis
       Test without modifying menu telemetry selection or setting Run active. */
    mock_run=mock_test=mock_remote=0;start_flag=START_STOP;g_vofa_mode=VOFA_OFF;
    s_station_att=0;assert(vofa_effective_mode()==VOFA_OFF);
    start_flag=START_BALANCE;assert(vofa_effective_mode()==VOFA_OFF);
    feed("cfg:hello,30\n");drain();assert(vofa_effective_mode()==VOFA_RUN);
    assert(!mock_run && g_vofa_mode==VOFA_OFF);
    vofa_poll();drain();
    for(unsigned i=0;i<20;i++){g_control_uptime_ms++;vofa_snapshot();}
    vofa_poll();assert(strstr(drain(),"run:") && strstr(response,"12.500"));
    mock_remote=1;
    for(unsigned i=0;i<20;i++){g_control_uptime_ms++;vofa_snapshot();}
    vofa_poll();assert(strstr(drain(),"run:"));
    mock_remote=0;start_flag=START_STOP;mock_test=1;assert(vofa_effective_mode()==VOFA_RUN);
    for(unsigned i=0;i<20;i++){g_control_uptime_ms++;vofa_snapshot();}
    vofa_poll();assert(strstr(drain(),"run:"));
    mock_test=0;assert(vofa_effective_mode()==VOFA_OFF);vofa_poll();drain();
    for(unsigned i=0;i<30;i++){g_control_uptime_ms++;vofa_snapshot();}
    vofa_poll();assert(!strstr(drain(),"run:"));
    g_vofa_mode=VOFA_ATT;assert(vofa_effective_mode()==VOFA_ATT);
    start_flag=START_BALANCE;assert(vofa_effective_mode()==VOFA_RUN);
    /* ATT menu selection must not suppress auxiliary att frames during tests. */
    g_control_uptime_ms+=101;cfg_poll();assert(strstr(drain(),"att:"));
    s_station_att=0;assert(vofa_effective_mode()==VOFA_ATT);
    g_vofa_mode=VOFA_RUN;assert(vofa_effective_mode()==VOFA_RUN);
    /* Runtime permission is advertised and rechecked on the MCU per write. */
    start_flag=START_BALANCE;mock_test=mock_jog=0;mock_run=1;
    assert((cfg_pid_mask() & 0x01000000u)!=0);
    feed("cfg:set,40,run_speed_straight,0.25\n");assert(g_param.run_speed_straight==.25f);assert(strstr(drain(),"APPLIED"));
    feed("cfg:set,41,lean_speed_kp,200\n");assert(g_param.lean_speed_kp==2);assert(strstr(drain(),"CLAMPED"));
    feed("cfg:set,42,direction_rate_kd,0.5\n");assert(g_param.direction_balance_kd==.5f);assert(strstr(drain(),"APPLIED"));
    feed("cfg:set,43,cam_exposure,80\n");assert(strstr(drain(),"RUNNING_LOCKED"));
    feed("cfg:set,44,motor_dir_a,1\n");assert(strstr(drain(),"UNSAFE_PARAM"));
    feed("cfg:save,45,all\n");assert(strstr(drain(),"SAVE_BLOCKED"));
    mock_run=0;mock_remote=1;
    feed("cfg:set,46,lean_speed_kp,1.25\n");assert(g_param.lean_speed_kp==1.25f);assert(strstr(drain(),"APPLIED"));
    mock_remote=0;mock_test=1;g_tune_axis=TUNE_AXIS_ROLL;g_tune_ring=TUNE_RING_RATE;
    assert((cfg_pid_mask() & 0x01000000u)==0);
    feed("cfg:set,47,lean_speed_kp,1.5\n");assert(g_param.lean_speed_kp==1.25f);assert(strstr(drain(),"RUNNING_LOCKED"));
    mock_test=0;mock_jog=1;assert(cfg_pid_mask()==0);
    feed("cfg:set,48,run_speed_straight,0.4\n");assert(g_param.run_speed_straight==.25f);assert(strstr(drain(),"RUNNING_LOCKED"));
    mock_jog=0;start_flag=START_STOP;
    feed("cfg:schema,49\n");
    {int found=0;for(unsigned i=0;i<9;i++){g_control_uptime_ms+=31;cfg_poll();if(strstr(drain(),"lean_speed_kp,float,1.25,0,2,Lean,0.0001,21"))found=1;}assert(found);}
    mock_path_busy=1;assert(!cfg_stopped() && cfg_pid_mask()==0);
    feed("cfg:set,50,r_rate_kp,5\n");assert(strstr(drain(),"RUNNING_LOCKED"));
    feed("cfg:save,51,all\n");assert(strstr(drain(),"SAVE_BLOCKED"));
    mock_path_busy=0;mock_path_active=1;start_flag=START_BALANCE;
    assert(cfg_pid_mask()==0);
    feed("cfg:set,52,lean_speed_kp,1.5\n");assert(strstr(drain(),"RUNNING_LOCKED"));
    mock_path_active=0;
    #include "path_contract.inc"
    #include "trace_contract.inc"
    #include "c_bench_contract.inc"
    printf("MCU host contract checks passed (not an ADS/TASKING build)\n");return 0;
}
