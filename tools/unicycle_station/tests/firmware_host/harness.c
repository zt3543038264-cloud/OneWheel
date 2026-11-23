/* Host contract test only. Never link or flash this file to the TC264. */
#include "zf_common_headfile.h"
#define MT9V03X_W 188
#define MT9V03X_H 120
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
static int mock_gate_seen;
int gate_get(gate_observation_t *out)
{
    if (!mock_gate_seen) return 0;
    memset(out, 0, sizeof(*out));
    out->sample_seq = 17;
    out->age_ms = 3;
    out->available = 1;
    out->flags = 4096;
    out->bearing_deg = -12.5f;
    out->field_norm = 0.25f;
    out->raw[2] = 1023;
    out->filtered[2] = 999.5f;
    return 1;
}
att_diag_t att_diag;
imu_data_t imu;
float g_yaw_target, g_flywheel_common_rpm, g_pwm_yaw;
float imu_get_angle_yaw(void) { return att.yaw; }
volatile uint8 g_imu_ok=1;
uint8 g_cam_ok=1;
volatile uint32 g_control_uptime_ms;
float Y_Motor_GetSpeedMps(void) { return 0.0f; }
static uint8 mock_test, mock_jog, mock_run, mock_remote;
static float remote_turn, remote_speed;
static odom_test_state_t mock_odom;
odom_test_state_t control_odometry_test_state(void) { return mock_odom; }
uint8 control_imu_recovery_busy(void) { return 0; }
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
    {"motor_c_deadzone_fwd", &g_param.motor_c_deadzone_fwd, 0, 0, 2000},
    {"motor_c_deadzone_rev", &g_param.motor_c_deadzone_rev, 0, 0, 2000},
    {"r_rate_kp", &g_param.r_rate_kp, 1, -2000,2000},
    {"r_angle_kp", &g_param.r_angle_kp, 1,-2000,2000},
    {"motor_dir_a", &g_param.motor_dir_a, 0,-1,1},
    {"fly_speed_limit", &g_param.fly_speed_limit, 0,4,1600},
    {"run_accel_mps2", &g_param.run_accel_mps2,1,0,1.5f},
    {"lean_speed_kp", &g_param.lean_speed_kp,1,0,2},
    {"lean_turn_kp", &g_param.lean_turn_kp,1,0,1},
    {"yaw_hold_deg", &g_param.yaw_hold_deg,1,-180,180},
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
uint8 control_remote_command(float a,float b) {
    if (!mock_remote) return 0;
    remote_turn=a;remote_speed=b;return 1;
}
void control_run_diag_snapshot(volatile control_run_diag_t *out) {memset((void *)out,0,sizeof(*out));out->uptime_ms=g_control_uptime_ms;out->roll=12.5f;out->state_flags=mock_run ? 1 : 2;}
imu_calib_state_t imu_calib_state(void) {return IMU_CALIB_OK;}
uint8 imu_link_lost(void) {return 0;}
const char *imu_fault_text(void) {return "IMU TEST FAULT";}
static uint8 mock_saturation;
uint8 imu_get_saturation_raw(int16 out[6]) {
    const int16 sample[] = {14, -28, 42, 4098, -32768, 0};
    if (!mock_saturation) return 0;
    memcpy(out,sample,sizeof(sample));
    return 1;
}
float g_pwm_roll;
uint8 imu_get_raw_sample(imu_data_t *out) { memset(out,0,sizeof(*out)); out->acc_z=1.0f; out->gyro_x=2.0f; return 1; }
uint8 W_Motor_LinkLost(void) {return 0;}
int16 W_Motor_GetSpeed1(void) {return (int16)wheel_speed;}
int16 W_Motor_GetSpeed2(void) {return (int16)wheel_speed;}
int16 Y_Motor_GetSpeed20ms(void) {return (int16)wheel_speed;}
static uint8 mock_attitude_fault;
uint8 attitude_converged(void) {return (uint8)!mock_attitude_fault;}
uint8 attitude_diverged(void) {return mock_attitude_fault;}
const char *attitude_fault_text(void) {return mock_attitude_fault ? "ATT OUTPUT B" : "NONE";}

static char response[4096];
static const char *drain(void) {
    unsigned n=0;while(s_tx_tail!=s_tx_head && n<sizeof(response)-1)response[n++]=(char)s_tx_buf[(s_tx_tail++)&VOFA_TX_MASK];
    response[n]=0;return response;
}
static void feed(const char *text) {
    while(*text) {s_rx_buf[s_rx_head&VOFA_RX_MASK]=(uint8)*text++;s_rx_head++;}
    vofa_cmd_poll();
}
#if STATION_WIFI_ENABLED
static void test_wifi_line_batch(void)
{
    extern uint8 mock_wifi_is_ready, mock_wifi_frame[];
    extern uint16 mock_wifi_length;
    const uint8 a[] = "stat:1\nyaw:1\n";
    const uint8 b[] = "fly:2\n";
    uint8 line[120];
    memset(line, 'a', sizeof(line));
    line[sizeof(line)-1] = '\n';
    s_tx_head = s_tx_tail = VOFA_TX_SIZE - 3u;
    assert(tx_push(a, sizeof(a)-1));
    assert(tx_push(b, sizeof(b)-1));
    vofa_wifi_poll();
    assert(s_tx_tail == VOFA_TX_SIZE - 3u);
    mock_wifi_is_ready = 1;
    vofa_wifi_poll();
    assert(mock_wifi_length == sizeof(a)+sizeof(b)-2 && s_tx_tail == s_tx_head);
    assert(!memcmp(mock_wifi_frame, a, sizeof(a)-1));
    assert(!memcmp(mock_wifi_frame+sizeof(a)-1, b, sizeof(b)-1));
    assert(sizeof(s_wifi_frame) == VOFA_WIFI_LINE_MAX);
    assert(tx_push(line, sizeof(line)) && tx_push(line, sizeof(line)) && tx_push(line, sizeof(line)));
    vofa_wifi_poll();
    assert(mock_wifi_length == 2u*sizeof(line));
    assert(s_tx_head - s_tx_tail == sizeof(line));
    vofa_wifi_poll();
    assert(mock_wifi_length == sizeof(line) && s_tx_tail == s_tx_head);
    mock_wifi_is_ready = 0;
}
#endif
int main(void) {
#if STATION_WIFI_ENABLED
    test_wifi_line_batch();
#endif
    vofa_init();start_flag=START_STOP;g_param.imu_forward_dir=1;
    feed("cfg:hello,1\r\n");assert(strstr(drain(),"rsp:1,ok,hello,1,"));
    feed("cfg:gate_status,62\n");assert(strstr(drain(),"rsp:62,ok,gate_status,NONE\n"));
    mock_gate_seen = 1;
    feed("cfg:gate_status,63\n");
    assert(strstr(drain(),"rsp:63,ok,gate_status,17,3,1,4096,-12.50,0.25,0,0,1023,0,0.0,0.0,999.5,0.0,0.0,0.0,0.0\n"));
    feed("cfg:imu_status,58\n");assert(strstr(drain(),"rsp:58,ok,imu_status,0,1,0,NONE,"));
    feed("cfg:imu_fault_raw,60\n");assert(strstr(drain(),"rsp:60,ok,imu_fault_raw,NONE"));
    mock_saturation=1;
    feed("cfg:imu_fault_raw,61\n");assert(strstr(drain(),"rsp:61,ok,imu_fault_raw,14,-28,42,4098,-32768,0"));
    mock_saturation=0;
    mock_attitude_fault=1;
    feed("cfg:imu_status,59\n");assert(strstr(drain(),"rsp:59,ok,imu_status,0,0,1,ATT OUTPUT B,"));
    mock_attitude_fault=0;
    feed("cfg:remote,21\n");assert(strstr(drain(),"rsp:21,ok,remote,1,0,10,0.15,25,1"));
    mock_remote=1;feed("cfg:remote,22\n");assert(strstr(drain(),"remote,1,1,"));mock_remote=0;
    feed("cfg:remote,23,start\n");assert(strstr(drain(),"err,"));assert(!mock_remote);
    feed("cfg:set,70,motor_c_deadzone_fwd,850\n");
    assert(g_param.motor_c_deadzone_fwd==850 && strstr(drain(),"APPLIED"));
    feed("cfg:set,71,motor_c_deadzone_rev,650\n");
    assert(g_param.motor_c_deadzone_rev==650 && strstr(drain(),"APPLIED"));
    start_flag=START_BALANCE;mock_remote=1;
    feed("cfg:set,72,motor_c_deadzone_fwd,0\n");
    assert(g_param.motor_c_deadzone_fwd==850 && strstr(drain(),"UNSAFE_PARAM"));
    start_flag=START_STOP;mock_remote=0;
    {
        uint32 accepted=g_vofa_cmd_ok;
        feed("speed:0.000,6.000\r\n");assert(g_vofa_cmd_ok==accepted);
        mock_remote=1;
        feed("speed:0.000,6.000\r\n");assert(remote_turn==0 && remote_speed==.15f);
        feed("speed:0.000,-6.000\r\n");assert(remote_turn==0 && remote_speed==-.15f);
        feed("speed:10.000,0.000\r\n");assert(remote_turn==10 && remote_speed==0);
        feed("speed:-10.000,0.000\r\n");assert(remote_turn==-10 && remote_speed==0);
        feed("speed:10.000,-6.000\r\n");assert(remote_turn==10 && remote_speed==-.15f);
        feed("speed:0.000,0.000\r\n");assert(remote_turn==0 && remote_speed==0);
        assert(g_vofa_cmd_ok==accepted+6);mock_remote=0;
    }
    feed("cfg:set,2,r_rate_kp,9999\r\n");assert(g_param.r_rate_kp==2000);assert(strstr(drain(),"2000,CLAMPED"));
    feed("cfg:set,3,r_rate_kp,nan\n");assert(g_param.r_rate_kp==2000);assert(strstr(drain(),"INVALID_VALUE"));
    feed("cfg:set,4,fly_speed_limit,4.5\n");assert(strstr(drain(),"INVALID_VALUE"));
    feed("cfg:set,5,motor_dir_a,0\n");assert(strstr(drain(),"OUT_OF_RANGE"));
    feed("cfg:set,50,imu_forward_dir,0\n");assert(strstr(drain(),"OUT_OF_RANGE"));
    feed("cfg:set,51,imu_forward_dir,-1\n");assert(g_param.imu_forward_dir==-1);assert(strstr(drain(),"RESTART_REQUIRED"));
    mock_jog=1;feed("cfg:set,52,imu_forward_dir,1\n");assert(g_param.imu_forward_dir==-1);assert(strstr(drain(),"UNSAFE_PARAM"));mock_jog=0;
    start_flag=START_BALANCE;feed("cfg:set,53,imu_forward_dir,1\n");assert(g_param.imu_forward_dir==-1);assert(strstr(drain(),"UNSAFE_PARAM"));start_flag=START_STOP;
    feed("cfg:set,54,imu_forward_dir,1\n");assert(strstr(drain(),"APPLIED"));
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
    feed("cfg:schema,15\n");for(unsigned i=0;i<=param_count();i++){g_control_uptime_ms+=31;cfg_poll();}
    assert(strstr(drain(),"rsp:15,ok,schema,11"));
    assert(strstr(response,"motor_c_deadzone_fwd,int,850,0,2000,Motor,1,6"));
    g_control_uptime_ms+=600;cfg_poll();drain();
    feed("cfg:save,16,all\n");s_rx_idle_ms=101;cfg_poll();assert(save_count==1);assert(strstr(drain(),"save,all,VERIFIED"));
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
#if STATION_WIFI_ENABLED
    vofa_poll();assert(!strstr(drain(),"run:"));
    feed("cfg:remote,68\n");assert(strstr(drain(),"rsp:68,ok,remote,1,1,"));
    for(unsigned i=0;i<20;i++){g_control_uptime_ms++;vofa_snapshot();}
#endif
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
    feed("cfg:set,40,run_accel_mps2,0.25\n");assert(g_param.run_accel_mps2==.25f);assert(strstr(drain(),"APPLIED"));
    feed("cfg:set,41,lean_speed_kp,200\n");assert(g_param.lean_speed_kp==2);assert(strstr(drain(),"CLAMPED"));
    feed("cfg:set,42,lean_turn_kp,0.5\n");assert(g_param.lean_turn_kp==.5f);assert(strstr(drain(),"APPLIED"));
    feed("cfg:set,43,fly_speed_limit,80\n");assert(strstr(drain(),"UNSAFE_PARAM"));
    feed("cfg:set,44,motor_dir_a,1\n");assert(strstr(drain(),"UNSAFE_PARAM"));
    feed("cfg:save,45,all\n");assert(strstr(drain(),"SAVE_BLOCKED"));
    mock_run=0;mock_remote=1;
    feed("cfg:set,55,yaw_hold_deg,2\n");assert(g_param.yaw_hold_deg==0);assert(strstr(drain(),"RUNNING_LOCKED"));
    feed("cfg:set,46,lean_speed_kp,1.25\n");assert(g_param.lean_speed_kp==1.25f);assert(strstr(drain(),"APPLIED"));
    mock_remote=0;mock_test=1;g_tune_axis=TUNE_AXIS_ROLL;g_tune_ring=TUNE_RING_RATE;
    assert((cfg_pid_mask() & 0x01000000u)==0);
    feed("cfg:set,47,lean_speed_kp,1.5\n");assert(g_param.lean_speed_kp==1.25f);assert(strstr(drain(),"RUNNING_LOCKED"));
    mock_test=0;mock_jog=1;assert(cfg_pid_mask()==0);
    feed("cfg:set,48,run_accel_mps2,0.4\n");assert(g_param.run_accel_mps2==.25f);assert(strstr(drain(),"RUNNING_LOCKED"));
    mock_jog=0;start_flag=START_STOP;
    start_flag=START_BALANCE;
    feed("cfg:set,56,yaw_hold_deg,2\n");assert(g_param.yaw_hold_deg==2);assert(strstr(drain(),"APPLIED"));
    g_yaw_target=12;att.yaw=11;imu.gyro_z=3;
    cfg_yaw_hold_report();assert(strstr(drain(),",12.000,11.000,3.00,"));
    mock_path_active=1;feed("cfg:set,57,yaw_hold_deg,0\n");assert(g_param.yaw_hold_deg==2);assert(strstr(drain(),"RUNNING_LOCKED"));mock_path_active=0;
    start_flag=START_STOP;
    feed("cfg:schema,49\n");
    {int found=0;for(unsigned i=0;i<param_count();i++){g_control_uptime_ms+=31;cfg_poll();if(strstr(drain(),"lean_speed_kp,float,1.25,0,2,Lean,0.0001,21"))found=1;}assert(found);}
    mock_path_busy=1;assert(!cfg_stopped() && cfg_pid_mask()==0);
    feed("cfg:set,50,r_rate_kp,5\n");assert(strstr(drain(),"RUNNING_LOCKED"));
    feed("cfg:save,51,all\n");assert(strstr(drain(),"SAVE_BLOCKED"));
    mock_path_busy=0;mock_path_active=1;start_flag=START_BALANCE;
    assert(cfg_pid_mask()==0);
    feed("cfg:set,52,lean_speed_kp,1.5\n");assert(strstr(drain(),"RUNNING_LOCKED"));
    mock_path_active=0;
    #include "path_contract.inc"
    #include "trace_contract.inc"
    printf("MCU host contract checks passed (not an ADS/TASKING build)\n");return 0;
}
