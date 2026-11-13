/* Actual control.c and pid.c, hardware/transport only stubbed. No plant model. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "Common/vehicle_profile.h"
#ifdef HOST_UNLOCK_JOG_TEST
/* Exercise dormant jog defense in a HOST-ONLY build. Production remains 0. */
#undef VEHICLE_MOTOR_OUTPUT_ALLOWED
#define VEHICLE_MOTOR_OUTPUT_ALLOWED 1
#endif
#include "Controller/control.c"
#include "Common/pid.c"
param_t g_param;
volatile uint32 g_param_revision;
attitude_t att; att_diag_t att_diag; imu_data_t imu;
volatile tune_axis_t g_tune_axis; volatile tune_ring_t g_tune_ring;
int host_stm;
static uint64 ticks;
static float stm_hz=100000000.0f;
static uint32 work_ticks;
static int imu_fault, calib_ok=1, converged=1, diverged, restart, link_lost=1, path_busy;
static int stop_writes, release_writes, nonzero_writes, keepzero_writes;
static int bench_busy, bench_ready, bench_services, bench_aborts;
static uint32 bench_session=7, passed_session;
static int16 rpm_a, rpm_b;
uint32 interrupt_global_disable(void){return 0;}void interrupt_global_enable(uint32 x){(void)x;}
uint32 IfxStm_getLower(int *x){(void)x;return (uint32)ticks;}
uint32 IfxStm_getOffset32Timer(int *x){(void)x;return (uint32)(ticks>>32);}
float IfxStm_getFrequency(int *x){(void)x;return stm_hz;}
uint8 imu_forward_pending(void){return restart;}uint8 imu_link_lost(void){return imu_fault;}
uint8 attitude_diverged(void){return diverged;}uint8 attitude_converged(void){return converged;}
imu_calib_state_t imu_calib_state(void){return calib_ok?IMU_CALIB_OK:IMU_CALIB_MOVED;}
uint8 W_Motor_LinkLost(void){return link_lost;}int path_nav_busy(void){return path_busy;}
void W_Motor_Stop(void){stop_writes++;bench_busy=bench_ready=0;}void Y_Motor_Stop(void){stop_writes++;}
void W_Motor_KeepZero(void){keepzero_writes++;assert(!bench_busy);}
void W_Motor_SetDuty(int32 a,int32 b){if(a||b)nonzero_writes++;}void Y_Motor_SetDuty(int32 c){if(c)nonzero_writes++;}
void W_Motor_Release(void){release_writes++;}
int16 W_Motor_GetSpeed1(void){return rpm_a;}int16 W_Motor_GetSpeed2(void){return rpm_b;}
void path_nav_abort(void){path_busy=0;}
int16 Y_Motor_GetSpeed20ms(void){return 0;}
float Y_Motor_MpsToCount20ms(float m){return m;}
uint8 W_Motor_BenchBusy(void){return bench_busy;}
uint8 W_Motor_BenchRequestB(int16 d,uint16 ms,uint32 session){
 assert(d&&d>=-500&&d<=500&&ms&&ms<=150);passed_session=session;
 if(!bench_ready||session!=bench_session)return 0;bench_busy=1;return 1;
}
void W_Motor_BenchAbort(uint8 reason){(void)reason;bench_busy=bench_ready=0;bench_aborts++;}
void W_Motor_BenchService(void){assert(bench_busy&&!imu_fault);bench_services++;}
void W_Motor_Tick1ms(void){} void key_scanner(void){}
void Y_Motor_EncoderUpdate5ms(void){} void Y_Motor_EncoderClear(void){}
void imu_update_gyro(void){ticks+=work_ticks;} void attitude_update_rate(void){}
void attitude_tick1ms(void){}void attitude_update(void){}
int32 Y_Motor_GetTotalCount(void){return 0;}float Y_Motor_GetSpeedMps(void){return 0;}
float Y_Motor_CountToMeter(int32 x){return x;}float Y_Motor_Count20msToMps(float x){return x;}
float imu_get_angle_yaw(void){return 0;}float imu_get_angle_element(void){return 0;}
int path_nav_running(void){return 0;}int path_nav_recording(void){return 0;}
void path_nav_status(path_status_t *out){memset(out,0,sizeof(*out));}
void path_nav_tick(float a,float b,float c,float d){(void)a;(void)b;(void)c;(void)d;}
void path_nav_finish_record(void){}
void vision_feedback_publish(const vision_feedback_t *f){(void)f;}
uint8 vision_result_read(vision_result_t *r){(void)r;return 0;}
vision_core_state_t vision_core_state(void){return VISION_CORE_OFF;}
uint32 vision_core_heartbeat(void){return 0;}
void vofa_snapshot(void){}void vofa_tick1ms(void){}
static void zero_outputs(void){assert(!g_motor_a&&!g_motor_b&&!g_motor_c);assert(!g_pwm_roll&&!g_pwm_pitch&&!g_pwm_yaw);}
static void tick_us(uint32 us){ticks+=(uint64)us*100;control_loop();}
static void reset_case(void){
 control_stop();memset(&g_param,0,sizeof(g_param));memset(&att,0,sizeof(att));memset(&imu,0,sizeof(imu));
 g_imu_ok=calib_ok=converged=1;imu_fault=diverged=restart=path_busy=0;link_lost=1;
 g_param.roll_protect_angle=20;g_param.pitch_protect_angle=50;g_roll_zero=g_pitch_zero=0;
 g_param.r_rate_kp=3;g_param.r_angle_kp=2;g_param.p_rate_kp=4;g_param.p_angle_kp=2;
 g_param.jog_duty_drive=800;g_param.jog_duty_fly=200;g_param.fly_speed_limit=5000;
 g_param.r_rate_notch_hz=30;g_param.r_rate_notch_q=1;g_param.r_rate_notch_db=0;
 s_jog_cooldown=0;rpm_a=rpm_b=0;work_ticks=0;stm_hz=100000000.0f;
 nonzero_writes=release_writes=bench_services=bench_aborts=0;bench_ready=bench_busy=0;
}
static void preview_cases(void){
 control_preview_diag_t d;reset_case();
 assert(!control_test_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));
 assert(!control_jog_start(MOTOR_JOG_C,1));
 assert(control_test_last_status()==CTRL_TEST_STATUS_ADAPT_LOCKED);
 assert(!control_preview_start(TUNE_AXIS_YAW,TUNE_RING_RATE));
 assert(!control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_VEL));
 assert(!control_preview_start((tune_axis_t)-1,TUNE_RING_RATE));
 assert(!control_preview_start(TUNE_AXIS_ROLL,(tune_ring_t)-1));
 assert(control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));
 assert(control_test_running()&&control_preview_running());
 assert(!control_preview_start(TUNE_AXIS_PITCH,TUNE_RING_RATE));
 att.roll_rate=2;tick_us(1000);control_preview_status(&d);
 assert(d.active&&d.requested_output==6&&d.rate_dps==2&&!d.flywheel_link_ok);zero_outputs();
 att.roll_rate=-3;tick_us(1000);control_preview_status(&d);assert(d.requested_output==-9);
 assert(!nonzero_writes&&!release_writes);
 /* Real gain refresh changes real computation. Output does not feed simulated attitude. */
 g_param.r_rate_kp=5;tick_us(1000);control_preview_status(&d);assert(d.requested_output==-15);
 control_test_stop();assert(!control_preview_running());control_preview_status(&d);
 assert(d.reason==CTRL_TEST_STATUS_MANUAL&&d.requested_output==0);zero_outputs();
 tick_us(1000);assert(!control_preview_running());
 reset_case();att.roll=1;assert(control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_ANGLE));
 tick_us(1000);control_preview_status(&d);assert(d.requested_output==6);control_stop();
 reset_case();att.pitch=2;assert(control_preview_start(TUNE_AXIS_PITCH,TUNE_RING_ANGLE));
 tick_us(1000);control_preview_status(&d);assert(d.requested_output==-16);zero_outputs();
 att.pitch=1;tick_us(1000);control_preview_status(&d);assert(d.requested_output==-8);
 control_stop();assert(p_rate_pid.integrator==0&&p_rate_pid.out==0);
 /* Real-time duration, not invocation count; no one-tick output after expiry. */
 reset_case();ticks=((uint64)1<<32)-100000;assert(control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));
 att.roll_rate=2;tick_us(9999000);assert(control_preview_running());
 tick_us(1000);control_preview_status(&d);assert(!d.active&&d.reason==CTRL_TEST_STATUS_EXPIRED&&d.elapsed_ms==10000);
 tick_us(1000);assert(!control_preview_running());zero_outputs();
 /* A whole low-counter wrap does not resurrect a pulse. */
 assert(control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));ticks+=((uint64)1<<32)+100000;control_loop();
 assert(!control_preview_running());
 reset_case();assert(control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));imu_fault=1;tick_us(1000);
 control_preview_status(&d);assert(!d.active&&d.reason==CTRL_TEST_STATUS_IMU_LOST);imu_fault=0;tick_us(1000);assert(!control_preview_running());
 assert(control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));att.roll=NAN;tick_us(1000);assert(!control_preview_running());
 reset_case();assert(control_preview_start(TUNE_AXIS_PITCH,TUNE_RING_RATE));att.roll=80;tick_us(1000);assert(!control_preview_running());
 reset_case();assert(control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));g_param.r_rate_kp=INFINITY;tick_us(1000);
 control_preview_status(&d);assert(!d.active&&d.reason==CTRL_TEST_STATUS_OUTPUT_INVALID);
 reset_case();assert(control_preview_start(TUNE_AXIS_PITCH,TUNE_RING_RATE));g_pitch_zero=NAN;tick_us(1000);assert(!control_preview_running());
 reset_case();g_imu_ok=0;assert(!control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));
 reset_case();calib_ok=0;assert(!control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));
 reset_case();converged=0;assert(!control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));
 reset_case();diverged=1;assert(!control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));
 reset_case();restart=1;assert(!control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));
 reset_case();att.roll=79;g_param.roll_protect_angle=79;att.pitch_rate=2e38f;imu.gyro_z=2e38f;
 assert(!control_preview_start(TUNE_AXIS_PITCH,TUNE_RING_RATE));
 reset_case();att.roll=21;assert(!control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));
 reset_case();att.pitch=51;assert(!control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));
 /* Direct state bypass still hits controller zero lock and clears PID. */
 reset_case();s_test_running=s_control_test_active=1;start_flag=START_BALANCE;
 s_jog_target=MOTOR_JOG_C;s_jog_duty=9000;g_motor_c=9000;control_loop();zero_outputs();assert(!s_test_running&&!s_jog_target);
 assert(!nonzero_writes&&!release_writes);
 assert(control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));s_test_running=1;control_loop();
 assert(!control_preview_running());zero_outputs();assert(!nonzero_writes&&!release_writes);
}
static void bench_cases(void){
 control_timing_diag_t t;reset_case();s_control_clock_seen=0;s_control_timing.valid=0;
 bench_ready=1;assert(!control_bench_b_start(100,100,7));tick_us(1000);assert(!control_bench_b_start(100,100,7));
 tick_us(1000);control_timing_status(&t);assert(t.valid&&t.period_us==1000&&t.work_us==0);
 assert(control_ticks_us((uint64)0xffffffffu * 100u + 100u)==0xffffffffu);
 assert(bench_ready); /* Passive zero did not erase physical LOCAL_READY. */
 assert(!control_bench_b_start(100,100,0));assert(!control_bench_b_start(501,100,7));assert(!control_bench_b_start(100,151,7));
 assert(!control_bench_b_start(100,100,6));assert(passed_session==6);
 assert(control_bench_b_start(100,100,7));assert(passed_session==7&&control_bench_b_running());
 assert(!control_preview_start(TUNE_AXIS_ROLL,TUNE_RING_RATE));
 tick_us(1000);assert(bench_services==1);zero_outputs();
 imu_fault=1;tick_us(1000);assert(!bench_busy&&bench_services==1&&bench_aborts>0);zero_outputs();
 reset_case();tick_us(1000);tick_us(1000);bench_ready=1;assert(control_bench_b_start(100,100,7));
 tick_us(2501);assert(!bench_busy&&!bench_services);control_timing_status(&t);assert(t.overrun_count>0);
 reset_case();tick_us(1000);tick_us(1000);bench_ready=1;assert(control_bench_b_start(100,100,7));
 work_ticks=100000;tick_us(1000);assert(!bench_busy&&!bench_services);zero_outputs();
 reset_case();tick_us(1000);tick_us(1000);bench_ready=1;assert(control_bench_b_start(100,100,7));
 control_bench_b_stop();assert(!bench_busy&&!bench_ready);zero_outputs();
 assert(!nonzero_writes&&!release_writes);
}
#ifdef HOST_UNLOCK_JOG_TEST
static void jog_cases(void){
 reset_case();assert(control_jog_start(MOTOR_JOG_C,1));assert(!control_jog_start(MOTOR_JOG_C,1));
 assert(control_test_last_status()==CTRL_TEST_STATUS_BUSY);
 ticks+=14900000;control_jog_run();assert(control_jog_running()==MOTOR_JOG_C&&g_motor_c==800);
 ticks+=100000;control_jog_run();assert(!control_jog_running());
 assert(control_jog_stop_reason()==CTRL_TEST_STATUS_EXPIRED);zero_outputs();
 assert(!control_jog_start(MOTOR_JOG_C,1)&&control_test_last_status()==CTRL_TEST_STATUS_COOLDOWN);
 ticks+=199900000;assert(!control_jog_start(MOTOR_JOG_C,1));
 ticks+=100000;assert(control_jog_start(MOTOR_JOG_C,1));imu_fault=1;control_jog_run();
 assert(!control_jog_running()&&control_jog_stop_reason()==CTRL_TEST_STATUS_IMU_LOST);zero_outputs();
 imu_fault=0;ticks+=300000000;control_jog_run();assert(!control_jog_running());
 reset_case();assert(control_jog_start(MOTOR_JOG_C,1));att.pitch=NAN;control_jog_run();assert(!control_jog_running());
 reset_case();assert(control_jog_start(MOTOR_JOG_C,1));att.roll=80;control_jog_run();assert(!control_jog_running());
 reset_case();assert(control_jog_start(MOTOR_JOG_C,1));ticks+=((uint64)1<<32);control_jog_run();assert(!control_jog_running());
 reset_case();link_lost=0;assert(control_jog_start(MOTOR_JOG_B,1));link_lost=1;control_jog_run();assert(!control_jog_running());
 reset_case();link_lost=0;assert(control_jog_start(MOTOR_JOG_B,1));rpm_b=6000;control_jog_run();assert(!control_jog_running());
 reset_case();assert(control_jog_start(MOTOR_JOG_C,1));control_jog_stop();assert(control_jog_stop_reason()==CTRL_TEST_STATUS_MANUAL);
 assert(!control_jog_start(MOTOR_JOG_C,1));
}
#endif
int main(void){
#ifdef HOST_UNLOCK_JOG_TEST
 jog_cases();puts("PASS bounded dormant jog (host-only gate override)");
#else
 assert(VEHICLE_MOTOR_OUTPUT_ALLOWED==0);preview_cases();bench_cases();puts("PASS real controller preview, lock, faults, clocks and bench supervision");
#endif
 return 0;
}
