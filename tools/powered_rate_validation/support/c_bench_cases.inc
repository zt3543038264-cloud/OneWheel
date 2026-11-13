/* Actual control.c; only sensors/STM/driver boundary mocked. No motor model. */
static control_c_bench_diag_t cb_status(void){control_c_bench_diag_t d;control_c_bench_status(&d);return d;}
static void cb_ready(void){reset_case();s_control_clock_seen=0;s_control_timing.valid=0;for(int i=0;i<25;i++)tick_us(1000);}
static uint32 cb_start(int mode,float target,float kp,float ki,uint16 duration,uint16 limit){
 return control_c_bench_start((control_c_bench_mode_t)mode,target,kp,ki,duration,limit,cb_status().session);
}
static void cb_ticks(int n){for(int i=0;i<n;i++)tick_us(1000);}
static void cb_quiet(void){assert(!c_raw_applied&&!g_motor_a&&!g_motor_b&&!g_motor_c);assert(!release_writes&&!nonzero_writes);}
static void cb_dry_and_parameters(void){
 control_c_bench_diag_t d;uint32 session;cb_ready();
 assert(!control_bench_b_start(100,100,7));
 assert((session=cb_start(C_BENCH_DRY_RAW,200,0,0,300,250)));
 cb_ticks(2);d=cb_status();assert(d.active&&d.calculated==200&&d.applied==0);cb_quiet();
 assert(!c_raw_calls);assert(!control_c_bench_start(C_BENCH_DRY_RAW,1,0,0,30,100,session));
 assert(!cb_start(-1,NAN,0,0,1,1));d=cb_status();assert(d.active&&d.session==session&&d.target==200);
 assert(!control_preview_start(TUNE_AXIS_PITCH,TUNE_RING_RATE));
 assert(!control_test_start(TUNE_AXIS_PITCH,TUNE_RING_RATE));
 assert(!control_jog_start(MOTOR_JOG_C,1));assert(!control_balance_start());
 assert(!control_odometry_counter_reset());assert(!control_path_record_start(0));
 control_stop();d=cb_status();assert(!d.active&&d.reason==C_BENCH_REASON_MANUAL&&d.calculated==0);cb_quiet();
 assert(!control_c_bench_start(C_BENCH_DRY_RAW,1,0,0,30,100,session-1));
 assert(cb_status().reason==C_BENCH_REASON_SESSION);
 assert(!cb_start(-1,0,0,0,30,100));assert(!cb_start(4,0,0,0,30,100));
 assert(!cb_start(C_BENCH_DRY_RAW,NAN,0,0,30,100));assert(!cb_start(C_BENCH_DRY_RATE,INFINITY,0,0,30,100));
 assert(!cb_start(C_BENCH_DRY_RATE,0,NAN,0,30,100));assert(!cb_start(C_BENCH_DRY_RATE,0,0,INFINITY,30,100));
 assert(!cb_start(C_BENCH_DRY_RATE,31,0,0,30,100));assert(!cb_start(C_BENCH_DRY_RATE,0,101,0,30,100));
 assert(!cb_start(C_BENCH_DRY_RATE,0,0,1001,30,100));assert(!cb_start(C_BENCH_DRY_RATE,0,-1,0,30,100));
 assert(!cb_start(C_BENCH_DRY_RAW,0,0,0,0,100));assert(!cb_start(C_BENCH_DRY_RAW,0,0,0,3001,100));
 assert(!cb_start(C_BENCH_DRY_RAW,0,0,0,30,0));assert(!cb_start(C_BENCH_DRY_RAW,0,0,0,30,501));
 assert(!cb_start(C_BENCH_DRY_RAW,101,0,0,30,100));assert(!cb_start(C_BENCH_DRY_RAW,-101,0,0,30,100));
 assert(!cb_start(C_BENCH_DRY_RAW,0,1,0,30,100));assert(!cb_start(C_BENCH_DRY_RAW,0,0,1,30,100));
 assert(cb_start(C_BENCH_DRY_RATE,0,0,0,30,100));att.pitch_rate=2;cb_ticks(2);assert(cb_status().calculated==0);
 control_test_stop();assert(!control_c_bench_running());
 assert(control_preview_start(TUNE_AXIS_PITCH,TUNE_RING_RATE));assert(!cb_start(C_BENCH_DRY_RAW,1,0,0,30,100));
 control_stop();s_run_active=1;assert(!cb_start(0,0,0,0,30,100));s_run_active=0;
 s_remote_active=1;assert(!cb_start(0,0,0,0,30,100));s_remote_active=0;
 path_busy=1;assert(!cb_start(0,0,0,0,30,100));path_busy=0;
 s_jog_target=MOTOR_JOG_C;assert(!cb_start(0,0,0,0,30,100));s_jog_target=MOTOR_JOG_NONE;
 s_test_running=1;assert(!cb_start(0,0,0,0,30,100));s_test_running=0;
 assert(cb_start(0,0,0,0,30,100));control_jog_stop();assert(!control_c_bench_running());
 assert(cb_start(0,0,0,0,30,100));control_preview_stop();assert(!control_c_bench_running());
 assert(cb_start(0,0,0,0,30,100));s_preview.active=1;cb_ticks(1);assert(!control_c_bench_running());control_stop();
 cb_quiet();
}
static void cb_time_and_replay(void){
 uint32 session;control_c_bench_diag_t d;cb_ready();
 session=cb_start(0,20,0,0,300,50);assert(session);cb_ticks(199);assert(control_c_bench_running());
 assert(!control_c_bench_heartbeat(session-1,1));assert(!control_c_bench_heartbeat(session,0));
 cb_ticks(1);d=cb_status();assert(!d.active&&d.reason==C_BENCH_REASON_LEASE&&d.elapsed_ms==200);cb_quiet();
 assert(!control_c_bench_heartbeat(session,1));cb_ticks(10);assert(!control_c_bench_running());
 session=cb_start(0,1,0,0,300,50);assert(session);
 assert(control_c_bench_heartbeat(session,0xffffffffu));
 assert(!control_c_bench_heartbeat(session,0xffffffffu));
 assert(!control_c_bench_heartbeat(session,1));control_stop();
 session=cb_start(0,20,0,0,3000,50);assert(session);cb_ticks(100);assert(control_c_bench_heartbeat(session,1));
 cb_ticks(100);assert(!control_c_bench_heartbeat(session,1));assert(!control_c_bench_heartbeat(session,0));
 cb_ticks(99);assert(control_c_bench_running());cb_ticks(1);assert(cb_status().reason==C_BENCH_REASON_LEASE);
 session=cb_start(0,20,0,0,30,50);assert(session);cb_ticks(29);assert(control_c_bench_running());
 cb_ticks(1);d=cb_status();assert(!d.active&&d.reason==C_BENCH_REASON_EXPIRED&&d.elapsed_ms==30);
 session=cb_start(0,20,0,0,3000,50);assert(session);for(int i=1;i<=3000;i++){
  cb_ticks(1);if(i%100==0&&i<3000)assert(control_c_bench_heartbeat(session,(uint32)i));
 }assert(cb_status().reason==C_BENCH_REASON_EXPIRED);
 /* Exact lease boundary in foreground: expiry is checked before accepting keepalive. */
 session=cb_start(0,1,0,0,300,50);assert(session);ticks+=20000000;
 assert(!control_c_bench_heartbeat(session,1));assert(cb_status().reason==C_BENCH_REASON_LEASE);cb_quiet();
 cb_ready();ticks=((uint64)4<<32)-2600000;cb_ticks(25); /* period guard needs normal STM samples */
 session=cb_start(0,1,0,0,30,50);assert(session);cb_ticks(30);assert(cb_status().reason==C_BENCH_REASON_EXPIRED);
 cb_ready();session=cb_start(0,1,0,0,3000,50);assert(session);ticks+=((uint64)1<<32);control_loop();
 assert(!control_c_bench_running());cb_quiet();
 cb_ready();assert(cb_start(0,1,0,0,300,50));tick_us(CONTROL_BENCH_MAX_GAP_US);
 assert(cb_status().reason==C_BENCH_REASON_TIMING);cb_quiet();
 cb_ready();ticks+=(uint64)CONTROL_BENCH_MAX_GAP_US*100;
 assert(!cb_start(0,1,0,0,300,50));assert(cb_status().reason==C_BENCH_REASON_TIMING);
 cb_ready();assert(cb_start(0,1,0,0,300,50));work_ticks=100000;cb_ticks(1);
 assert(cb_status().reason==C_BENCH_REASON_TIMING);assert(!c_raw_nonzero);cb_quiet();
 cb_ready();assert(cb_start(0,1,0,0,300,50));keepzero_ticks=100000;cb_ticks(1);
 assert(cb_status().reason==C_BENCH_REASON_TIMING);assert(!c_raw_nonzero);cb_quiet();
 cb_ready();assert(cb_start(0,1,0,0,300,50));post_write_ticks=100000;cb_ticks(1);
 assert(cb_status().reason==C_BENCH_REASON_TIMING);cb_quiet();
}
static void cb_sensor_and_encoder(void){
 cb_ready();s_c_imu_seen=0;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();s_c_encoder_seen=0;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();encoder_count=3;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();encoder_count=-3;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();assert(cb_start(0,1,0,0,100,50));encoder_count=101;cb_ticks(1);assert(cb_status().reason==C_BENCH_REASON_ENCODER);
 cb_ready();assert(cb_start(0,1,0,0,100,50));encoder_count=-101;cb_ticks(1);assert(cb_status().reason==C_BENCH_REASON_ENCODER);
 cb_ready();assert(cb_start(0,1,0,0,100,50));encoder_sequence_frozen=1;cb_ticks(30);assert(cb_status().reason==C_BENCH_REASON_ENCODER);
 cb_ready();assert(cb_start(0,1,0,0,100,50));encoder_sequence=0;encoder_sequence_frozen=1;cb_ticks(1);assert(!control_c_bench_running());
 cb_ready();assert(cb_start(0,1,0,0,100,50));imu_sequence_frozen=1;cb_ticks(6);assert(cb_status().reason==C_BENCH_REASON_SENSOR);
 cb_ready();assert(cb_start(0,1,0,0,100,50));imu_fault=1;cb_ticks(1);assert(cb_status().sensor_reason==CTRL_TEST_STATUS_IMU_LOST);
 imu_fault=0;cb_ticks(10);assert(!control_c_bench_running());
 cb_ready();calib_ok=0;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();converged=0;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();diverged=1;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();g_imu_ok=0;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();restart=1;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();att.roll=80;g_roll_zero=80;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();att.pitch=15;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();att.roll=-15;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();att.pitch_rate=100;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();att.roll_rate=100;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();imu.gyro_z=-100;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();att.pitch_rate=NAN;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();att.roll_rate=INFINITY;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();imu.gyro_z=NAN;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();g_pitch_zero=NAN;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();assert(cb_start(0,1,0,0,100,50));att.pitch=NAN;cb_ticks(1);
 assert(cb_status().reason==C_BENCH_REASON_SENSOR);control_c_bench_stop();
 assert(isfinite(cb_status().angle_deg)&&isfinite(cb_status().rate_dps));
 cb_ready();g_roll_zero=INFINITY;assert(!cb_start(0,1,0,0,100,50));
 cb_ready();assert(cb_start(0,1,0,0,100,50));g_pitch_zero=1;cb_ticks(1);assert(cb_status().reason==C_BENCH_REASON_SENSOR);
 cb_quiet();
}
static void cb_pi(void){
 float prev;control_c_bench_diag_t d;cb_ready();
 /* Original Pitch Euler rate p+(q sin(phi)+r cos(phi))*tan(theta), no axis edits. */
 att.pitch=3;att.roll=5;att.pitch_rate=2;att.roll_rate=4;imu.gyro_z=6;
 float expected=2+(4*sinf(3*.017453292519943295f)+6*cosf(3*.017453292519943295f))*tanf(5*.017453292519943295f);
 assert(cb_start(C_BENCH_DRY_RATE,0,3,0,100,100));cb_ticks(6);d=cb_status();
 assert(fabsf(d.rate_dps-expected)<1e-5f&&fabsf(d.calculated+3*expected)<1e-5f);cb_quiet();
 control_stop();assert(s_c_integral==0&&s_c_pi_seq==0);
 cb_ready();assert(cb_start(C_BENCH_DRY_RATE,30,100,1000,100,50));prev=0;
 for(int i=0;i<20;i++){cb_ticks(1);d=cb_status();assert(d.calculated-prev<=4.001f&&d.calculated>=prev&&d.calculated<=50);prev=d.calculated;}
 assert(s_c_integral==0); /* Saturation and slew must not wind up integral. */
 assert(!d.applied&&!c_raw_calls);control_stop();
 cb_ready();assert(cb_start(C_BENCH_DRY_RATE,1,0,100,100,50));cb_ticks(10);d=cb_status();
 assert(d.calculated>0.5f&&d.calculated<1.2f&&s_c_integral==d.calculated);
 float held=d.calculated;imu_sequence_frozen=1;cb_ticks(1);assert(cb_status().calculated==held);
 control_stop();assert(s_c_integral==0);
 assert(cb_start(C_BENCH_DRY_RATE,0,0,0,100,50));cb_ticks(2);assert(cb_status().calculated==0);
 control_stop();cb_quiet();
}
static void cb_signal(void){
 cb_ready();
#if C_BENCH_SIGNAL_OUTPUT_ALLOWED
 assert(cb_start(C_BENCH_SIGNAL_RAW,1,0,0,100,500));cb_ticks(1);assert(c_raw_applied==1&&g_motor_c==1&&!g_motor_a&&!g_motor_b);control_stop();
 assert(cb_start(C_BENCH_SIGNAL_RAW,-500,0,0,100,500));cb_ticks(1);assert(c_raw_applied==-500);control_stop();cb_quiet();
 assert(cb_start(C_BENCH_SIGNAL_RAW,500,0,0,30,500));cb_ticks(29);int writes=c_raw_nonzero;
 cb_ticks(1);assert(c_raw_nonzero==writes&&cb_status().reason==C_BENCH_REASON_EXPIRED);cb_quiet();
 assert(cb_start(C_BENCH_SIGNAL_RAW,500,0,0,300,500));cb_ticks(199);writes=c_raw_nonzero;
 cb_ticks(1);assert(c_raw_nonzero==writes&&cb_status().reason==C_BENCH_REASON_LEASE);cb_quiet();
 cb_ready();assert(cb_start(C_BENCH_SIGNAL_RATE,1,0,100,100,100));c_driver_hold=1;cb_ticks(20);
 float integral=s_c_integral;cb_ticks(2);assert(s_c_integral==integral);control_stop();cb_quiet();
 cb_ready();assert(cb_start(C_BENCH_SIGNAL_RAW,500,0,0,300,500));keepzero_ticks=100000;cb_ticks(1);
 assert(!c_raw_nonzero&&cb_status().reason==C_BENCH_REASON_TIMING);cb_quiet();
#else
 assert(!cb_start(C_BENCH_SIGNAL_RAW,1,0,0,100,500));assert(cb_status().reason==C_BENCH_REASON_LOCKED);
 assert(!cb_start(C_BENCH_SIGNAL_RATE,1,1,0,100,500));assert(cb_status().reason==C_BENCH_REASON_LOCKED);cb_quiet();
#endif
}
static void cb_stop_idempotent(void){
 control_c_bench_diag_t before,after;int writes;cb_ready();
 assert(cb_start(C_BENCH_DRY_RAW,20,0,0,1,50));cb_ticks(1);
 before=cb_status();assert(!before.active&&before.reason==C_BENCH_REASON_EXPIRED);
 /* Deliberately inconsistent host boundary: STOP must really write PWM zero,
  * not merely report an already inactive controller. */
 c_raw_applied=123;s_c_bench.applied=123;g_motor_c=123;writes=stop_writes;
 control_c_bench_stop();after=cb_status();
 assert(stop_writes>writes&&!c_raw_applied&&!after.applied&&!g_motor_c);
 assert(after.reason==before.reason&&after.state==before.state&&after.session==before.session);
 writes=stop_writes;control_c_bench_stop();after=cb_status();
 assert(stop_writes>writes&&!after.active&&!after.applied);
 assert(after.reason==before.reason&&after.state==before.state);cb_quiet();
}
static void c_bench_cases(void){
 assert(VEHICLE_MOTOR_OUTPUT_ALLOWED==0);cb_dry_and_parameters();cb_time_and_replay();
 cb_sensor_and_encoder();cb_pi();cb_signal();cb_stop_idempotent();
 cb_ready();s_c_bench.session=0xffffffffu;assert(!cb_start(0,0,0,0,100,10));assert(cb_status().reason==C_BENCH_REASON_SESSION);
}
