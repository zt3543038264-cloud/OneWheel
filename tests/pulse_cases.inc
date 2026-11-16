#include <stdlib.h>
static void pulse_cases(void) {
 control_c_bench_diag_t d; uint32 session;
 for(int sign=-1;sign<=1;sign+=2) for(int peak=200;peak<=1200;peak+=200) {
  cb_ready();session=cb_start(6,(float)(sign*peak),0,0,400,peak);assert(session);
  for(int i=1;i<=400;i++) {
   cb_ticks(1);d=cb_status();
   assert(!g_motor_a&&!g_motor_b&&!release_writes&&!nonzero_writes);
   if(i<400) {
    float envelope=i<100?(float)i/100.0f:i>300?(float)(400-i)/100.0f:1.0f;
    assert(d.active && d.mode==6 && d.signal_allowed==2);
    assert(fabsf(d.calculated-sign*peak*envelope)<.02f);
    assert(d.applied*sign>=0&&abs(d.applied)<=peak);
    if(i%50==0) assert(control_c_bench_heartbeat(session,i/50));
   }
  }
  assert(!d.active&&d.reason==C_BENCH_REASON_EXPIRED);cb_quiet();
  assert(!control_c_bench_heartbeat(session,100));cb_ticks(20);cb_quiet();
 }
 cb_ready();assert(!cb_start(6,1201,0,0,400,1201));
 assert(!cb_start(6,100,0,0,400,100));assert(!cb_start(6,300,0,0,400,300));
 assert(!cb_start(6,0,0,0,400,200));assert(!cb_start(6,200,1,0,400,200));
 assert(!cb_start(6,200,0,1,400,200));assert(!cb_start(6,200,0,0,401,200));
 assert(!cb_start(6,200,0,0,399,200));assert(!cb_start(6,NAN,0,0,400,200));
 att.pitch_rate=5;assert(!cb_start(6,200,0,0,400,200));
 cb_ready();session=cb_start(6,1200,0,0,400,1200);assert(session);cb_ticks(200);
 assert(cb_status().reason==C_BENCH_REASON_LEASE);cb_quiet();
 assert(!control_c_bench_heartbeat(session,1));cb_ticks(20);cb_quiet();
 cb_ready();session=cb_start(6,-1200,0,0,400,1200);assert(session);cb_ticks(20);
 control_c_bench_stop();cb_quiet();assert(!control_c_bench_heartbeat(session,1));
 cb_ready();assert(cb_start(6,200,0,0,400,200));att.pitch=15;cb_ticks(1);
 assert(cb_status().reason==C_BENCH_REASON_SENSOR);cb_quiet();
 cb_ready();assert(cb_start(6,200,0,0,400,200));encoder_count=101;cb_ticks(1);
 assert(cb_status().reason==C_BENCH_REASON_ENCODER);cb_quiet();
 cb_ready();assert(cb_start(6,200,0,0,400,200));tick_us(2500);
 assert(cb_status().reason==C_BENCH_REASON_TIMING);cb_quiet();
 cb_ready();assert(cb_start(6,200,0,0,400,200));imu_sequence_frozen=1;cb_ticks(6);
 assert(cb_status().reason==C_BENCH_REASON_SENSOR);cb_quiet();
 puts("PASS actual graded pulse controller");
}
