/* Execute actual control.c with only hardware/sensor/clock boundaries mocked.
 * No plant simulation, no proof of closed-loop sign or stability. */
static void powered_rate_cases(void)
{
    control_c_bench_diag_t d;
    uint32 session;
    float previous;
    assert(C_BENCH_POWERED_RATE_PROFILE == 1);
    assert(C_BENCH_ONLY_PROFILE == 1 && VEHICLE_MOTOR_OUTPUT_ALLOWED == 0);
    assert(VEHICLE_YAW_ENABLED == 0);
    cb_ready(); assert(cb_status().signal_allowed == 2);
    /* No old command can accidentally convert an isolated check into power. */
    assert(!cb_start(C_BENCH_SIGNAL_RAW, 1, 0, 0, 100, 100));
    assert(cb_status().reason == C_BENCH_REASON_LOCKED);
    assert(!cb_start(C_BENCH_SIGNAL_RATE, 0, 1, 0, 100, 100));
    assert(cb_status().reason == C_BENCH_REASON_LOCKED);
    assert(!control_bench_b_start(100,100,7));
    assert(!control_test_start(TUNE_AXIS_PITCH,TUNE_RING_RATE));
    assert(!control_jog_start(MOTOR_JOG_C,1));
    assert(!control_balance_start());
    /* Powered start has no inherited menu gains, nonzero target, I, or defaults. */
    assert(!cb_start(4,0,0,0,100,100));
    assert(!cb_start(4,1,1,0,100,100));
    assert(!cb_start(4,0,1,.01f,100,100));
    assert(!cb_start(4,0,20.01f,0,100,100));
    assert(!cb_start(4,0,1,0,1001,100));
    assert(!cb_start(4,0,1,0,100,201));
    assert(!cb_start(4,NAN,1,0,100,100));
    assert(!cb_start(4,0,INFINITY,0,100,100));
    assert(!cb_start(4,0,1,NAN,100,100));
    assert(!cb_start(6,0,1,0,100,100));
    att.pitch_rate=5; assert(!cb_start(4,0,1,0,100,100));
    att.pitch_rate=-5; assert(!cb_start(5,0,1,0,100,100));
    cb_quiet();
    for (int mode=4; mode<=5; ++mode) {
        cb_ready(); att.pitch_rate=1;
        session=cb_start(mode,0,20,0,1000,200); assert(session);
        previous=0;
        for (int i=1; i<=80; ++i) {
            cb_ticks(1); d=cb_status();
            assert(d.active && d.mode==mode && d.signal_allowed==2);
            assert(fabsf(d.calculated-previous)<=4.001f);
            assert(fabsf(d.calculated)<=200 && (d.applied>=-200 && d.applied<=200));
            assert(!g_motor_a && !g_motor_b && !release_writes && !nonzero_writes);
            assert(!s_c_integral);
            previous=d.calculated;
            if(i%50==0) assert(control_c_bench_heartbeat(session,i));
        }
        assert((mode==4 && d.applied==-20) || (mode==5 && d.applied==20));
        /* Stop is a physical sink write. Delayed keep cannot resurrect output. */
        control_c_bench_stop(); cb_quiet();
        assert(!control_c_bench_heartbeat(session,1000));
        cb_ticks(200); cb_quiet(); assert(!control_c_bench_running());
        assert(!control_c_bench_start((control_c_bench_mode_t)mode,0,1,0,100,100,session-1));
    }
    cb_ready(); session=cb_start(4,0,20,0,1000,200); assert(session);
    att.pitch_rate=99; previous=0;
    for(int i=1;i<=199;i++) {
        cb_ticks(1); d=cb_status();
        assert(d.active&&fabsf(d.calculated)<=200);
        assert(fabsf(d.calculated-previous)<=4.001f);previous=d.calculated;
    }
    assert(d.applied==-200); cb_ticks(1);
    assert(cb_status().reason==C_BENCH_REASON_LEASE);cb_quiet();
    assert(!control_c_bench_heartbeat(session,1));cb_ticks(50);cb_quiet();
    /* Fixed deadline cannot be extended by valid heartbeats. */
    cb_ready();session=cb_start(5,0,2,0,1000,200);assert(session);att.pitch_rate=1;
    for(int i=1;i<=1000;i++){cb_ticks(1);if(i%50==0&&i<1000)assert(control_c_bench_heartbeat(session,i));}
    assert(cb_status().reason==C_BENCH_REASON_EXPIRED);cb_quiet();
    /* Powered runs exercise each common protection, not merely dry coverage. */
    cb_ready();assert(cb_start(4,0,1,0,1000,200));att.pitch_rate=NAN;cb_ticks(1);
    assert(cb_status().reason==C_BENCH_REASON_SENSOR);cb_quiet();
    cb_ready();assert(cb_start(4,0,1,0,1000,200));att.pitch=15;cb_ticks(1);
    assert(cb_status().reason==C_BENCH_REASON_SENSOR);cb_quiet();
    cb_ready();assert(cb_start(4,0,1,0,1000,200));imu_sequence_frozen=1;cb_ticks(6);
    assert(cb_status().reason==C_BENCH_REASON_SENSOR);cb_quiet();
    cb_ready();assert(cb_start(4,0,1,0,1000,200));encoder_sequence_frozen=1;cb_ticks(30);
    assert(cb_status().reason==C_BENCH_REASON_ENCODER);cb_quiet();
    cb_ready();assert(cb_start(4,0,1,0,1000,200));encoder_count=101;cb_ticks(1);
    assert(cb_status().reason==C_BENCH_REASON_ENCODER);cb_quiet();
    cb_ready();assert(cb_start(4,0,1,0,1000,200));tick_us(2500);
    assert(cb_status().reason==C_BENCH_REASON_TIMING);cb_quiet();
    cb_ready();assert(cb_start(4,0,1,0,1000,200));att.pitch_rate=1;keepzero_ticks=100000;cb_ticks(1);
    assert(cb_status().reason==C_BENCH_REASON_TIMING && !c_raw_nonzero);cb_quiet();
    cb_ready();assert(cb_start(4,0,1,0,1000,200));g_pitch_zero=1;cb_ticks(1);
    assert(cb_status().reason==C_BENCH_REASON_SENSOR);cb_quiet();
    /* Zero samples do not imply encoder connected or hardware motion stopped. */
    puts("PASS powered rate modes: polarity, fixed limits, lease, expiry, replay, faults and other-motor locks");
}
