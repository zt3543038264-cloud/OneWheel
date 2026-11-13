/* Foreground-only commissioning commands. Mutating requests are never retried
 * by the supplied CLI. No Flash writes and no direct motor writes here. */
static uint8 commission_u32(const char *s, uint32 *out)
{
    uint32 value=0u, digit;
    const char *p=s;
    if (!p || !*p) return 0;
    for (;*p;++p) {
        if (*p<'0' || *p>'9') return 0;
        digit=(uint32)(*p-'0');
        if (value>429496729u || (value==429496729u && digit>5u)) return 0;
        value=value*10u+digit;
    }
    *out=value; return 1;
}
#include "Protocols/vofa_link_probe.inc"
#include "Protocols/vofa_keep_log_cfg.inc"
static uint8 commission_duty(const char *s, int16 *out)
{
    uint32 magnitude;
    uint8 negative=(uint8)(s && *s=='-');
    if (!s || !commission_u32(s+negative,&magnitude) || magnitude==0u || magnitude>500u) return 0;
    *out=(int16)(negative?-(int32)magnitude:(int32)magnitude);
    return 1;
}
/* C bench v1. Every success contains a fresh, actual controller snapshot,
 * correlated by the ordinary cfg sequence. No parser-owned output or PID. */
static void commission_c_reply(uint16 seq, const char *operation)
{
    control_c_bench_diag_t d;
    char payload[440];
    int n;
    control_c_bench_status(&d);
    n=snprintf(payload,sizeof(payload),
        "%s,1,%u,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%u,%u,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%.9g,%ld,%u",
        operation,(unsigned)d.active,(unsigned)d.mode,(unsigned)d.state,
        (unsigned)d.reason,(unsigned)d.sensor_reason,(unsigned long)d.session,
        (unsigned long)d.heartbeat_sequence,(unsigned long)d.elapsed_ms,
        (unsigned long)d.heartbeat_age_ms,(unsigned long)d.imu_age_ms,
        (unsigned)d.duration_ms,(unsigned)d.limit,(double)d.target,(double)d.kp,
        (double)d.ki,(double)d.angle_deg,(double)d.rate_dps,(int)d.encoder20ms,
        (double)d.calculated,(long)d.applied,(unsigned)d.signal_allowed);
    if (n<0 || n>=(int)sizeof(payload)) {
        (void)cfg_reply(seq,"err","STATUS_TOO_LONG,No state inferred"); return;
    }
    (void)cfg_reply(seq,"ok",payload);
}
static void commission_c_blocked(uint16 seq)
{
    control_c_bench_diag_t d;
    char payload[80];
    control_c_bench_status(&d);
    (void)snprintf(payload,sizeof(payload),"CBENCH_BLOCKED,%u,%u",
                   (unsigned)d.reason,(unsigned)d.sensor_reason);
    (void)cfg_reply(seq,"err",payload);
}
static void commission_c_cfg(char **f, uint8 count, uint16 seq)
{
    uint32 session=0, heartbeat=0, duration, limit;
    float target,kp,ki;
    control_c_bench_mode_t mode;
    if (!strcmp(f[0],"cbench_status") && count==2u) {
        commission_c_reply(seq,f[0]);
    } else if (!strcmp(f[0],"cbench_stop") && count==2u) {
        control_c_bench_stop(); commission_c_reply(seq,f[0]);
    } else if (!strcmp(f[0],"cbench_keep") && count==4u) {
        if (!commission_u32(f[2],&session) || !session ||
            !commission_u32(f[3],&heartbeat) || !heartbeat) {
            keep_log_fields(session,heartbeat);
            (void)cfg_reply(seq,"err","BAD_CBENCH,Require nonzero session and heartbeat"); return;
        }
        keep_log_fields(session,heartbeat);
        if (!control_c_bench_heartbeat(session,heartbeat)) { keep_log_result(2u); commission_c_blocked(seq); return; }
        keep_log_result(1u);
        commission_c_reply(seq,f[0]);
    } else if (!strcmp(f[0],"cbench_start") && count==9u) {
        if (!strcmp(f[3],"dry_raw")) mode=C_BENCH_DRY_RAW;
        else if (!strcmp(f[3],"dry_rate")) mode=C_BENCH_DRY_RATE;
        else if (!strcmp(f[3],"signal_raw")) mode=C_BENCH_SIGNAL_RAW;
        else if (!strcmp(f[3],"signal_rate")) mode=C_BENCH_SIGNAL_RATE;
        else if (!strcmp(f[3],"powered_rate_pos")) mode=C_BENCH_POWERED_RATE_POS;
        else if (!strcmp(f[3],"powered_rate_neg")) mode=C_BENCH_POWERED_RATE_NEG;
        else { (void)cfg_reply(seq,"err","BAD_MODE,Require dry_raw dry_rate signal_raw signal_rate powered_rate_pos or powered_rate_neg"); return; }
        if (!commission_u32(f[2],&session) ||
            !cmd_parse_float_strict(f[4],&target) || !cmd_parse_float_strict(f[5],&kp) ||
            !cmd_parse_float_strict(f[6],&ki) || !commission_u32(f[7],&duration) ||
            duration<1u || duration>3000u || !commission_u32(f[8],&limit) ||
            limit<1u || limit>500u ||
            ((mode==C_BENCH_DRY_RAW || mode==C_BENCH_SIGNAL_RAW) &&
             (target<-(float)limit || target>(float)limit || kp!=0.0f || ki!=0.0f)) ||
            ((mode==C_BENCH_DRY_RATE || mode==C_BENCH_SIGNAL_RATE) &&
             (target<-30.0f || target>30.0f || kp<0.0f || kp>100.0f || ki<0.0f || ki>1000.0f))) {
            (void)cfg_reply(seq,"err","BAD_CBENCH,Explicit finite bounded configuration required"); return;
        }
        if (!control_c_bench_start(mode,target,kp,ki,(uint16)duration,(uint16)limit,session)) {
            commission_c_blocked(seq); return;
        }
        commission_c_reply(seq,f[0]);
    } else (void)cfg_reply(seq,"err","BAD_FORMAT,Unknown C bench operation or field count");
}
static void commission_cfg(char **f, uint8 count, uint16 seq)
{
    char payload[144];
    if (strncmp(f[0],"cbench_",7)==0) { commission_c_cfg(f,count,seq); return; }
    if (!strcmp(f[0],"preview_status") && count==2u) {
        control_preview_diag_t d;
        control_preview_status(&d);
        (void)snprintf(payload,sizeof(payload),"preview_status,1,%u,%u,%u,%u,%lu,%.6g,%.6g,%.6g,%u",
            (unsigned)d.active,(unsigned)d.axis,(unsigned)d.ring,(unsigned)d.reason,
            (unsigned long)d.elapsed_ms,(double)d.angle_deg,(double)d.rate_dps,
            (double)d.requested_output,(unsigned)d.flywheel_link_ok);
        (void)cfg_reply(seq,"ok",payload);
    }
    else if (!strcmp(f[0],"preview_start") && count==4u) {
        tune_axis_t axis;
        tune_ring_t ring;
        uint8 ok;
        if (!strcmp(f[2],"roll")) axis=TUNE_AXIS_ROLL;
        else if (!strcmp(f[2],"pitch")) axis=TUNE_AXIS_PITCH;
        else { (void)cfg_reply(seq,"err","BAD_AXIS,Only roll or pitch preview"); return; }
        if (!strcmp(f[3],"rate")) ring=TUNE_RING_RATE;
        else if (!strcmp(f[3],"angle")) ring=TUNE_RING_ANGLE;
        else { (void)cfg_reply(seq,"err","BAD_RING,Only rate or angle preview"); return; }
        ok=control_preview_start(axis,ring);
        if (ok) (void)cfg_reply(seq,"ok","preview_start,1,ZERO_OUTPUT");
        else { (void)snprintf(payload,sizeof(payload),"BLOCKED,%u",(unsigned)control_test_last_status()); (void)cfg_reply(seq,"err",payload); }
    }
    else if (!strcmp(f[0],"preview_stop") && count==2u) {
        control_preview_stop(); (void)cfg_reply(seq,"ok","preview_stop");
    }
    else if (!strcmp(f[0],"bench_status") && count==2u) {
        w_motor_bench_status_t d;
        W_Motor_BenchReadStatus(&d);
        (void)snprintf(payload,sizeof(payload),"bench_status,1,%u,%u,%u,%u,%u,%u,%lu,%lu,%d,%u",
            (unsigned)d.host_state,(unsigned)d.driver_state,(unsigned)d.ready,(unsigned)d.busy,
            (unsigned)d.reason,(unsigned)d.driver_reason,(unsigned long)d.session,
            (unsigned long)d.last_seq,(int)d.right_duty,(unsigned)d.elapsed_ms);
        (void)cfg_reply(seq,"ok",payload);
    }
    else if (!strcmp(f[0],"bench_start") && count==5u) {
        /* Obsolete/unbound requests are never accepted. A fresh displayed
         * driver session is required as the sixth field. */
        (void)cfg_reply(seq,"err","SESSION_REQUIRED,Read bench_status and supply session");
    }
    else if (!strcmp(f[0],"bench_start") && count==6u) {
        int16 duty;
        uint32 duration, session;
        if (!commission_duty(f[2],&duty) || !commission_u32(f[3],&duration) ||
            duration<1u || duration>150u || strcmp(f[4],"B") ||
            !commission_u32(f[5],&session) || !session) {
            (void)cfg_reply(seq,"err","BAD_BENCH,Require nonzero PWM +/-500 1..150ms B and session"); return;
        }
        if (control_bench_b_start(duty,(uint16)duration,session))
            (void)cfg_reply(seq,"ok","bench_start,1,REQUESTED");
        else { (void)snprintf(payload,sizeof(payload),"BLOCKED,%u",(unsigned)control_test_last_status()); (void)cfg_reply(seq,"err",payload); }
    }
    else if (!strcmp(f[0],"bench_stop") && count==2u) {
        control_bench_b_stop(); (void)cfg_reply(seq,"ok","bench_stop");
    }
    else if (!strcmp(f[0],"timing") && count==2u) {
        control_timing_diag_t d;
        uint32 irq, sequence;
        float period, solve;
        uint8 ready, fresh;
        control_timing_status(&d);
        irq=interrupt_global_disable();
        period=att_diag.period_us; solve=att_diag.update_us;
        sequence=att_diag.sample_sequence; fresh=att_diag.sample_fresh;
        ready=attitude_converged();
        interrupt_global_enable(irq);
        (void)snprintf(payload,sizeof(payload),"timing,1,%u,%lu,%lu,%lu,%lu,%lu,%.6g,%.6g,%lu,%u,%u",
            (unsigned)d.valid,(unsigned long)d.period_us,(unsigned long)d.max_period_us,
            (unsigned long)d.work_us,(unsigned long)d.max_work_us,(unsigned long)d.overrun_count,
            (double)period,(double)solve,(unsigned long)sequence,(unsigned)fresh,(unsigned)ready);
        (void)cfg_reply(seq,"ok",payload);
    }
    else (void)cfg_reply(seq,"err","BAD_FORMAT,Unknown commissioning operation or field count");
}
