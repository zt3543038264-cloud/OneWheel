/* Foreground-only, RAM-only communication observation. No controller writes,
 * motor calls, Flash, start/keep/stop calls, or control-lease renewal.
 * Timestamp is command dispatch, NOT radio ingress or synchronized latency.
 * The shadow deadline is diagnostic only and never grants output permission.
 */
#include "IfxStm.h"
#define LPROBE_COUNT       200u
#define LPROBE_WINDOW_MS 10000u
#define LPROBE_ARM_MS    30000u
#define LPROBE_SHADOW_MS   200u
typedef struct {
    uint64 armed_at, first_at, last_at, observed_at, sum_gap;
    uint32 run, hz, received, last, duplicate, reordered, invalid;
    uint32 max_gap_us, gaps_ge200, rx_base, tx_base;
    uint8 state, gap_expired, shadow_expired;
} link_probe_t;
static link_probe_t s_link_probe;

static uint64 lprobe_ticks(void)
{
    uint32 hi, lo, again;
    do {
        hi=IfxStm_getOffset32Timer(&MODULE_STM0);
        lo=IfxStm_getLower(&MODULE_STM0);
        again=IfxStm_getOffset32Timer(&MODULE_STM0);
    } while (hi!=again);
    return ((uint64)hi<<32)|lo;
}
static uint64 lprobe_ms_ticks(uint32 ms)
{ return (uint64)s_link_probe.hz*ms/1000u; }
static uint32 lprobe_us(uint64 ticks)
{
    if (!s_link_probe.hz) return 0xffffffffu;
    if (ticks>((uint64)0xffffffffu*s_link_probe.hz)/1000000u) return 0xffffffffu;
    return (uint32)(ticks*1000000u/s_link_probe.hz);
}
static void lprobe_increment(uint32 *v)
{ if (*v!=0xffffffffu) ++*v; }
static void lprobe_gap(uint64 until)
{
    uint64 gap=until-s_link_probe.last_at;
    uint32 us=lprobe_us(gap);
    if (us>s_link_probe.max_gap_us) s_link_probe.max_gap_us=us;
    if (gap>=lprobe_ms_ticks(LPROBE_SHADOW_MS) && !s_link_probe.gap_expired) {
        s_link_probe.gap_expired=1;
        s_link_probe.shadow_expired=1;
        lprobe_increment(&s_link_probe.gaps_ge200);
    }
}
static void lprobe_poll(void)
{
    uint64 now, end;
    if (s_link_probe.state!=1u && s_link_probe.state!=2u) return;
    now=lprobe_ticks();
    if (now<s_link_probe.observed_at) { s_link_probe.state=5u; return; }
    s_link_probe.observed_at=now;
    /* If another operator starts anything, invalidate this measurement only.
     * This observer deliberately has no authority to stop or modify it. */
    if (!cfg_stopped()) { s_link_probe.state=4u; return; }
    if (s_link_probe.state==1u) {
        if (now-s_link_probe.armed_at>=lprobe_ms_ticks(LPROBE_ARM_MS))
            s_link_probe.state=6u;
        return;
    }
    end=s_link_probe.first_at+lprobe_ms_ticks(LPROBE_WINDOW_MS);
    lprobe_gap(now<end?now:end);
    if (now>=end) s_link_probe.state=3u;
}
static void lprobe_reply(uint16 seq, const char *op)
{
    char payload[280];
    uint32 elapsed=0, remaining=0, missing, mean=0;
    int n;
    lprobe_poll();
    if (s_link_probe.received) {
        elapsed=lprobe_us(s_link_probe.observed_at-s_link_probe.first_at)/1000u;
        if (elapsed>LPROBE_WINDOW_MS) elapsed=LPROBE_WINDOW_MS;
        remaining=LPROBE_WINDOW_MS-elapsed;
        if (s_link_probe.received>1u)
            mean=lprobe_us(s_link_probe.sum_gap/(s_link_probe.received-1u));
    }
    missing=(s_link_probe.state==3u?LPROBE_COUNT:s_link_probe.last)-s_link_probe.received;
    n=snprintf(payload,sizeof(payload),
        "%s,1,%lu,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%lu,%lu,%lu,%lu",
        op,(unsigned long)s_link_probe.run,(unsigned)s_link_probe.state,
        (unsigned long)s_link_probe.received,(unsigned long)s_link_probe.last,
        (unsigned long)missing,(unsigned long)s_link_probe.duplicate,
        (unsigned long)s_link_probe.reordered,(unsigned long)s_link_probe.invalid,
        (unsigned long)s_link_probe.max_gap_us,(unsigned long)mean,
        (unsigned long)s_link_probe.gaps_ge200,(unsigned)s_link_probe.shadow_expired,
        (unsigned long)elapsed,(unsigned long)remaining,
        (unsigned long)(s_cfg_rx_error-s_link_probe.rx_base),
        (unsigned long)(s_cfg_tx_drop-s_link_probe.tx_base));
    if (n<0 || n>=(int)sizeof(payload)) {
        (void)cfg_reply(seq,"err","PROBE_SIZE,No measurement inferred"); return;
    }
    (void)cfg_reply(seq,"ok",payload);
}
static void lprobe_cfg(char **f, uint8 count, uint16 seq)
{
    uint32 run, index;
    uint64 now;
#if C_BENCH_ONLY_PROFILE && !C_BENCH_SIGNAL_OUTPUT_ALLOWED && !VEHICLE_MOTOR_OUTPUT_ALLOWED
    float hz;
#endif
    lprobe_poll();
    if (!strcmp(f[0],"lprobe_tick")) {
        /* Every tick, including malformed/stale ticks, is ACK-free. Never
         * convert packet loss into a response storm or a control heartbeat. */
        if (count!=4u || !commission_u32(f[2],&run) || !run ||
            run!=s_link_probe.run || !commission_u32(f[3],&index) ||
            index<1u || index>LPROBE_COUNT) {
            if (s_link_probe.state==1u || s_link_probe.state==2u)
                lprobe_increment(&s_link_probe.invalid);
            return;
        }
        if (s_link_probe.state!=1u && s_link_probe.state!=2u) return;
        if (index==s_link_probe.last) { lprobe_increment(&s_link_probe.duplicate); return; }
        if (index<s_link_probe.last) { lprobe_increment(&s_link_probe.reordered); return; }
        now=s_link_probe.observed_at;
        if (s_link_probe.state==1u) {
            s_link_probe.state=2u; s_link_probe.first_at=now;
        } else s_link_probe.sum_gap+=now-s_link_probe.last_at;
        s_link_probe.last_at=now; s_link_probe.last=index;
        s_link_probe.received++; s_link_probe.gap_expired=0;
        return;
    }
    if (count!=3u || !commission_u32(f[2],&run) || !run) {
        (void)cfg_reply(seq,"err","BAD_PROBE,Require nonzero run id"); return;
    }
    if (!strcmp(f[0],"lprobe_status")) {
        if (run!=s_link_probe.run) { (void)cfg_reply(seq,"err","PROBE_RUN,No matching capture"); return; }
        lprobe_reply(seq,f[0]); return;
    }
    if (strcmp(f[0],"lprobe_begin")) {
        (void)cfg_reply(seq,"err","BAD_PROBE,Unknown diagnostic operation"); return;
    }
#if !C_BENCH_ONLY_PROFILE || C_BENCH_SIGNAL_OUTPUT_ALLOWED || VEHICLE_MOTOR_OUTPUT_ALLOWED
    (void)cfg_reply(seq,"err","PROBE_PROFILE,Require calculation-only C profile");
    return;
#else
    if (!cfg_stopped() || s_save_seq || s_path_save_seq) {
        (void)cfg_reply(seq,"err","PROBE_BUSY,Require idle no pending storage"); return;
    }
    if (s_link_probe.state==1u || s_link_probe.state==2u || run==s_link_probe.run) {
        (void)cfg_reply(seq,"err","PROBE_BUSY,Existing capture or reused run id"); return;
    }
    hz=IfxStm_getFrequency(&MODULE_STM0);
    if (!cmd_float_is_finite(hz) || hz<1000000.0f || hz>1000000000.0f) {
        (void)cfg_reply(seq,"err","PROBE_CLOCK,Invalid STM frequency"); return;
    }
    now=lprobe_ticks();
    memset(&s_link_probe,0,sizeof(s_link_probe));
    s_link_probe.run=run; s_link_probe.hz=(uint32)hz; s_link_probe.state=1u;
    s_link_probe.armed_at=s_link_probe.observed_at=now;
    s_link_probe.rx_base=s_cfg_rx_error; s_link_probe.tx_base=s_cfg_tx_drop;
    lprobe_reply(seq,f[0]);
#endif
}
