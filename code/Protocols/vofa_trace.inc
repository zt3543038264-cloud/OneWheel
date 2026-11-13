/* Included by vofa.c. CPU0 only: bounded 1 kHz sampling, indexed STOP-only reads.
 * 256 * 44 = 11264 bytes; no allocation, formatting or transport in the ISR. */
#if defined(__TASKING__)
#include "IfxStm.h"
#endif
#define TRACE_CAPACITY 256u
#define TRACE_IDLE 0u
#define TRACE_ARMED 1u
#define TRACE_RECORDING 2u
#define TRACE_DONE 3u
typedef struct {
    uint32 ticks;
    float roll, gx, gy, gz, ax, ay, az, output;
    int16 rpm_a, rpm_b;
    uint32 flags;
} roll_trace_sample_t;
typedef char trace_sample_size_check[(sizeof(roll_trace_sample_t)==44u)?1:-1];
static roll_trace_sample_t s_trace_data[TRACE_CAPACITY];
static volatile uint8 s_trace_state, s_trace_reason;
static volatile uint16 s_trace_count;
static uint32 s_trace_armed_ms, s_trace_started_ms, s_trace_revision, s_trace_delay, s_trace_hz;
static uint8 s_trace_triggered;
static float s_trace_kp, s_trace_ki, s_trace_kd;
static uint32 trace_ticks(void)
{
#if defined(__TASKING__)
    return IfxStm_getLower(&MODULE_STM0);
#else
    return g_control_uptime_ms * 1000u;
#endif
}
static void trace_finish(uint8 reason)
{
    s_trace_reason=reason;
    __dsync();
    s_trace_state=TRACE_DONE;
}
static void trace_tick1ms(void)
{
    roll_trace_sample_t *p;
    imu_data_t raw;
    uint8 valid, running;
    if(s_trace_state!=TRACE_ARMED && s_trace_state!=TRACE_RECORDING) return;
    if(g_param_revision!=s_trace_revision) { trace_finish(3u); return; }
    running=control_test_running();
    if(!running) {
        if(s_trace_triggered) trace_finish(2u);
        else if((uint32)(g_control_uptime_ms-s_trace_armed_ms)>=30000u) trace_finish(4u);
        else if(start_flag!=START_STOP || control_jog_running()!=MOTOR_JOG_NONE) trace_finish(5u);
        return;
    }
    if(g_tune_axis!=TUNE_AXIS_ROLL || g_tune_ring!=TUNE_RING_RATE) { trace_finish(5u); return; }
    if(!s_trace_triggered) { s_trace_started_ms=g_control_uptime_ms; s_trace_triggered=1u; }
    if((uint32)(g_control_uptime_ms-s_trace_started_ms)<s_trace_delay) return;
    s_trace_state=TRACE_RECORDING;
    p=&s_trace_data[s_trace_count];
    p->ticks=trace_ticks();
    valid=imu_get_raw_sample(&raw);
    p->roll=att.roll;
    p->gx=valid?raw.gyro_x:0.0f; p->gy=valid?raw.gyro_y:0.0f; p->gz=valid?raw.gyro_z:0.0f;
    p->ax=valid?raw.acc_x:0.0f; p->ay=valid?raw.acc_y:0.0f; p->az=valid?raw.acc_z:0.0f;
    p->output=g_pwm_roll;
    p->rpm_a=W_Motor_GetSpeed1(); p->rpm_b=W_Motor_GetSpeed2();
    p->flags=(uint32)valid | ((uint32)!W_Motor_LinkLost()<<1) | ((uint32)(g_imu_ok && !imu_link_lost())<<2);
    __dsync();
    ++s_trace_count;
    if(s_trace_count>=TRACE_CAPACITY) trace_finish(1u);
}
/* Called only by foreground cfg parser; arm/cancel critical sections are constant work. */
static void trace_cfg(char **fields,uint8 count,uint16 seq)
{
    char payload[176];
    uint32 irq, index;
    float value;
    uint8 state;
    if(strcmp(fields[0],"trace_arm")==0 && count==3u) {
        if(!cmd_parse_float_strict(fields[2],&value) || value!=floorf(value) || value<0.0f || value>10000.0f) {
            (void)cfg_reply(seq,"err","INVALID_VALUE,Delay must be integer 0..10000 ms"); return;
        }
        irq=interrupt_global_disable();
        if(!cfg_stopped() || s_trace_state==TRACE_ARMED || s_trace_state==TRACE_RECORDING) {
            interrupt_global_enable(irq); (void)cfg_reply(seq,"err","BUSY,Stop and finish or cancel recording first"); return;
        }
        s_trace_count=0; s_trace_reason=0; s_trace_triggered=0;
        s_trace_revision=g_param_revision; s_trace_delay=(uint32)value;
        s_trace_kp=R_RATE_KP; s_trace_ki=R_RATE_KI; s_trace_kd=R_RATE_KD;
        s_trace_armed_ms=g_control_uptime_ms;
#if defined(__TASKING__)
        s_trace_hz=(uint32)IfxStm_getFrequency(&MODULE_STM0);
#else
        s_trace_hz=1000000u;
#endif
        __dsync(); s_trace_state=TRACE_ARMED;
        interrupt_global_enable(irq);
        (void)cfg_reply(seq,"ok","trace_arm,1,256,1000"); return;
    }
    if(strcmp(fields[0],"trace_status")==0 && count==2u) {
        irq=interrupt_global_disable();
        state=s_trace_state; index=s_trace_count;
        interrupt_global_enable(irq);
        (void)snprintf(payload,sizeof(payload),"trace_status,1,%u,%lu,256,1000,%lu,%lu,%u,%lu,%.9g,%.9g,%.9g",
            (unsigned)state,(unsigned long)index,(unsigned long)s_trace_hz,(unsigned long)s_trace_revision,
            (unsigned)s_trace_reason,(unsigned long)s_trace_delay,(double)s_trace_kp,(double)s_trace_ki,(double)s_trace_kd);
        (void)cfg_reply(seq,"ok",payload); return;
    }
    if(strcmp(fields[0],"trace_cancel")==0 && count==2u) {
        irq=interrupt_global_disable();
        if(s_trace_state==TRACE_ARMED || s_trace_state==TRACE_RECORDING) trace_finish(6u);
        interrupt_global_enable(irq);
        (void)cfg_reply(seq,"ok","trace_cancel"); return;
    }
    if(strcmp(fields[0],"trace_get")==0 && count==3u) {
        roll_trace_sample_t *p;
        if(!cmd_parse_float_strict(fields[2],&value) || value!=floorf(value) || value<0.0f || value>=TRACE_CAPACITY) {
            (void)cfg_reply(seq,"err","INVALID_VALUE,Index must be integer 0..255"); return;
        }
        if(!cfg_stopped() || s_trace_state!=TRACE_DONE) {
            (void)cfg_reply(seq,"err","BUSY,Stop and freeze recording before download"); return;
        }
        index=(uint32)value;
        if(index>=s_trace_count) { (void)cfg_reply(seq,"err","OUT_OF_RANGE,Sample not recorded"); return; }
        p=&s_trace_data[index];
        (void)snprintf(payload,sizeof(payload),"trace_get,%lu,%lu,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%d,%d,%lu",
            (unsigned long)index,(unsigned long)p->ticks,(double)p->roll,(double)p->gx,(double)p->gy,(double)p->gz,
            (double)p->ax,(double)p->ay,(double)p->az,(double)p->output,(int)p->rpm_a,(int)p->rpm_b,(unsigned long)p->flags);
        (void)cfg_reply(seq,"ok",payload); return;
    }
    (void)cfg_reply(seq,"err","BAD_FORMAT,Unknown trace operation or arity");
}
