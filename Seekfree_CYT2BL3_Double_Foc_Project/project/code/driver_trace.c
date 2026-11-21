/* SPDX-License-Identifier: GPL-3.0-or-later
 * Passive left FAST_FOC recorder. No motor command, calibration or protection
 * changes. Record in IRQs; export only in main loop with both commands zero.
 */
#ifdef DRIVER_TRACE_HOST
#include "driver_trace_host.h"
#else
#include "zf_common_headfile.h"
#endif
#include "driver_trace.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define TRACE_CAPACITY 8192u
#define TRACE_PRE 128u
#define TRACE_EVENTS 16u
#define TRACE_HZ 8000000u
#define TRACE_PERIOD 8000u
#define TRACE_DURATION (7500u * TRACE_PERIOD)
#define TRACE_UNSET UINT32_MAX
/* State: 0 idle, 1 armed, 2 recording, 3 complete, 4 cancelled. */
typedef struct {
    uint32_t ticks;
    int32_t counts;
    uint16_t angle;
    int16_t duty;
    uint16_t active[3], buffer[3], flags, event;
} trace_row;
typedef struct {
    uint32_t received, dispatched, written, active;
    int16_t left, right;
    uint16_t target[3];
} trace_event;
static trace_row rows[TRACE_CAPACITY];
static trace_event events[TRACE_EVENTS];
static volatile uint32_t trace_state, capture_id;
static uint32_t arm_tick, trigger_tick, last_sample, previous_irq, max_gap;
static uint32_t rx_tick;
static uint8_t rx_valid;
static uint32_t pre_head, pre_count, post_count, event_count, overflow;
static int32_t counts;
static int16_t last_left, last_right;
static volatile uint16_t generation=DRIVER_TRACE_NONE;
static volatile uint16_t request; /* 1 arm, 2 cancel, 3 get */
static volatile uint16_t request_index;
static int trace_ready;
static char response[1536];
static size_t response_length;

#ifndef DRIVER_TRACE_HOST
static uint32_t trace_ticks(void) {
    return Cy_Tcpwm_Counter_GetCounter((volatile stc_TCPWM_GRP_CNT_t*)&TCPWM0->GRP[2].CNT[5]);
}
static uint32_t trace_lock(void) { uint32_t n=__get_PRIMASK();__disable_irq();return n; }
static void trace_unlock(uint32_t n) { __set_PRIMASK(n); }
static int trace_zero(void) { return motor_left.motor_duty==0.0f && motor_right.motor_duty==0.0f; }
static uint16_t trace_flags(void) {
    return (uint16_t)((motor_left.motor_protect_state==PROTECT_MODE?1u:0u) |
        (motor_left.encoder_state==ENCODER_ERROR?2u:0u) |
        ((battery_value.protect_flag && battery_value.battery_state==BATTERY_ERROR)?4u:0u));
}
static void trace_widths(uint16_t active[3],uint16_t buffer[3]) {
    volatile stc_TCPWM_GRP_CNT_t *p[3]={MOTOR_LEFT_A_PHASE_TCPWM_TIMER,MOTOR_LEFT_B_PHASE_TCPWM_TIMER,MOTOR_LEFT_C_PHASE_TCPWM_TIMER};
    unsigned i;
    for(i=0;i<3;i++) {
        active[i]=(uint16_t)(p[i]->unCC1.u32Register-p[i]->unCC0.u32Register);
        buffer[i]=(uint16_t)(p[i]->unCC1_BUFF.u32Register-p[i]->unCC0_BUFF.u32Register);
    }
}
static void trace_send(const char *s,size_t n) {
    uart_write_buffer(MOTOR_DRIVER_UART,(const uint8 *)s,(uint32)n);
}
#endif

void driver_trace_init(void) {
#ifndef DRIVER_TRACE_HOST
    if (motor_left.driver_mode!=FAST_FOC) return;
    /* CH2 is unused by this project. A free-running 8 MHz 32-bit counter. */
    timer_init(TC_TIME2_CH2,TIMER_US);timer_start(TC_TIME2_CH2);
#endif
    trace_ready=1;
}
uint16_t driver_trace_generation(void) { return generation; }
void driver_trace_on_rx_byte(uint8_t byte) {
    if ((trace_state==1u || trace_state==2u) && byte=='\n') {rx_tick=trace_ticks();rx_valid=1;}
}
void driver_trace_on_command(int16_t left,int16_t right) {
    trace_event *e;
    uint32_t now,received=rx_valid?rx_tick-arm_tick:TRACE_UNSET;
    rx_valid=0;
    if (trace_state!=1u && trace_state!=2u) return;
    if (left==last_left && right==last_right) return;
    last_left=left;last_right=right;
    if (trace_state==1u && left==0 && right==0) return;
    now=trace_ticks();
    if (trace_state==1u) { trigger_tick=now;trace_state=2u; }
    if (event_count==TRACE_EVENTS) { overflow=1u;return; }
    e=&events[event_count];
    e->received=received;e->dispatched=now-arm_tick;e->written=e->active=TRACE_UNSET;
    e->left=left;e->right=right;
    generation=(uint16_t)event_count++;
}
void driver_trace_on_pwm_written(uint16_t gen) {
    uint16_t active[3], buffer[3];
    trace_event *e;
    if (trace_state!=2u || gen==DRIVER_TRACE_NONE || gen>=event_count) return;
    e=&events[gen];
    if (e->written!=TRACE_UNSET) return;
    e->written=trace_ticks()-arm_tick;
    trace_widths(active,buffer);
    memcpy(e->target,buffer,sizeof(e->target));
}
void driver_trace_on_encoder(int32_t raw,int32_t delta) {
    uint32_t now,stamp,slot;
    uint16_t active[3], buffer[3];
    uint16_t gen;
    trace_row *r;
    if (trace_state!=1u && trace_state!=2u) return;
    now=trace_ticks();stamp=now-arm_tick;
    if (previous_irq && (uint32_t)(now-previous_irq)>max_gap) max_gap=now-previous_irq;
    previous_irq=now;
    counts+=delta;
    gen=generation;
    if (gen!=DRIVER_TRACE_NONE && gen<event_count && events[gen].written!=TRACE_UNSET && events[gen].active==TRACE_UNSET) {
        trace_widths(active,buffer);
        if (memcmp(active,events[gen].target,sizeof(active))==0) events[gen].active=stamp;
    }
    if ((uint32_t)(now-last_sample)<TRACE_PERIOD) return;
    last_sample=now;
    if (trace_state==1u) {
        slot=pre_head;pre_head=(pre_head+1u)%TRACE_PRE;
        if (pre_count<TRACE_PRE) pre_count++;
    } else {
        if (TRACE_PRE+post_count>=TRACE_CAPACITY) {overflow=1u;trace_state=3u;return;}
        slot=TRACE_PRE+post_count++;
    }
    r=&rows[slot];r->ticks=stamp;r->counts=counts;r->angle=(uint16_t)raw;
    r->duty=last_left;r->event=gen;r->flags=trace_flags();
    trace_widths(r->active,r->buffer);
    if (trace_state==2u && (uint32_t)(now-trigger_tick)>=TRACE_DURATION) trace_state=3u;
}

static void add(const char *fmt,...) {
    va_list args;int n;
    if(response_length>=sizeof(response)-8u)return;
    va_start(args,fmt);n=vsnprintf(response+response_length,sizeof(response)-8u-response_length,fmt,args);va_end(args);
    if(n<0 || (size_t)n>=sizeof(response)-8u-response_length)response_length=sizeof(response)-8u;
    else response_length+=(size_t)n;
}
static void finish(void) {
    uint16_t crc=0xffffu;size_t i;unsigned j;
    for(i=0;i<response_length;i++) {crc^=(uint16_t)((uint8_t)response[i]<<8);for(j=0;j<8;j++)crc=(uint16_t)((crc&0x8000u)?((uint32_t)crc<<1)^0x1021u:(uint32_t)crc<<1);}
    (void)snprintf(response+response_length,sizeof(response)-response_length,"*%04X\n",(unsigned)crc);
    trace_send(response,strlen(response));
}
/* Only two exact command families; no formatting or export in the UART IRQ. */
int driver_trace_text(const uint8_t *text) {
    const char *s=(const char *)text;uint32_t n=0;unsigned digits=0;
    uint16_t kind;
    if (strncmp(s,"SET-TRACE,",10)==0) {s+=10;kind=1;}
    else if (strncmp(s,"GET-TRACE,",10)==0) {s+=10;kind=3;}
    else return 0;
    while (*s>='0' && *s<='9') {if(++digits>5)return 1;n=n*10u+(uint32_t)(*s++-'0');}
    if(*s=='\r')s++;
    if(!digits || n>65535u || *s!='\n' || (kind==1 && n>1u))return 1;
    if(request)return 1;
    request_index=(uint16_t)n;request=(kind==1 && n==0)?2:kind;
    return 1;
}
void driver_trace_service(void) {
    uint32_t irq,state,id,count,ec,maxgap,of;
    uint16_t kind,index;
    unsigned j;
    if(!request)return;
    /* Dumping never overlaps a nonzero command, even if WiFi asks too early. */
    if(!trace_zero())return;
    irq=trace_lock();kind=request;index=request_index;request=0;
    if(kind==1 && trace_ready && trace_zero()) {
        if(trace_state!=1u && trace_state!=2u) {
            capture_id++;trace_state=1;arm_tick=last_sample=trace_ticks();
            pre_head=pre_count=post_count=event_count=overflow=0;
            previous_irq=max_gap=0;rx_valid=0;counts=0;last_left=last_right=0;generation=DRIVER_TRACE_NONE;
        }
    } else if(kind==2)trace_state=4;
    state=trace_state;id=capture_id;count=pre_count+post_count;ec=event_count;maxgap=max_gap;of=overflow;
    trace_unlock(irq);
    response_length=0;
    if(kind==1 || kind==2) {add("TR,A,%lu,%lu",(unsigned long)id,(unsigned long)state);finish();return;}
    if(index==65535u) {
        add("TR,M,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",(unsigned long)id,(unsigned long)state,(unsigned long)count,(unsigned long)ec,(unsigned long)TRACE_HZ,32768ul,(unsigned long)maxgap,(unsigned long)of);
    } else if(state!=3u && state!=4u) {add("TR,X,%lu,busy",(unsigned long)id);}
    else if(index>=60000u && index<60000u+TRACE_EVENTS && index-60000u<ec) {
        trace_event *e=&events[index-60000u];
        add("TR,E,%lu,%u,%lu,%lu,%lu,%lu,%d,%d,%u,%u,%u",(unsigned long)id,(unsigned)(index-60000u),(unsigned long)e->received,(unsigned long)e->dispatched,(unsigned long)e->written,(unsigned long)e->active,(int)e->left,(int)e->right,e->target[0],e->target[1],e->target[2]);
    } else if(index<count) {
        unsigned n=(unsigned)(count-index);if(n>8u)n=8u;
        add("TR,D,%lu,%u,%u",(unsigned long)id,index,n);
        for(j=0;j<n;j++) {
            uint32_t logical=index+j,slot=logical<pre_count?(pre_head+TRACE_PRE-pre_count+logical)%TRACE_PRE:TRACE_PRE+logical-pre_count;
            trace_row *r=&rows[slot];
            add(";%lu,%ld,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u",(unsigned long)r->ticks,(long)r->counts,r->angle,(int)r->duty,r->active[0],r->active[1],r->active[2],r->buffer[0],r->buffer[1],r->buffer[2],r->flags,r->event);
        }
    } else add("TR,X,%lu,index",(unsigned long)id);
    finish();
}

#ifndef DRIVER_TRACE_HOST
static uint8_t driver_right_trace_flags(void)
{
    return (uint8_t)((motor_right.encoder_state==ENCODER_ERROR ? 4u : 0u) |
        ((battery_value.protect_flag && battery_value.battery_state==BATTERY_ERROR) ? 8u : 0u) |
        (motor_right.motor_protect_state==PROTECT_MODE
            ? (16u | ((unsigned)motor_right.motor_protect_cause<<5)) : 0u));
}
static int driver_right_trace_quiet(void)
{
    /* Screen out reported rotation, without claiming a physical stop sensor. */
    return motor_left.motor_speed_filter > -10.0f && motor_left.motor_speed_filter < 10.0f &&
           motor_right.motor_speed_filter > -10.0f && motor_right.motor_speed_filter < 10.0f &&
           motor_left.encoder_state != ENCODER_ERROR && motor_right.encoder_state != ENCODER_ERROR;
}
/* Existing SDK UART accessor is not declared in its legacy public header. */
extern volatile stc_SCB_t *get_scb_module(uart_index_enum uart_n);
static int driver_right_trace_try_tx(const char *s,unsigned n)
{
    volatile stc_SCB_t *p=get_scb_module(MOTOR_DRIVER_UART);
    unsigned i;
    uint32_t size=Cy_SCB_GetFifoSize(p), used=Cy_SCB_GetNumInTxFifo(p);
    /* Caller checked STOP with IRQs masked; never wait for serial wire. Keep
     * space for the next legacy sender's first byte. FIFO preserves ordering. */
    if (used>=size || size-used<=n) return 0;
    for (i=0;i<n;i++) Cy_SCB_WriteTxFifo(p,(uint8_t)s[i]);
    return 1;
}
#include "driver_right_trace.inc"
#endif
