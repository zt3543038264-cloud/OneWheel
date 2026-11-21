/* SPDX-License-Identifier: GPL-3.0-or-later
 * Passive right recorder. Uses the existing trace clock, not a PWM source.
 * Debugger mailbox is consumed only at STOP; it never enters the command path.
 */
#include <stddef.h>
#include <string.h>
#define BTR_MAGIC 0x31525442u
#define BTR_ARM 0x314D5241u
#define BTR_CAP 1024u
#define BTR_PRE 128u
#define BTR_PERIOD 32000u
#define BTR_POST_STOP (500u * TRACE_PERIOD)
#define BTR_ARM_EXPIRE (120000u * TRACE_PERIOD)

typedef struct {
    uint32_t ticks, sequence;
    int16_t command_b;
    uint16_t pwm[3], bus_mv;
    int16_t encoder_delta;
    uint16_t encoder_angle;
    uint8_t flags, safe_status;
} right_trace_row;
typedef struct {
    uint32_t request, magic, state, capture_id, row_size, capacity;
    uint32_t pre_head, pre_count, post_count, trigger_ticks, ticks_hz, stop_reason;
    right_trace_row rows[BTR_CAP];
} right_trace_capture;
typedef char right_trace_layout_check[(sizeof(right_trace_row)==24u &&
    offsetof(right_trace_capture,rows)==48u &&
    offsetof(right_trace_row,command_b)==8u &&
    offsetof(right_trace_row,pwm)==10u &&
    offsetof(right_trace_row,bus_mv)==16u &&
    offsetof(right_trace_row,encoder_delta)==18u &&
    offsetof(right_trace_row,encoder_angle)==20u &&
    offsetof(right_trace_row,flags)==22u &&
    offsetof(right_trace_row,safe_status)==23u &&
    sizeof(right_trace_capture)==24624u) ? 1 : -1];

/* 0 idle, 1 armed, 2 recording, 3 frozen. Default: one passive capture at boot. */
volatile right_trace_capture g_driver_right_trace; /* BSS, no large ROM initializer */
static uint32_t btr_arm_tick, btr_last_tick, btr_zero_tick, btr_adc_tick;
static uint32_t btr_sequence;
static int16_t btr_command;
static uint16_t btr_bus_mv;
static uint8_t btr_zero, btr_adc_valid;
static volatile uint8_t btr_read_request;
static volatile uint16_t btr_read_index;
static volatile uint32_t btr_read_id;

static int btr_hex_parse(const char *s,unsigned n,uint32_t *v)
{
    unsigned i;uint32_t x=0;
    for (i=0;i<n;i++) {
        unsigned c=(uint8_t)s[i];
        if (c>='0' && c<='9') c-='0';
        else if (c>='A' && c<='F') c=c-'A'+10u;
        else return 0;
        x=(x<<4)|c;
    }
    *v=x;return 1;
}

/* Called only from the existing complete-line UART receive parser. */
int driver_right_trace_read_request(const uint8_t *text)
{
    const char *s=(const char *)text;
    uint32_t id=0,index=0;
    unsigned kind;
    if (strcmp(s,"BTR,M\r\n")==0) kind=1u;
    else if (strlen(s)==21u && memcmp(s,"BTR,D,",6u)==0 &&
        s[14]==',' && s[19]=='\r' && s[20]=='\n' &&
        btr_hex_parse(s+6,8,&id) && btr_hex_parse(s+15,4,&index)) kind=2u;
    else return 0;
    if (!btr_read_request) {
        btr_read_id=id;btr_read_index=(uint16_t)index;btr_read_request=(uint8_t)kind;
    }
    return 1;
}

static int btr_stopped(void)
{
    unsigned state=driver_right_trace_safe_status() & 15u;
    return (state==0u || state==4u) && trace_zero() && driver_right_trace_quiet();
}

static void btr_store16(uint8_t *p,uint16_t v)
{
    p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);
}
static void btr_store32(uint8_t *p,uint32_t v)
{
    btr_store16(p,(uint16_t)v);btr_store16(p+2,(uint16_t)(v>>16));
}
static char *btr_hex_write(char *p,uint32_t v,unsigned digits)
{
    static const char hex[]="0123456789ABCDEF";
    while (digits) { --digits;*p++=hex[(v>>(4u*digits))&15u]; }
    return p;
}
static char *btr_base64(char *p,const uint8_t bytes[24])
{
    static const char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    unsigned i;
    for(i=0;i<24u;i+=3u) {
        uint32_t v=((uint32_t)bytes[i]<<16)|((uint32_t)bytes[i+1u]<<8)|bytes[i+2u];
        *p++=alphabet[(v>>18)&63u];*p++=alphabet[(v>>12)&63u];
        *p++=alphabet[(v>>6)&63u];*p++=alphabet[v&63u];
    }
    return p;
}

static void btr_read_service(void)
{
    unsigned kind,index,i,j;
    uint32_t irq,id,current_id,state,pre,post,head;
    uint8_t bytes[24];
    char line[57],*p;
    uint16_t crc=0xffffu;

    if (!btr_read_request) return;
    irq=trace_lock();
    kind=btr_read_request;id=btr_read_id;index=btr_read_index;btr_read_request=0;
    state=g_driver_right_trace.state;current_id=g_driver_right_trace.capture_id;
    pre=g_driver_right_trace.pre_count;post=g_driver_right_trace.post_count;
    head=g_driver_right_trace.pre_head;
    if (!btr_stopped() || (kind==1u && state==2u) ||
        (kind==2u && (state!=3u || id!=current_id || index>=pre+post))) {
        trace_unlock(irq);return;
    }
    if (kind==1u) {
        btr_store32(bytes,current_id);bytes[4]=(uint8_t)state;
        bytes[5]=(uint8_t)g_driver_right_trace.stop_reason;
        btr_store16(bytes+6,(uint16_t)pre);btr_store16(bytes+8,(uint16_t)post);
        btr_store16(bytes+10,24u);
        btr_store32(bytes+12,g_driver_right_trace.trigger_ticks);
        btr_store32(bytes+16,TRACE_HZ);
        btr_store16(bytes+20,BTR_CAP);btr_store16(bytes+22,0);
    }
    trace_unlock(irq);
    if (kind==2u) {
        unsigned slot=index<pre?(head+BTR_PRE-pre+index)%BTR_PRE:BTR_PRE+index-pre;
        const volatile uint8_t *row=(const volatile uint8_t *)&g_driver_right_trace.rows[slot];
        /* Frozen rows cannot be changed in IRQ; rearming is main-loop only. */
        for(i=0;i<24u;i++) bytes[i]=row[i];
    }
    p=line;*p++='B';*p++='T';*p++=',';
    *p++=(kind==1u)?'M':'D';*p++=',';
    if(kind==2u) {
        p=btr_hex_write(p,id,8u);*p++=',';
        p=btr_hex_write(p,index,4u);*p++=',';
    }
    p=btr_base64(p,bytes);
    for(i=0;i<(unsigned)(p-line);i++) {
        crc^=(uint16_t)((uint8_t)line[i]<<8);
        for(j=0;j<8u;j++) crc=(uint16_t)((crc&0x8000u)?((uint32_t)crc<<1)^0x1021u:(uint32_t)crc<<1);
    }
    *p++='*';p=btr_hex_write(p,crc,4u);*p++='\n';
    /* Gate again upon actual UART FIFO submission. No lock held while
     * formatting, and no wait while masked. No unsafe request is retained. */
    irq=trace_lock();
    if (btr_stopped() && g_driver_right_trace.state==state &&
        g_driver_right_trace.capture_id==current_id)
        (void)driver_right_trace_try_tx(line,(unsigned)(p-line));
    trace_unlock(irq);
}

void driver_right_trace_service(void)
{
    uint32_t irq;
    uint8_t state;
    if (!trace_ready) return;
    if (g_driver_right_trace.magic==0u) {
        g_driver_right_trace.row_size=sizeof(right_trace_row);
        g_driver_right_trace.capacity=BTR_CAP;
        g_driver_right_trace.ticks_hz=TRACE_HZ;
        g_driver_right_trace.request=BTR_ARM;
        g_driver_right_trace.magic=BTR_MAGIC;
    }
    btr_read_service();
    if (g_driver_right_trace.request != BTR_ARM) return;
    irq=trace_lock();
    g_driver_right_trace.request=0;
    state=driver_right_trace_safe_status() & 15u;
    if (g_driver_right_trace.state!=1u && g_driver_right_trace.state!=2u &&
        trace_zero() && driver_right_trace_quiet() && (state==0u || state==4u)) {
        g_driver_right_trace.capture_id++;
        g_driver_right_trace.pre_head=g_driver_right_trace.pre_count=g_driver_right_trace.post_count=0;
        g_driver_right_trace.trigger_ticks=UINT32_MAX;
        g_driver_right_trace.stop_reason=0;
        btr_zero=0;
        btr_arm_tick=btr_last_tick=trace_ticks();
        g_driver_right_trace.state=1u;
    }
    trace_unlock(irq);
}

/* Both hooks execute inside the existing SAFE critical section. */
void driver_right_trace_command(int16_t b,uint32_t sequence)
{
    btr_command=b;btr_sequence=sequence;
    if (b && g_driver_right_trace.state==1u) {
        g_driver_right_trace.trigger_ticks=trace_ticks()-btr_arm_tick;
        g_driver_right_trace.state=2u;
    }
}
void driver_right_trace_zero(void) { btr_command=0;btr_sequence=0; }

/* Called after publishing the existing ADC result; does not perform a conversion. */
void driver_right_trace_adc(uint16_t raw,float volts)
{
    uint32_t irq;
    uint8_t valid;
    uint16_t mv;
    if (!trace_ready) return;
    valid=(uint8_t)(raw!=0u && raw<4095u && volts>0.0f && volts<=65.0f);
    mv=valid ? (uint16_t)(volts*1000.0f+0.5f) : 0u;
    irq=trace_lock();
    btr_bus_mv=mv;btr_adc_valid=valid;btr_adc_tick=trace_ticks();
    trace_unlock(irq);
}

void driver_right_trace_sample(int32_t angle,int32_t delta)
{
    uint32_t now,irq,slot,state;
    uint16_t phase[3]={0,0,0};
    uint8_t flags,status,zero;
    volatile right_trace_row *row;

    state=g_driver_right_trace.state;
    if (state!=1u && state!=2u) return;
    now=trace_ticks();
    if (now-btr_last_tick<BTR_PERIOD) return;
    irq=trace_lock();
    state=g_driver_right_trace.state;
    btr_last_tick=now;
    if (state==1u) {
        if (now-btr_arm_tick>=BTR_ARM_EXPIRE) {
            g_driver_right_trace.stop_reason=3u;g_driver_right_trace.state=3u;
            trace_unlock(irq);return;
        }
        slot=g_driver_right_trace.pre_head;
        g_driver_right_trace.pre_head=(slot+1u)&(BTR_PRE-1u);
        if (g_driver_right_trace.pre_count<BTR_PRE) g_driver_right_trace.pre_count++;
    } else {
        if (g_driver_right_trace.post_count>=BTR_CAP-BTR_PRE) {
            g_driver_right_trace.stop_reason=2u;g_driver_right_trace.state=3u;
            trace_unlock(irq);return;
        }
        slot=BTR_PRE+g_driver_right_trace.post_count++;
    }
    status=driver_right_trace_safe_status();
    zero=(uint8_t)(trace_zero() && (status&15u)!=3u);
    flags=driver_right_trace_pwm(phase) ? 1u : 0u;
    if (btr_adc_valid && now-btr_adc_tick<100u*TRACE_PERIOD) flags|=2u;
    flags|=driver_right_trace_flags();
    row=&g_driver_right_trace.rows[slot];
    row->ticks=now-btr_arm_tick;row->sequence=btr_sequence;row->command_b=btr_command;
    row->pwm[0]=phase[0];row->pwm[1]=phase[1];row->pwm[2]=phase[2];
    row->bus_mv=(flags&2u)?btr_bus_mv:0u;
    row->encoder_delta=(int16_t)delta;row->encoder_angle=(uint16_t)angle;
    row->flags=flags;row->safe_status=status;
    if (state==2u) {
        if (!zero) btr_zero=0;
        else if (!btr_zero) { btr_zero=1;btr_zero_tick=now; }
        else if (now-btr_zero_tick>=BTR_POST_STOP) {
            g_driver_right_trace.stop_reason=1u;g_driver_right_trace.state=3u;
        }
    }
    trace_unlock(irq);
}
