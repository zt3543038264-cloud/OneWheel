/* SPDX-License-Identifier: GPL-3.0-or-later
 * Included by motor_driver_uart_control.c so existing IDE source lists keep working.
 * Shared state changes are short PRIMASK critical sections. No UART writes in IRQ.
 * This is a commissioning pulse gate, not a production vehicle safety controller.
 */
#include "driver_safety.h"
#include <stdio.h>
#include <string.h>
static volatile uint32_t safe_ms, safe_session, safe_sequence;
static volatile uint32_t safe_last_command, safe_window_start, safe_pulse_start;
static volatile uint32_t safe_main_tick, safe_control_tick[2];
static volatile uint8_t safe_state, safe_reason, safe_pulse_started, safe_reply, safe_timer_ready;
static uint32_t safe_generation, safe_last_status;
static uint8_t safe_button_released, safe_button_release_count, safe_button_press_count;
static uint8_t safe_rx_length, safe_rx_drop;
static uint32_t safe_rx_start;

static uint32_t safe_lock(void) { uint32_t n=__get_PRIMASK();__disable_irq();return n; }
static void safe_unlock(uint32_t n) { __set_PRIMASK(n); }
static int safe_fault(void)
{
    return motor_left.motor_protect_state==PROTECT_MODE ||
           motor_right.motor_protect_state==PROTECT_MODE ||
           motor_left.encoder_state==ENCODER_ERROR || motor_right.encoder_state==ENCODER_ERROR ||
           (battery_value.protect_flag && battery_value.battery_state==BATTERY_ERROR);
}
static void safe_zero_outputs(void)
{
    motor_left.motor_duty=0.0f; motor_right.motor_duty=0.0f;
    motor_value.receive_left_duty_data=0; motor_value.receive_right_duty_data=0;
    motor_value.immediate_command=NULL_CMD; motor_value.refresh_flag=CMD_NULL;
    motor_output_inhibit();
}
static void safe_stop(unsigned reason)
{
    safe_state=(reason==DRIVER_SAFE_MANUAL)?DRIVER_SAFE_DISARMED:DRIVER_SAFE_LATCHED;
    safe_reason=(uint8_t)reason; safe_session=0; safe_sequence=0;
    safe_pulse_started=0; safe_button_released=0;
    safe_button_release_count=0; safe_button_press_count=0;
    safe_zero_outputs(); safe_reply=1;
}
void driver_safety_disarm(unsigned reason)
{
    uint32_t irq=safe_lock(); safe_stop(reason); safe_unlock(irq);
}
void driver_safety_init(void)
{
    uint32_t irq=safe_lock();
    safe_ms=0; safe_generation=0; safe_session=0; safe_sequence=0;
    safe_main_tick=0; safe_control_tick[0]=safe_control_tick[1]=0;
    safe_rx_length=0; safe_rx_drop=0; safe_reply=0; safe_last_status=0; safe_timer_ready=0;
    safe_stop(DRIVER_SAFE_BOOT); safe_state=DRIVER_SAFE_DISARMED;
    safe_unlock(irq);
}
void driver_safety_main_alive(void) { safe_main_tick=safe_ms; }
void driver_safety_control_alive(unsigned side) { if(side<2) safe_control_tick[side]=safe_ms; }
void driver_safety_tick_1ms(void)
{
    uint32_t irq=safe_lock();
    ++safe_ms;
    if(safe_ms>=20u) safe_timer_ready=1;
    if(safe_state==DRIVER_SAFE_ARMED) {
        if(safe_fault()) safe_stop(DRIVER_SAFE_MOTOR_FAULT);
        else if((uint32_t)(safe_ms-safe_control_tick[0])>=DRIVER_SAFE_CONTROL_MS ||
                (uint32_t)(safe_ms-safe_control_tick[1])>=DRIVER_SAFE_CONTROL_MS)
            safe_stop(DRIVER_SAFE_CONTROL_TIMEOUT);
        else if((uint32_t)(safe_ms-safe_main_tick)>=DRIVER_SAFE_MAIN_MS)
            safe_stop(DRIVER_SAFE_MAIN_TIMEOUT);
        else if((uint32_t)(safe_ms-safe_last_command)>=DRIVER_SAFE_COMMAND_MS)
            safe_stop(DRIVER_SAFE_COMMAND_TIMEOUT);
        else if(safe_pulse_started && (uint32_t)(safe_ms-safe_pulse_start)>=DRIVER_SAFE_PULSE_MS)
            safe_stop(DRIVER_SAFE_PULSE_LIMIT);
    } else if((safe_state==DRIVER_SAFE_LOCAL_READY || safe_state==DRIVER_SAFE_ZEROED) &&
              (uint32_t)(safe_ms-safe_window_start)>=DRIVER_SAFE_ARM_WINDOW_MS)
        safe_stop(DRIVER_SAFE_ARM_WINDOW_EXPIRED);
    safe_unlock(irq);
}
/* Two consecutive released samples, then two pressed samples. A held key cannot
 * grant again after timeout, disarm, or boot. Any fresh press while armed stops. */
void driver_safety_button_sample(int released)
{
    uint32_t irq=safe_lock();
    if(released) {
        safe_button_press_count=0;
        if(safe_button_release_count<2) ++safe_button_release_count;
        if(safe_button_release_count==2) safe_button_released=1;
    } else {
        safe_button_release_count=0;
        if(safe_button_press_count<2) ++safe_button_press_count;
        if(safe_button_press_count==2 && safe_button_released) {
            safe_button_released=0;
            if(safe_state==DRIVER_SAFE_ARMED) safe_stop(DRIVER_SAFE_MANUAL);
            else if(!motor_calibration_valid) safe_stop(DRIVER_SAFE_CALIBRATION_INVALID);
            else if(safe_fault()) safe_stop(DRIVER_SAFE_MOTOR_FAULT);
            else if(safe_generation==UINT32_MAX) safe_stop(DRIVER_SAFE_SEQUENCE_EXHAUSTED);
            else {
                safe_zero_outputs(); ++safe_generation;
                safe_session=safe_generation; safe_sequence=0;
                safe_window_start=safe_ms; safe_state=DRIVER_SAFE_LOCAL_READY;
                safe_reason=DRIVER_SAFE_NONE; safe_pulse_started=0; safe_reply=1;
            }
        }
    }
    safe_unlock(irq);
}
int driver_safety_output_allowed(unsigned side)
{
    /* Atomic snapshots at aligned word granularity; register writes are locked by caller. */
    return safe_state==DRIVER_SAFE_ARMED && side==1 &&
        motor_right.motor_duty>=-0.05f && motor_right.motor_duty<=0.05f &&
        (uint32_t)(safe_ms-safe_last_command)<DRIVER_SAFE_COMMAND_MS &&
        (!safe_pulse_started || (uint32_t)(safe_ms-safe_pulse_start)<DRIVER_SAFE_PULSE_MS) &&
        !safe_fault() && (side==0?motor_left.motor_duty:motor_right.motor_duty)!=0.0f;
}
/* Exact decimal grammar, no sign/whitespace/leading zeros for unsigned fields. */
static int safe_uint(const char **p,uint32_t *v,char delimiter)
{
    const char *s=*p; uint32_t n=0; unsigned digits=0;
    if(*s<'0' || *s>'9') return 0;
    if(*s=='0' && s[1]!=delimiter) return 0;
    while(*s>='0' && *s<='9') {
        unsigned d=(unsigned)(*s++-'0');
        if(++digits>10 || n>(UINT32_MAX-d)/10u) return 0;
        n=n*10u+d;
    }
    if(*s!=delimiter) return 0;
    *v=n; *p=s+1; return 1;
}
static int safe_duty(const char **p,int16_t *v,char delimiter)
{
    uint32_t n; int negative=0;
    if(**p=='-') {negative=1; ++*p;}
    if(!safe_uint(p,&n,delimiter) || n>10000u || (negative && n==0)) return 0;
    *v=(int16_t)(negative?-(int32_t)n:(int32_t)n); return 1;
}
static int safe_wire(const char *s)
{
    const char *p; uint32_t session,seq,irq; int16_t left,right;
    if(strcmp(s,"SAFE-STATUS,1\r\n")==0) {safe_reply=1;return 1;}
    if(strcmp(s,"SAFE-DISARM,1\r\n")==0) {driver_safety_disarm(DRIVER_SAFE_MANUAL);return 1;}
    if(strncmp(s,"SAFE-ZERO,1,",12)==0 || strncmp(s,"SAFE-ARM,1,",11)==0) {
        int zero=s[5]=='Z'; p=s+(zero?12:11);
        if(!safe_uint(&p,&session,'\r') || *p++!='\n' || *p!='\0') return 1;
        irq=safe_lock();
        if(session!=0 && session==safe_session && motor_calibration_valid && !safe_fault() &&
           (uint32_t)(safe_ms-safe_window_start)<DRIVER_SAFE_ARM_WINDOW_MS) {
            if(zero && safe_state==DRIVER_SAFE_LOCAL_READY) {
                safe_zero_outputs(); safe_state=DRIVER_SAFE_ZEROED; safe_reply=1;
            } else if(!zero && safe_state==DRIVER_SAFE_ZEROED && safe_timer_ready &&
                (uint32_t)(safe_ms-safe_main_tick)<DRIVER_SAFE_MAIN_MS &&
                (uint32_t)(safe_ms-safe_control_tick[0])<DRIVER_SAFE_CONTROL_MS &&
                (uint32_t)(safe_ms-safe_control_tick[1])<DRIVER_SAFE_CONTROL_MS) {
                safe_zero_outputs(); safe_last_command=safe_ms;
                safe_state=DRIVER_SAFE_ARMED; safe_reason=DRIVER_SAFE_NONE; safe_reply=1;
            }
        }
        safe_unlock(irq);return 1;
    }
    if(strncmp(s,"SAFE-DUTY,1,",12)==0) {
        p=s+12;
        if(!safe_uint(&p,&session,',') || !safe_uint(&p,&seq,',') ||
           !safe_duty(&p,&left,',') || !safe_duty(&p,&right,'\r') ||
           *p++!='\n' || *p!='\0') return 1;
        if(left!=0 || right < -DRIVER_SAFE_B_DUTY_MAX || right > DRIVER_SAFE_B_DUTY_MAX) return 1;
        irq=safe_lock();
        if(safe_state==DRIVER_SAFE_ARMED && session!=0 && session==safe_session &&
           seq>safe_sequence && (uint32_t)(safe_ms-safe_last_command)<DRIVER_SAFE_COMMAND_MS &&
           !safe_fault()) {
            safe_sequence=seq; safe_last_command=safe_ms;
            if(!safe_pulse_started && (left || right)) {safe_pulse_started=1;safe_pulse_start=safe_ms;}
            motor_value.receive_left_duty_data=left;motor_value.receive_right_duty_data=right;
            motor_left.motor_duty=(float)left/10000.0f;motor_right.motor_duty=(float)right/10000.0f;
            motor_left.lose_control_protect_count=0;motor_right.lose_control_protect_count=0;
            driver_trace_on_command(left,right);
            if(!left && !right) motor_output_inhibit();
            if(seq==UINT32_MAX) safe_stop(DRIVER_SAFE_SEQUENCE_EXHAUSTED);
        }
        safe_unlock(irq);return 1;
    }
    if(strncmp(s,"SET-DUTY,",9)==0) {
        p=s+9;
        if(safe_duty(&p,&left,',') && safe_duty(&p,&right,'\r') && *p++=='\n' && *p=='\0') {
            irq=safe_lock();
            if(left || right || safe_state==DRIVER_SAFE_ARMED) safe_stop(DRIVER_SAFE_MANUAL);
            safe_unlock(irq);
        }
        return 1;
    }
    return strncmp(s,"SAFE-",5)==0;
}
void driver_safety_service(void)
{
    char response[64]; uint32_t session,seq,irq; unsigned state,reason; int n;
    irq=safe_lock();
    if(!safe_reply && !((safe_state>=DRIVER_SAFE_LOCAL_READY && safe_state<=DRIVER_SAFE_ARMED) &&
        (uint32_t)(safe_ms-safe_last_status)>=20u)) {safe_unlock(irq);return;}
    safe_last_status=safe_ms;
    safe_reply=0;session=safe_session;seq=safe_sequence;state=safe_state;reason=safe_reason;
    safe_unlock(irq);
    n=snprintf(response,sizeof response,"SAFE-STATUS,1,%u,%lu,%lu,%u,1\r\n",
               state,(unsigned long)session,(unsigned long)seq,reason);
    if(n>0 && (size_t)n<sizeof response) uart_write_buffer(MOTOR_DRIVER_UART,(uint8 *)response,(uint32)n);
}
