#include <assert.h>
#include <stdlib.h>
#include "driver_safety_host.h"
#include "driver_pwm_host.h"
#include "motor_driver_uart_control.h"
#include "driver_safety.h"
#include "driver_trace.h"
test_motor motor_left,motor_right;
test_battery battery_value;
uint8_t motor_calibration_valid=1;
uint32_t test_irq;
stc_TCPWM_GRP_CNT_t test_pwm[6];
stc_GPIO_PRT_t test_gpio[2];
static char reply[128];
static unsigned accepted;
int uart_query_byte(int uart,uint8_t *byte) {(void)uart;(void)byte;return 0;}
void uart_write_buffer(int uart,const uint8_t *data,uint32_t n) {
    (void)uart;assert(n<sizeof reply);memcpy(reply,data,n);reply[n]=0;
}
void driver_trace_on_rx_byte(uint8_t b) {(void)b;}
void driver_trace_on_command(int16_t l,int16_t r) {(void)l;(void)r;++accepted;}
int driver_trace_text(const uint8_t *t) {(void)t;return 0;}
static void send(const char *s) {while(*s)motor_driver_receive_byte((uint8_t)*s++);}
static unsigned status(unsigned *state,unsigned *seq,unsigned *reason) {
    unsigned session,cap;
    send("SAFE-STATUS,1\r\n");driver_safety_service();
    assert(sscanf(reply,"SAFE-STATUS,1,%u,%u,%u,%u,%u",state,&session,seq,reason,&cap)==5);
    assert(cap==1 && strlen(reply)<64);return session;
}
static void expect(unsigned expected,unsigned cause) {
    unsigned state,seq,reason;status(&state,&seq,&reason);assert(state==expected);assert(reason==cause);
}
static void low(void) {
    unsigned i,j;for(i=0;i<2;++i)for(j=0;j<6;++j)assert(test_gpio[i].mux[j]==0 && test_gpio[i].level[j]==0);
}
static void tick(unsigned n,int main_alive,int left_alive,int right_alive) {
    while(n--) {
        if(main_alive)driver_safety_main_alive();
        if(left_alive)driver_safety_control_alive(0);
        if(right_alive)driver_safety_control_alive(1);
        driver_safety_tick_1ms();
    }
}
static void physical(void) {
    driver_safety_button_sample(1);driver_safety_button_sample(1);
    driver_safety_button_sample(0);driver_safety_button_sample(0);
}
static void reset(void) {
    memset(&motor_left,0,sizeof motor_left);memset(&motor_right,0,sizeof motor_right);
    memset(&battery_value,0,sizeof battery_value);motor_calibration_valid=1;
    memset(test_gpio,0xff,sizeof test_gpio);memset(test_pwm,0xff,sizeof test_pwm);
    driver_safety_init();low();expect(DRIVER_SAFE_DISARMED,DRIVER_SAFE_BOOT);
}
static unsigned arm(void) {
    unsigned state,seq,reason,session;char s[80];
    tick(20,1,1,1);physical();session=status(&state,&seq,&reason);assert(state==DRIVER_SAFE_LOCAL_READY && session);
    send("SET-DUTY,0,0\r\n");assert(status(&state,&seq,&reason)==session && state==DRIVER_SAFE_LOCAL_READY);
    snprintf(s,sizeof s,"SAFE-ARM,1,%u\r\n",session);send(s);expect(DRIVER_SAFE_LOCAL_READY,0);
    snprintf(s,sizeof s,"SAFE-ZERO,1,%u\r\n",session);send(s);expect(DRIVER_SAFE_ZEROED,0);
    send("SET-DUTY,0,0\r\n");expect(DRIVER_SAFE_ZEROED,0);
    snprintf(s,sizeof s,"SAFE-ARM,1,%u\r\n",session);send(s);expect(DRIVER_SAFE_ARMED,0);low();return session;
}
static void duty(unsigned session,unsigned seq,int left,int right) {
    char s[100];snprintf(s,sizeof s,"SAFE-DUTY,1,%u,%u,%d,%d\r\n",session,seq,left,right);send(s);
    motor_left_duty_set(1000,2000,3000);motor_right_duty_set(1000,2000,3000);
}
static void malformed_tests(void) {
    static const char *bad[]={
      "SAFE-DUTY,1,1,2,0,501\r\n", "SAFE-DUTY,1,1,2,1,500\r\n",
      "SAFE-DUTY,1,1,2,0,-501\r\n", "SAFE-DUTY,1,1,2,0,10001\r\n",
      "SAFE-DUTY,1,1,2,0,32768\r\n", "SAFE-DUTY,1,1,2,0,99999999999999999999\r\n",
      "SAFE-DUTY,1,1,2,0,\r\n", "SAFE-DUTY,1,1,2,,500\r\n",
      "SAFE-DUTY,1,1,2,0,+500\r\n", "SAFE-DUTY,1,1,2,0, 500\r\n",
      "SAFE-DUTY,1,1,2,0,0500\r\n", "SAFE-DUTY,1,1,2,-0,500\r\n",
      "SAFE-DUTY,1,1,2,0,1x\r\n", "SAFE-DUTY,1,1,2,0,1,9\r\n",
      "SAFE-DUTY,1,1,4294967296,0,1\r\n", "SAFE-DUTY,1,01,2,0,1\r\n",
      "SAFE-DUTY,1,2,2,0,1\r\n", "SAFE-DUTY,2,1,2,0,1\r\n",
      "XSAFE-DUTY,1,1,2,0,1\r\n", "SAFE-DUTY,1,1,2,0,1\n",
      "SAFE-DUTY,1,1,2,0,1\r\r\n", "SAFE-DUTY,1,1,2,0,1\r\nx\n",
      "SET-DUTY,999999999999999999999999,1\r\n", "SET-DUTY,1\r\n"
    };
    unsigned i;
    for(i=0;i<sizeof bad/sizeof *bad;++i) {
        unsigned s,before;reset();s=arm();duty(s,1,0,100);before=accepted;
        tick(90,1,1,1);send(bad[i]);
        /* A valid prefix followed by another line remains a valid whole frame. */
        if(i==21) {assert(accepted==before+1);continue;}
        assert(accepted==before);tick(10,1,1,1);expect(DRIVER_SAFE_LATCHED,DRIVER_SAFE_COMMAND_TIMEOUT);low();
    }
}
int main(void) {
    unsigned s,state,seq,reason,i,before;char buf[220];
    reset();send("SET-DUTY,500,500\r\n");low();assert(motor_right.motor_duty==0);reset();
    for(i=0;i<10;++i) {driver_safety_button_sample(0);}
    expect(0,1); /* held on boot */
    send("SAFE-ZERO,1,1\r\nSAFE-ARM,1,1\r\nSAFE-DUTY,1,1,1,0,500\r\n");low();expect(0,1);
    s=arm();duty(s,1,0,500);assert(motor_right.motor_duty==0.05f);
    for(i=0;i<6;++i){assert(test_gpio[0].mux[i]==0);assert(test_gpio[1].mux[i]==8);}
    /* Real final PWM gate refuses non-permitted A and fault injection. */
    motor_left.motor_duty=0.5f;motor_left_duty_set(4000,4000,4000);
    for(i=0;i<3;++i)assert(test_pwm[i].unCC0_BUFF.u32Register==test_pwm[i].unCC1_BUFF.u32Register);
    motor_right.motor_duty=0.2f;motor_right_duty_set(2000,2000,2000);low();motor_right.motor_duty=0.05f;
    before=accepted;tick(90,1,1,1);duty(s,1,0,500);assert(accepted==before);tick(10,1,1,1);
    expect(4,3);low();duty(s,2,0,500);send("SAFE-ZERO,1,1\r\nSAFE-ARM,1,1\r\n");expect(4,3);low();
    for(i=0;i<10;++i) {driver_safety_button_sample(0);}
    expect(4,3);
    s=arm();assert(s==2);duty(1,10,0,500);assert(motor_right.motor_duty==0);duty(s,1,0,-500);assert(motor_right.motor_duty==-0.05f);
    /* Reboot invalidates permission, even a complete old transcript cannot arm. */
    reset();send("SAFE-ZERO,1,2\r\nSAFE-ARM,1,2\r\nSAFE-DUTY,1,2,99,0,500\r\n");expect(0,1);low();
    malformed_tests();
    /* Overlong/embedded NUL/binary/partial frames and recovery. */
    reset();s=arm();memset(buf,'9',sizeof buf);memcpy(buf,"SAFE-DUTY,1,1,1,0,",strlen("SAFE-DUTY,1,1,1,0,"));
    for(i=0;i<sizeof buf;++i) {motor_driver_receive_byte(buf[i]);}
    motor_driver_receive_byte('\n');
    assert(motor_right.motor_duty==0);duty(s,1,0,200);assert(motor_right.motor_duty==0.02f);
    send("SAFE-DUTY,1,1,2,0,");motor_driver_receive_byte(0);send("500\r\n");assert(motor_right.motor_duty==0.02f);
    send("SAFE-DUTY,1,1,2,0,");tick(100,1,1,1);send("500\r\n");expect(4,3);low();
    reset();{const uint8_t binary[]={0xa5,1,0,0,1,0xf4,0x9b};for(i=0;i<7;++i)motor_driver_receive_byte(binary[i]);}motor_driver_receive_byte('\n');low();expect(0,1);
    /* Absolute pulse limit cannot be reset by zero, fresh duty or status chatter. */
    reset();s=arm();duty(s,1,0,100);
    for(i=2;i<=10;++i){tick(20,1,1,1);duty(s,i,0,(i&1)?100:0);}
    tick(20,1,1,1);expect(4,5);low();
    reset();s=arm();duty(s,1,0,100);tick(10,1,0,1);expect(4,10);low();
    reset();s=arm();duty(s,1,0,100);tick(10,1,1,0);expect(4,10);low();
    reset();s=arm();duty(s,1,0,100);tick(90,0,1,1);duty(s,2,0,100);tick(10,0,1,1);expect(4,9);low();
    reset();physical();tick(5000,1,1,1);expect(4,4);
    reset();motor_calibration_valid=0;physical();expect(4,6);low();
    reset();s=arm();duty(s,1,0,100);battery_value.protect_flag=1;battery_value.battery_state=BATTERY_ERROR;tick(1,1,1,1);expect(4,7);low();
    reset();s=arm();duty(s,1,0,100);send("SET-DUTY,0,0\r\n");expect(0,2);low();
    reset();s=arm();duty(s,1,0,100);send("SAFE-DISARM,1\r\n");expect(0,2);low();
    reset();s=arm();duty(s,UINT32_MAX,0,500);expect(4,8);low();
    reset();s=arm();duty(s,1,0,500);send("SET-ZERO\r\nTEST-PHASE\r\n");assert(status(&state,&seq,&reason)==s && state==3);tick(100,1,1,1);expect(4,3);low();
    /* Deterministic byte fuzz with no local permission: never arms or writes duty. */
    reset();{uint32_t rng=0x12345678u;for(i=0;i<100000u;++i) {
        rng=rng*1664525u+1013904223u;motor_driver_receive_byte((uint8_t)(rng>>24));
        if((i%17u)==0)tick(1,1,1,1);
    }}
    motor_driver_receive_byte('\n');low();assert(motor_right.motor_duty==0);
    printf("Driver safety real-source UART and PWM mocks passed\n");return 0;
}
