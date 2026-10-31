"""Exercise real Mode8 controller -> real seven-byte UART serializer."""
from pathlib import Path
import subprocess
import tempfile

root=Path(__file__).resolve().parents[1]
boundary=r'''
#ifndef HOST_BOUNDARY
#define HOST_BOUNDARY
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "config.h"
#include "qcar_types.h"
typedef uint8_t uint8;typedef uint32_t uint32;typedef int32_t int32;typedef int16_t int16;
#define UART_3 3
#define UART3_TX_P15_6 0
#define UART3_RX_P15_7 1
#define IMU_ATTITUDE_CALIBRATING 0
extern struct host_attitude {struct {float x,y;} gyro;} attitude_state;
extern volatile float imu_roll_deg,imu_pitch_deg,imu_sample_dt_ms;
extern volatile uint32 imu_sample_sequence;
extern volatile int32 imu_attitude_status;
uint32 interrupt_global_disable(void);void interrupt_global_enable(uint32);
void small_driver_set_duty(int16,int16);void motor_m_ctrl(int);
uint8 uart_query_byte(int,uint8 *);
void uart_write_buffer(int,const uint8 *,uint32);
void uart_init(int,uint32,int,int);void uart_rx_interrupt(int,int);
#endif
'''
test=r'''
#include "balance_control.h"
#include <assert.h>
#include <stdio.h>
struct host_attitude attitude_state;
volatile float imu_roll_deg,imu_pitch_deg,imu_sample_dt_ms;
volatile uint32 imu_sample_sequence;
volatile int32 imu_attitude_status=-1;
static uint8 frame[7];static int ground;
uint32 interrupt_global_disable(void){return 0;}
void interrupt_global_enable(uint32 n){(void)n;}
void motor_m_ctrl(int n){ground=n;}
uint8 uart_query_byte(int port,uint8 *b){(void)port;(void)b;return 0;}
void uart_write_buffer(int port,const uint8 *data,uint32 size){assert(port==UART_3 && size==7);memcpy(frame,data,7);}
void uart_init(int p,uint32 b,int tx,int rx){(void)p;(void)b;(void)tx;(void)rx;}
void uart_rx_interrupt(int p,int on){(void)p;(void)on;}
static void tick(void){for(unsigned i=0;i<APP_CONTROL_DIVIDER;i++)balance_control_tick_1ms();}
static void check(float left,float right,int wheel,const uint8 expected[7]){
    pwm_debug_fly_left=left;pwm_debug_fly_right=right;pwm_debug_ground=(float)wheel;tick();
    assert(!memcmp(frame,expected,7));assert(ground==wheel);
    assert(costrained_flywheel_speed_left==(int16)left);
    assert(-costrained_flywheel_speed_right==(int16)right);
    printf("logical L=%g R=%g, reversed wire:",(double)left,(double)right);
    for(unsigned i=0;i<7;i++)printf(" %02X",frame[i]);puts("");
}
int main(void){
    balance_control_defaults();pwm_debug_arm();shacheflag=1;control_startup_ramp_ms=0;
    check(300,0,0,(uint8[]){0xA5,1,0xFE,0xD4,0,0,0x78});
    check(0,300,0,(uint8[]){0xA5,1,0,0,0xFE,0xD4,0x78});
    check(300,-700,-500,(uint8[]){0xA5,1,0xFE,0xD4,2,0xBC,0x36});
    check(-300,0,0,(uint8[]){0xA5,1,1,0x2C,0,0,0xD3});
    roll_rate_test_stop();balance_control_tick_1ms();
    assert(!memcmp(frame,(uint8[]){0xA5,1,0,0,0,0,0xA6},7) && !ground);
    /* Closed-loop callers also pass through the same single reversal. */
    for(int mode=3;mode<=6;mode++) {
        if(mode==4)continue;
        balance_control_defaults();shacheflag=1;control_startup_ramp_ms=0;
        roll_rate_test_kp=36;ground_rate_test_kp=30;ground_deadzone_enable=0;
        imu_attitude_status=1;imu_roll_deg=BALANCE_ZERO_X_DEG;imu_pitch_deg=BALANCE_ZERO_Y_DEG;
        attitude_state.gyro.y=10;attitude_state.gyro.x=4;
        if(mode==5)dual_balance_arm();else if(mode==6)roll_rate_only_arm();else roll_rate_test_arm();
        ++imu_sample_sequence;tick();
        assert(!memcmp(frame,(uint8[]){0xA5,1,1,0x68,0xFE,0x98,0xA5},7));
        assert(ground==(mode==5?-120:0));
        printf("Mode%d: logical -360/+360 -> wire +360/-360, ground=%d\n",mode,ground);
    }
    roll_rate_test_stop();balance_control_tick_1ms();
    assert(!memcmp(frame,(uint8[]){0xA5,1,0,0,0,0,0xA6},7) && !ground);
    puts("PASS: Mode3/5/6/8 final flywheel reversal, independent channels, checksum, unchanged ground and stop zero");
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-pwm-uart-') as directory:
    work=Path(directory)
    (work/'zf_common_headfile.h').write_text(boundary,encoding='utf-8')
    (work/'test.c').write_text(test,encoding='utf-8')
    exe=work/'test.exe'
    subprocess.run(['gcc','-std=c99','-DQCAR_DRIVER_TEXT_DUTY=0','-I'+str(work),'-I'+str(root/'code'),
                    str(root/'code/balance_control.c'),str(root/'code/wheel_hold.c'),str(root/'tests/encoder_snapshot_stub.c'),str(root/'code/pid.c'),
                    str(root/'code/small_driver_uart_control.c'),str(work/'test.c'),
                    '-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=5)
