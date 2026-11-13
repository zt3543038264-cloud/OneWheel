"""Byte-for-byte link of delivered TC264 transport and CYT2BL3 SAFE/PWM sources.
Only hardware registers, UART endpoints and time are mocked. Not board validation.
"""
from pathlib import Path
import os
import subprocess
from wmotor_host import ROOT, HEADER, HARDWARE
from test_external_driver_safety import body, CODE, MOCK


def test_real_host_driver_handshake(tmp_path):
    (tmp_path/'zf_common_headfile.h').write_text(HEADER)
    (tmp_path/'IfxStm.h').write_text('#include \"zf_common_headfile.h\"\n')
    host = tmp_path/'host.c'
    host.write_text(HARDWARE + r'''
void driver_test_init(void); void driver_test_tick(void); void driver_test_physical(void);
void driver_test_rx(uint8); void driver_test_service(void); int driver_test_take(uint8*);
float driver_test_right(void); int driver_test_inhibited(void); int driver_test_speed_requested(void);
static unsigned consumed;
static void bridge(void){
 for(unsigned pass=0;pass<20;pass++) {
  uint8 bytes[512],byte;unsigned n=0;
  drain();while(consumed<wn)driver_test_rx((uint8)wire[consumed++]);
  driver_test_service();while(driver_test_take(&byte)){assert(n<sizeof bytes);bytes[n++]=byte;}
  if(n)feed(bytes,n);else if(!fill&&!tx_enabled)return;
 }
 assert(!"Bridge did not quiesce");
}
static void millisecond(int host_service,int exchange){
 advance(1);driver_test_tick();W_Motor_Tick1ms();
 if(host_service)W_Motor_BenchService();
 if(exchange)bridge();
}
int main(void){
 driver_test_init();boot();bridge();assert(driver_test_inhibited());
 assert(driver_test_speed_requested()); /* Legacy CRLF read query interoperates. */
 for(unsigned i=0;i<40;i++)millisecond(0,1);
 assert(!W_Motor_BenchRequestB(500,150,1));
 driver_test_physical();bridge();assert(snapshot().ready&&snapshot().session==1);
 W_Motor_KeepZero();bridge();assert(snapshot().ready);
 assert(!W_Motor_BenchRequestB(500,150,2));assert(W_Motor_BenchRequestB(500,150,1));bridge();
 assert(snapshot().driver_state==2);assert(driver_test_right()==0);
 W_Motor_Tick1ms();bridge();assert(snapshot().driver_state==2);assert(driver_test_right()==0);
 W_Motor_BenchService();bridge();assert(snapshot().driver_state==3);assert(driver_test_right()==0);
 W_Motor_BenchService();bridge();assert(driver_test_right()==0.05f);assert(!driver_test_inhibited());
 for(unsigned i=0;i<149;i++){millisecond(1,1);assert(W_Motor_BenchBusy());assert(driver_test_right()==0.05f);}
 millisecond(1,1);assert(!W_Motor_BenchBusy());assert(driver_test_right()==0);assert(driver_test_inhibited());
 assert(snapshot().reason==W_BENCH_REASON_COMPLETE);
 driver_test_physical();bridge();assert(snapshot().session==2);
 assert(!W_Motor_BenchRequestB(-500,150,1));assert(W_Motor_BenchRequestB(-500,150,2));bridge();
 W_Motor_BenchService();bridge();W_Motor_BenchService();bridge();assert(driver_test_right()==-0.05f);
 /* Link is physically cut: driver stops without receiving host DISARM. */
 for(unsigned i=0;i<100;i++)millisecond(0,0);
 assert(driver_test_right()==0);assert(driver_test_inhibited());assert(!W_Motor_BenchBusy());
 bridge();assert(!W_Motor_BenchBusy());
 puts("PASS: real TC264/CYT2BL3 bytes, physical grant, handshake, B-only pulse,150ms host stop,100ms link loss");
 return 0;
}
''')
    adapter=tmp_path/'driver_adapter.c'
    adapter.write_text(r'''
#include <assert.h>
#include "driver_safety_host.h"
#include "driver_pwm_host.h"
#include "driver_safety.h"
#include "motor_driver_uart_control.h"
test_motor motor_left,motor_right;
test_battery battery_value;
uint8_t motor_calibration_valid=1;
uint32_t test_irq;
stc_TCPWM_GRP_CNT_t test_pwm[6];
stc_GPIO_PRT_t test_gpio[2];
static uint8_t reply[2048];static unsigned count,index;
int uart_query_byte(int uart,uint8_t *byte){(void)uart;(void)byte;return 0;}
void uart_write_buffer(int uart,const uint8_t *data,uint32_t n){(void)uart;assert(count+n<sizeof reply);memcpy(reply+count,data,n);count+=n;}
void driver_trace_on_rx_byte(uint8_t b){(void)b;}
void driver_trace_on_command(int16_t a,int16_t b){(void)a;(void)b;}
int driver_trace_text(const uint8_t *p){(void)p;return 0;}
void driver_test_init(void){driver_safety_init();}
void driver_test_rx(uint8_t b){motor_driver_receive_byte(b);}
void driver_test_service(void){driver_safety_service();motor_left_duty_set(1000,2000,3000);motor_right_duty_set(1000,2000,3000);}
int driver_test_take(uint8_t *b){if(index==count){index=count=0;return 0;}*b=reply[index++];return 1;}
void driver_test_tick(void){driver_safety_main_alive();driver_safety_control_alive(0);driver_safety_control_alive(1);driver_safety_tick_1ms();}
void driver_test_physical(void){driver_safety_button_sample(1);driver_safety_button_sample(1);driver_safety_button_sample(0);driver_safety_button_sample(0);}
float driver_test_right(void){assert(motor_left.motor_duty==0);return motor_right.motor_duty;}
int driver_test_inhibited(void){
 for(unsigned n=0;n<6;n++){assert(test_gpio[0].mux[n]==0&&test_gpio[0].level[n]==0);if(test_gpio[1].mux[n]!=0||test_gpio[1].level[n]!=0)return 0;}return 1;
}
int driver_test_speed_requested(void){return motor_value.continue_command==GET_SPEED;}
''')
    source=(CODE/'pwm_output.c').read_bytes().decode('latin1')
    helpers=source[source.index('static uint8_t safe_right_connected;'):source.index('//----------------')]
    gate=tmp_path/'gate.c'
    gate.write_text('#include "driver_pwm_host.h"\n#include "driver_safety.h"\n'+helpers+'\n'.join(
        body(source,s) for s in ['void motor_left_duty_set(', 'void motor_right_duty_set(',
                                'void motor_left_channel_set(', 'void motor_right_channel_set(']))
    flags=['gcc','-std=c99','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unknown-pragmas',
           '-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-no-pie']
    subprocess.run([*flags,'-I'+str(tmp_path),'-I'+str(ROOT/'code'),'-c',str(host),'-o',str(tmp_path/'host.o')],check=True,capture_output=True,text=True)
    objects=[tmp_path/'host.o']
    for i,path in enumerate([CODE/'motor_driver_uart_control.c',adapter,gate]):
        obj=tmp_path/f'driver{i}.o';objects.append(obj)
        subprocess.run([*flags,'-DDRIVER_SAFETY_HOST','-Duart_query_byte=cyt2_uart_query_byte',
                        '-I'+str(MOCK),'-I'+str(CODE),'-c',str(path),'-o',str(obj)],check=True,capture_output=True,text=True)
    exe=tmp_path/'integration'
    subprocess.run([*flags,*(str(p) for p in objects),'-o',str(exe)],check=True,capture_output=True,text=True)
    result=subprocess.run([str(exe)],capture_output=True,text=True,env=dict(os.environ,ASAN_OPTIONS='detect_leaks=0'))
    assert result.returncode==0,result.stdout+result.stderr


if __name__=='__main__':
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        test_real_host_driver_handshake(Path(d))
    print('PASS: TC264/CYT2BL3 real-source integration (ASan+UBSan, mocked hardware)')
