"""Verify independent final directions across both protocols and all flywheel modes."""
from pathlib import Path
import ast
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
tree = ast.parse((root/'tests/check_pwm_uart.py').read_text(encoding='utf-8-sig'))
boundary = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign)
                and any(isinstance(t,ast.Name) and t.id=='boundary' for t in n.targets))
test = r'''
#include "balance_control.h"
#include <assert.h>
#include <stdio.h>
struct host_attitude attitude_state;
volatile float imu_roll_deg,imu_pitch_deg,imu_sample_dt_ms=5;
volatile uint32 imu_sample_sequence;
volatile int32 imu_attitude_status=1;
static int wire_left,wire_right,ground;
uint32 interrupt_global_disable(void){return 0;}
void interrupt_global_enable(uint32 n){(void)n;}
void motor_m_ctrl(int n){ground=n;}
uint8 uart_query_byte(int p,uint8 *b){(void)p;(void)b;return 0;}
void uart_init(int p,uint32 b,int tx,int rx){(void)p;(void)b;(void)tx;(void)rx;}
void uart_rx_interrupt(int p,int on){(void)p;(void)on;}
void uart_write_buffer(int port,const uint8 *data,uint32 size){
    assert(port==UART_3);
#if QCAR_DRIVER_TEXT_DUTY
    char text[32];assert(size<sizeof(text));memcpy(text,data,size);text[size]=0;
    assert(sscanf(text,"SET-DUTY,%d,%d",&wire_left,&wire_right)==2);
#else
    uint8 sum=0;assert(size==7 && data[0]==0xA5 && data[1]==1);
    for(unsigned i=0;i<6;++i)sum+=data[i];assert(sum==data[6]);
    wire_left=(int16)((data[2]<<8)|data[3]);wire_right=(int16)((data[4]<<8)|data[5]);
#endif
}
static void tick(void){++imu_sample_sequence;for(unsigned i=0;i<APP_CONTROL_DIVIDER;++i)balance_control_tick_1ms();}
static void expect(int left,int right,int wheel){
    assert(wire_left==left*QCAR_FLYWHEEL_LEFT_SIGN);
    assert(wire_right==right*QCAR_FLYWHEEL_RIGHT_SIGN);
    assert(ground==wheel);
    assert(costrained_flywheel_speed_left==left && -costrained_flywheel_speed_right==right);
}
int main(void){
    balance_control_defaults();shacheflag=1;control_startup_ramp_ms=0;pwm_debug_arm();
    pwm_debug_fly_left=300;pwm_debug_fly_right=-700;pwm_debug_ground=-500;tick();expect(300,-700,-500);
    pwm_debug_fly_right=1200;tick();expect(300,1200,-500);
    for(int mode=3;mode<=6;++mode){
        if(mode==4)continue;
        balance_control_defaults();shacheflag=1;control_startup_ramp_ms=0;
        ground_rate_test_kp=30;ground_deadzone_enable=0; /* Isolate direction mapping. */
        imu_roll_deg=BALANCE_ZERO_X_DEG;imu_pitch_deg=BALANCE_ZERO_Y_DEG;
        attitude_state.gyro.x=4;attitude_state.gyro.y=10;
        if(mode==5)dual_balance_arm();else if(mode==6)roll_rate_only_arm();else roll_rate_test_arm();
        tick();expect(-(int)(10*FLY_RATE_KP),(int)(10*FLY_RATE_KP),mode==5?-120:0);
    }
    roll_rate_test_stop();clear_all_cascade_out();balance_control_tick_1ms();expect(0,0,0);
    printf("PASS: protocol=%d, left sign=%d, right sign=%d; Mode3/5/6/8 and stop\n",
           QCAR_DRIVER_TEXT_DUTY,QCAR_FLYWHEEL_LEFT_SIGN,QCAR_FLYWHEEL_RIGHT_SIGN);
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-direction-') as directory:
    work=Path(directory)
    (work/'zf_common_headfile.h').write_text(boundary,encoding='utf-8')
    (work/'test.c').write_text(test,encoding='utf-8')
    includes=['-I'+str(work),'-I'+str(root/'code')]
    for protocol in (0,1):
        for left in (-1,1):
            for right in (-1,1):
                defines=[f'-DQCAR_DRIVER_TEXT_DUTY={protocol}',f'-DQCAR_FLYWHEEL_LEFT_SIGN={left}',f'-DQCAR_FLYWHEEL_RIGHT_SIGN={right}']
                exe=work/'direction.exe'
                subprocess.run(['gcc','-std=c99',*includes,*defines,str(root/'code/balance_control.c'),str(root/'code/wheel_hold.c'),str(root/'tests/encoder_snapshot_stub.c'),str(root/'code/pid.c'),str(root/'code/small_driver_uart_control.c'),str(work/'test.c'),'-lm','-o',str(exe)],check=True)
                subprocess.run([str(exe)],check=True,timeout=5)
    for side in ('LEFT','RIGHT'):
        name='QCAR_FLYWHEEL_'+side+'_SIGN'
        invalid=subprocess.run(['gcc','-std=c99',*includes,'-D'+name+'=0','-fsyntax-only',str(root/'code/small_driver_uart_control.c')],capture_output=True)
        assert invalid.returncode and name.encode('ascii') in invalid.stderr, 'invalid direction accepted'
