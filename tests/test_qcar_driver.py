"""Actual nonblocking UART3 driver: zero-only ordinary paths and legacy RPM."""
from wmotor_host import compile_run


def test_text_mailbox_and_binary_rpm(tmp_path):
    compile_run(tmp_path, r'''
int main(void){
 boot();assert(strstr(wire,"SAFE-DISARM,1\r\nSET-DUTY,0,0\r\n")&&strstr(wire,"GET-SPEED\r\n"));assert(W_Motor_LinkLost());
 W_Motor_Release();clearwire();unsigned before=written;W_Motor_SetDuty(-4000,4000);assert(written-before<=16);drain();assert(!strcmp(wire,"SET-DUTY,0,0\r\n"));assert(w_motor_brake_locked);
 /* Even direct use of the ordinary transport cannot enqueue nonzero duty. */
 clearwire();wm_text_command(W_MOTOR_SET_DUTY_CMD,10000,-10000);for(int i=0;i<100;i++)wm_text_command(W_MOTOR_SET_DUTY_CMD,i,-i);W_Motor_Stop();drain();
 assert(!strstr(wire,"10000"));assert(strstr(wire,"SET-DUTY,0,0\r\n"));assert(strstr(wire,"SAFE-DISARM,1\r\n"));assert(!strstr(wire,"SET-DUTY,99,-99\n"));
 line("123,-456\n");W_Motor_Tick1ms();assert(W_Motor_GetSpeed1()==123&&W_Motor_GetSpeed2()==-456&&!W_Motor_LinkLost());
 uint32 frames=w_motor_rx_frames;line("bad,123\n");assert(w_motor_rx_frames==frames);
 uint8 bad[90];memset(bad,'1',sizeof(bad));bad[86]=',';bad[87]='2';bad[88]='\n';feed(bad,89);assert(w_motor_rx_frames==frames);
 uint8 packet[7]={0xa5,2,0x0a,0xf0,0xf5,0x10,0};for(int i=0;i<6;i++)packet[6]+=packet[i];feed(packet,7);W_Motor_Tick1ms();assert(W_Motor_GetSpeed1()==2800&&W_Motor_GetSpeed2()==-2800);
 for(int i=0;i<W_MOTOR_LINK_TIMEOUT_MS;i++){W_Motor_Tick1ms();}assert(W_Motor_LinkLost());
 puts("PASS: bounded UART3 TX, zero-only ordinary paths, strict RPM parser, binary LF payload, watchdog");return 0;}
''')
