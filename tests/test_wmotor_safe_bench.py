"""SAFE v1 real host implementation, STM/FIFO mocked only. No hardware claims."""
from wmotor_host import compile_run


def test_identity_and_strict_frame_bounds(tmp_path):
    compile_run(tmp_path, r'''
int main(void){
 boot();clearwire();line("0,0\n");W_Motor_Tick1ms();assert(!W_Motor_LinkLost());
 assert(!W_Motor_BenchRequestB(100,100,1));assert(!snapshot().ready);
 const char *bad[]={"SAFE-STATUS,1,1,1,0,0\n", "SAFE-STATUS,2,1,1,0,0,1\n",
 "SAFE-STATUS,1,1,1,0,0,0\n", "SAFE-STATUS,1,1,0,0,0,1\n",
 "SAFE-STATUS,1,1,1,1,0,1\n", "SAFE-STATUS,1,1,1,0,11,1\n",
 "SAFE-STATUS,1,1,1,0,0,1,extra\n", "SAFE-STATUS,1,1,+1,0,0,1\n",
 "SAFE-STATUS,1,1,4294967296,0,0,1\n", "SAFE-STATUS,1,1,1,4294967296,0,1\n",
 "SAFE-STATUS,1,1,1,0,0,1\rX\n", "SAFE-STATUS,1,5,1,0,0,1\n"};
 for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++){line(bad[i]);assert(!snapshot().ready);assert(!W_Motor_BenchRequestB(100,100,1));}
 status(1,1,0,0);assert(snapshot().ready);W_Motor_KeepZero();drain();assert(snapshot().ready);
 assert(!W_Motor_BenchRequestB(501,100,1));assert(!W_Motor_BenchRequestB(-501,100,1));assert(!W_Motor_BenchRequestB(0,100,1));
 assert(!W_Motor_BenchRequestB(100,0,1));assert(!W_Motor_BenchRequestB(100,151,1));assert(!W_Motor_BenchRequestB(100,100,0));assert(!W_Motor_BenchRequestB(100,100,2));
 advance(60);assert(!snapshot().ready);assert(!W_Motor_BenchRequestB(100,100,1));
 status(1,1,0,0);W_Motor_Stop();drain();status(1,1,0,0);assert(!snapshot().ready);assert(!W_Motor_BenchRequestB(100,100,1));
 status(1,2,0,0);assert(snapshot().ready);assert(W_Motor_BenchRequestB(-500,150,2));drain();assert(strstr(wire,"SAFE-ZERO,1,2\r\n"));
 assert(!strstr(wire,"SAFE-ARM"));assert(!strstr(wire,"SAFE-DUTY"));
 puts("PASS: strict v1/capability identity, explicit session, bounds, freshness, Stop consumes grant");return 0;}
''')


def test_positive_ack_and_absolute_stm_deadline(tmp_path):
    compile_run(tmp_path, r'''
int main(void){
 boot();clearwire();ticks=0xfffffff0ULL;status(1,77,0,0);assert(W_Motor_BenchRequestB(500,150,77));drain();
 assert(snapshot().host_state==W_BENCH_WAIT_ZERO);
 status(2,77,0,0);W_Motor_Tick1ms();drain();assert(!strstr(wire,"SAFE-ARM"));
 W_Motor_BenchService();drain();assert(strstr(wire,"SAFE-ARM,1,77\r\n"));assert(!strstr(wire,"SAFE-DUTY"));
 status(3,77,0,0);W_Motor_Tick1ms();drain();assert(!strstr(wire,"SAFE-DUTY"));
 W_Motor_BenchService();drain();assert(strstr(wire,"SAFE-DUTY,1,77,1,0,500\r\n"));assert(snapshot().right_duty==500);
 status(3,77,1,0);
 for(unsigned n=1;n<=7;n++){advance(20);W_Motor_Tick1ms();assert(W_Motor_BenchBusy());W_Motor_BenchService();drain();status(3,77,n+1,0);}
 advance(9);W_Motor_Tick1ms();assert(W_Motor_BenchBusy());assert(snapshot().elapsed_ms==149);
 advance(1);W_Motor_Tick1ms();drain();assert(!W_Motor_BenchBusy());assert(snapshot().reason==W_BENCH_REASON_COMPLETE);assert(snapshot().right_duty==0);
 assert(strstr(wire,"SAFE-DISARM,1\r\nSET-DUTY,0,0\r\n"));
 /* Delayed service / a dropped tick cannot extend pulse: real STM is authority. */
 status(1,78,0,0);assert(W_Motor_BenchRequestB(-100,5,78));drain();status(2,78,0,0);W_Motor_BenchService();drain();status(3,78,0,0);W_Motor_BenchService();drain();
 advance(6);W_Motor_BenchService();drain();assert(!W_Motor_BenchBusy());assert(snapshot().reason==W_BENCH_REASON_COMPLETE);
 puts("PASS: positive zero+arm ACK, Tick never energizes, 64-bit STM150ms/short pulse deadline");return 0;}
''')


def test_link_session_sequence_and_abort_priority(tmp_path):
    compile_run(tmp_path, r'''
static void pulse(uint32 session){status(1,session,0,0);assert(W_Motor_BenchRequestB(200,150,session));drain();status(2,session,0,0);W_Motor_BenchService();drain();status(3,session,0,0);W_Motor_BenchService();drain();status(3,session,1,0);}
int main(void){
 boot();clearwire();pulse(1);
 advance(60);line("0,0\n");W_Motor_Tick1ms();drain();assert(!W_Motor_BenchBusy());assert(snapshot().reason==W_BENCH_REASON_LINK);
 pulse(2);status(3,3,1,0);drain();assert(!W_Motor_BenchBusy());assert(snapshot().reason==W_BENCH_REASON_SESSION);
 pulse(4);status(3,4,2,0);drain();assert(!W_Motor_BenchBusy());assert(snapshot().reason==W_BENCH_REASON_PROTOCOL);
 pulse(5);status(3,5,0,0);drain();assert(!W_Motor_BenchBusy());assert(snapshot().reason==W_BENCH_REASON_PROTOCOL);
 pulse(6);line("SAFE-STATUS,1,3,6,1,0,1,bad\n");drain();assert(!W_Motor_BenchBusy());assert(snapshot().reason==W_BENCH_REASON_PROTOCOL);
 pulse(7);W_Motor_SetDuty(1000,1000);drain();assert(!W_Motor_BenchBusy());assert(snapshot().right_duty==0);
 pulse(8);clearwire();
 /* Fill FIFO with a read-only request so next nonzero is pending, not begun. */
 wm_status_pending=1;wm_query_pending=1;W_Motor_TxHandler();advance(20);W_Motor_BenchService();assert(wm_pending_kind==WM_TX_SAFE);
 W_Motor_Stop();drain();assert(!strstr(wire,"SAFE-DUTY,1,8,2,"));assert(strstr(wire,"SAFE-DISARM,1\r\n"));
 status(1,8,0,0);assert(!W_Motor_BenchRequestB(200,150,8));
 puts("PASS: legacy cannot renew SAFE link, session/sequence mismatch latches, pending duty purged by Stop");return 0;}
''')


def test_timeout_and_no_autorearm(tmp_path):
    compile_run(tmp_path, r'''
int main(void){
 boot();clearwire();status(1,1,0,0);assert(W_Motor_BenchRequestB(200,100,1));drain();
 advance(40);status(1,1,0,0);advance(20);W_Motor_Tick1ms();drain();assert(!W_Motor_BenchBusy());assert(snapshot().reason==W_BENCH_REASON_TIMEOUT);
 status(1,2,0,0);for(unsigned i=0;i<10;i++){W_Motor_Tick1ms();W_Motor_BenchService();}drain();assert(!strstr(wire,"SAFE-ARM"));assert(!strstr(wire,"SAFE-DUTY"));
 assert(W_Motor_BenchRequestB(200,100,2));drain();status(2,2,0,0);W_Motor_BenchService();drain();
 advance(30);status(2,2,0,0);advance(30);W_Motor_BenchService();drain();assert(snapshot().reason==W_BENCH_REASON_TIMEOUT);
 status(1,3,0,0);assert(W_Motor_BenchRequestB(200,150,3));drain();status(2,3,0,0);W_Motor_BenchService();drain();status(3,3,0,0);W_Motor_BenchService();drain();
 advance(40);status(3,3,0,0);advance(20);W_Motor_BenchService();drain();assert(!W_Motor_BenchBusy());assert(snapshot().reason==W_BENCH_REASON_TIMEOUT);
 /* Invalid clock fails closed. */
 clock_hz=0;fill=0;W_Motor_Init();drain();status(1,4,0,0);assert(!W_Motor_BenchRequestB(200,100,4));
 puts("PASS: handshake/ACK timeout, no reconnect/automatic rearm, invalid clock rejection");return 0;}
''')


def test_binary_build_remains_zero_only(tmp_path):
    compile_run(tmp_path, r'''
int main(void){boot();clearwire();W_Motor_Release();W_Motor_SetDuty(9999,-9999);drain();
 assert(wn==7&&wire[0]==(char)0xa5&&wire[1]==1&&wire[2]==0&&wire[3]==0&&wire[4]==0&&wire[5]==0);
 assert(!W_Motor_BenchRequestB(500,150,1));assert(!W_Motor_BenchBusy());return 0;}
''', extra=('-DQCAR_DRIVER_TEXT_DUTY=0',))


if __name__ == '__main__':
    import tempfile
    from pathlib import Path
    from test_qcar_driver import test_text_mailbox_and_binary_rpm
    checks = [test_text_mailbox_and_binary_rpm, test_identity_and_strict_frame_bounds,
              test_positive_ack_and_absolute_stm_deadline,
              test_link_session_sequence_and_abort_priority,
              test_timeout_and_no_autorearm, test_binary_build_remains_zero_only]
    for check in checks:
        with tempfile.TemporaryDirectory() as directory:
            check(Path(directory))
    print('PASS: 6 real-source W_Motor safety groups (mocked UART/STM, GCC -Werror)')
