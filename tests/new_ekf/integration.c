#include "HW_Drivers/IMU/imu.h"
#include "Estimation/Attitude/attitude.h"
#include "Estimation/Attitude/new_ekf/imu.h"
#include "Estimation/Attitude/new_ekf/attitude_algorithm.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
int host_stm,host_forward_dir=-1;
static uint64 ticks;
static uint32 bursts, spi_delay_us, rollover_on_lower;
static uint8 regs[128],ready=0xc0,selected,fail_bus,fail_init,bad_readback,all_ff;
static int16 raw_a[3]={-4096,0,0},raw_g[3]={0,0,0};
int16 imu660ra_acc_x,imu660ra_acc_y,imu660ra_acc_z,imu660ra_gyro_x,imu660ra_gyro_y,imu660ra_gyro_z;
float imu660ra_transition_factor[2]={4096,16.384f};
uint32 IfxStm_getLower(int *p) {(void)p;if(rollover_on_lower){ticks+=rollover_on_lower;rollover_on_lower=0;}return (uint32)ticks;}
uint32 IfxStm_getOffset32Timer(int *p) {(void)p;return (uint32)(ticks>>32);}
float IfxStm_getFrequency(int *p) {(void)p;return 1000000.f;}
uint32 interrupt_global_disable(void) {return 0;}
void interrupt_global_enable(uint32 p) {(void)p;}
void system_delay_ms(uint32 ms) {ticks+=(uint64)1000*ms;}
void gpio_high(int pin) {assert(pin==13 && selected);selected=0;}
void gpio_low(int pin) {assert(pin==13 && !selected);selected=1;}
uint8 imu660ra_init(void) {memset(regs,0,sizeof(regs));regs[0]=0x24;regs[0x41]=2;return fail_init;}
uint8 spi_transfer_8bit_timeout(int spi,const uint8 *tx,uint8 *rx,uint32 n,uint32 ms) {
 assert(spi==0 && selected && ms==1);ticks+=spi_delay_us;if(fail_bus)return 1;
 uint8 reg=tx[0]&127;
 if(n==2) {regs[reg]=tx[1];return 0;}
 memset(rx,0,n);
 if(n==3) {rx[2]=reg==3?ready:(regs[reg]^(bad_readback && reg==0x40 ? 1:0));return 0;}
 assert(n==14 && reg==0xc);bursts++;if(all_ff){memset(rx,255,n);return 0;}
 for(unsigned i=0;i<3;i++) {rx[2+2*i]=(uint8)raw_a[i];rx[3+2*i]=(uint8)((uint16)raw_a[i]>>8);rx[8+2*i]=(uint8)raw_g[i];rx[9+2*i]=(uint8)((uint16)raw_g[i]>>8);}
 return 0;
}
static void tick(void) {ticks+=1000;imu_update_gyro();attitude_update_rate();attitude_tick1ms();attitude_update();}
static void boot(void) {ready=0xc0;fail_bus=fail_init=bad_readback=all_ff=0;spi_delay_us=rollover_on_lower=0;raw_g[0]=raw_g[1]=raw_g[2]=0;raw_a[0]=-4096;raw_a[1]=raw_a[2]=0;assert(!imu_init());imu_calibrate();assert(imu_calib_state()==IMU_CALIB_OK);attitude_init();assert(attitude_converged() && !attitude_diverged());}
/* All recovery here is simulated. The production application has no reset or
 * recovery call after control_init; boot() is a fresh stopped-timer boot. */
static void expect_locked(const char *text)
{
 imu_data_t raw,mean;float x,y,z;
 assert(imu_link_lost() && !attitude_converged() && attitude_diverged());
 assert(!imu_get_raw_sample(&raw) && !imu_get_startup_mean(&mean));
 assert(!imu_get_static_acc(&x,&y,&z));
 if(text)assert(!strcmp(imu_fault_text(),text));
 attitude_update_rate();assert(att.roll_rate==0 && att.pitch_rate==0 && att.yaw_rate==0);
}
static void test_calibration_axes_and_cadence(void)
{
 imu_navigation_sample_t nav;imu_data_t raw;
 boot();assert(regs[0x40]==0xab && regs[0x42]==0xeb && regs[0x7d]==0xe);
 assert(imu_attitude_method==3 && imu_get_navigation_sample(&nav));assert(nav.accel_g[2]==1);
 uint32 seq=imu_sample_sequence,reads=bursts;tick();assert(imu_sample_sequence==seq && bursts==reads && !att_diag.sample_fresh);
 tick();assert(imu_sample_sequence==seq+1 && bursts==reads+1 && att_diag.sample_fresh && fabsf(imu_sample_dt_ms-2)<.001f);
 raw_g[0]=-164;raw_g[1]=328;raw_g[2]=-492;
 for(int i=0;i<100;i++){tick();}
 assert(imu_get_raw_sample(&raw));assert(fabsf(raw.gyro_x-492/16.384f)<.001f && fabsf(raw.gyro_y-328/16.384f)<.001f);
 assert(imu_get_navigation_sample(&nav));assert(att.roll==imu_pitch_deg && att.pitch==imu_roll_deg);
 assert(fabsf(att.roll_rate-nav.gyro_dps[1])<.001f && fabsf(att.pitch_rate-nav.gyro_dps[0])<.001f && fabsf(att.yaw_rate-nav.gyro_dps[2])<.001f);
 raw_a[0]=-3547;raw_a[1]=-2048;raw_a[2]=0;raw_g[0]=raw_g[1]=raw_g[2]=0;
 assert(!imu_init());imu_calibrate();attitude_init();assert(attitude_converged());
 assert(fabsf(att.roll-30)<.05f && fabsf(att.pitch)<.01f);
}
static void test_short_dropout_needs_fresh_sequence(void)
{
 boot();uint32 seq=imu_sample_sequence;ready=0;
 for(int i=0;i<6;i++){tick();assert(!att_diag.sample_fresh && imu_sample_sequence==seq);}
 assert(attitude_converged()); /* Bounded hold, not a claim of a new sample. */
 ready=0xc0;tick();assert(!att_diag.sample_fresh && imu_sample_sequence==seq);
 tick();assert(att_diag.sample_fresh && imu_sample_sequence==seq+1 && attitude_converged());
 boot();seq=imu_sample_sequence;ready=0;
 for(int i=0;i<20;i++){tick();}ready=0xc0;
 tick();tick(); /* Original 22 ms dt is skipped and cannot refresh publication. */
 assert(imu_sample_sequence==seq && !att_diag.sample_fresh && !imu_link_lost());
 tick();tick();assert(imu_sample_sequence==seq+1 && attitude_converged());
}
static void test_reboot_only_stale_and_bus_recovery(void)
{
 boot();uint32 seq=imu_sample_sequence;ready=0;
 for(int i=0;i<24;i++){tick();assert(!imu_link_lost());}
 tick();expect_locked("IMU STALE / REBOOT");assert(imu_sample_sequence==seq);
 uint32 reads=bursts;ready=0xc0;for(int i=0;i<1000;i++){tick();}
 assert(bursts==reads && imu_sample_sequence==seq);expect_locked("IMU STALE / REBOOT");
 imu_calibrate();imu_recover_after_stall();imu_prepare_start();imu_service();
 for(int i=0;i<1000;i++){tick();}expect_locked("IMU STALE / REBOOT");
 boot();ready=0;for(int i=0;i<110;i++){tick();}ready=0xc0;
 for(int i=0;i<1000;i++){tick();}expect_locked("IMU STALE / REBOOT");
 boot();fail_bus=1;for(int i=0;i<110;i++){tick();}assert(imu_spi_timeouts>0);
 fail_bus=0;for(int i=0;i<1000;i++){tick();}expect_locked("IMU STALE / REBOOT");
 boot();assert(attitude_converged()); /* Reboot plus a new successful calibration only. */
}
static void test_elapsed_deadline_and_wrap(void)
{
 boot();uint32 seq=imu_sample_sequence,reads=bursts;
 ticks+=24999;assert(!imu_link_lost());ticks++;expect_locked("IMU STALE / REBOOT");
 assert(imu_sample_sequence==seq && bursts==reads); /* No poll needed to expire. */
 boot();seq=imu_sample_sequence;reads=bursts;ticks+=60000;tick();tick();
 assert(imu_sample_sequence==seq && bursts==reads);expect_locked("IMU STALE / REBOOT");
 boot();spi_delay_us=13000;tick();tick();expect_locked("IMU STALE / REBOOT");
 spi_delay_us=0; /* A new internal frame after an overrun cannot erase the deadline. */
 ticks=(uint64)UINT32_MAX-1021000;boot();
 for(int i=0;i<40;i++){tick();}assert(ticks>UINT32_MAX && attitude_converged());
 /* A lower-32 wrap during the high/low/high read retries coherently. */
 ticks=(uint64)UINT32_MAX-1007000;boot();
 assert((uint32)ticks>UINT32_MAX-10000u);rollover_on_lower=8000;
 assert(!imu_link_lost());
 ticks+=17000;expect_locked("IMU STALE / REBOOT");
 boot();ticks+=((uint64)1<<32)+2000;expect_locked("IMU STALE / REBOOT");
 /* Once observed, neither many low-counter wraps nor timer rollback clears it. */
 ticks+=((uint64)7<<32);expect_locked("IMU STALE / REBOOT");ticks=0;
 expect_locked("IMU STALE / REBOOT");
}
static void test_original_clipping_and_bad_bursts(void)
{
 boot();raw_g[0]=32767;uint32 seq=imu_sample_sequence;tick();tick();
 assert(imu_sample_sequence==seq && imu_gyro_clips>0 && !att_diag.sample_fresh);
 raw_g[0]=0;tick();tick();assert(imu_sample_sequence==seq+1 && attitude_converged());
 raw_g[0]=32767;for(int i=0;i<24;i++){tick();assert(!imu_link_lost());}
 tick();assert(imu_attitude_status==IMU_ATTITUDE_RUNNING);expect_locked("IMU STALE / REBOOT");
 for(int i=0;i<70000;i++){tick();}expect_locked("IMU STALE / REBOOT");
 raw_g[0]=0;for(int i=0;i<1000;i++){tick();}expect_locked("IMU STALE / REBOOT");
 boot();all_ff=1;seq=imu_sample_sequence;tick();tick();
 assert(imu_sample_sequence==seq && imu_bad_bursts>0 && !att_diag.sample_fresh);
 all_ff=0;tick();tick();assert(attitude_converged());
}
static void test_calibration_timeout_and_original_faults(void)
{
 boot();raw_g[0]=164;assert(!imu_init());uint64 started=ticks;imu_calibrate();
 assert(imu_calib_state()==IMU_CALIB_MOVED && ticks-started>=10000000u);
 expect_locked("CALIB TIMEOUT / REBOOT");raw_g[0]=0;
 for(int i=0;i<1500;i++){tick();}
 assert(imu_attitude_status==IMU_ATTITUDE_CALIBRATING && imu_calib_state()==IMU_CALIB_MOVED);
 expect_locked("CALIB TIMEOUT / REBOOT");
 imu_calibrate();expect_locked("CALIB TIMEOUT / REBOOT");
 /* Transfer latency counts toward the 10 s deadline, not just delay calls. */
 assert(!imu_init());raw_g[0]=164;spi_delay_us=3000;started=ticks;
 uint32 reads=bursts;imu_calibrate();
 assert(ticks-started>=10000000u && ticks-started<10008000u);
 assert(bursts-reads<1300u);expect_locked("CALIB TIMEOUT / REBOOT");
 spi_delay_us=0;raw_g[0]=0;
 /* Even direct use of unchanged imported service/update cannot unlock master. */
 imu_request_recalibration();imu_service();
 for(int i=0;i<510;i++){ticks+=2000;imu_update();}
 assert(imu_attitude_status==IMU_ATTITUDE_RUNNING);
 expect_locked("CALIB TIMEOUT / REBOOT");
 boot();ticks+=60000;imu_update();assert(imu_attitude_status==IMU_ATTITUDE_BAD_DT);
 expect_locked("RA BAD DT / REBOOT");imu_recover_after_stall();
 for(int i=0;i<5;i++){ticks+=2000;imu_update();}
 assert(imu_attitude_status==IMU_ATTITUDE_RUNNING);expect_locked("RA BAD DT / REBOOT");
 boot();imu_attitude_status=IMU_ATTITUDE_FILTER_ERROR;expect_locked("EKF ERROR / REBOOT");
 boot();imu_request_recalibration();imu_service();expect_locked("IMU RESTART / REBOOT");
 bad_readback=1;assert(imu_init());expect_locked("RA INIT FAIL / REBOOT");bad_readback=0;
 fail_init=1;assert(imu_init());expect_locked("RA INIT FAIL / REBOOT");fail_init=0;
 boot();assert(attitude_converged());
}
int main(void)
{
 test_calibration_axes_and_cadence();
 test_short_dropout_needs_fresh_sequence();
 test_reboot_only_stale_and_bus_recovery();
 test_elapsed_deadline_and_wrap();
 test_original_clipping_and_bad_bursts();
 test_calibration_timeout_and_original_faults();
 puts("new RA/EKF shim: 500 Hz, axes, original clipping, elapsed/wrap deadline, boot timeout and reboot-only recovery PASS");
 return 0;
}
