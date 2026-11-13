#include "HW_Drivers/IMU/imu.h"
#include "Estimation/Attitude/attitude.h"
#include "Estimation/Attitude/r10h/core/candidate/compare_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
uint32_t host_clock,host_clock_cost;
int host_forward_dir=-1;
int16 imu660rb_gyro_x,imu660rb_gyro_y,imu660rb_gyro_z,imu660rb_acc_x,imu660rb_acc_y,imu660rb_acc_z;
float imu660rb_transition_factor[2];
static uint8 regs[128],status=3;
static int16 sensor[6];
static unsigned reads,delay_ms;
static int bad_mode,moving;
extern compare_result_t v2_compare_result;
void spi_write_8bit_register(int bus,uint8 reg,uint8 v) { (void)bus; regs[reg&127]=v; }
uint8 spi_read_8bit_register(int bus,uint8 reg) { (void)bus; return (reg&127)==0x1e ? status : regs[reg&127]; }
void spi_read_8bit_registers(int bus,uint8 reg,uint8 *out,uint32 len)
{
    (void)bus; assert((reg&127)==0x22 && len==12); ++reads;
    for (unsigned i=0;i<6;i++) {
        int16 v=sensor[i]; if (moving && i==0) v=(reads&1)?100:-100;
        out[2*i]=(uint8)v; out[2*i+1]=(uint8)((uint16_t)v>>8);
    }
    if (bad_mode==1) memset(out,255,len);
    if (bad_mode==2) memset(out,0,len);
    if (bad_mode==3) memset(out+6,0,6);
    if (bad_mode==4) { out[0]=255;out[1]=127; }
}
uint8 r10h_host_bus_read(uint8 reg,uint8 *out,uint8 len)
{
    if (bad_mode==5) return 0; /* Emulated transaction deadline. */
    if (len==1) *out=spi_read_8bit_register(0,reg);
    else spi_read_8bit_registers(0,(uint8)(reg|128),out,len);
    return 1;
}
uint8 imu660rb_init(void) { memset(regs,0,sizeof(regs));regs[0xf]=0x6b;regs[0x12]=0x44;return 0; }
float imu660rb_gyro_transition(int16 v) { return v/imu660rb_transition_factor[1]; }
float imu660rb_acc_transition(int16 v) { return v/imu660rb_transition_factor[0]; }
void system_delay_ms(uint32 v) { host_clock+=v*1000u;delay_ms+=v; }
static void init(void)
{
    bad_mode=moving=0;status=3;host_clock=host_clock_cost=reads=delay_ms=0;
    sensor[0]=14;sensor[1]=-28;sensor[2]=42;sensor[3]=4098;sensor[4]=sensor[5]=0;
    assert(!imu_init());imu_calibrate();assert(imu_calib_state()==IMU_CALIB_OK);
    assert(reads==6000 && delay_ms>=6000);attitude_init();assert(!attitude_diverged());
}
static void tick(unsigned dt)
{
    host_clock+=dt;imu_update_gyro();attitude_update_rate();attitude_tick1ms();
    attitude_update();
}
int main(void)
{
    imu_data_t raw,mean;
    init();assert(imu_get_startup_mean(&mean));assert(fabsf(mean.gyro_z-14/14.3f)<.001f);
    for(unsigned i=0;i<4000;i++)tick(1000);
    assert(attitude_converged() && !attitude_diverged() && fabsf(att.yaw)<.01f);
    assert(v2_compare_result.samples==4000 && fabsf(att.roll_rate)<.001f);
    /* Raw gyro zero by itself is legitimate, unlike an all-zero six-axis frame. */
    sensor[0]=sensor[1]=sensor[2]=0;
    for(unsigned i=0;i<200;i++)tick(1000);
    assert(!imu_link_lost());
    for(int direction=-1;direction<=1;direction+=2) {
        host_forward_dir=direction;init();tick(1000);assert(imu_get_raw_sample(&raw));
        assert(fabsf(raw.gyro_x-direction*(-28/14.3f))<.001f);
        assert(fabsf(raw.gyro_y-direction*(42/14.3f))<.001f);
        assert(raw.acc_z==1 && fabsf(raw.gyro_z-14/14.3f)<.001f);
        host_forward_dir=-direction;assert(imu_forward_pending());host_forward_dir=direction;
    }
    host_forward_dir=-1;init();
    for(unsigned i=0;i<4000;i++)tick(1000);
    sensor[0]=1301; /* +90 deg/s body Z after bias subtraction. */
    for(unsigned i=0;i<10000;i++)tick(1000);
    assert(!attitude_diverged() && fabsf(att.yaw-900)<1.0f);
    assert(fabsf(att.yaw_wrapped)<=180 && fabsf(imu_get_angle_yaw()-att.yaw)<.001f);
    assert(fabsf(imu_get_angle_element()-att.yaw)<.01f);
    printf("continuous yaw %.4f, wrapped %.4f\n",att.yaw,att.yaw_wrapped);
    init();tick(1000);float old=att.yaw;tick(2000);attitude_update();
    assert(attitude_diverged() && !attitude_converged() && att.yaw==old);
    tick(1000);assert(attitude_diverged());
    init();tick(1000);tick(500);assert(attitude_diverged());
    init();host_clock=UINT32_MAX-500u;tick(1000);tick(1000);assert(!attitude_diverged());
    init();tick(1000);host_clock_cost=500;tick(1000);assert(attitude_diverged());
    for(int mode=1;mode<=5;mode++) {
        const char *expected[]={"", "IMU DATA ALL FF", "IMU ACC ALL ZERO", "IMU ACC ALL ZERO", "IMU SATURATED", "IMU SPI READ FAIL"};
        init();assert(!strcmp(imu_fault_text(),"IMU LINK LOST"));
        bad_mode=mode;tick(1000);assert(imu_link_lost() && attitude_diverged());
        assert(!strcmp(imu_fault_text(),expected[mode]));
        assert(!imu_get_raw_sample(&raw));bad_mode=0;tick(1000);assert(imu_link_lost());
        assert(!strcmp(imu_fault_text(),expected[mode])); /* First reason survives later samples. */
    }
    init();status=0;tick(1000);assert(imu_link_lost() && attitude_diverged());assert(!strcmp(imu_fault_text(),"IMU NOT READY"));
    init();regs[0xf]=0;for(unsigned i=0;i<100;i++)tick(1000);assert(imu_link_lost());assert(!strcmp(imu_fault_text(),"IMU BAD CHIP ID"));
    init();imu660rb_transition_factor[1]=0;tick(1000);assert(imu_link_lost());assert(!strcmp(imu_fault_text(),"IMU NONFINITE"));
    init();moving=1;imu_calibrate();assert(imu_calib_state()==IMU_CALIB_MOVED);
    assert(!imu_get_startup_mean(&mean));attitude_init();assert(attitude_diverged());
    init();sensor[5]=-2049;sensor[3]=3549;imu_calibrate();attitude_init();
    tick(1000);assert(fabsf(att.roll-30)<.05f); /* Cold gravity alignment. */
    puts("PASS: 6000-point raw mean, signs, single bias removal, 1/1 ms scheduling, continuous yaw, timing/STM wrap, faults, moved/tilted startup");
    return 0;
}
