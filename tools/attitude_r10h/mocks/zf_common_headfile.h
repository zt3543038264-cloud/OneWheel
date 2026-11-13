#ifndef HOST_ZF_H
#define HOST_ZF_H
#include <stdint.h>
typedef uint8_t uint8; typedef uint16_t uint16; typedef uint32_t uint32; typedef int16_t int16;
#define IMU660RB_USE_SOFT_IIC 0
#define IMU660RB_SPI 0
#define IMU660RB_SPI_W 0
#define IMU660RB_SPI_R 0x80
#define IMU660RB_CS(x) ((void)(x))
#define IMU660RB_CTRL1_XL 0x10
#define IMU660RB_CTRL2_G 0x11
#define IMU660RB_CTRL3_C 0x12
#define IMU660RB_WHO_AM_I 0x0f
#define IMU660RB_OUTX_L_G 0x22
extern int16 imu660rb_gyro_x,imu660rb_gyro_y,imu660rb_gyro_z,imu660rb_acc_x,imu660rb_acc_y,imu660rb_acc_z;
extern float imu660rb_transition_factor[2];
void spi_write_8bit_register(int,uint8,uint8);
uint8 spi_read_8bit_register(int,uint8);
void spi_read_8bit_registers(int,uint8,uint8 *,uint32);
uint8 imu660rb_init(void);
float imu660rb_gyro_transition(int16);
float imu660rb_acc_transition(int16);
void system_delay_ms(uint32);
#endif
