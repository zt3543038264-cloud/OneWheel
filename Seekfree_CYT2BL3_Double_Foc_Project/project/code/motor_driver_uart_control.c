/*********************************************************************************************************************
* CYT2BL3 Opensource Library 即（ CYT2BL3 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT2BL3 开源库的一部分
*
* CYT2BL3 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          motor_driver_uart_control
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT2BL3
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2025-01-03       pudding            first version
********************************************************************************************************************/

#include "motor_driver_uart_control.h"
#include "driver_trace.h"

small_device_value_struct motor_value;

/* Reuse the existing receive buffer; no second command queue. */
uint8 read_buffer[128];
#include "driver_safety.inc"

void motor_driver_parse_statement(small_device_value_struct *device_value, uint8 *statement_buffer)
{
    const char *s=(const char *)statement_buffer; unsigned n;
    if(device_value->cmd_type!=STRING_TYPE) return; /* legacy binary never actuates */
    for(n=0;n<DRIVER_SAFE_FRAME_BYTES && s[n];++n) {}
    if(n<2 || n>=DRIVER_SAFE_FRAME_BYTES || s[n-2]!='\r' || s[n-1]!='\n') return;
    if(safe_wire(s)) return;
    if(strcmp(s,"GET-SPEED\r\n")==0) device_value->continue_command=GET_SPEED;
    else if(strcmp(s,"GET-ANGLE\r\n")==0) device_value->continue_command=GET_ANGLE;
    else if(strcmp(s,"GET-RDT-ANGLE\r\n")==0) device_value->continue_command=GET_RDT_ANGLE;
    else if(strcmp(s,"GET-ENCODER\r\n")==0) device_value->continue_command=GET_ENCODER;
    else if(strcmp(s,"GET-VOLTAGE\r\n")==0) device_value->continue_command=GET_VOLTAGE;
    else if(strcmp(s,"STOP-SEND\r\n")==0) device_value->continue_command=NULL_CMD;
    /* SET-DUTY, SET-ZERO, phase tests, and binary commands intentionally ignored.
     * Trace capture is passive and cannot renew the motion watchdog. */
    else if(strncmp(s,"SET-TRACE,",10)==0 || strncmp(s,"GET-TRACE,",10)==0)
        (void)driver_trace_text(statement_buffer);
}

void motor_driver_receive_byte(uint8_t byte)
{
    if(safe_rx_length && (uint32_t)(safe_ms-safe_rx_start)>=DRIVER_SAFE_COMMAND_MS) {
        safe_rx_length=0;safe_rx_drop=1;
    }
    if(safe_rx_drop) {
        if(byte=='\n') safe_rx_drop=0;
        return;
    }
    if(!safe_rx_length) safe_rx_start=safe_ms;
    if(byte==0 || (byte<32 && byte!='\r' && byte!='\n') || byte>126 ||
       safe_rx_length>=DRIVER_SAFE_FRAME_BYTES-1u) {
        safe_rx_length=0;safe_rx_drop=(byte!='\n');return;
    }
    read_buffer[safe_rx_length++]=byte;
    if(byte=='\n') {
        read_buffer[safe_rx_length]=0;
        motor_value.cmd_type=STRING_TYPE;
        motor_driver_parse_statement(&motor_value,read_buffer);
        safe_rx_length=0;
    }
}
/* Parsing is byte-stream based now; retained for source compatibility. */
void motor_driver_control_loop(small_device_value_struct *device_value) { (void)device_value; }

void motor_driver_set_duty(small_device_value_struct *device_value, int left_duty, int right_duty)
{
    device_value->send_data_buffer[0] = 0xA5;
    device_value->send_data_buffer[1] = 0X01;
    device_value->send_data_buffer[2] = (uint8)((left_duty  & 0xFF00) >> 8);
    device_value->send_data_buffer[3] = (uint8)( left_duty  & 0x00FF);
    device_value->send_data_buffer[4] = (uint8)((right_duty & 0xFF00) >> 8);
    device_value->send_data_buffer[5] = (uint8)( right_duty & 0x00FF);
    device_value->send_data_buffer[6] = device_value->send_data_buffer[0] + 
                                        device_value->send_data_buffer[1] + 
                                        device_value->send_data_buffer[2] + 
                                        device_value->send_data_buffer[3] + 
                                        device_value->send_data_buffer[4] + 
                                        device_value->send_data_buffer[5];

    uart_write_buffer(MOTOR_DRIVER_UART, device_value->send_data_buffer, 7);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送速度信息  字节包类型
// 参数说明     device_value        通讯参数结构体
// 参数说明     left_speed          左侧电机速度
// 参数说明     right_speed         右侧电机速度
// 返回参数     void
// 使用示例     motor_driver_send_speed(&motor_value, motor_left.motor_speed_filter, motor_right.motor_speed_filter);   // 字节包通讯则按照协议打包输出
// 备注信息       
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_send_speed(small_device_value_struct *device_value, int left_speed, int right_speed)
{
    device_value->send_data_buffer[0] = 0xA5;
    device_value->send_data_buffer[1] = 0X02;
    device_value->send_data_buffer[2] = (uint8)((left_speed  & 0xFF00) >> 8);
    device_value->send_data_buffer[3] = (uint8)( left_speed  & 0x00FF);
    device_value->send_data_buffer[4] = (uint8)((right_speed & 0xFF00) >> 8);
    device_value->send_data_buffer[5] = (uint8)( right_speed & 0x00FF);
    device_value->send_data_buffer[6] = device_value->send_data_buffer[0] + 
                                        device_value->send_data_buffer[1] + 
                                        device_value->send_data_buffer[2] + 
                                        device_value->send_data_buffer[3] + 
                                        device_value->send_data_buffer[4] + 
                                        device_value->send_data_buffer[5];
    uart_write_buffer(MOTOR_DRIVER_UART, device_value->send_data_buffer, 7);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送角度信息  字节包类型
// 参数说明     device_value        通讯参数结构体
// 参数说明     left_speed          左侧电机角度
// 参数说明     right_speed         右侧电机角度
// 返回参数     void
// 使用示例     motor_driver_send_angle(&motor_value, left_angle, right_angle);   // 字节包通讯则按照协议打包输出
// 备注信息       
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_send_angle(small_device_value_struct *device_value, float left_angle, float right_angle)
{
    int16 left_angle_int = (int16)(func_limit_ab(left_angle, -327.67f, 327.67f) * 100.0f);
    int16 righ_angle_int = (int16)(func_limit_ab(right_angle, -327.67f, 327.67f) * 100.0f);
    
    device_value->send_data_buffer[0] = 0xA5;
    device_value->send_data_buffer[1] = 0X04;
    device_value->send_data_buffer[2] = (uint8)((left_angle_int  & 0xFF00) >> 8);
    device_value->send_data_buffer[3] = (uint8)( left_angle_int  & 0x00FF);
    device_value->send_data_buffer[4] = (uint8)((righ_angle_int & 0xFF00) >> 8);
    device_value->send_data_buffer[5] = (uint8)( righ_angle_int & 0x00FF);
    device_value->send_data_buffer[6] = device_value->send_data_buffer[0] + 
                                        device_value->send_data_buffer[1] + 
                                        device_value->send_data_buffer[2] + 
                                        device_value->send_data_buffer[3] + 
                                        device_value->send_data_buffer[4] + 
                                        device_value->send_data_buffer[5];
    uart_write_buffer(MOTOR_DRIVER_UART, device_value->send_data_buffer, 7);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送减速后角度信息  字节包类型
// 参数说明     device_value        通讯参数结构体
// 参数说明     left_speed          左侧电机减速后角度
// 参数说明     right_speed         右侧电机减速后角度
// 返回参数     void
// 使用示例     motor_driver_send_reduction_angle(&motor_value, left_angle, right_angle);   // 字节包通讯则按照协议打包输出
// 备注信息       
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_send_reduction_angle(small_device_value_struct *device_value, float left_angle, float right_angle)
{
    uint16 left_angle_int = (uint16)(left_angle * 100.0f);
    uint16 righ_angle_int = (uint16)(right_angle * 100.0f);
    
    device_value->send_data_buffer[0] = 0xA5;
    device_value->send_data_buffer[1] = 0X05;
    device_value->send_data_buffer[2] = (uint8)((left_angle_int  & 0xFF00) >> 8);
    device_value->send_data_buffer[3] = (uint8)( left_angle_int  & 0x00FF);
    device_value->send_data_buffer[4] = (uint8)((righ_angle_int & 0xFF00) >> 8);
    device_value->send_data_buffer[5] = (uint8)( righ_angle_int & 0x00FF);
    device_value->send_data_buffer[6] = device_value->send_data_buffer[0] + 
                                        device_value->send_data_buffer[1] + 
                                        device_value->send_data_buffer[2] + 
                                        device_value->send_data_buffer[3] + 
                                        device_value->send_data_buffer[4] + 
                                        device_value->send_data_buffer[5];
    uart_write_buffer(MOTOR_DRIVER_UART, device_value->send_data_buffer, 7);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口通讯控制 回调函数
// 参数说明     device_value        通讯参数结构体
// 返回参数     void
// 使用示例     motor_driver_control_callback(&motor_value);
// 备注信息       
//-------------------------------------------------------------------------------------------------------------------
void motor_driver_control_callback(small_device_value_struct *device_value)
{
    uint8 receive_data,receive_max=10;
    (void)device_value;
    while(receive_max-- && uart_query_byte(MOTOR_DRIVER_UART,&receive_data)) {
        driver_trace_on_rx_byte(receive_data);
        motor_driver_receive_byte(receive_data);
    }
}

void motor_driver_uart_init(void)
{
    uart_init(MOTOR_DRIVER_UART,460800,MOTOR_DRIVER_RX,MOTOR_DRIVER_TX);
    uart_rx_interrupt(MOTOR_DRIVER_UART,1);
    /* Dedicated safety PIT is CPUIntIdx2 priority 0; UART cannot starve it. */
    NVIC_SetPriority(CPUIntIdx7_IRQn,2u);
}
