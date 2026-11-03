#ifndef _my_flash_h_
#define _my_flash_h_
#include "zf_common_headfile.h"
#include "IfxScuWdt.h"
#include "IfxFlash.h"
#include "IfxFlash_cfg.h"

#define QCAR_EEPROM_BASE_ADDR        IFXFLASH_DFLASH_START
#define QCAR_EEPROM_SIZE             IFXFLASH_DFLASH_SIZE//12*0x2000(8kb)  96kb  1(Byte)字节=8（bit)位  1KB=1024B
#define QCAR_EEPROM_SECTOR_SIZE      (QCAR_EEPROM_SIZE /IFXFLASH_DFLASH_NUM_LOG_SECTORS) //8kb
#define QCAR_EEPROM_SECTOR_NUM       (IFXFLASH_DFLASH_NUM_LOG_SECTORS)//12个扇区

#define QCAR_EEPROM_PAGE_SIZE        IFXFLASH_DFLASH_PAGE_LENGTH
#define QCAR_EEPROM_SECTOR_PAGE_NUM  (QCAR_EEPROM_SECTOR_SIZE/QCAR_EEPROM_PAGE_SIZE) //每个扇区1024页


//---------------------------------------------------------------------------------------
//  @brief      使用宏定义对flash进行数据读取
//  @param      sector_num      需要写入的扇区编号
//  @param      page_num        需要读取的页
//  @param      type            读取的数据类型
//  @return                     返回给定地址的数据
//  @since      v1.0
//  Sample usage:               my_flash_read(0,0,uint32);//读取0号扇区  第0页数据 类型为uint32

/* Parenthesize every argument: callers pass page expressions such as i+1. */
#define my_flash_read(sector_num, page_num, type) (*(volatile type *)(QCAR_EEPROM_BASE_ADDR + ((sector_num) * QCAR_EEPROM_SECTOR_SIZE) + ((page_num) * QCAR_EEPROM_PAGE_SIZE)))
//-------------------------------------------------------------------------------------------------------------------
//  @brief      将浮点数转换为二进制数据
//  @param      float_data      需要转换的浮点数
//  @return                     返回整型数据
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------

uint8 my_flash_check(uint32 sector_num, uint32 page_num);
int eeprom_erase_sector(uint32 sector_num);
int eeprom_page_program(uint32 sector_num, uint32 page_num, uint32 *buf);
int write_flash(void);
void get_flash(void);

#endif
