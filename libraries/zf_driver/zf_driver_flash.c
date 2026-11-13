/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
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
* 文件名称          zf_driver_flash
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
* 2023-07-14       pudding            修复之前不完整的断言判断以及错误的注释内容
* 2023-07-15       pudding            写入时单独增加是否擦除判断，防止用户使用报错
********************************************************************************************************************/

/* CPU0 foreground only; no ISR/other-core Flash access during a transaction.
 * The vendor logical "page" is an 8 KiB erase sector. Each stored uint32
 * occupies the low half of one 8-byte hardware page. Keep this wire format. */
#include "Common/storage_profile.h"
#include "IfxScuWdt.h"
#include "IfxFlash.h"
#include "IfxStm.h"
#include "zf_common_interrupt.h"
#include "zf_driver_flash.h"
#include <string.h>

flash_data_union flash_union_buffer[EEPROM_PAGE_LENGTH];
static flash_status_t s_status;
static uint8 s_fault; /* Timeout/hardware errors latch until reset. */
#define BUSY_MASK (1u << 1)
#define PAGE_MASK (1u << 10)
/* Bundled TC26B FSR: OPER/SQER/PROER 11..13, DFlash ECC 18..21,
 * SRIADDERR 22, PVER/EVER 25..26. Reject even corrected DFlash ECC:
 * marginal/uncertain contents must not authorize a destructive operation.
 * A silicon bus/ECC trap may occur before C regains control. These status
 * checks do NOT implement trap recovery or validate blank-sector ECC behavior.
 * Keep STORAGE_FLASH_VALIDATED=0 until silicon/trap/ENDINIT timing is reviewed. */
#define ERROR_MASK ((7u << 11) | (31u << 18) | (3u << 25))
static int hardware_ok(void)
{
    if (FLASH0_FSR.U & ERROR_MASK) {
        s_status = FLASH_STATUS_HARDWARE; s_fault = 1; return 0;
    }
    return 1;
}
flash_status_t flash_last_status(void) { return s_status; }
static int ready(uint32 sector, uint32 page)
{
    if (!STORAGE_FLASH_VALIDATED) { s_status = FLASH_STATUS_DISABLED; return 0; }
    if (s_fault) return 0;
    if (sector != 0u || page >= EEPROM_PAGE_NUM) { s_status = FLASH_STATUS_ARGUMENT; return 0; }
    if (FLASH0_FSR.U & BUSY_MASK) { s_status = FLASH_STATUS_BUSY; return 0; }
    if (!hardware_ok()) return 0;
    s_status = FLASH_STATUS_OK;
    return 1;
}
static int wait_status(uint32 mask, uint32 value, uint32 ms)
{
    uint32 start = IfxStm_getLower(&MODULE_STM0);
    uint32 ticks = (uint32)(IfxStm_getFrequency(&MODULE_STM0) / 1000.0f) * ms;
    uint32 polls = 10000000u;
    do {
        uint32 status = FLASH0_FSR.U;
        if (status & ERROR_MASK) { s_status = FLASH_STATUS_HARDWARE; s_fault = 1; return 0; }
        if ((status & mask) == value) { __dsync(); return hardware_ok(); }
        if ((uint32)(IfxStm_getLower(&MODULE_STM0) - start) >= ticks) break;
    } while (--polls);
    s_status = FLASH_STATUS_TIMEOUT; s_fault = 1;
    return 0;
}
static uint32 address(uint32 page) { return IfxFlash_dFlashTableEepLog[page].start; }
static int read_word(uint32 addr, uint32 *value)
{
    uint32 v = *(volatile uint32 *)addr;
    /* Observe read-triggered ECC/address errors before publishing the value or
     * accepting an early occupied/blank/verify result. Never clear the flags. */
    __dsync();
    if (!hardware_ok()) return 0;
    *value = v;
    return 1;
}
uint8 flash_check(uint32 sector, uint32 page)
{
    uint32 i, addr, value;
    if (!ready(sector, page)) return 1; /* failure is never mistaken for blank */
    addr = address(page);
    /* Inspect BOTH hardware words, including unused high halves. */
    for (i = 0; i < EEPROM_PAGE_SIZE; i += 4u) {
        if (!read_word(addr+i, &value)) return 1;
        if (value != 0u) return 1;
    }
    return 0;
}
void flash_read_page(uint32 sector, uint32 page, uint32 *buf, uint16 len)
{
    uint32 i, addr, low, high;
    if (!ready(sector, page)) return;
    if (!buf || len > EEPROM_PAGE_LENGTH) { s_status = FLASH_STATUS_ARGUMENT; return; }
    addr = address(page);
    for (i = 0; i < len; i++) {
        if (!read_word(addr+i*FLASH_DATA_SIZE+4u, &high)) return;
        if (high != 0u) {
            s_status = FLASH_STATUS_VERIFY; return; /* incompatible packed layout */
        }
        if (!read_word(addr+i*FLASH_DATA_SIZE, &low)) return;
        buf[i] = low;
    }
}
void flash_erase_page(uint32 sector, uint32 page)
{
    uint32 irq;
    uint16 password;
    if (!ready(sector, page)) return;
    irq = interrupt_global_disable();
    IfxFlash_resetToRead(0);
    IfxFlash_clearStatus(0);
    password = IfxScuWdt_getSafetyWatchdogPassword();
    IfxScuWdt_clearSafetyEndinit(password);
    IfxFlash_eraseSector(address(page));
    IfxScuWdt_setSafetyEndinit(password);
    interrupt_global_enable(irq);
    if (!wait_status(BUSY_MASK, 0, 1000u)) return;
    IfxFlash_resetToRead(0);
    if (flash_check(sector, page) && s_status == FLASH_STATUS_OK)
        s_status = FLASH_STATUS_VERIFY;
}
void flash_write_page(uint32 sector, uint32 page, const uint32 *buf, uint16 len)
{
    uint32 i, irq, addr, error, low, high;
    uint16 password;
    if (!ready(sector, page)) return;
    if (!buf || !len || len > EEPROM_PAGE_LENGTH) { s_status = FLASH_STATUS_ARGUMENT; return; }
    /* Ownership belongs to the record layer. Never implicitly erase data. */
    if (flash_check(sector, page)) {
        if (s_status == FLASH_STATUS_OK) s_status = FLASH_STATUS_VERIFY;
        return;
    }
    for (i = 0; i < len; i++) {
        addr = address(page) + i*FLASH_DATA_SIZE;
        irq = interrupt_global_disable();
        error = IfxFlash_enterPageMode(addr);
        interrupt_global_enable(irq);
        if (error) { s_status = FLASH_STATUS_HARDWARE; s_fault = 1; return; }
        if (!wait_status(PAGE_MASK, PAGE_MASK, 10u)) return;
        irq = interrupt_global_disable();
        IfxFlash_loadPage(addr, buf[i], 0);
        password = IfxScuWdt_getSafetyWatchdogPassword();
        IfxScuWdt_clearSafetyEndinit(password);
        IfxFlash_writePage(addr);
        IfxScuWdt_setSafetyEndinit(password);
        interrupt_global_enable(irq);
        if (!wait_status(BUSY_MASK, 0, 10u)) return;
        IfxFlash_resetToRead(0);
        if (!read_word(addr, &low) || !read_word(addr+4u, &high)) return;
        if (low != buf[i] || high != 0u) {
            s_status = FLASH_STATUS_VERIFY; return;
        }
    }
}
void flash_read_page_to_buffer(uint32 sector, uint32 page)
{ flash_read_page(sector, page, (uint32 *)flash_union_buffer, EEPROM_PAGE_LENGTH); }
uint8 flash_write_page_from_buffer(uint32 sector, uint32 page)
{
    flash_write_page(sector, page, (const uint32 *)flash_union_buffer, EEPROM_PAGE_LENGTH);
    return (uint8)(s_status != FLASH_STATUS_OK);
}
void flash_buffer_clear(void) { memset(flash_union_buffer, 0, sizeof(flash_union_buffer)); }
