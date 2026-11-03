/* Upright-only parameter storage. New magic prevents interpreting old
 * image/PID records as this compact schema. Pages 0..14 only; marker last.
 * Save is still permitted only with control stopped by menu.c.
 */
#include "flash.h"
uint8 my_flash_check(uint32 sector_num, uint32 page_num)
{
    zf_assert(QCAR_EEPROM_SECTOR_NUM > sector_num);
    zf_assert(QCAR_EEPROM_SECTOR_PAGE_NUM > page_num);
    uint32 sector_addr = IfxFlash_dFlashTableEepLog[sector_num].start;
    uint32 page_addr = sector_addr + page_num * IFXFLASH_DFLASH_PAGE_LENGTH;

    uint32 temp = *(uint32 *)page_addr;
    if(0 !=temp) return 1;
    return 0;


}

/* The iLLD waitUnbusy() has no deadline, and entering page mode is
 * asynchronous: D0BUSY == 0 does NOT mean DFPAGE == 1. Never assert or
 * poll forever in a menu action. STM keeps running independently of IRQs. */
#include "IfxStm.h"
#define FLASH_BUSY_MASK (1u << 1)
#define FLASH_PAGE_MASK (1u << 10)
#define FLASH_ERROR_MASK ((7u << 11) | (3u << 25))
static int flash_wait(uint32 mask, uint32 expected, uint32 timeout_ms)
{
    uint32 start = IfxStm_getLower(&MODULE_STM0);
    uint32 ticks = (uint32)(IfxStm_getFrequency(&MODULE_STM0) / 1000.0f) * timeout_ms;
    uint32 polls = 10000000u; /* also bounded if STM is misconfigured */
    do {
        uint32 status = FLASH0_FSR.U;
        if (status & FLASH_ERROR_MASK) return 0;
        if ((status & mask) == expected) { __dsync(); return 1; }
        if ((uint32)(IfxStm_getLower(&MODULE_STM0) - start) >= ticks) return 0;
    } while (--polls);
    return 0;
}
static void flash_read_mode(void)
{
    /* Do not access DFlash or send another command after a busy timeout. */
    if (!(FLASH0_FSR.U & FLASH_BUSY_MASK)) IfxFlash_resetToRead(0);
}
int eeprom_erase_sector(uint32 sector_num)
{
    uint32 irq, sector_addr;
    uint16 password;
    if (sector_num >= QCAR_EEPROM_SECTOR_NUM) return 0;
    if (FLASH0_FSR.U & FLASH_BUSY_MASK) return 0;
    IfxFlash_resetToRead(0);
    IfxFlash_clearStatus(0);
    sector_addr = IfxFlash_dFlashTableEepLog[sector_num].start;
    /* Protect the command sequence and the safety-ENDINIT window only,
     * not the millisecond erase wait, from timer/UART interrupts. */
    irq = interrupt_global_disable();
    password = IfxScuWdt_getSafetyWatchdogPassword();
    IfxScuWdt_clearSafetyEndinit(password);
    IfxFlash_eraseSector(sector_addr);
    IfxScuWdt_setSafetyEndinit(password);
    interrupt_global_enable(irq);
    if (!flash_wait(FLASH_BUSY_MASK, 0, 1000)) { flash_read_mode(); return 0; }
    return 1;
}
int eeprom_page_program(uint32 sector_num, uint32 page_num, uint32 *buf)
{
    uint32 irq, page_addr, error;
    uint16 password;
    if (sector_num >= QCAR_EEPROM_SECTOR_NUM ||
        page_num >= QCAR_EEPROM_SECTOR_PAGE_NUM || !buf) return 0;
    if (!flash_wait(FLASH_BUSY_MASK, 0, 10)) return 0;
    page_addr = IfxFlash_dFlashTableEepLog[sector_num].start + page_num * IFXFLASH_DFLASH_PAGE_LENGTH;
    irq = interrupt_global_disable();
    error = IfxFlash_enterPageMode(page_addr);
    interrupt_global_enable(irq);
    if (error || !flash_wait(FLASH_PAGE_MASK, FLASH_PAGE_MASK, 10)) {
        flash_read_mode(); return 0;
    }
    irq = interrupt_global_disable();
    IfxFlash_loadPage(page_addr, buf[0], 0);
    password = IfxScuWdt_getSafetyWatchdogPassword();
    IfxScuWdt_clearSafetyEndinit(password);
    IfxFlash_writePage(page_addr);
    IfxScuWdt_setSafetyEndinit(password);
    interrupt_global_enable(irq);
    if (!flash_wait(FLASH_BUSY_MASK, 0, 10)) { flash_read_mode(); return 0; }
    return 1;
}

#define BALANCE_STORAGE_MAGIC 0x4F574232u
#define BALANCE_PARAMETER_COUNT 12u
static float *const values[BALANCE_PARAMETER_COUNT]={
    &roll_rate_test_kp,&roll_rate_test_limit,&roll_rate_test_sign,
    &pitch_balance_kp,&pitch_balance_rate_limit,
    &ground_rate_test_kp,&ground_rate_test_limit,&ground_rate_test_sign,
    &ground_angle_kp,&ground_angle_rate_limit,
    &mono_params.balance_angle_x,&mono_params.balance_angle_y
};
static uint32 checksum_word(uint32 crc,uint32 word) {return (crc*16777619u)^word;}
static int stored_valid(const float v[BALANCE_PARAMETER_COUNT],uint32 ramp)
{
    unsigned i;
    for(i=0;i<BALANCE_PARAMETER_COUNT;++i) if(!isfinite(v[i])) return 0;
    return v[0]>=0 && v[0]<=500 && v[1]>=0 && v[1]<=BALANCE_PWM_LIMIT_MAX && (v[2]==1 || v[2]==-1) &&
        v[3]>=0 && v[3]<=100 && v[4]>0 && v[4]<=BALANCE_RATE_LIMIT_MAX_DPS &&
        v[5]>=0 && v[5]<=500 && v[6]>=0 && v[6]<=BALANCE_PWM_LIMIT_MAX && (v[7]==1 || v[7]==-1) &&
        v[8]>=0 && v[8]<=100 && v[9]>0 && v[9]<=BALANCE_RATE_LIMIT_MAX_DPS &&
        fabsf(v[10])<=20 && fabsf(v[11])<=20 && ramp<=5000;
}
int write_flash(void)
{
    float snapshot[BALANCE_PARAMETER_COUNT];
    uint32 words[BALANCE_PARAMETER_COUNT],crc=BALANCE_STORAGE_MAGIC,magic=BALANCE_STORAGE_MAGIC;
    uint32 ramp,irq; unsigned i;
    irq=interrupt_global_disable();
    for(i=0;i<BALANCE_PARAMETER_COUNT;++i) snapshot[i]=*values[i];
    ramp=control_startup_ramp_ms;
    interrupt_global_enable(irq);
    if(!stored_valid(snapshot,ramp)) return 0;
    memcpy(words,snapshot,sizeof(words));
    if (!eeprom_erase_sector(0)) return 0;
    for(i=0;i<BALANCE_PARAMETER_COUNT;++i) {
        if (!eeprom_page_program(0,i+1,&words[i])) return 0;
        crc=checksum_word(crc,words[i]);
    }
    if (!eeprom_page_program(0,13,&ramp)) return 0;
    crc=checksum_word(crc,ramp);
    if (!eeprom_page_program(0,14,&crc)) return 0;
    if (!eeprom_page_program(0,0,&magic)) return 0;
    IfxFlash_resetToRead(0);
    if(my_flash_read(0,0,uint32)!=magic || my_flash_read(0,14,uint32)!=crc) return 0;
    for(i=0;i<BALANCE_PARAMETER_COUNT;++i) if(my_flash_read(0,i+1,uint32)!=words[i])return 0;
    return my_flash_read(0,13,uint32)==ramp;
}
void get_flash(void)
{
    balance_control_defaults();
#if QCAR_BOOT_LOAD_FLASH
    float snapshot[BALANCE_PARAMETER_COUNT];
    uint32 words[BALANCE_PARAMETER_COUNT],crc=BALANCE_STORAGE_MAGIC,ramp;unsigned i;
    if (FLASH0_FSR.U & FLASH_BUSY_MASK) return;
    IfxFlash_resetToRead(0);
    if(my_flash_read(0,0,uint32)!=BALANCE_STORAGE_MAGIC) return;
    for(i=0;i<BALANCE_PARAMETER_COUNT;++i) {words[i]=my_flash_read(0,i+1,uint32);crc=checksum_word(crc,words[i]);}
    ramp=my_flash_read(0,13,uint32);crc=checksum_word(crc,ramp);
    if(crc!=my_flash_read(0,14,uint32))return;
    memcpy(snapshot,words,sizeof(snapshot));
    if(!stored_valid(snapshot,ramp))return;
    for(i=0;i<BALANCE_PARAMETER_COUNT;++i)*values[i]=snapshot[i];
    control_startup_ramp_ms=ramp;
#endif
}
