#ifndef PATH_STORE_STUBS_H
#define PATH_STORE_STUBS_H
#include <stdint.h>
typedef uint32_t uint32;
typedef uint16_t uint16;
#define FLASH_STATUS_OK 0
int flash_last_status(void);
#define EEPROM_PAGE_LENGTH 1024u
int control_path_storage_ready(void);
int flash_check(unsigned sector, unsigned page);
void flash_read_page(unsigned sector, unsigned page, uint32_t *out, uint16_t count);
void flash_erase_page(unsigned sector, unsigned page);
void flash_write_page(unsigned sector, unsigned page, const uint32_t *in, uint16_t count);
#endif
