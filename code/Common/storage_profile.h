#ifndef STORAGE_PROFILE_H_
#define STORAGE_PROFILE_H_
/* Parameter manual save enabled; startup read remains off pending board test. */
#ifndef STORAGE_FLASH_VALIDATED
#define STORAGE_FLASH_VALIDATED 1
#endif
#ifndef STORAGE_BOOT_LOAD
#define STORAGE_BOOT_LOAD 0
#endif
/* Route persistence needs a separate RAM/storage validation. RAM routes remain usable. */
#ifndef STORAGE_PATH_FLASH_ENABLED
#define STORAGE_PATH_FLASH_ENABLED 0
#endif
#endif
