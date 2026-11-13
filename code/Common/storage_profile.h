#ifndef STORAGE_PROFILE_H_
#define STORAGE_PROFILE_H_
/* Hardware has NOT been validated on the new vehicle. Deliberate lab opt-in,
 * not menu switches. Keep startup read disabled even during save diagnosis. */
#ifndef STORAGE_FLASH_VALIDATED
#define STORAGE_FLASH_VALIDATED 0
#endif
#ifndef STORAGE_BOOT_LOAD
#define STORAGE_BOOT_LOAD 0
#endif
#endif
