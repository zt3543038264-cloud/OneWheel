#ifndef PATH_STORE_H_
#define PATH_STORE_H_
#include "Controller/Planner/path_nav.h"

#define PATH_RECORD_WORDS (5u + PATH_MAX_POINTS * 3u)
/* DFlash logical pages 9/10 are redundant path slots; param/IPM owns page 11. */
#define PATH_FLASH_PAGE_A 9u
#define PATH_FLASH_PAGE_B 10u

typedef enum { PATH_STORAGE_OK = 0, PATH_STORAGE_DISABLED, PATH_STORAGE_INCOMPATIBLE, PATH_STORAGE_IO } path_storage_status_t;
path_storage_status_t path_store_status(void);

/* Pure record codec, also used by host corruption/power-loss tests. */
unsigned path_record_encode(uint32_t *words, const path_point_t *points,
                            uint16_t count, uint32_t generation);
int path_record_decode(const uint32_t *words, path_point_t *points,
                       uint16_t *count, uint32_t *generation);
/* CPU0 foreground only. Init before starting the control timer; later calls
 * require control_path_storage_ready(). Saves never overwrite the latest slot. */
void path_store_init(void);
int path_store_load(void);
int path_store_save(void);
#endif
