#include "Controller/Planner/path_store.h"
#include <string.h>
#include "Common/storage_profile.h"

#if !defined(PATH_HOST_TEST) || defined(PATH_STORE_TEST)
#ifdef PATH_STORE_TEST
#include "path_store_stubs.h"
#else
#include "Controller/control.h"
#include "zf_driver_flash.h"
#endif
#pragma section all "cpu0_dsram"
typedef char path_page_fits[(PATH_RECORD_WORDS <= EEPROM_PAGE_LENGTH) ? 1 : -1];
#endif

typedef char path_point_wire_size[(sizeof(path_point_t) == 12u) ? 1 : -1];

#define PATH_MAGIC 0x4F525031u /* ORP1: preserved new-EKF frame / orthogonal chassis */
#define PATH_FORMAT 1u

static uint32_t crc_words(const uint32_t *w, unsigned count)
{
    uint32_t crc = 0xffffffffu;
    unsigned i, b;
    for (i = 0; i < count; i++)
    {
        uint32_t v = w[i];
        for (b = 0; b < 32u; b++)
        {
            uint32_t bit = (crc ^ v) & 1u;
            crc >>= 1;
            if (bit) crc ^= 0xedb88320u;
            v >>= 1;
        }
    }
    return ~crc;
}

unsigned path_record_encode(uint32_t *w, const path_point_t *p, uint16_t count, uint32_t gen)
{
    unsigned n = 4u + (unsigned)count * 3u;
    if (!w || (count != 0u && !path_nav_validate(p, count))) return 0;
    w[0] = PATH_MAGIC; w[1] = PATH_FORMAT; w[2] = gen; w[3] = count;
    if (count) memcpy(w+4, p, (size_t)count * sizeof(p[0]));
    w[n] = crc_words(w, n);
    return n+1u;
}

int path_record_decode(const uint32_t *w, path_point_t *p, uint16_t *count, uint32_t *gen)
{
    unsigned n;
    uint16_t c;
    if (!w || !p || !count || !gen || w[0] != PATH_MAGIC || w[1] != PATH_FORMAT ||
        w[3] > PATH_MAX_POINTS || w[3] == 1u) return 0;
    c = (uint16_t)w[3];
    n = 4u + (unsigned)c * 3u;
    if (w[n] != crc_words(w, n)) return 0;
    if (c) memcpy(p, w+4, (size_t)c * sizeof(p[0]));
    if (c && !path_nav_validate(p, c)) return 0;
    *count = c; *gen = w[2];
    return 1;
}

#if !defined(PATH_HOST_TEST) || defined(PATH_STORE_TEST)
static path_storage_status_t s_status;
path_storage_status_t path_store_status(void) { return s_status; }

#if STORAGE_FLASH_VALIDATED
/* Keep Flash-only work buffers out of locked builds, including -O0 builds.
 * Do not reduce the hardware page size, stack/CSA reservations or record checks. */
static uint32_t s_record[EEPROM_PAGE_LENGTH];
static uint32_t s_verify[EEPROM_PAGE_LENGTH];
static path_point_t s_candidate[PATH_MAX_POINTS];

static int read_slot(unsigned page, uint16_t *count, uint32_t *gen)
{
    unsigned i, words;
    int occupied = flash_check(0, page);
    if (flash_last_status() != FLASH_STATUS_OK) { s_status = PATH_STORAGE_IO; return 0; }
    if (!occupied) return 0;
    /* Read every low word and validate every high half before trusting the
     * slot. A valid prefix does not grant ownership of unknown trailing data. */
    flash_read_page(0, page, (uint32 *)s_record, (uint16)EEPROM_PAGE_LENGTH);
    if (flash_last_status() != FLASH_STATUS_OK) { s_status = PATH_STORAGE_IO; return 0; }
    if (!path_record_decode(s_record, s_candidate, count, gen)) return 0;
    words = 5u + (unsigned)*count * 3u;
    for (i = words; i < EEPROM_PAGE_LENGTH; i++)
        if (s_record[i] != 0u) return 0;
    return 1;
}

static unsigned latest_slot(uint32_t *generation)
{
    uint16_t count;
    uint32_t a = 0, b = 0;
    int va = read_slot(PATH_FLASH_PAGE_A, &count, &a);
    int vb = read_slot(PATH_FLASH_PAGE_B, &count, &b);
    if (vb && (!va || (int32_t)(b-a) > 0)) { *generation = b; return PATH_FLASH_PAGE_B; }
    if (va) { *generation = a; return PATH_FLASH_PAGE_A; }
    *generation = 0;
    return 0;
}

static int load_latest(void)
{
    uint32_t gen;
    uint16_t count;
    unsigned page;
    s_status = PATH_STORAGE_OK;
    page = latest_slot(&gen);
    if (s_status == PATH_STORAGE_IO) return 0;
    if (!page || !read_slot(page, &count, &gen)) return 0;
    if (!count) { path_nav_clear(); path_nav_saved(); return 1; }
    return path_nav_import(s_candidate, count);
}

void path_store_init(void)
{
    path_nav_init();
    if (STORAGE_FLASH_VALIDATED && STORAGE_BOOT_LOAD) (void)load_latest();
    else s_status = PATH_STORAGE_DISABLED;
}

int path_store_load(void)
{
    if (!STORAGE_FLASH_VALIDATED) { s_status = PATH_STORAGE_DISABLED; return 0; }
    if (!control_path_storage_ready()) return 0;
    return load_latest();
}

int path_store_save(void)
{
    uint32_t gen;
    uint16_t i;
    path_status_t status;
    unsigned latest, target, words;
    if (!STORAGE_FLASH_VALIDATED) { s_status = PATH_STORAGE_DISABLED; return 0; }
    if (!control_path_storage_ready()) return 0;
    path_nav_status(&status);
    if (status.count && !status.valid) return 0;
    s_status = PATH_STORAGE_OK;
    latest = latest_slot(&gen);
    if (s_status == PATH_STORAGE_IO) return 0;
    target = latest == PATH_FLASH_PAGE_A ? PATH_FLASH_PAGE_B : PATH_FLASH_PAGE_A;
    /* Never destroy an occupied unknown/corrupt/other-version destination.
     * A torn write needs explicit out-of-band recovery, not silent erasure. */
    if (flash_check(0, target)) {
        uint16_t old_count;
        uint32_t old_gen;
        if (flash_last_status() != FLASH_STATUS_OK) { s_status = PATH_STORAGE_IO; return 0; }
        if (!read_slot(target, &old_count, &old_gen)) {
            if (s_status != PATH_STORAGE_IO) s_status = PATH_STORAGE_INCOMPATIBLE;
            return 0;
        }
    }
    if (flash_last_status() != FLASH_STATUS_OK) { s_status = PATH_STORAGE_IO; return 0; }
    for (i = 0; i < status.count; i++)
        if (!path_nav_point(i, &s_candidate[i])) return 0;
    memset(s_record, 0, sizeof(s_record));
    words = path_record_encode(s_record, s_candidate, status.count, gen+1u);
    if (!words) return 0;
    /* Explicit erase only after full-sector ownership validation. */
    flash_erase_page(0, target);
    if (flash_last_status() != FLASH_STATUS_OK) { s_status = PATH_STORAGE_IO; return 0; }
    flash_write_page(0, target, (const uint32 *)s_record, (uint16)words);
    if (flash_last_status() != FLASH_STATUS_OK) { s_status = PATH_STORAGE_IO; return 0; }
    flash_read_page(0, target, (uint32 *)s_verify, (uint16)EEPROM_PAGE_LENGTH);
    if (flash_last_status() != FLASH_STATUS_OK || memcmp(s_record, s_verify, sizeof(s_record)) != 0) { s_status = PATH_STORAGE_IO; return 0; }
    s_status = PATH_STORAGE_OK;
    path_nav_saved();
    return 1;
}
#else
/* The locked build keeps the public API and RAM-path initialization, but has
 * no Flash buffers, Flash I/O or storage-readiness side effects. */
void path_store_init(void)
{
    path_nav_init();
    s_status = PATH_STORAGE_DISABLED;
}

int path_store_load(void)
{
    s_status = PATH_STORAGE_DISABLED;
    return 0;
}

int path_store_save(void)
{
    s_status = PATH_STORAGE_DISABLED;
    return 0;
}
#endif /* STORAGE_FLASH_VALIDATED */
#pragma section all restore
#endif
