# Staged TC264 storage safety

The adapted build defaults to **no automatic DFlash reads and no writes**.
`code/Common/storage_profile.h` defines `STORAGE_FLASH_VALIDATED=0` and
`STORAGE_BOOT_LOAD=0`. Initializers load RAM defaults only. Diagnostic/lab
builds must explicitly opt in; these are not proof that hardware is validated.
Do not enable startup load merely to diagnose saves.

## Address and format preservation

The checked-in TC26B iLLD table maps 12 logical erase sectors at AF000000,
8 KiB apart. The Seekfree API calls these sectors "pages"; its `sector_num`
is only a compatibility argument and must be zero. Parameter storage remains
logical sector 11 (AF016000), path slots remain 9/10 (AF012000/AF014000).
A physical programming page is 8 bytes: one stored uint32 in its low half,
zero in its high half. No dense packing or sector-zero port from `new` is used.
The same table is used for read, erase and program. Blank checks inspect both
halves, and programming verifies both. No trap/ECC/security settings changed.

Before an occupied sector may be erased, the parameter/path layer reads the
entire 8 KiB sector: all 1,024 low words and all 1,024 high words. A recognized
record owns only its header, values/points and CRC. Every low word after that
record must be zero, including the unused capacity after a short path; every
high word must also be zero. Any foreign tail or packed high-half content
blocks the destructive operation. A valid ORA1/ORP1 prefix alone is insufficient.
Full-sector readback after a save verifies the record and the zero tail.
Path storage uses two 4 KiB RAM buffers for these complete low-word images
(2,008 bytes more static RAM than the previous two 773-word buffers).

The orthogonal chassis / IMU660RA profile uses new magic `0x4F524131` (ORA1).
Previous master magic `0x51434146` and older `0x51434145` are both rejected,
including after explicit lab opt-in: old physical zeros, signs and gains must
not load into this chassis profile. No automatic migration is provided.
Legacy magic, unknown keys, duplicate
keys, invalid counts, invalid values and CRC failures block load/save/erase.
There is no automatic sign/frame migration or overwrite of incompatible data.
Path records use ORP1 magic (0x4F525031), so old master PATH records are not
loaded into the preserved new-EKF coordinate frame or overwritten.
Group saves preserve all recognized values outside that group. Unknown keys
block the entire save, rather than being silently dropped. Path saves preserve
the latest valid slot and refuse to erase an occupied invalid destination.
A torn path slot can therefore require deliberate out-of-band backup/recovery.

## Driver behavior

Driver status is available through `flash_last_status()`. Parameter and path
status accessors distinguish disabled/incompatible/I/O states for callers.
The old void driver functions remain for compatibility; callers must check
status immediately. The application storage paths do so. The previous global
erase flag was removed. Save return values now reflect the driver result.
`flash_write_page()` now refuses any occupied destination with VERIFY status;
it never silently erases. Parameter and path saves explicitly erase only after
full-sector ownership checks. The raw erase API cannot establish record
ownership itself and must not be used to reclaim unknown contents. There is no
in-firmware erase/migration/recovery bypass for rejected records.

STM deadlines and a finite polling budget bound every wait (including page-mode
entry); a timeout/hardware fault latches further driver access until reboot.
No reset/read/second command is sent after a timeout or latched hardware error.
The TC26B FSR error mask includes OPER/SQER/PROER (11..13), all DFlash ECC
indications DFSBER/DFDBER/DFTBER/DFMBER (18..21), SRI address ECC (22), and
program/erase verification errors (25..26). This deliberately refuses corrected
DFlash ECC as well as uncorrectable errors; degraded or uncertain data cannot
authorize erasure. The status is checked before reads and after every physical
32-bit read, after a data synchronization barrier. A failing word is not
published, occupied/blank early returns check the read error, and erase/program
readback preserves HARDWARE status rather than hiding it as VERIFY. Completion
waits also check status after their synchronization barrier. Flags are not
silently cleared to authorize another transaction.

IRQ masking covers command and safety-ENDINIT sequences only, not the long erase/program waits. This assumes
CPU0 foreground ownership, no concurrent Flash clients and a stopped controller.
Parameter save/erase callers must enforce stopped state; path uses its existing
storage-ready guard. A single parameter sector is **not power-loss atomic**.

## Host verification only

- `python tools/tests/test_storage_safety.py`: compiles actual parameter code
  with a RAM backend; disabled no-I/O defaults, save/load, legacy/unknown/corrupt
  preservation, write/readback failure and injected I/O errors.
- `python tools/tests/test_flash_wait.py`: compiles driver implementation with
  stub registers; busy/page-mode deadlines, stalled timer fallback, timeout
  latch, hardware error, argument bounds and default disabled behavior.
- `python tools/tests/test_flash_preservation.py`: actual parameter/path sources
  linked with the actual driver against a 12-sector, 8 KiB-per-sector RAM model.
  Exhaustively injects a nonzero word at every unused low slot and every high
  half, for parameters and both path destination slots; verifies no erase/write
  and byte-for-byte preservation. Also covers pre/post-read ECC at boundary
  positions, post-erase/program verification errors, reboot-only fault latch,
  occupied-write refusal, valid ORA1/ORP1/tombstones and default-disabled zero I/O.
  This harness needs GCC non-PIE support for 32-bit simulated Flash addresses.
- Three station Flash tests now assert the current no-migration contract:
  legacy snapshots remain unchanged, old/unknown keys and out-of-range records
  are refused as a whole, current-format values and partial group saves round
  trip, and fixed IMU frame writes cannot flip mechanical zeros.
- `python tools/tests/test_path_nav.py`: actual portable path codec/storage,
  redundant-slot loading, interrupted writes/readback, unknown-slot preservation,
  stopped-state gate and tombstone clear, plus existing navigation tests.

These tests neither access nor program the vehicle. Hardware toolchain/link,
DFlash timing/errata, ECC behavior on existing and erased records, real interrupt
latency, watchdogs and power-interruption behavior still need controlled bench
validation. A physical bus/ECC trap can occur before the next C status check;
this code does not catch/recover such traps or prove the ordering/coverage of
FSR flags on silicon. No host test authorizes enabling Flash. Review TC26B
silicon revision/errata, erased-page ECC reads, trap handling, safety-ENDINIT
windows and sole-client ownership first. Keep both storage gates at 0 in this
delivery; back up any unknown sectors independently before deciding recovery.
