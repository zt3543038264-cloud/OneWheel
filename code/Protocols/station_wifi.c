/* TC264 station WiFi transport, 2026-09-21.
 * SPI packet sequence follows SeekFree's zf_device_wifi_spi (GPL-3.0-or-later,
 * Copyright (c) 2022 SEEKFREE). That driver's original notices remain intact.
 * Adaptation: foreground one-transaction steps instead of blocking waits/drains.
 * Architecture reference: HDUASC WiFi command layer, ZhangStudyLife,
 * https://github.com/ZhangStudyLife/HDUASC-SmartCar-21st-FlyOverMinefield
 * The vehicle's existing cfg/telemetry protocol is used without importing its
 * vehicle-specific parameter, IMU calibration or JustFloat implementations.
 */
#include "Protocols/station_wifi.h"
#if STATION_WIFI_ENABLED
#include "zf_common_headfile.h"
#include "zf_device_wifi_spi.h"
#include <string.h>

#define STATION_WIFI_INIT_ATTEMPTS 3u
#define STATION_WIFI_STARTUP_DELAY_MS 300u
#define STATION_WIFI_RETRY_DELAY_MS 500u
/* Match the pre-ASC driver wait budget without blocking the foreground loop. */
#define STATION_WIFI_TRANSACTION_TIMEOUT_MS 1000u

/* Exclusive owner of SPI3 after boot; never call the blocking library send/read
 * routines from elsewhere while this transport is enabled. */
static uint8 s_ready;
static uint8 s_phase; /* 0 data/read, 1 UDP flush, 2 flush acknowledgement */
static station_wifi_diag_t s_diag;
static uint32 s_wait_since_ms, s_last_poll_ms;
static uint8 s_waiting, s_poll_seen;
typedef char station_wifi_header_size[(sizeof(wifi_spi_head_struct) == 4u) ? 1 : -1];

void station_wifi_get_diag(station_wifi_diag_t *out)
{
    *out = s_diag;
    out->phase = s_phase;
    out->ready = s_ready;
}

uint8 station_wifi_init(void)
{
    s_ready = 0; s_phase = 0;
    memset(&s_diag, 0, sizeof(s_diag));
    s_waiting = 0; s_poll_seen = 0;
    if (STATION_WIFI_SSID[0] == '\0') return 0;
    system_delay_ms(STATION_WIFI_STARTUP_DELAY_MS);
    for (uint8 attempt = 0; attempt < STATION_WIFI_INIT_ATTEMPTS; ++attempt)
    {
        if (attempt) system_delay_ms(STATION_WIFI_RETRY_DELAY_MS);
        if (!wifi_spi_init((char *)STATION_WIFI_SSID, (char *)STATION_WIFI_PASSWORD) &&
            !wifi_spi_socket_connect("UDP", (char *)STATION_WIFI_PC_IP,
                                     (char *)STATION_WIFI_PC_PORT, (char *)STATION_WIFI_LOCAL_PORT))
        {
            s_ready = 1;
            return 1;
        }
        s_diag.errors++;
    }
    return 0;
}

int station_wifi_step(const uint8 *tx, uint16 length, uint8 *rx, uint16 *received, uint32 now_ms)
{
    wifi_spi_head_struct head = {0};
    uint8 data[STATION_WIFI_RX_MAX] = {0};
    uint8 phase = s_phase;
    uint16 accepted = 0;
    uint8 module_ready;
    uint32 elapsed;
    *received = 0;
    if (!s_ready) return 0;
    if (length > STATION_WIFI_FRAME_MAX || (length && !tx)) return -1;
    elapsed = now_ms - s_last_poll_ms;
    if (s_poll_seen && elapsed > s_diag.poll_gap_max_ms)
        s_diag.poll_gap_max_ms = elapsed;
    s_last_poll_ms = now_ms;
    s_poll_seen = 1;

    module_ready = gpio_get_level(WIFI_SPI_INT_PIN);
    /* Ignore idle DATA polls with no outgoing frame. */
    if (!module_ready && !s_waiting && (phase != 0u || length != 0u))
    {
        s_waiting = 1;
        s_wait_since_ms = now_ms;
        s_diag.wait_count[phase]++;
    }
    if (s_waiting)
    {
        elapsed = now_ms - s_wait_since_ms;
        if (elapsed > s_diag.wait_max_ms[phase])
            s_diag.wait_max_ms[phase] = elapsed;
        if (!module_ready && elapsed >= STATION_WIFI_TRANSACTION_TIMEOUT_MS)
        {
            /* Like the old driver releasing its SPI mutex on timeout, allow
             * the next DATA transaction once the module becomes ready. */
            s_diag.errors++;
            s_phase = 0;
            s_waiting = 0;
            return 0;
        }
    }
    if (!module_ready) return 0;
    s_waiting = 0;
    head.command = (phase == 1u) ? WIFI_SPI_UDP_SEND : WIFI_SPI_DATA;
    if (phase == 0u) { head.length = length; accepted = length; }
    if (accepted) memcpy(data, tx, accepted < STATION_WIFI_RX_MAX ? accepted : STATION_WIFI_RX_MAX);
    gpio_low(WIFI_SPI_CS_PIN);
    spi_transfer_8bit(WIFI_SPI_INDEX, (const uint8 *)&head, (uint8 *)&head, sizeof(head));
    /* The flush acknowledgement clocks only the header, as in the vendor driver. */
    if (phase != 2u)
    {
        spi_transfer_8bit(WIFI_SPI_INDEX, data, data, STATION_WIFI_RX_MAX);
        if (accepted > STATION_WIFI_RX_MAX)
            spi_write_8bit_array(WIFI_SPI_INDEX, tx + STATION_WIFI_RX_MAX, accepted - STATION_WIFI_RX_MAX);
    }
    gpio_high(WIFI_SPI_CS_PIN);
    if (phase == 2u)
    {
        s_phase = 0;
        if (head.command != WIFI_SPI_REPLY_OK) { s_diag.errors++; return 0; }
        s_diag.acknowledged++;
        return 0;
    }
    s_phase = phase == 1u ? 2u : (accepted ? 1u : 0u);
    if (head.command == WIFI_SPI_REPLY_DATA_START || head.command == WIFI_SPI_REPLY_DATA_END)
    {
        if (head.length > STATION_WIFI_RX_MAX)
        {
            s_diag.errors++;
            /* Report receive loss without disabling the transport. If DATA
             * was queued, finish flushing it before accepting another frame. */
            return accepted ? (int)accepted : -1;
        }
        memcpy(rx, data, head.length);
        *received = head.length;
    }
    if (accepted) s_diag.submitted++;
    return (int)accepted;
}
#else
uint8 station_wifi_init(void) { return 0; }
int station_wifi_step(const uint8 *tx, uint16 length, uint8 *rx, uint16 *received, uint32 now_ms)
{
    (void)tx; (void)length; (void)rx; (void)now_ms; *received = 0; return 0;
}
#endif
