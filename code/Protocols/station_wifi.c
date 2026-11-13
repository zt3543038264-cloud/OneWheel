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

/* Exclusive owner of SPI3 after boot; never call the blocking library send/read
 * routines from elsewhere while this transport is enabled. */
static uint8 s_ready;
static uint8 s_phase; /* 0 data/read, 1 UDP flush, 2 flush acknowledgement */
typedef char station_wifi_header_size[(sizeof(wifi_spi_head_struct) == 4u) ? 1 : -1];

uint8 station_wifi_init(void)
{
    s_ready = 0; s_phase = 0;
    if (STATION_WIFI_SSID[0] == '\0') return 0;
    if (wifi_spi_init((char *)STATION_WIFI_SSID, (char *)STATION_WIFI_PASSWORD)) return 0;
    if (wifi_spi_socket_connect("UDP", (char *)STATION_WIFI_PC_IP,
                               (char *)STATION_WIFI_PC_PORT, (char *)STATION_WIFI_LOCAL_PORT)) return 0;
    s_ready = 1;
    return 1;
}

int station_wifi_step(const uint8 *tx, uint16 length, uint8 *rx, uint16 *received)
{
    wifi_spi_head_struct head = {0};
    uint8 data[STATION_WIFI_RX_MAX] = {0};
    uint8 phase = s_phase;
    uint16 accepted = 0;
    *received = 0;
    if (!s_ready || !gpio_get_level(WIFI_SPI_INT_PIN)) return 0;
    if (length > STATION_WIFI_FRAME_MAX || (length && !tx)) return -1;
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
        if (head.command != WIFI_SPI_REPLY_OK) { s_ready = 0; return -1; }
        return 0;
    }
    s_phase = phase == 1u ? 2u : (accepted ? 1u : 0u);
    if (head.command == WIFI_SPI_REPLY_DATA_START || head.command == WIFI_SPI_REPLY_DATA_END)
    {
        if (head.length > STATION_WIFI_RX_MAX) { s_ready = 0; return -1; }
        memcpy(rx, data, head.length);
        *received = head.length;
    }
    return (int)accepted;
}
#else
uint8 station_wifi_init(void) { return 0; }
int station_wifi_step(const uint8 *tx, uint16 length, uint8 *rx, uint16 *received)
{
    (void)tx; (void)length; (void)rx; *received = 0; return 0;
}
#endif
