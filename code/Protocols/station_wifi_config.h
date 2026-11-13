#ifndef STATION_WIFI_CONFIG_H
#define STATION_WIFI_CONFIG_H

/* 0 keeps the existing UART2 station. Set 1 after configuring this vehicle's LAN.
 * WiFi SPI pin definitions remain in zf_device_wifi_spi.h (SPI3, default pins).
 * Network credentials are local settings; do not commit populated passwords.
 */
#ifndef STATION_WIFI_ENABLED
#define STATION_WIFI_ENABLED 0
#endif
#if STATION_WIFI_ENABLED
#include "Protocols/station_wifi_private.h"
#endif

#ifndef STATION_WIFI_SSID
#define STATION_WIFI_SSID ""
#endif
#ifndef STATION_WIFI_PASSWORD
#define STATION_WIFI_PASSWORD ""
#endif
#ifndef STATION_WIFI_PC_IP
#define STATION_WIFI_PC_IP "192.168.0.107"
#endif
#ifndef STATION_WIFI_PC_PORT
#define STATION_WIFI_PC_PORT "8081"
#endif
#ifndef STATION_WIFI_LOCAL_PORT
#define STATION_WIFI_LOCAL_PORT "5001"
#endif

#endif
