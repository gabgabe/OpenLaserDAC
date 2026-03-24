/*
 * OpenLaserDAC - Open Source Laser DAC Firmware
 * Copyright (C) 2026 gabgabe
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PIN_DAC_CS          10
#define PIN_DAC_MOSI        11
#define PIN_DAC_SCK         12
#define PIN_DAC_MISO        9

#define PIN_ETH_MOSI        13
#define PIN_ETH_SCK         14
#define PIN_ETH_MISO        15
#define PIN_ETH_CS          16
#define PIN_ETH_INT         8

#define PIN_STATUS_LED      7

#define ETH_SPI_HOST        SPI2_HOST
// W5500 SPI: 50 MHz for better throughput (chip supports up to 80 MHz)
// Gain: +25% transfer speed vs 40 MHz
#define ETH_SPI_FREQ_HZ     (50 * 1000 * 1000)
#define ETH_MAC_ADDR        {0x02, 0x00, 0x00, 0x49, 0x4C, 0x44}

#define WIFI_AP_SSID        "OpenLaserDAC"
#define WIFI_AP_PASS        "openlaser"
#define WIFI_AP_CHANNEL     6
#define WIFI_AP_MAX_CONN    4
#define ETH_DHCP_TIMEOUT_MS 10000

#define ETHERDREAM_TCP_PORT 7765
#define ETHERDREAM_UDP_PORT 7654
#define HTTP_PORT           80

#define SCAN_RATE_MIN_HZ    1000
#define SCAN_RATE_DEFAULT_HZ 10000
#define SCAN_RATE_MAX_HZ    100000

#define FRAME_BUFFER_SIZE   8192

#define DAC_SPI_HOST        SPI3_HOST

#define CORE_SERVICES       0
#define CORE_REALTIME       1
#define TASK_PRIORITY_EDREAM 19

// =============================================================================
// Laser Point (Ether Dream format)
// =============================================================================

typedef struct {
    int16_t x;
    int16_t y;
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint8_t user1;
    uint8_t user2;
    uint8_t flags;
} __attribute__((packed)) laser_point_t;

#define POINT_FLAG_BLANK    0x01

typedef struct {
    uint32_t scan_rate_hz;
} system_config_t;

typedef struct {
    bool running;
    bool ed_connected;
    uint32_t buffer_level;
    uint32_t current_scan_rate;
    uint32_t ed_point_rate;
    uint32_t free_heap;
} system_status_t;

extern system_config_t g_config;
extern system_status_t g_status;

#ifdef __cplusplus
}
#endif
