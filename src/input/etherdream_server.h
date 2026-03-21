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

/**
 * @file etherdream_server.h
 * @brief Ether Dream protocol server
 * 
 * Implements the Ether Dream protocol:
 * - TCP command/data streaming on port 7765
 * - UDP broadcast/discovery on port 7654
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Ether Dream server
 * @return ESP_OK on success
 */
esp_err_t etherdream_server_init(void);

/**
 * @brief Start Ether Dream server (TCP + UDP broadcast)
 * @return ESP_OK on success
 */
esp_err_t etherdream_server_start(void);

/**
 * @brief Stop Ether Dream server
 * @return ESP_OK on success
 */
esp_err_t etherdream_server_stop(void);

/**
 * @brief Ether Dream server main loop (call from task)
 * Handles TCP accept/recv and periodic UDP broadcast
 */
void etherdream_server_loop(void);

/**
 * @brief Check if an Ether Dream client is connected and streaming
 * @return true if client connected
 */
bool etherdream_server_is_connected(void);

/**
 * @brief Get current point rate (from begin/queue commands)
 * @return Point rate in Hz, or 0 if not playing
 */
uint32_t etherdream_server_get_point_rate(void);

/**
 * @brief Get measured incoming point rate (updated every 500ms)
 * @return Actual measured points per second
 */
uint32_t etherdream_server_get_measured_pps(void);

#ifdef __cplusplus
}
#endif
