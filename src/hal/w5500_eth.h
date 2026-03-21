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
#include "esp_err.h"
#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

// Callback when Ethernet gets IP address
typedef void (*eth_got_ip_cb_t)(void);

/**
 * @brief Set callback for when Ethernet gets IP
 */
void w5500_eth_set_got_ip_callback(eth_got_ip_cb_t cb);

/**
 * @brief Initialize W5500 Ethernet
 * @return ESP_OK on success
 */
esp_err_t w5500_eth_init(void);

/**
 * @brief Start Ethernet
 * @return ESP_OK on success
 */
esp_err_t w5500_eth_start(void);

/**
 * @brief Stop Ethernet
 * @return ESP_OK on success
 */
esp_err_t w5500_eth_stop(void);

/**
 * @brief Check if link is up
 */
bool w5500_eth_is_link_up(void);

/**
 * @brief Check if we have an IP address
 */
bool w5500_eth_has_ip(void);

/**
 * @brief Get netif handle
 */
esp_netif_t* w5500_eth_get_netif(void);

/**
 * @brief Get link speed in Mbps
 */
uint32_t w5500_eth_get_speed(void);

#ifdef __cplusplus
}
#endif
