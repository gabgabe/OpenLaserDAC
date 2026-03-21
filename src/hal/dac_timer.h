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
 * @file dac_timer.h
 * @brief Hardware-timed DAC output using GPTimer
 * 
 * This module provides precise, hardware-timed laser point output.
 * It uses a GPTimer ISR that fires at the scan rate frequency.
 * A high-priority refill task on Core 1 keeps the ISR buffer fed
 * from the frame buffer.
 * 
 * Replaces the busy-wait approach in dac_engine for better timing precision
 * and reduced CPU usage.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get diagnostic counters for pipeline debugging
 */
typedef struct {
    uint32_t points_output;    // Points output by ISR since last reset
    uint32_t underruns;        // Total ISR underruns
    size_t   isr_buf_level;    // Current ISR buffer fill level
    bool     playback_active;  // Playback gate status
    bool     running;          // Timer running
} dac_timer_diag_t;

void dac_timer_get_diag(dac_timer_diag_t *diag);

/**
 * @brief Initialize hardware-timed DAC output system
 * 
 * Initializes:
 *   - Frame buffer
 *   - Base DAC (via dac_init())
 *   - GPTimer for scan rate timing
 *   - SPI device for queued transactions
 *   - Output task on Core 1
 * 
 * @return ESP_OK on success
 */
esp_err_t dac_timer_init(void);

/**
 * @brief Start timed output at current scan rate
 * 
 * Starts the GPTimer which triggers point output at scan_rate frequency.
 * 
 * @return ESP_OK on success
 */
esp_err_t dac_timer_start(void);

/**
 * @brief Stop timed output
 * 
 * Stops the GPTimer. DAC outputs rest point when stopped.
 * 
 * @return ESP_OK on success
 */
esp_err_t dac_timer_stop(void);

/**
 * @brief Set scan rate (points per second)
 * 
 * Updates the GPTimer period. Can be called while running.
 * 
 * @param rate_hz Scan rate in Hz (clamped to SCAN_RATE_MIN_HZ..SCAN_RATE_MAX_HZ)
 * @return ESP_OK on success
 */
esp_err_t dac_timer_set_scan_rate(uint32_t rate_hz);

/**
 * @brief Get current scan rate
 * @return Current scan rate in Hz
 */
uint32_t dac_timer_get_scan_rate(void);

/**
 * @brief Check if timed output is running
 * @return true if running, false if stopped
 */
bool dac_timer_is_running(void);

/**
 * @brief Set playback active gate
 * 
 * When false, the refill task will NOT read from frame_buffer,
 * allowing the buffer to accumulate points during PREPARED state.
 * When true, points flow from frame_buffer → ISR buffer → DAC.
 * 
 * @param active true to enable point flow, false to gate it
 */
void dac_timer_set_playback_active(bool active);

/**
 * @brief Check if playback is active
 * @return true if point flow is enabled
 */
bool dac_timer_is_playback_active(void);

#ifdef __cplusplus
}
#endif
