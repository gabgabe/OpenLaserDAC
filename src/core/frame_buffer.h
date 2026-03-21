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
 * @file frame_buffer.h
 * @brief Lock-free SPSC ring buffer for laser points
 * 
 * Uses atomic operations for lock-free thread-safe access (single producer, single consumer).
 * Supports batch read (up to 512 points) for efficient DAC output.
 */

#pragma once

#include "config.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t frame_buffer_init(void);

/**
 * @brief Write batch of points to buffer
 * @return Number of points actually written
 */
size_t frame_buffer_write(const laser_point_t* points, size_t count);

/**
 * @brief Read batch of points from buffer
 * @param points Output array
 * @param max Maximum number of points to read
 * @return Number of points actually read (0 if empty)
 */
size_t frame_buffer_read(laser_point_t* points, size_t max);

/**
 * @brief Check if count points can fit
 */
bool frame_buffer_can_fit(size_t count);

size_t frame_buffer_level(void);
void frame_buffer_clear(void);

#ifdef __cplusplus
}
#endif
