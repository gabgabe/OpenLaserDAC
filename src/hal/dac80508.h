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
 * @file dac80508.h
 * @brief DAC80508 8-channel 16-bit DAC driver
 * 
 * Hardware channel mapping (ESP32-S3 → DAC80508 → ILDA):
 *   DAC OUT6 (pin10) = X axis
 *   DAC OUT7 (pin11) = Y axis
 *   DAC OUT3 (pin5)  = Blue
 *   DAC OUT4 (pin8)  = Green
 *   DAC OUT5 (pin9)  = Red
 * 
 * SPI wiring:
 *   GPIO11 (SDI)  → DAC pin14 (SDI)
 *   GPIO10 (CS)   → DAC pin12 (SYNC)
 *   GPIO12 (SCK)  → DAC pin13 (SCLK)
 * 
 * DAC80508 SPI protocol: 24-bit frames, MSB first
 *   [7-bit addr | R/W] [DATA_HI] [DATA_LO]
 *   CPOL=0 CPHA=1 → SPI mode 1 per datasheet (SCLK idle low, data latched on falling edge)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// DAC80508 channel mapping: laser signal → physical DAC output channel
// ============================================================================
#define DAC_CH_X        6   // X axis → DAC OUT6
#define DAC_CH_Y        7   // Y axis → DAC OUT7
#define DAC_CH_RED      5   // Red    → DAC OUT5
#define DAC_CH_GREEN    4   // Green  → DAC OUT4
#define DAC_CH_BLUE     3   // Blue   → DAC OUT3

/**
 * @brief Initialize DAC80508 
 * @return ESP_OK on success
 */
esp_err_t dac_init(void);

/** @brief Get total dac_output_point calls */
uint32_t dac_get_output_count(void);
/** @brief Check if direct SPI register access is ready */
bool dac_is_direct_spi_ready(void);
/** @brief Check if SPI device handle is valid */
bool dac_is_spi_initialized(void);

/**
 * @brief Write a single DAC register (channel or config)
 * @param reg Register address (0x00-0x0F)
 * @param value 16-bit value
 * @return ESP_OK on success
 */
esp_err_t dac_write_register(uint8_t reg, uint16_t value);

/**
 * @brief Output a laser point to the correct DAC channels
 * Maps X→OUT6, Y→OUT7, R→OUT5, G→OUT4, B→OUT3
 * @param point Pointer to laser point
 */
void dac_output_point(const laser_point_t* point);

#ifdef __cplusplus
}
#endif
