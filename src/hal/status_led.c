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

#include "status_led.h"
#include "config.h"
#include "esp_log.h"
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

static const char* TAG = "LED";
static led_strip_handle_t s_strip = NULL;

static TimerHandle_t s_blink_timer = NULL;
static uint8_t s_blink_r, s_blink_g, s_blink_b;
static bool s_blink_on = false;

static void blink_timer_cb(TimerHandle_t timer) {
    if (!s_strip) return;
    s_blink_on = !s_blink_on;
    if (s_blink_on) {
        led_strip_set_pixel(s_strip, 0, s_blink_r, s_blink_g, s_blink_b);
    } else {
        led_strip_set_pixel(s_strip, 0, 0, 0, 0);
    }
    led_strip_refresh(s_strip);
}

esp_err_t status_led_init(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = PIN_STATUS_LED,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,  // 10 MHz
    };

    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED strip init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    led_strip_clear(s_strip);
    return ESP_OK;
}

void status_led_set(uint8_t r, uint8_t g, uint8_t b) {
    if (s_blink_timer && xTimerIsTimerActive(s_blink_timer)) {
        xTimerStop(s_blink_timer, 0);
    }
    if (!s_strip) return;
    led_strip_set_pixel(s_strip, 0, r, g, b);
    led_strip_refresh(s_strip);
}

void status_led_blink(uint8_t r, uint8_t g, uint8_t b, uint32_t period_ms) {
    s_blink_r = r;
    s_blink_g = g;
    s_blink_b = b;
    s_blink_on = true;

    TickType_t half = pdMS_TO_TICKS(period_ms / 2);
    if (half == 0) half = 1;

    if (!s_blink_timer) {
        s_blink_timer = xTimerCreate("led_blink", half, pdTRUE, NULL, blink_timer_cb);
    } else {
        xTimerChangePeriod(s_blink_timer, half, 0);
    }
    xTimerStart(s_blink_timer, 0);

    // Show immediately
    if (s_strip) {
        led_strip_set_pixel(s_strip, 0, r, g, b);
        led_strip_refresh(s_strip);
    }
}

void status_led_blink_stop(void) {
    if (s_blink_timer && xTimerIsTimerActive(s_blink_timer)) {
        xTimerStop(s_blink_timer, 0);
    }
}
