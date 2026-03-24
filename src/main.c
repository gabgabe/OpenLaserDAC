
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

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "config.h"
#include "hal/dac_timer.h"
#include "hal/dac80508.h"
#include "hal/w5500_eth.h"
#include "hal/status_led.h"
#include "core/frame_buffer.h"
#include "input/etherdream_server.h"
#include "web/http_server.h"
#include "web/dns_server.h"

static const char* TAG = "MAIN";

system_config_t g_config = {
    .scan_rate_hz = SCAN_RATE_DEFAULT_HZ,
};

system_status_t g_status = {0};

// =============================================================================
// WiFi AP — always-on management / fallback interface
// =============================================================================

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_AP_STARTED_BIT BIT0

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_AP_START) {
            ESP_LOGI(TAG, "WiFi AP interface UP");
            xEventGroupSetBits(s_wifi_event_group, WIFI_AP_STARTED_BIT);
        } else if (event_id == WIFI_EVENT_AP_STACONNECTED) {
            wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)event_data;
            ESP_LOGI(TAG, "Client connected to AP (AID=%d)", event->aid);
        } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
            wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)event_data;
            ESP_LOGW(TAG, "Client disconnected from AP (AID=%d, reason=%d)", event->aid, event->reason);
        }
    }
}

static void wifi_ap_start(void) {
    s_wifi_event_group = xEventGroupCreate();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.ampdu_rx_enable = 1;
    cfg.ampdu_tx_enable = 1;
    cfg.nvs_enable = 0;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler, NULL, NULL));

    esp_netif_create_default_wifi_ap();

    wifi_config_t ap_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .password = WIFI_AP_PASS,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = WIFI_AP_MAX_CONN,
            .beacon_interval = 100,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Apply radio settings IMMEDIATELY after start, before the WiFi task
    // has a chance to run — matches the working committed boot order.
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
    esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
    esp_wifi_set_max_tx_power(80);

    // Now wait for the AP interface to come up (max 5 s)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_AP_STARTED_BIT,
                                           pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(5000));
    if (!(bits & WIFI_AP_STARTED_BIT)) {
        ESP_LOGW(TAG, "WiFi AP_START event timeout — continuing anyway");
    }

    ESP_LOGI(TAG, "WiFi AP: %s (http://192.168.4.1)", WIFI_AP_SSID);
    ESP_LOGI(TAG, "Heap after WiFi: free=%lu internal=%lu min=%lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_free_internal_heap_size(),
             (unsigned long)esp_get_minimum_free_heap_size());
}

// =============================================================================
// Park Ethernet SPI GPIOs when W5500 is absent
// Floating SPI2 lines (GPIO 13-16) + GPIO 8 (INT) near the WiFi antenna
// pick up RF noise and degrade WiFi auth reliability.
// =============================================================================
static void park_eth_gpios(void) {
    const int pins[] = { PIN_ETH_MOSI, PIN_ETH_SCK, PIN_ETH_MISO, PIN_ETH_CS };
    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    for (int i = 0; i < 4; i++) {
        io.pin_bit_mask = (1ULL << pins[i]);
        gpio_config(&io);
        gpio_set_level(pins[i], 0);
    }
    // INT pin: input with pull-up (idle-high for W5500 INT)
    io.pin_bit_mask = (1ULL << PIN_ETH_INT);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io);

    ESP_LOGI(TAG, "Parked Ethernet GPIOs (no W5500 hardware)");
}

// =============================================================================
// Network init: WiFi AP always, Ethernet optional
// Returns true if Ethernet is also active.
// =============================================================================
static bool network_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Park unused Ethernet GPIOs BEFORE WiFi starts — floating SPI2 lines
    // (GPIO 13-16) near the antenna degrade radio performance.
    park_eth_gpios();

    // ── WiFi AP — always on ──────────────────────────────────────────────
    wifi_ap_start();
    status_led_set(0, 0, 40);          // Blue = WiFi AP only

    // Start captive portal DNS (phones need it to reach the web UI)
    dns_server_start(ESP_IP4TOADDR(192, 168, 4, 1));

    // ── Ethernet — optional add-on ──────────────────────────────────────
    ESP_LOGI(TAG, "--- Probing Ethernet (W5500) ---");

    esp_err_t ret = w5500_eth_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "W5500 not found (%s) — WiFi AP only", esp_err_to_name(ret));
        park_eth_gpios();
        return false;
    }

    ret = w5500_eth_start();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "W5500 start failed (%s) — WiFi AP only", esp_err_to_name(ret));
        park_eth_gpios();
        return false;
    }

    // Wait for link up (max 3 s)
    ESP_LOGI(TAG, "Waiting for Ethernet link...");
    for (int i = 0; i < 30; i++) {
        if (w5500_eth_is_link_up()) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!w5500_eth_is_link_up()) {
        ESP_LOGW(TAG, "No Ethernet link — WiFi AP only");
        w5500_eth_stop();
        park_eth_gpios();
        return false;
    }

    ESP_LOGI(TAG, "Link up! Waiting for DHCP (%d ms timeout)...", ETH_DHCP_TIMEOUT_MS);

    for (int i = 0; i < (ETH_DHCP_TIMEOUT_MS / 100); i++) {
        if (w5500_eth_has_ip()) break;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!w5500_eth_has_ip()) {
        ESP_LOGW(TAG, "DHCP timeout → Static IP fallback (192.168.77.1/24)");

        esp_netif_t* eth_netif = w5500_eth_get_netif();
        if (eth_netif) {
            esp_netif_dhcpc_stop(eth_netif);

            esp_netif_ip_info_t ip_info = {
                .ip.addr = ESP_IP4TOADDR(192, 168, 77, 1),
                .gw.addr = ESP_IP4TOADDR(192, 168, 77, 1),
                .netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0),
            };

            ret = esp_netif_set_ip_info(eth_netif, &ip_info);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "=== Ethernet (Static IP 192.168.77.1) ===");
                status_led_set(40, 0, 40);          // Magenta = Ethernet static
                return true;
            }
            ESP_LOGE(TAG, "Failed to set static IP: %s", esp_err_to_name(ret));
            w5500_eth_stop();
            park_eth_gpios();
            return false;
        }
        ESP_LOGE(TAG, "Cannot get netif for static IP config");
        w5500_eth_stop();
        park_eth_gpios();
        return false;
    }

    ESP_LOGI(TAG, "=== Ethernet (DHCP) ===");
    status_led_set(0, 40, 0);                       // Green = Ethernet DHCP
    return true;
}

// Tasks

static void network_task(void* arg) {
    while (1) {
        etherdream_server_loop();
    }
}

void app_main(void) {
    // Pin-safe DAC SPI lines immediately to prevent boot glitches.
    // During early boot, GPIOs default to high-impedance input mode which
    // lets the DAC80508 CS float LOW, causing spurious latch of random data
    // on the RGB channels (laser flashes full-white at power-on).
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_DAC_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_DAC_CS, 1);  // CS HIGH = DAC deselected

    io_conf.pin_bit_mask = (1ULL << PIN_DAC_MOSI) | (1ULL << PIN_DAC_SCK);
    gpio_config(&io_conf);
    gpio_set_level(PIN_DAC_MOSI, 0);
    gpio_set_level(PIN_DAC_SCK, 0);

    // Init status LED and set RED (booting)
    status_led_init();
    status_led_set(40, 0, 0);

    vTaskDelay(pdMS_TO_TICKS(2000));
    
    ESP_LOGI(TAG, "OpenLaserDAC Starting...");
    ESP_LOGI(TAG, "  Buffer: %d points", FRAME_BUFFER_SIZE);
    ESP_LOGI(TAG, "HEAP boot: free=%lu int=%lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_free_internal_heap_size());

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "HEAP pre-net: free=%lu int=%lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_free_internal_heap_size());
    bool eth_mode = network_init();
    ESP_LOGI(TAG, "HEAP post-net: free=%lu int=%lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_free_internal_heap_size());

    // Init DAC after WiFi is up — the SPI3 bus + GPTimer ISR must not
    // run during WiFi radio initialisation.
    dac_timer_init();
    ESP_LOGI(TAG, "HEAP post-dac: free=%lu int=%lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_free_internal_heap_size());
    
    if (eth_mode) {
        ESP_LOGI(TAG, "Mode: WiFi AP + ETHERNET (%.0f Mbps)", (float)w5500_eth_get_speed());
    } else {
        ESP_LOGI(TAG, "Mode: WiFi AP only (%s)", WIFI_AP_SSID);
    }
    
    etherdream_server_init();
    etherdream_server_start();
    http_server_init();
    http_server_start();
    
    // Network task on Core 0
    xTaskCreatePinnedToCore(network_task, "net", 8192, NULL, TASK_PRIORITY_EDREAM, NULL, CORE_SERVICES);
    
    // Start DAC timer
    dac_timer_start();
    
    ESP_LOGI(TAG, "Ready - TCP:%d HTTP:%d", ETHERDREAM_TCP_PORT, HTTP_PORT);
    
    // Status loop with pipeline diagnostics
    uint32_t last_dac_out = 0;
    bool was_engaged = false;
    while (1) {
        g_status.running = dac_timer_is_running();
        g_status.ed_connected = etherdream_server_is_connected();
        g_status.buffer_level = frame_buffer_level();
        g_status.current_scan_rate = dac_timer_get_scan_rate();
        g_status.ed_point_rate = etherdream_server_get_point_rate();
        g_status.free_heap = esp_get_free_heap_size();
        
        // Pipeline diagnostic every 2s
        dac_timer_diag_t diag;
        dac_timer_get_diag(&diag);
        uint32_t dac_out = dac_get_output_count();
        uint32_t dac_delta = dac_out - last_dac_out;
        last_dac_out = dac_out;

        // Blink green while a protocol is connected and laser is engaged
        bool engaged = g_status.ed_connected && diag.playback_active;
        if (engaged && !was_engaged) {
            status_led_blink(0, 40, 0, 300);
        } else if (!engaged && was_engaged) {
            status_led_blink_stop();
            // Restore base network color
            if (eth_mode) {
                status_led_set(w5500_eth_has_ip() ? 0 : 40, w5500_eth_has_ip() ? 40 : 0, w5500_eth_has_ip() ? 0 : 40);
            } else {
                status_led_set(0, 0, 40);   // Blue = WiFi AP only
            }
        }
        was_engaged = engaged;
        
        ESP_LOGI(TAG, "PIPE| fb=%zu ed=%d play=%d run=%d | isr_buf=%zu ur=%lu | dac_out=%lu/2s spi=%d direct=%d | heap=%lu int=%lu min=%lu",
                 frame_buffer_level(),
                 g_status.ed_connected,
                 diag.playback_active,
                 diag.running,
                 diag.isr_buf_level,
                 (unsigned long)diag.underruns,
                 (unsigned long)dac_delta,
                 dac_is_spi_initialized(),
                 dac_is_direct_spi_ready(),
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)esp_get_free_internal_heap_size(),
                 (unsigned long)esp_get_minimum_free_heap_size());
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
