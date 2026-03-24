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

#include "w5500_eth.h"
#include "config.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <string.h>

static const char* TAG = "ETH";

static esp_eth_handle_t s_eth_handle = NULL;
static esp_netif_t* s_eth_netif = NULL;
static volatile bool s_link_up = false;
static volatile bool s_got_ip = false;
static uint32_t s_link_speed = 0;
static eth_got_ip_cb_t s_got_ip_callback = NULL;

// Callbacks

void w5500_eth_set_got_ip_callback(eth_got_ip_cb_t cb) {
    s_got_ip_callback = cb;
}

// Event Handlers

static void eth_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    switch (event_id) {
        case ETHERNET_EVENT_CONNECTED: {
            s_link_up = true;
            eth_speed_t speed;
            eth_duplex_t duplex;
            if (esp_eth_ioctl(s_eth_handle, ETH_CMD_G_SPEED, &speed) == ESP_OK &&
                esp_eth_ioctl(s_eth_handle, ETH_CMD_G_DUPLEX_MODE, &duplex) == ESP_OK) {
                s_link_speed = (speed == ETH_SPEED_100M) ? 100 : 10;
                ESP_LOGI(TAG, "Link Up - %luMbps %s", 
                         (unsigned long)s_link_speed,
                         duplex == ETH_DUPLEX_FULL ? "Full" : "Half");
            }
            break;
        }
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Link Down");
            s_link_up = false;
            s_got_ip = false;
            s_link_speed = 0;
            break;
        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "Started");
            break;
        case ETHERNET_EVENT_STOP:
            s_link_up = false;
            s_got_ip = false;
            break;
        default:
            break;
    }
}

static void ip_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data)
{
    if (event_id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        s_got_ip = true;
        
        ESP_LOGI(TAG, "=== Ethernet Ready ===");
        ESP_LOGI(TAG, "  IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "  GW: " IPSTR, IP2STR(&event->ip_info.gw));
        ESP_LOGI(TAG, "  Speed: %lu Mbps", (unsigned long)s_link_speed);
        
        uint8_t mac[6];
        if (esp_eth_ioctl(s_eth_handle, ETH_CMD_G_MAC_ADDR, mac) == ESP_OK) {
            ESP_LOGI(TAG, "  MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        }
        ESP_LOGI(TAG, "======================");
        
        if (s_got_ip_callback) {
            s_got_ip_callback();
        }
    }
}

// Init / Start / Stop

esp_err_t w5500_eth_init(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "W5500 init: SPI@%dMHz CS=%d MOSI=%d MISO=%d CLK=%d INT=%d",
             ETH_SPI_FREQ_HZ / 1000000,
             PIN_ETH_CS, PIN_ETH_MOSI, PIN_ETH_MISO, PIN_ETH_SCK, PIN_ETH_INT);
    
    // ── 1. SPI bus ──────────────────────────────────────────────────────────
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_ETH_MOSI,
        .miso_io_num = PIN_ETH_MISO,
        .sclk_io_num = PIN_ETH_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };
    ret = spi_bus_initialize(ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // ── 2. Software reset via SPI @ 1MHz (no HW reset pin) ─────────────────
    vTaskDelay(pdMS_TO_TICKS(50));
    {
        spi_device_handle_t test_spi;
        spi_device_interface_config_t test_cfg = {
            .mode = 0,
            .clock_speed_hz = 1000000,  // 1MHz — safe for cold boot
            .spics_io_num = PIN_ETH_CS,
            .queue_size = 1,
        };
        if (spi_bus_add_device(ETH_SPI_HOST, &test_cfg, &test_spi) == ESP_OK) {
            // Diagnostic: try to read version BEFORE reset
            uint8_t ver_cmd[4] = {0x00, 0x39, 0x00, 0x00};
            uint8_t ver_rx[4] = {0};
            spi_transaction_t vt0 = { .length = 32, .tx_buffer = ver_cmd, .rx_buffer = ver_rx };
            spi_device_transmit(test_spi, &vt0);
            ESP_LOGI(TAG, "SPI diag: pre-reset version=0x%02X (raw: %02X %02X %02X %02X)",
                     ver_rx[3], ver_rx[0], ver_rx[1], ver_rx[2], ver_rx[3]);
            
            // Write RST bit to Mode Register (MR)
            uint8_t reset_cmd[4] = {0x00, 0x00, 0x04, 0x80};
            spi_transaction_t t = { .length = 32, .tx_buffer = reset_cmd };
            spi_device_transmit(test_spi, &t);
            vTaskDelay(pdMS_TO_TICKS(200));
            
            // Read Version Register (0x0039) after reset
            memset(ver_rx, 0, 4);
            spi_transaction_t vt = { .length = 32, .tx_buffer = ver_cmd, .rx_buffer = ver_rx };
            spi_device_transmit(test_spi, &vt);
            ESP_LOGI(TAG, "SPI diag: post-reset version=0x%02X (raw: %02X %02X %02X %02X)",
                     ver_rx[3], ver_rx[0], ver_rx[1], ver_rx[2], ver_rx[3]);
            uint8_t phy_cmd[4] = {0x00, 0x2E, 0x00, 0x00};
            uint8_t phy_rx[4] = {0};
            spi_transaction_t pt = { .length = 32, .tx_buffer = phy_cmd, .rx_buffer = phy_rx };
            spi_device_transmit(test_spi, &pt);
            ESP_LOGI(TAG, "SPI diag: PHYCFGR raw: %02X %02X %02X %02X",
                     phy_rx[0], phy_rx[1], phy_rx[2], phy_rx[3]);
            
            if (ver_rx[3] == 0x00 && phy_rx[3] == 0x00) {
                ESP_LOGE(TAG, "W5500 not responding! Check: VCC(3.3V), GND, MISO(GPIO%d), MOSI(GPIO%d), CLK(GPIO%d), CS(GPIO%d)",
                         PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_SCK, PIN_ETH_CS);
                spi_bus_remove_device(test_spi);
                spi_bus_free(ETH_SPI_HOST);
                return ESP_ERR_NOT_FOUND;
            }
            
            spi_bus_remove_device(test_spi);
        }
    }
    
    // ── 3. SPI device config (passed to W5500 driver) ───────────────────────
    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = ETH_SPI_FREQ_HZ,
        .spics_io_num = PIN_ETH_CS,
        .queue_size = 20,
    };
    
    // ── 3. GPIO ISR service (only if interrupt mode) ───────────────────────
    if (PIN_ETH_INT >= 0) {
        ret = gpio_install_isr_service(0);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "GPIO ISR service install failed");
        }
    }
    
    // ── 4. W5500 MAC ────────────────────────────────────────────────────────
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(ETH_SPI_HOST, &spi_devcfg);
    if (PIN_ETH_INT >= 0) {
        w5500_config.int_gpio_num = PIN_ETH_INT;
        w5500_config.poll_period_ms = 0;
        ESP_LOGI(TAG, "Mode: interrupt (GPIO %d)", PIN_ETH_INT);
    } else {
        w5500_config.int_gpio_num = -1;
        w5500_config.poll_period_ms = 1;
        ESP_LOGI(TAG, "Mode: polling (1ms)");
    }
    
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    mac_config.rx_task_stack_size = 4096;
    // CRITICAL: rx_task must be HIGHER priority than both tcpip_task (20) and
    // network_task (19).  At priority 15 (the default) it gets preempted while
    // our server processes a large batch, so frames pile up inside the W5500's
    // 16 KB MACRAW RX buffer.  An 878-point batch = 11 TCP segments =
    // ~16.4 KB of Ethernet frames → overflow → dropped segment → TCP retransmit
    // after 200-300 ms → client times out → disconnect.
    // At priority 22 the ISR wakes rx_task immediately, it reads the frame
    // from SPI in ~30 µs, hands it to LWIP, and returns — no starvation.
    mac_config.rx_task_prio = 22;
    
    esp_eth_mac_t* mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    if (!mac) {
        ESP_LOGE(TAG, "MAC create failed");
        return ESP_FAIL;
    }
    
    // ── 5. W5500 PHY ────────────────────────────────────────────────────────
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 1;
    phy_config.reset_gpio_num = -1;
    
    esp_eth_phy_t* phy = esp_eth_phy_new_w5500(&phy_config);
    if (!phy) {
        ESP_LOGE(TAG, "PHY create failed");
        return ESP_FAIL;
    }
    
    // ── 6. Install driver ───────────────────────────────────────────────────
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ret = esp_eth_driver_install(&eth_config, &s_eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // ── 7. MAC address ──────────────────────────────────────────────────────
    uint8_t mac_addr[] = ETH_MAC_ADDR;
    esp_eth_ioctl(s_eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr);
    
    // ── 8. Netif + DHCP ─────────────────────────────────────────────────────
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_eth_netif = esp_netif_new(&netif_cfg);
    if (!s_eth_netif) {
        ESP_LOGE(TAG, "Netif create failed");
        return ESP_FAIL;
    }
    
    esp_eth_netif_glue_handle_t eth_glue = esp_eth_new_netif_glue(s_eth_handle);
    esp_netif_attach(s_eth_netif, eth_glue);
    
    // ── 9. Event handlers ───────────────────────────────────────────────────
    esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &ip_event_handler, NULL);
    
    ESP_LOGI(TAG, "Init OK — interrupt on GPIO %d", PIN_ETH_INT);
    return ESP_OK;
}

esp_err_t w5500_eth_start(void)
{
    if (!s_eth_handle) return ESP_ERR_INVALID_STATE;
    return esp_eth_start(s_eth_handle);
}

esp_err_t w5500_eth_stop(void)
{
    if (!s_eth_handle) return ESP_ERR_INVALID_STATE;
    return esp_eth_stop(s_eth_handle);
}

bool w5500_eth_is_link_up(void)
{
    return s_link_up;
}

bool w5500_eth_has_ip(void)
{
    return s_got_ip;
}

esp_netif_t* w5500_eth_get_netif(void)
{
    return s_eth_netif;
}

uint32_t w5500_eth_get_speed(void)
{
    return s_link_speed;
}
