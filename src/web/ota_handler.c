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

#include "ota_handler.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include <string.h>

static const char* TAG = "OTA";

#define OTA_BUF_SIZE 4096

static esp_ota_handle_t ota_handle = 0;
static const esp_partition_t* update_partition = NULL;

// OTA upload handler
static esp_err_t ota_upload_handler(httpd_req_t* req) {
    char buf[OTA_BUF_SIZE];
    int remaining = req->content_len;
    int received = 0;

    ESP_LOGI(TAG, "Starting OTA update, size: %d bytes", remaining);

    // Get update partition
    update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        ESP_LOGE(TAG, "No OTA partition found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Writing to partition '%s' at offset 0x%lx", 
             update_partition->label, update_partition->address);

    // Begin OTA
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return err;
    }

    // Receive firmware data
    while (remaining > 0) {
        int chunk_size = (remaining < (int)sizeof(buf)) ? remaining : (int)sizeof(buf);
        int recv_len = httpd_req_recv(req, buf, chunk_size);
        if (recv_len < 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "Receive failed");
            esp_ota_abort(ota_handle);
            return ESP_FAIL;
        }

        if (recv_len > 0) {
            err = esp_ota_write(ota_handle, buf, recv_len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "OTA write failed: %s", esp_err_to_name(err));
                esp_ota_abort(ota_handle);
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
                return err;
            }
            received += recv_len;
            remaining -= recv_len;

            if (received % (100 * 1024) == 0) {
                ESP_LOGI(TAG, "Received %d KB / %d KB", received / 1024, req->content_len / 1024);
            }
        }
    }

    // Finalize OTA
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA end failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return err;
    }

    // Set boot partition
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Set boot partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot failed");
        return err;
    }

    ESP_LOGI(TAG, "OTA update successful! Rebooting in 2 seconds...");
    
    const char* resp = "{\"status\":\"success\",\"message\":\"OTA update complete, rebooting...\"}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));

    // Reboot after 2 seconds
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();

    return ESP_OK;
}

// OTA status handler
static esp_err_t ota_status_handler(httpd_req_t* req) {
    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* boot = esp_ota_get_boot_partition();

    char resp[256];
    snprintf(resp, sizeof(resp),
        "{\"running_partition\":\"%s\",\"boot_partition\":\"%s\",\"version\":\"%s\"}",
        running->label, boot->label, esp_app_get_description()->version);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

esp_err_t ota_handler_register(httpd_handle_t server) {
    httpd_uri_t ota_upload_uri = {
        .uri = "/api/ota",
        .method = HTTP_POST,
        .handler = ota_upload_handler,
        .user_ctx = NULL
    };

    httpd_uri_t ota_status_uri = {
        .uri = "/api/ota/status",
        .method = HTTP_GET,
        .handler = ota_status_handler,
        .user_ctx = NULL
    };

    httpd_register_uri_handler(server, &ota_upload_uri);
    httpd_register_uri_handler(server, &ota_status_uri);

    ESP_LOGI(TAG, "OTA handlers registered");
    return ESP_OK;
}
