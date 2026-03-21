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

#include "http_server.h"
#include "ota_handler.h"
#include "config.h"
#include "hal/dac_timer.h"
#include "core/frame_buffer.h"
#include "input/etherdream_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>

static const char* TAG = "HTTP";
static httpd_handle_t s_server = NULL;

static const char* INDEX_HTML = 
"<!DOCTYPE html>\n"
"<html><head>\n"
"<meta charset='utf-8'>\n"
"<meta name='viewport' content='width=device-width,initial-scale=1'>\n"
"<title>OpenLaserDAC</title>\n"
"<style>\n"
"*{box-sizing:border-box;margin:0;padding:0}\n"
"body{font-family:-apple-system,sans-serif;background:#1a1a2e;color:#eee;padding:16px}\n"
".header{text-align:center;padding:20px 0;border-bottom:1px solid #333}\n"
".header h1{font-size:1.8em;color:#00d9ff}\n"
".status{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:12px;margin:20px 0}\n"
".stat{background:#252547;padding:16px;border-radius:12px;text-align:center}\n"
".stat-value{font-size:1.8em;font-weight:700;color:#00d9ff}\n"
".stat-label{font-size:.75em;color:#888;margin-top:4px}\n"
".card{background:#252547;border-radius:12px;padding:20px;margin:16px 0}\n"
".indicator{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:8px}\n"
".on{background:#2ecc71}.off{background:#666}\n"
"a{color:#00d9ff}\n"
"</style>\n"
"</head><body>\n"
"<div class='header'><h1>⚡ OpenLaserDAC</h1></div>\n"
"<div class='status'>\n"
"<div class='stat'><div class='stat-value' id='kpps'>--</div><div class='stat-label'>kpps</div></div>\n"
"<div class='stat'><div class='stat-value' id='buffer'>--</div><div class='stat-label'>Buffer</div></div>\n"
"<div class='stat'><div class='stat-value' id='latency'>--</div><div class='stat-label'>Latency ms</div></div>\n"
"</div>\n"
"<div class='card'>\n"
"<p><span class='indicator' id='edInd'></span><span id='edStatus'>Waiting...</span></p>\n"
"</div>\n"
"<script>\n"
"async function refresh(){\n"
"  const s=await(await fetch('/api/status')).json();\n"
"  document.getElementById('kpps').textContent=(s.scan_rate/1000).toFixed(1);\n"
"  document.getElementById('buffer').textContent=s.buffer_level;\n"
"  document.getElementById('latency').textContent=s.latency_ms||'--';\n"
"  document.getElementById('edInd').className='indicator '+(s.ed_connected?'on':'off');\n"
"  document.getElementById('edStatus').textContent=s.ed_connected?'Streaming':'Waiting...';\n"
"}\n"
"setInterval(refresh,500);refresh();\n"
"</script>\n"
"</body></html>\n";

static esp_err_t index_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
    return ESP_OK;
}
// Android checks /generate_204 and expects HTTP 204.
// iOS/macOS check /hotspot-detect.html and expect "Success".
// Windows checks /connecttest.txt and /ncsi.txt.
static esp_err_t captive_204_handler(httpd_req_t* req) {
    // Android: expects HTTP 204 No Content
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t captive_success_handler(httpd_req_t* req) {
    // iOS/macOS: expects body containing "Success"
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
    return ESP_OK;
}

static esp_err_t captive_ncsi_handler(httpd_req_t* req) {
    // Windows NCSI: expects 200 OK with text body
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "Microsoft NCSI");
    return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t* req) {
    cJSON* json = cJSON_CreateObject();
    
    cJSON_AddBoolToObject(json, "running", g_status.running);
    cJSON_AddBoolToObject(json, "ed_connected", g_status.ed_connected);
    cJSON_AddNumberToObject(json, "buffer_level", g_status.buffer_level);
    cJSON_AddNumberToObject(json, "scan_rate", g_status.current_scan_rate);
    cJSON_AddNumberToObject(json, "ed_point_rate", g_status.ed_point_rate);
    
    // AP-only: always show as AP mode
    cJSON_AddBoolToObject(json, "wifi_connected", false);
    cJSON_AddStringToObject(json, "wifi_ssid", "");
    
    if (g_status.current_scan_rate > 0 && g_status.buffer_level > 0) {
        float latency = (float)g_status.buffer_level / (float)g_status.current_scan_rate * 1000.0f;
        cJSON_AddNumberToObject(json, "latency_ms", (int)latency);
    } else {
        cJSON_AddNumberToObject(json, "latency_ms", 0);
    }
    
    const char* str = cJSON_PrintUnformatted(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, str, strlen(str));
    free((void*)str);
    cJSON_Delete(json);
    return ESP_OK;
}

esp_err_t http_server_init(void) {
    return ESP_OK;
}

esp_err_t http_server_start(void) {
    if (s_server) return ESP_OK;
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.core_id = CORE_SERVICES;
    config.max_uri_handlers = 16;
    
    esp_err_t ret = httpd_start(&s_server, &config);
    if (ret != ESP_OK) return ret;
    
    httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler };
    httpd_register_uri_handler(s_server, &index_uri);
    
    httpd_uri_t status_uri = { .uri = "/api/status", .method = HTTP_GET, .handler = status_handler };
    httpd_register_uri_handler(s_server, &status_uri);
    // Android
    httpd_uri_t gen204_uri = { .uri = "/generate_204", .method = HTTP_GET, .handler = captive_204_handler };
    httpd_register_uri_handler(s_server, &gen204_uri);
    // iOS / macOS
    httpd_uri_t hotspot_uri = { .uri = "/hotspot-detect.html", .method = HTTP_GET, .handler = captive_success_handler };
    httpd_register_uri_handler(s_server, &hotspot_uri);
    // Windows NCSI
    httpd_uri_t ncsi_uri = { .uri = "/connecttest.txt", .method = HTTP_GET, .handler = captive_ncsi_handler };
    httpd_register_uri_handler(s_server, &ncsi_uri);
    httpd_uri_t ncsi2_uri = { .uri = "/ncsi.txt", .method = HTTP_GET, .handler = captive_ncsi_handler };
    httpd_register_uri_handler(s_server, &ncsi2_uri);
    // Additional Apple check
    httpd_uri_t apple_uri = { .uri = "/library/test/success.html", .method = HTTP_GET, .handler = captive_success_handler };
    httpd_register_uri_handler(s_server, &apple_uri);
    
    // Register OTA handlers
    ota_handler_register(s_server);
    
    ESP_LOGI(TAG, "HTTP on port %d", HTTP_PORT);
    return ESP_OK;
}

esp_err_t http_server_stop(void) {
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
    return ESP_OK;
}
