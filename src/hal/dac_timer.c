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
 * @file dac_timer.c
 * @brief Hardware-timed DAC output using GPTimer ISR per-point
 * 
 * Architecture (ISR-per-point):
 *   GPTimer fires at scan_rate Hz - ISR writes point directly to DAC
 *   Background task refills ISR buffer from frame_buffer
 *   
 * Benefits:
 *   - Hardware timer guarantees exact scan rate (no drift!)
 *   - ISR uses direct SPI (~6us) - fits in 22us @ 45kpps
 *   - Zero jitter between points
 */

#include "dac_timer.h"
#include "dac80508.h"
#include "core/frame_buffer.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <inttypes.h>
#include <string.h>

static const char* TAG = "DAC_TIMER";

// Configuration
#define ISR_BUFFER_SIZE 1024
#define REFILL_THRESHOLD 128     // Refill when 1/8 free — keeps ISR fed
#define REFILL_MAX_BATCH 256     // But read up to this many at once
#define TIMER_RESOLUTION_HZ 40000000ULL  // 40 MHz (APB/2) — max HW allows (prescaler >= 2)

// Hardware resources
static gptimer_handle_t s_timer = NULL;
static TaskHandle_t s_task = NULL;
static SemaphoreHandle_t s_init_done = NULL;  // Signal from Core 1 that timer ISR is installed
static volatile bool s_running = false;
static volatile bool s_playback_active = false;  // Gate: only drain frame_buffer when true
static volatile uint32_t s_scan_rate = SCAN_RATE_DEFAULT_HZ;

// ISR ring buffer - written by task, read by ISR
static laser_point_t s_isr_buffer[ISR_BUFFER_SIZE];
static volatile size_t s_isr_head = 0;
static volatile size_t s_isr_tail = 0;
static volatile uint32_t s_isr_underruns = 0;

// Timing statistics
static volatile uint32_t s_points_output = 0;
static volatile int64_t s_last_log_time = 0;

#if LOG_PERF
// Performance profiling (all in microseconds)
static volatile uint32_t s_isr_time_sum = 0;     // Accumulated ISR time (µs)
static volatile uint32_t s_isr_time_max = 0;      // Max ISR time (µs)
static volatile uint32_t s_isr_time_count = 0;    // Number of ISR calls
static volatile uint32_t s_refill_time_sum = 0;   // Accumulated refill batch time (µs)
static volatile uint32_t s_refill_time_max = 0;   // Max refill time (µs)
static volatile uint32_t s_refill_count = 0;      // Number of refills
static volatile uint32_t s_refill_pts_sum = 0;    // Total points refilled
static volatile int64_t s_perf_last_time = 0;     // Last perf log time
#endif

// Rest point (blanked, centered) - used for stop and underruns
static const laser_point_t REST_POINT = {
    .x = 0, .y = 0, .r = 0, .g = 0, .b = 0,
    .user1 = 0, .user2 = 0, .flags = POINT_FLAG_BLANK
};

// ISR buffer helpers (lock-free single producer/consumer)
static inline size_t IRAM_ATTR isr_buffer_count(void) {
    size_t h = s_isr_head;
    size_t t = s_isr_tail;
    if (h >= t) return h - t;
    return ISR_BUFFER_SIZE - t + h;
}

static inline size_t isr_buffer_free(void) {
    return ISR_BUFFER_SIZE - 1 - isr_buffer_count();
}

// GPTimer ISR callback - outputs ONE point per call
static bool IRAM_ATTR timer_isr_callback(gptimer_handle_t timer, 
                                          const gptimer_alarm_event_data_t *edata, 
                                          void *user_ctx)
{
    if (!s_running) {
        return false;
    }
    
#if LOG_PERF
    uint32_t isr_start = (uint32_t)esp_timer_get_time();
#endif
    
    // Check if we have data
    if (s_isr_tail == s_isr_head) {
        // Buffer empty - output rest point
        dac_output_point(&REST_POINT);
        s_isr_underruns++;
        return false;
    }
    
    // Get next point
    const laser_point_t* point = &s_isr_buffer[s_isr_tail];
    
    // Output point (uses direct SPI - ~6us)
    dac_output_point(point);
    
    // Advance tail
    s_isr_tail = (s_isr_tail + 1) % ISR_BUFFER_SIZE;
    s_points_output++;
    
#if LOG_PERF
    uint32_t isr_elapsed = (uint32_t)esp_timer_get_time() - isr_start;
    s_isr_time_sum += isr_elapsed;
    s_isr_time_count++;
    if (isr_elapsed > s_isr_time_max) s_isr_time_max = isr_elapsed;
#endif
    
    return false;
}

// Buffer refill task - keeps ISR buffer full
// CRITICAL: Must be very fast - no delays when buffer needs filling!
// NOTE: This task runs on Core 1. It also creates the GPTimer here so
// the ISR is registered on Core 1, keeping DAC SPI and timer on same core.
static void dac_refill_task(void* arg)
{
    ESP_LOGI(TAG, "Refill task started on Core %d", xPortGetCoreID());
    
    // === Create GPTimer on Core 1 so ISR runs here ===
    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RESOLUTION_HZ,  // 40 MHz for sub-µs precision
        .intr_priority = 1,
    };
    
    esp_err_t ret = gptimer_new_timer(&timer_cfg, &s_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer on Core 1: %s", esp_err_to_name(ret));
        xSemaphoreGive(s_init_done);
        vTaskDelete(NULL);
        return;
    }
    
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_isr_callback,
    };
    ret = gptimer_register_event_callbacks(s_timer, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register timer callback");
        xSemaphoreGive(s_init_done);
        vTaskDelete(NULL);
        return;
    }
    
    ret = gptimer_enable(s_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable timer");
        xSemaphoreGive(s_init_done);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "GPTimer ISR installed on Core %d", xPortGetCoreID());
    
    // Signal to dac_timer_init() that we're ready
    xSemaphoreGive(s_init_done);
    
    // === Normal refill loop ===
    static laser_point_t temp_batch[REFILL_MAX_BATCH];
    
    s_last_log_time = esp_timer_get_time();
#if LOG_PERF
    s_perf_last_time = s_last_log_time;
#endif
    uint32_t last_underruns = 0;
    uint32_t idle_loops = 0;
    
    while (1) {
        if (!s_running) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // GATE: Only drain frame_buffer into ISR buffer when playback is active.
        // During PREPARED state, the client is filling the buffer — we must NOT
        // consume those points or the client can never accumulate enough to start.
        if (!s_playback_active) {
            idle_loops++;
            if (idle_loops > 100) {
                vTaskDelay(1);
                idle_loops = 0;
            } else {
                taskYIELD();
            }
            
#if LOG_DAC_STATS
            // Reset stats counters even when idle
            int64_t now_idle = esp_timer_get_time();
            if (now_idle - s_last_log_time >= 1000000) {
                s_points_output = 0;
                last_underruns = s_isr_underruns;
                s_last_log_time = now_idle;
            }
#endif
            continue;
        }
        
        // Check if ISR buffer needs refill
        size_t free_space = isr_buffer_free();
        
        if (free_space >= REFILL_THRESHOLD) {
            // Read as much as we can fit (up to REFILL_MAX_BATCH)
            size_t to_read = (free_space < REFILL_MAX_BATCH) ? free_space : REFILL_MAX_BATCH;
            
#if LOG_PERF
            int64_t refill_start = esp_timer_get_time();
#endif
            size_t n = frame_buffer_read(temp_batch, to_read);
            
            if (n > 0) {
                // Copy to ISR buffer
                for (size_t i = 0; i < n; i++) {
                    s_isr_buffer[s_isr_head] = temp_batch[i];
                    s_isr_head = (s_isr_head + 1) % ISR_BUFFER_SIZE;
                }
#if LOG_PERF
                uint32_t refill_elapsed = (uint32_t)(esp_timer_get_time() - refill_start);
                s_refill_time_sum += refill_elapsed;
                s_refill_count++;
                s_refill_pts_sum += n;
                if (refill_elapsed > s_refill_time_max) s_refill_time_max = refill_elapsed;
#endif
                idle_loops = 0;  // Reset idle counter
            } else {
                // No data available from frame_buffer
                idle_loops++;
            }
        } else {
            // Buffer is full enough
            idle_loops++;
        }
        
#if LOG_DAC_STATS
        // Log stats every 2 seconds (reduce UART contention)
        int64_t now = esp_timer_get_time();
        if (now - s_last_log_time >= 2000000) {
            uint32_t pts = s_points_output;
            uint32_t underruns = s_isr_underruns - last_underruns;
            size_t buf_level = isr_buffer_count();
            
            int64_t elapsed_us = now - s_last_log_time;
            uint32_t expected_pts = (uint32_t)((int64_t)s_scan_rate * elapsed_us / 1000000);
            int32_t error_ppm = (expected_pts > 0) ? 
                (int32_t)(((int64_t)pts - expected_pts) * 1000000 / expected_pts) : 0;
            
            ESP_LOGI(TAG, "ISR: pts/s=%"PRIu32" expected=%"PRIu32" err=%+"PRId32"ppm | buf=%zu underruns=%"PRIu32,
                     pts, expected_pts, error_ppm, buf_level, underruns);
            
            s_points_output = 0;
            last_underruns = s_isr_underruns;
            s_last_log_time = now;
        }
#endif
        
#if LOG_PERF
        // Performance profiling log every 5 seconds
        {
            int64_t now_perf = esp_timer_get_time();
            if (now_perf - s_perf_last_time >= 5000000) {
                uint32_t isr_cnt = s_isr_time_count;
                uint32_t isr_avg = (isr_cnt > 0) ? (s_isr_time_sum / isr_cnt) : 0;
                uint32_t isr_max = s_isr_time_max;
                uint32_t ref_cnt = s_refill_count;
                uint32_t ref_avg = (ref_cnt > 0) ? (s_refill_time_sum / ref_cnt) : 0;
                uint32_t ref_max = s_refill_time_max;
                uint32_t ref_pts_avg = (ref_cnt > 0) ? (s_refill_pts_sum / ref_cnt) : 0;
                uint64_t period_ticks = (s_scan_rate > 0) ? ((TIMER_RESOLUTION_HZ + s_scan_rate - 1) / s_scan_rate) : 0;
                uint32_t period_us = (uint32_t)(period_ticks / (TIMER_RESOLUTION_HZ / 1000000));
                uint32_t isr_pct = (period_us > 0) ? (isr_avg * 100 / period_us) : 0;
                
                ESP_LOGI(TAG, "PERF ISR: avg=%luus max=%luus (%lu%% of %luus) | REFILL: avg=%luus max=%luus batch=%lu cnt=%lu",
                         (unsigned long)isr_avg, (unsigned long)isr_max,
                         (unsigned long)isr_pct, (unsigned long)period_us,
                         (unsigned long)ref_avg, (unsigned long)ref_max,
                         (unsigned long)ref_pts_avg, (unsigned long)ref_cnt);
                
                // Reset perf counters
                s_isr_time_sum = 0;
                s_isr_time_max = 0;
                s_isr_time_count = 0;
                s_refill_time_sum = 0;
                s_refill_time_max = 0;
                s_refill_count = 0;
                s_refill_pts_sum = 0;
                s_perf_last_time = now_perf;
            }
        }
#endif
        
        // Yield strategy: only delay if truly idle for many loops
        // This keeps the task responsive while not hogging CPU when no data
        if (idle_loops > 1000) {
            vTaskDelay(1);  // Only delay after extended idle period
            idle_loops = 0;
        } else {
            taskYIELD();    // Let other tasks run but come back ASAP
        }
    }
}

// Update timer period based on scan rate
static esp_err_t update_timer_period(void)
{
    if (!s_timer) return ESP_ERR_INVALID_STATE;
    
    // Timer period in ticks at TIMER_RESOLUTION_HZ.
    // Use CEILING division so ISR fires at ≤ declared rate.
    // At 1 MHz the old code truncated 33.33→33 µs for 30kpps (+1.01% error!).
    // At 80 MHz with ceiling: 30kpps → ceil(80M/30k) = 2667 ticks → 29989 Hz (-0.037%).
    uint64_t period_ticks = (TIMER_RESOLUTION_HZ + s_scan_rate - 1) / s_scan_rate;
    
    // Clamp: at 100kpps with 80MHz → 800 ticks (12.5µs). Min 100 ticks = 800kHz theoretical max.
    if (period_ticks < 100) {
        ESP_LOGW(TAG, "Scan rate too high, clamping period to 100 ticks");
        period_ticks = 100;
    }
    
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = period_ticks,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    
#if LOG_RATE_CHANGE
    // Log actual rate for diagnostics
    uint32_t actual_hz = (uint32_t)(TIMER_RESOLUTION_HZ / period_ticks);
    int32_t err_ppm = (int32_t)(((int64_t)actual_hz - (int64_t)s_scan_rate) * 1000000 / (int64_t)s_scan_rate);
    ESP_LOGI(TAG, "Timer: %"PRIu64" ticks @ 40MHz → %"PRIu32" Hz (requested %"PRIu32", err=%+"PRId32"ppm)",
             period_ticks, actual_hz, s_scan_rate, err_ppm);
#endif
    
    return gptimer_set_alarm_action(s_timer, &alarm_cfg);
}

// Public API

esp_err_t dac_timer_init(void)
{
    ESP_LOGI(TAG, "Initializing ISR-per-point DAC output");
    
    esp_err_t ret = frame_buffer_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init frame buffer");
        return ret;
    }
    
    ret = dac_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init DAC");
        return ret;
    }
    
    // Create semaphore for synchronizing with Core 1 timer init
    s_init_done = xSemaphoreCreateBinary();
    if (!s_init_done) {
        ESP_LOGE(TAG, "Failed to create init semaphore");
        return ESP_ERR_NO_MEM;
    }
    
    // Launch refill task on Core 1 — it will create the GPTimer there
    // so the ISR is pinned to Core 1 (same core as refill + SPI)
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        dac_refill_task,
        "dac_refill",
        4096,
        NULL,
        configMAX_PRIORITIES - 2,  // Very high priority
        &s_task,
        CORE_REALTIME  // Core 1 - away from WiFi
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create refill task");
        vSemaphoreDelete(s_init_done);
        return ESP_ERR_NO_MEM;
    }
    
    // Wait for Core 1 to finish timer setup
    if (xSemaphoreTake(s_init_done, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Timeout waiting for Core 1 timer init");
        return ESP_ERR_TIMEOUT;
    }
    vSemaphoreDelete(s_init_done);
    s_init_done = NULL;
    
    if (!s_timer) {
        ESP_LOGE(TAG, "Timer creation on Core 1 failed");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Initialized: ISR buffer=%d, refill threshold=%d (ISR+refill on Core 1)", 
             ISR_BUFFER_SIZE, REFILL_THRESHOLD);
    return ESP_OK;
}

esp_err_t dac_timer_start(void)
{
    if (s_running) return ESP_OK;
    if (!s_timer) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Starting at %lu pps", (unsigned long)s_scan_rate);
    
    s_isr_head = 0;
    s_isr_tail = 0;
    s_isr_underruns = 0;
    s_points_output = 0;
    
    esp_err_t ret = update_timer_period();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set timer period");
        return ret;
    }
    
    gptimer_set_raw_count(s_timer, 0);
    s_running = true;
    
    ret = gptimer_start(s_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start timer");
        s_running = false;
        return ret;
    }
    
    ESP_LOGI(TAG, "Started");
    return ESP_OK;
}

esp_err_t dac_timer_stop(void)
{
    if (!s_running) return ESP_OK;
    
    ESP_LOGI(TAG, "Stopping");
    s_running = false;
    
    if (s_timer) {
        gptimer_stop(s_timer);
    }
    
    dac_output_point(&REST_POINT);
    return ESP_OK;
}

esp_err_t dac_timer_set_scan_rate(uint32_t rate_hz)
{
    if (rate_hz < SCAN_RATE_MIN_HZ) rate_hz = SCAN_RATE_MIN_HZ;
    if (rate_hz > SCAN_RATE_MAX_HZ) rate_hz = SCAN_RATE_MAX_HZ;
    
    uint32_t old_rate = s_scan_rate;
    
    // Skip if rate hasn't actually changed — avoid timer stop/start gap
    if (rate_hz == old_rate && s_running) {
        return ESP_OK;
    }
    
    s_scan_rate = rate_hz;
    g_config.scan_rate_hz = rate_hz;
    
#if LOG_RATE_CHANGE
    ESP_LOGI(TAG, "Scan rate: %lu -> %lu pps", (unsigned long)old_rate, (unsigned long)rate_hz);
#endif
    
    if (s_running && s_timer) {
        // Update alarm period without stopping timer — gptimer_set_alarm_action
        // can be called while running, takes effect on next alarm
        update_timer_period();
    }
    
    return ESP_OK;
}

uint32_t dac_timer_get_scan_rate(void)
{
    return s_scan_rate;
}

bool dac_timer_is_running(void)
{
    return s_running;
}

void dac_timer_set_playback_active(bool active)
{
    if (active && !s_playback_active) {
        // Transitioning to active: clear ISR buffer for clean start
        s_isr_head = 0;
        s_isr_tail = 0;
    }
    s_playback_active = active;
}

bool dac_timer_is_playback_active(void)
{
    return s_playback_active;
}

void dac_timer_get_diag(dac_timer_diag_t *diag)
{
    if (!diag) return;
    diag->points_output = s_points_output;
    diag->underruns = s_isr_underruns;
    diag->isr_buf_level = isr_buffer_count();
    diag->playback_active = s_playback_active;
    diag->running = s_running;
}
