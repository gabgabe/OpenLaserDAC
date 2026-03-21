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
 * @file frame_buffer.c
 * @brief Lock-free SPSC ring buffer for laser points
 * 
 * Single-Producer Single-Consumer (SPSC) lock-free design:
 *   - Producer (network_task on Core 0): writes via frame_buffer_write()
 *   - Consumer (dac_refill_task on Core 1): reads via frame_buffer_read()
 * 
 * No mutex needed! Uses memory barriers for cross-core synchronization.
 */

#include "frame_buffer.h"
#include <string.h>
#include <stdatomic.h>
#include "esp_attr.h"

static laser_point_t buffer[FRAME_BUFFER_SIZE];

// Atomic indices for lock-free operation
// head: written by producer, read by consumer
// tail: written by consumer, read by producer
static atomic_size_t head = 0;
static atomic_size_t tail = 0;

esp_err_t frame_buffer_init(void)
{
    atomic_store_explicit(&head, 0, memory_order_relaxed);
    atomic_store_explicit(&tail, 0, memory_order_relaxed);
    memset(buffer, 0, sizeof(buffer));
    return ESP_OK;
}

// Producer function - called from network_task (Core 0)
size_t frame_buffer_write(const laser_point_t* points, size_t count)
{
    if (!points || count == 0) return 0;

    size_t h = atomic_load_explicit(&head, memory_order_relaxed);
    size_t t = atomic_load_explicit(&tail, memory_order_acquire);  // Sync with consumer
    
    // Calculate available space
    size_t free_space;
    if (h >= t)
        free_space = FRAME_BUFFER_SIZE - 1 - (h - t);
    else
        free_space = t - h - 1;
    
    size_t to_write = (count < free_space) ? count : free_space;
    if (to_write == 0) return 0;
    
    // Bulk copy using memcpy - handle wrap-around
    size_t first_chunk = FRAME_BUFFER_SIZE - h;
    if (first_chunk > to_write) first_chunk = to_write;
    
    memcpy(&buffer[h], points, first_chunk * sizeof(laser_point_t));
    
    if (to_write > first_chunk) {
        // Wrap around to beginning
        memcpy(&buffer[0], &points[first_chunk], (to_write - first_chunk) * sizeof(laser_point_t));
    }
    
    // Advance head
    size_t new_head = (h + to_write) % FRAME_BUFFER_SIZE;
    
    // Publish new head position to consumer
    atomic_store_explicit(&head, new_head, memory_order_release);
    return to_write;
}

// Consumer function - called from dac_refill_task (Core 1)
size_t frame_buffer_read(laser_point_t* points, size_t max)
{
    if (!points || max == 0) return 0;

    size_t t = atomic_load_explicit(&tail, memory_order_relaxed);
    size_t h = atomic_load_explicit(&head, memory_order_acquire);  // Sync with producer
    
    // Calculate available data
    size_t available;
    if (h >= t)
        available = h - t;
    else
        available = FRAME_BUFFER_SIZE - t + h;
    
    size_t to_read = (max < available) ? max : available;
    if (to_read == 0) return 0;
    
    // Bulk copy using memcpy - handle wrap-around
    size_t first_chunk = FRAME_BUFFER_SIZE - t;
    if (first_chunk > to_read) first_chunk = to_read;
    
    memcpy(points, &buffer[t], first_chunk * sizeof(laser_point_t));
    
    if (to_read > first_chunk) {
        // Wrap around from beginning
        memcpy(&points[first_chunk], &buffer[0], (to_read - first_chunk) * sizeof(laser_point_t));
    }
    
    // Advance tail
    size_t new_tail = (t + to_read) % FRAME_BUFFER_SIZE;
    
    // Publish new tail position to producer
    atomic_store_explicit(&tail, new_tail, memory_order_release);
    return to_read;
}

bool frame_buffer_can_fit(size_t count)
{
    size_t h = atomic_load_explicit(&head, memory_order_relaxed);
    size_t t = atomic_load_explicit(&tail, memory_order_acquire);
    
    size_t free_space;
    if (h >= t)
        free_space = FRAME_BUFFER_SIZE - (h - t) - 1;
    else
        free_space = t - h - 1;
    
    return count <= free_space;
}

size_t frame_buffer_level(void)
{
    size_t h = atomic_load_explicit(&head, memory_order_acquire);
    size_t t = atomic_load_explicit(&tail, memory_order_acquire);

    if (h >= t)
        return h - t;
    else
        return FRAME_BUFFER_SIZE - t + h;
}

void frame_buffer_clear(void)
{
    // Reset both to 0 - safe because clear is called when system is idle
    atomic_store_explicit(&tail, 0, memory_order_relaxed);
    atomic_store_explicit(&head, 0, memory_order_release);
}
