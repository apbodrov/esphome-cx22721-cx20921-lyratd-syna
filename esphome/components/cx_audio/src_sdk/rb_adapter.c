// Адаптер для ring buffer функций SDK
// Реализуем простой ring buffer на основе FreeRTOS queue

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>

// Ring buffer handle type
typedef void* rb_handle_t;

#define TAG "rb_adapter"

// Структура ring buffer
typedef struct {
    char name[32];
    uint8_t* buffer;
    uint32_t size;
    uint32_t read_pos;
    uint32_t write_pos;
    uint32_t available;
    bool aborted;
} esphome_rb_t;

rb_handle_t esphome_rb_init(const char* name, uint32_t size) {
    if (!name || size == 0) {
        ESP_LOGE(TAG, "Invalid parameters: name=%p, size=%u", name, size);
        return NULL;
    }

    esphome_rb_t* rb = (esphome_rb_t*)malloc(sizeof(esphome_rb_t));
    if (!rb) {
        ESP_LOGE(TAG, "Failed to allocate rb structure");
        return NULL;
    }

    rb->buffer = (uint8_t*)malloc(size);
    if (!rb->buffer) {
        ESP_LOGE(TAG, "Failed to allocate rb buffer: size=%u", size);
        free(rb);
        return NULL;
    }

    strncpy(rb->name, name, sizeof(rb->name) - 1);
    rb->name[sizeof(rb->name) - 1] = '\0';
    rb->size = size;
    rb->read_pos = 0;
    rb->write_pos = 0;
    rb->available = 0;
    rb->aborted = false;

    ESP_LOGI(TAG, "Ring buffer '%s' initialized: size=%u", rb->name, size);
    return (rb_handle_t)rb;
}

int esphome_rb_read(rb_handle_t handle, uint8_t* buf, int len, uint32_t timeout) {
    if (!handle || !buf || len <= 0) {
        return -1;
    }

    esphome_rb_t* rb = (esphome_rb_t*)handle;
    
    if (rb->aborted) {
        return -1;
    }

    TickType_t ticks = timeout == portMAX_DELAY ? portMAX_DELAY : pdMS_TO_TICKS(timeout);
    TickType_t start = xTaskGetTickCount();

    // Ждем данных
    while (rb->available == 0 && !rb->aborted) {
        if (xTaskGetTickCount() - start > ticks) {
            return 0; // Timeout
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (rb->aborted) {
        return -1;
    }

    // Читаем данные
    int to_read = len < (int)rb->available ? len : (int)rb->available;
    for (int i = 0; i < to_read; i++) {
        buf[i] = rb->buffer[rb->read_pos];
        rb->read_pos = (rb->read_pos + 1) % rb->size;
        rb->available--;
    }

    return to_read;
}

int esphome_rb_write(rb_handle_t handle, uint8_t* buf, int len, uint32_t timeout) {
    if (!handle || !buf || len <= 0) {
        return -1;
    }

    esphome_rb_t* rb = (esphome_rb_t*)handle;
    
    if (rb->aborted) {
        return -1;
    }

    TickType_t ticks = timeout == portMAX_DELAY ? portMAX_DELAY : pdMS_TO_TICKS(timeout);
    TickType_t start = xTaskGetTickCount();

    // Ждем свободного места
    uint32_t free_space = rb->size - rb->available;
    while (free_space < (uint32_t)len && !rb->aborted) {
        if (xTaskGetTickCount() - start > ticks) {
            return 0; // Timeout
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        free_space = rb->size - rb->available;
    }

    if (rb->aborted) {
        return -1;
    }

    // Записываем данные
    int to_write = len < (int)free_space ? len : (int)free_space;
    for (int i = 0; i < to_write; i++) {
        rb->buffer[rb->write_pos] = buf[i];
        rb->write_pos = (rb->write_pos + 1) % rb->size;
        rb->available++;
    }

    return to_write;
}

ssize_t esphome_rb_available(rb_handle_t handle) {
    if (!handle) {
        return -1;
    }
    esphome_rb_t* rb = (esphome_rb_t*)handle;
    return (ssize_t)rb->available;
}

void esphome_rb_abort(rb_handle_t handle) {
    if (!handle) {
        return;
    }
    esphome_rb_t* rb = (esphome_rb_t*)handle;
    rb->aborted = true;
    ESP_LOGI(TAG, "Ring buffer '%s' aborted", rb->name);
}

void esphome_rb_reset(rb_handle_t handle) {
    if (!handle) {
        return;
    }
    esphome_rb_t* rb = (esphome_rb_t*)handle;
    rb->read_pos = 0;
    rb->write_pos = 0;
    rb->available = 0;
    rb->aborted = false;
    ESP_LOGI(TAG, "Ring buffer '%s' reset", rb->name);
}
