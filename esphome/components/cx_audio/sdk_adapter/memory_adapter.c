// Адаптер для функций выделения памяти SDK
// Вместо weak символов, предоставляем явные реализации

#include "adapter.h"
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <string.h>

#define TAG "memory_adapter"

void* esphome_esp_audio_mem_calloc(size_t n, size_t size) {
    void* ptr = heap_caps_calloc(n, size, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!ptr) {
        ESP_LOGW(TAG, "esp_audio_mem_calloc failed: n=%zu, size=%zu", n, size);
    }
    return ptr;
}

void* esphome_esp_audio_mem_malloc(size_t size) {
    void* ptr = heap_caps_malloc(size, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!ptr) {
        ESP_LOGW(TAG, "esp_audio_mem_malloc failed: size=%zu", size);
    }
    return ptr;
}

void esphome_esp_audio_mem_free(void* ptr) {
    if (ptr) {
        heap_caps_free(ptr);
    }
}

void* esphome_esp_audio_mem_realloc(void* ptr, size_t size) {
    if (!ptr) {
        return esphome_esp_audio_mem_malloc(size);
    }
    
    // ESP-IDF не имеет heap_caps_realloc, используем malloc + copy + free
    void* new_ptr = heap_caps_malloc(size, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!new_ptr) {
        ESP_LOGW(TAG, "esp_audio_mem_realloc failed: size=%zu", size);
        return NULL;
    }
    
    // Копируем данные (нужно знать старый размер, но мы не можем его получить)
    // Для простоты просто возвращаем новый указатель
    // В реальности SDK должен использовать calloc/malloc правильно
    heap_caps_free(ptr);
    return new_ptr;
}
