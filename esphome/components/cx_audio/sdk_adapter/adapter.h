#ifndef ESPHOME_SDK_ADAPTER_H
#define ESPHOME_SDK_ADAPTER_H

#include <va_dsp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <stddef.h>

// Ring buffer handle type
typedef void* rb_handle_t;

// ============================================================================
// va_dsp адаптеры
// ============================================================================

/**
 * @brief Инициализация va_dsp с правильным порядком создания очереди
 * 
 * Эта функция гарантирует, что очередь создана ДО вызова va_boot_dsp_signal()
 * из библиотеки SDK.
 * 
 * @param recognize_cb Callback для распознавания wake word
 * @param record_cb Callback для записи аудио
 * @param mute_cb Callback для уведомления о mute статусе
 */
void esphome_va_dsp_init(va_dsp_recognize_cb_t recognize_cb,
                         va_dsp_record_cb_t record_cb,
                         va_dsp_notify_mute_cb_t mute_cb);

/**
 * @brief Получить очередь команд va_dsp (для внутреннего использования)
 */
QueueHandle_t esphome_va_dsp_get_queue(void);

// ============================================================================
// Memory адаптеры (вместо weak esp_audio_mem_*)
// ============================================================================

void* esphome_esp_audio_mem_calloc(size_t n, size_t size);
void* esphome_esp_audio_mem_malloc(size_t size);
void esphome_esp_audio_mem_free(void* ptr);
void* esphome_esp_audio_mem_realloc(void* ptr, size_t size);

// ============================================================================
// Ring buffer адаптеры (если нужны библиотеке)
// ============================================================================

rb_handle_t esphome_rb_init(const char* name, uint32_t size);
int esphome_rb_read(rb_handle_t handle, uint8_t* buf, int len, uint32_t timeout);
int esphome_rb_write(rb_handle_t handle, uint8_t* buf, int len, uint32_t timeout);
ssize_t esphome_rb_available(rb_handle_t handle);
void esphome_rb_abort(rb_handle_t handle);
void esphome_rb_reset(rb_handle_t handle);

#endif // ESPHOME_SDK_ADAPTER_H
