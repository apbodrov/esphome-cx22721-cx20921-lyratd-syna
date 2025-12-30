// Адаптер для va_dsp функций SDK
// Гарантирует правильный порядок инициализации: очередь создается ДО va_boot_dsp_signal()

#include "adapter.h"
#include <va_dsp.h>
#include <va_dsp_hal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_audio_nvs.h>
#include <string.h>

#define TAG "va_dsp_adapter"
#define DSP_NVS_NAMESPACE "dsp"

#define AUDIO_BUF_SIZE 4096
#define EVENTQ_LENGTH 10
#define STACK_SIZE (6 * 1024)
#define DSP_NVS_KEY "dsp_mute"

// Внутренняя структура данных va_dsp (наша версия)
static struct {
    va_dsp_record_cb_t record_cb;
    va_dsp_recognize_cb_t recognize_cb;
    va_dsp_notify_mute_cb_t mute_cb;
    QueueHandle_t cmd_queue;
    TaskHandle_t task_handle;
    uint8_t* audio_buf;
    bool initialized;
    bool va_dsp_booted;
} esphome_va_dsp_data = {0};

// Внешняя функция из библиотеки SDK
extern void va_boot_dsp_signal(void);
extern void va_dsp_mic_mute(bool mute);

// Внутренняя функция для обработки событий DSP (упрощенная версия)
static void va_dsp_task(void* arg) {
    struct dsp_event_data event_data;
    while (1) {
        if (xQueueReceive(esphome_va_dsp_data.cmd_queue, &event_data, portMAX_DELAY) == pdTRUE) {
            switch (event_data.event) {
                case GET_AUDIO: {
                    if (esphome_va_dsp_data.record_cb && esphome_va_dsp_data.audio_buf) {
                        int read_len = va_dsp_hal_stream_audio(esphome_va_dsp_data.audio_buf, AUDIO_BUF_SIZE, portMAX_DELAY);
                        if (read_len > 0) {
                            esphome_va_dsp_data.record_cb(esphome_va_dsp_data.audio_buf, read_len);
                            // Продолжаем чтение
                            struct dsp_event_data new_event = {.event = GET_AUDIO};
                            xQueueSend(esphome_va_dsp_data.cmd_queue, &new_event, portMAX_DELAY);
                        }
                    }
                    break;
                }
                case START_MIC:
                    va_dsp_hal_start_capture();
                    break;
                case STOP_MIC:
                    va_dsp_hal_stop_capture();
                    break;
                case MUTE:
                    va_dsp_hal_mic_mute();
                    if (esphome_va_dsp_data.mute_cb) {
                        esphome_va_dsp_data.mute_cb(true);
                    }
                    break;
                case UNMUTE:
                    va_dsp_hal_mic_unmute();
                    if (esphome_va_dsp_data.mute_cb) {
                        esphome_va_dsp_data.mute_cb(false);
                    }
                    break;
                default:
                    ESP_LOGD(TAG, "Unhandled event: %d", event_data.event);
                    break;
            }
        }
    }
}

void esphome_va_dsp_init(va_dsp_recognize_cb_t recognize_cb,
                         va_dsp_record_cb_t record_cb,
                         va_dsp_notify_mute_cb_t mute_cb) {
    if (esphome_va_dsp_data.initialized) {
        ESP_LOGW(TAG, "va_dsp already initialized");
        return;
    }

    ESP_LOGI(TAG, "Initializing va_dsp adapter...");

    // Сохраняем callbacks
    esphome_va_dsp_data.recognize_cb = recognize_cb;
    esphome_va_dsp_data.record_cb = record_cb;
    esphome_va_dsp_data.mute_cb = mute_cb;

    // КРИТИЧНО: Создаем очередь ПЕРЕД вызовом va_boot_dsp_signal()
    esphome_va_dsp_data.cmd_queue = xQueueCreate(EVENTQ_LENGTH, sizeof(struct dsp_event_data));
    if (!esphome_va_dsp_data.cmd_queue) {
        ESP_LOGE(TAG, "Failed to create command queue");
        return;
    }
    ESP_LOGI(TAG, "Command queue created");

    // Выделяем буфер для аудио
    esphome_va_dsp_data.audio_buf = heap_caps_calloc(1, AUDIO_BUF_SIZE, MALLOC_CAP_DMA);
    if (!esphome_va_dsp_data.audio_buf) {
        ESP_LOGE(TAG, "Failed to allocate audio buffer");
        return;
    }
    ESP_LOGI(TAG, "Audio buffer allocated");

    // Инициализируем va_dsp_hal с нашей очередью
    esp_err_t ret = va_dsp_hal_init(esphome_va_dsp_data.cmd_queue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "va_dsp_hal_init failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "va_dsp_hal initialized");

    // Создаем задачу для обработки событий DSP
    StackType_t* task_stack = (StackType_t*)heap_caps_calloc(1, STACK_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    static StaticTask_t task_buf;
    esphome_va_dsp_data.task_handle = xTaskCreateStatic(
        va_dsp_task,
        "va_dsp",
        STACK_SIZE,
        NULL,
        CONFIG_ESP32_PTHREAD_TASK_PRIO_DEFAULT,
        task_stack,
        &task_buf
    );
    if (!esphome_va_dsp_data.task_handle) {
        ESP_LOGE(TAG, "Failed to create va_dsp task");
        return;
    }
    ESP_LOGI(TAG, "va_dsp task created");

    // Проверяем сохраненный mute статус
    int8_t dsp_mute_en = 0;
    if (esp_audio_nvs_get_i8(DSP_NVS_NAMESPACE, DSP_NVS_KEY, &dsp_mute_en) == ESP_OK) {
        if (dsp_mute_en) {
            va_dsp_mic_mute(true);
            if (esphome_va_dsp_data.mute_cb) {
                esphome_va_dsp_data.mute_cb(true);
            }
        }
    }

    esphome_va_dsp_data.initialized = true;
    esphome_va_dsp_data.va_dsp_booted = true;

    ESP_LOGI(TAG, "Queue created, calling va_boot_dsp_signal() from SDK...");

    // КРИТИЧНО: Теперь очередь создана, можно безопасно вызывать va_boot_dsp_signal()
    va_boot_dsp_signal();

    ESP_LOGI(TAG, "va_dsp adapter initialized successfully");
}

QueueHandle_t esphome_va_dsp_get_queue(void) {
    return esphome_va_dsp_data.cmd_queue;
}
