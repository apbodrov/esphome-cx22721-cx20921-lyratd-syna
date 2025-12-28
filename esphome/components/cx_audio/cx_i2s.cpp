#include "cx_i2s.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include <driver/i2s.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h>

extern "C" {
  #include <va_board.h>
  #include <va_dsp.h>
  #include <va_dsp_hal.h>
  #include "cnx20921_init.h"
  void va_dsp_init(va_dsp_recognize_cb_t va_dsp_recognize_cb, 
                  va_dsp_record_cb_t va_dsp_record_cb, 
                  va_dsp_notify_mute_cb_t va_dsp_mute_notify_cb);
}

namespace esphome {
namespace cx_i2s {

static const char *const TAG = "cx_i2s";

// --- Microphone ---

void CXI2SMicrophone::setup() {
    ESP_LOGI(TAG, "Setting up CX I2S Microphone...");
    
    // ЭТАЛОН: Инициализация DSP для микрофона
    // va_dsp_init() из монолита уже вызывает va_dsp_hal_init() внутри себя
    // и создает очередь команд и DMA буфер
    va_dsp_init(nullptr, nullptr, nullptr);
    
    // Инициализируем сам DSP чип CX20921 после va_dsp_init()
    // На LyraTD V1.2: reset_pin=21 (GPIO21), int_pin=36, mute_pin=27
    gpio_num_t reset_pin = GPIO_NUM_21;  // ЭТАЛОН: GPIO21 для V1.2
    int int_pin = 36;
    int mute_pin = 27;
    
    ESP_LOGI(TAG, "Initializing CX20921 DSP chip (reset=%d, int=%d, mute=%d)...", reset_pin, int_pin, mute_pin);
    
    // Настраиваем reset pin (если еще не настроен)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << reset_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(reset_pin, 1);  // Держим включенным
    
    // Инициализируем DSP чип через cnx20921_init()
    // Эта функция уже определена в va_patch.cpp как stub (hardware reset)
    // Полная инициализация будет вызвана из va_dsp_init() через va_dsp_hal_init()
    SemaphoreHandle_t semph = xSemaphoreCreateMutex();
    esp_err_t ret = cnx20921_init(semph, int_pin, mute_pin, NO_FLASH_FW);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "cnx20921_init failed: %s", esp_err_to_name(ret));
        this->mark_failed();
        return;
    }
    
    ESP_LOGI(TAG, "CX20921 DSP chip initialized successfully");
    ESP_LOGI(TAG, "Microphone setup complete");
    
    // Автоматически запускаем микрофон после инициализации
    // Это нужно для работы sound_level компонента
    this->start();
}

void CXI2SMicrophone::start() {
    this->state_ = microphone::STATE_RUNNING;
    ESP_LOGI(TAG, "Microphone started");
}

void CXI2SMicrophone::stop() {
    this->state_ = microphone::STATE_STOPPED;
    ESP_LOGI(TAG, "Microphone stopped");
}

void CXI2SMicrophone::loop() {
    if (this->state_ != microphone::STATE_RUNNING) return;
    
    // ЭТАЛОН: Прямое чтение из I2S1 (как в рабочем коммите da36104)
    // DSP шлет обработанный голос в I2S1
    uint8_t buffer[320]; // 160 samples * 2 bytes
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_NUM_1, buffer, sizeof(buffer), &bytes_read, pdMS_TO_TICKS(10));
    
    if (err == ESP_OK && bytes_read > 0) {
        std::vector<uint8_t> data(buffer, buffer + bytes_read);
        this->data_callbacks_.call(data);
        
        // Логируем периодически для отладки
        static int log_counter = 0;
        if (++log_counter >= 100) {  // Каждые ~100 вызовов (примерно раз в секунду)
            log_counter = 0;
            ESP_LOGD(TAG, "Microphone: received %zu bytes from I2S1", bytes_read);
        }
    } else if (err != ESP_OK) {
        // Ошибка чтения
        static int error_counter = 0;
        if (++error_counter >= 1000) {  // Логируем только раз в ~10 секунд
            error_counter = 0;
            ESP_LOGW(TAG, "Microphone: i2s_read error: %s", esp_err_to_name(err));
        }
    }
}

// --- Speaker ---

void CXI2SSpeaker::setup() {
    ESP_LOGI(TAG, "Setting up CX I2S Speaker...");
}

void CXI2SSpeaker::start() {
    this->state_ = speaker::STATE_RUNNING;
}

void CXI2SSpeaker::stop() {
    this->state_ = speaker::STATE_STOPPED;
}

void CXI2SSpeaker::loop() {}

size_t CXI2SSpeaker::play(const uint8_t *data, size_t length) {
    size_t bytes_written = 0;
    // Пишем в I2S0 (Кодек)
    i2s_write(I2S_NUM_0, data, length, &bytes_written, pdMS_TO_TICKS(10));
    return bytes_written;
}

bool CXI2SSpeaker::has_buffered_data() const { return false; }

}  // namespace cx_i2s
}  // namespace esphome