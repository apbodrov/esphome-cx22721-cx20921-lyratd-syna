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
  // Microphone gain functions from SDK
  int cx20921SetMicGain(int gain_db);
  int cx20921GetMicGain(void);
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
    
    // Устанавливаем гейн микрофона после инициализации DSP
    if (this->mic_gain_ > 0) {
        int gain_db = (int)this->mic_gain_;
        int ret = cx20921SetMicGain(gain_db);
        if (ret == 0) {
            ESP_LOGI(TAG, "Microphone gain set to %d dB", gain_db);
        } else {
            ESP_LOGW(TAG, "Failed to set microphone gain: %d", ret);
        }
    }
    
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

bool CXI2SMicrophone::set_mic_gain(float mic_gain) {
    // CX20921 поддерживает гейн от 0 до 30 dB (обычно)
    // Ограничиваем диапазон для безопасности
    this->mic_gain_ = clamp<float>(mic_gain, 0.0f, 30.0f);
    
    // Если микрофон уже инициализирован (не в состоянии STOPPED), применяем гейн сразу
    if (this->state_ != microphone::STATE_STOPPED && !this->is_failed()) {
        int gain_db = (int)this->mic_gain_;
        int ret = cx20921SetMicGain(gain_db);
        if (ret == 0) {
            ESP_LOGI(TAG, "Microphone gain updated to %d dB", gain_db);
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to set microphone gain: %d", ret);
            return false;
        }
    }
    
    // Если еще не инициализирован, гейн будет установлен в setup()
    return true;
}

void CXI2SMicrophone::loop() {
    if (this->state_ != microphone::STATE_RUNNING) return;
    
    // ЭТАЛОН: I2S1 настроен на стерео (2 канала) согласно va_board.c:
    // "Reading mono channel on I2S produces data mirroring, 0-4KHz mirrored to 4-8KHz
    //  For better audio performance, we read stereo data and then down sample it to mono"
    // i2s_set_clk(I2S_NUM_1, 16000, 16, 2); // 16kHz, 16bit, 2 channels (stereo)
    
    // Читаем стерео данные: 160 samples * 2 channels * 2 bytes = 640 bytes
    uint8_t stereo_buffer[640]; // Стерео данные от DSP
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_NUM_1, stereo_buffer, sizeof(stereo_buffer), &bytes_read, pdMS_TO_TICKS(10));
    
    if (err == ESP_OK && bytes_read > 0) {
        // Конвертируем стерео в моно: берем только левый канал (первые 2 байта каждого сэмпла)
        // Формат стерео: [L0 L1 R0 R1 L2 L3 R2 R3 ...] где L/R - left/right каналы
        size_t stereo_samples = bytes_read / 4; // Каждый стерео-сэмпл = 4 байта (2 байта L + 2 байта R)
        size_t mono_bytes = stereo_samples * 2; // Моно = 2 байта на сэмпл
        
        std::vector<uint8_t> mono_data;
        mono_data.reserve(mono_bytes);
        
        // Извлекаем левый канал (первые 2 байта каждого стерео-сэмпла)
        for (size_t i = 0; i < stereo_samples; i++) {
            mono_data.push_back(stereo_buffer[i * 4 + 0]);     // L low byte
            mono_data.push_back(stereo_buffer[i * 4 + 1]);     // L high byte
        }
        
        this->data_callbacks_.call(mono_data);
        
        // Логируем периодически для отладки
        static int log_counter = 0;
        if (++log_counter >= 100) {  // Каждые ~100 вызовов (примерно раз в секунду)
            log_counter = 0;
            ESP_LOGD(TAG, "Microphone: read %zu stereo bytes -> %zu mono bytes from I2S1", bytes_read, mono_bytes);
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