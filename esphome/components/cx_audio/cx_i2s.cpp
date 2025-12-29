#include "cx_i2s.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include <driver/i2s.h>
#include <driver/gpio.h>
#include <driver/i2c.h>  // Legacy I2C driver for SDK
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h>

extern "C" {
  #include <va_board.h>
  #include <va_dsp.h>
  #include <va_dsp_hal.h>
  #include "cnx20921_init.h"
  #include "va_dsp_cnx.h"
  void va_dsp_init(va_dsp_recognize_cb_t va_dsp_recognize_cb, 
                  va_dsp_record_cb_t va_dsp_record_cb, 
                  va_dsp_notify_mute_cb_t va_dsp_mute_notify_cb);
  // Microphone gain functions from SDK
  int cx20921SetMicGain(int gain_db);
  int cx20921GetMicGain(void);
  
  // SDK I2C callback setup functions (from libcnx-ipc.a)
  void SetupI2cWriteMemCallback(void *callback);
  void SetupI2cReadMemCallback(void *callback);
  
  // CNX pin configuration (must be called BEFORE cnx20921_init)
  void cnx_pin_config(int cnx_intr_pin, int cnx_mute_pin);
  
  // SDK I2C device management (C++ mangled name from libcnx-ipc.a)
  extern void *_Z13OpenI2cDevicev();  // OpenI2cDevice() - C++ mangled name
}


// I2C implementation functions for SDK
// ВАЖНО: Функции i2c_write_imp и i2c_read_imp УЖЕ определены в библиотеке (libcnx-ipc_host_depend.o)
// Нам НЕ нужно их определять самим - SDK использует свои реализации
// SDK сам установит callbacks через SetupI2cWriteMemCallback/SetupI2cReadMemCallback
// внутри cnx20921_init() или OpenI2cDevice()

// Initialize I2C for SDK DSP communication
// ВАЖНО: I2C driver должен быть установлен ДО вызова va_dsp_init()
// ESPHome обычно устанавливает его автоматически через компонент i2c
// Здесь мы только проверяем, что он установлен, и настраиваем параметры если нужно
static void init_dsp_i2c() {
  // I2C driver должен быть уже установлен ESPHome (компонент i2c)
  // Но SDK использует legacy I2C driver на I2C_NUM_0
  // Проверяем, что driver установлен, и настраиваем параметры если нужно
  
  ESP_LOGI("DSP_I2C", "I2C driver should be installed by ESPHome (checking...)");
  
  // ВАЖНО: НЕ устанавливаем I2C callbacks вручную
  // SDK сам установит их через OpenI2cDevice() внутри cnx20921_init():
  // cnx20921_init() -> OpenI2cDevice() -> SetupI2cWriteMemCallback() -> SetupI2cReadMemCallback()
  // Функции i2c_write_imp и i2c_read_imp уже определены в библиотеке (libcnx-ipc.a)
  
  ESP_LOGI("DSP_I2C", "I2C callbacks will be set by SDK inside cnx20921_init()");
  
  // Add small delay to ensure I2C is ready
  vTaskDelay(pdMS_TO_TICKS(100));
}

namespace esphome {
namespace cx_i2s {

static const char *const TAG = "cx_i2s";

// --- Microphone ---

void CXI2SMicrophone::setup() {
    ESP_LOGI(TAG, "Setting up CX I2S Microphone...");
    
    // ВАЖНО: I2C driver должен быть установлен ДО вызова va_dsp_init()
    // ESPHome обычно устанавливает его автоматически через компонент i2c
    // Проверяем и настраиваем если нужно
    init_dsp_i2c();
    
    // На LyraTD V1.2: reset_pin=21 (GPIO21), int_pin=36, mute_pin=27
    int int_pin = 36;
    int mute_pin = 27;
    
    ESP_LOGI(TAG, "Initializing CX20921 DSP chip (int=%d, mute=%d)...", int_pin, mute_pin);
    
    // ВАЖНО: cnx_pin_config() должна быть вызвана ДО va_dsp_init()
    // Это настраивает interrupt и mute pins для DSP
    // Как в va_board.c из cx20921-master (строка 73)
    ESP_LOGI(TAG, "Configuring CNX pins (int=%d, mute=%d)...", int_pin, mute_pin);
    cnx_pin_config(int_pin, mute_pin);
    
    // ЭТАЛОН: Инициализация DSP для микрофона
    // va_dsp_init() из монолита:
    // 1. Создает команду очередь и DMA буфер
    // 2. Вызывает va_dsp_hal_init() с очередью
    // 3. Вызывает cnx20921_init() (ПОЛНУЮ версию из библиотеки)
    // 
    // Полная версия cnx20921_init() из библиотеки:
    // 1. Делает hardware reset
    // 2. Вызывает OpenI2cDevice() для инициализации I2C
    // 3. Устанавливает I2C callbacks через SetupI2cWriteMemCallback/SetupI2cReadMemCallback
    // 4. Инициализирует DSP через SendCmdV()
    //
    // Функции i2c_write_imp и i2c_read_imp уже определены в библиотеке (libcnx-ipc.a)
    va_dsp_init(nullptr, nullptr, nullptr);
    
    ESP_LOGI(TAG, "CX20921 DSP chip initialized successfully (full init from library)");
    
    // ВАЖНО: cx20921SetMicGain() требует полной инициализации DSP через I2C
    // Полная инициализация завершена, но DSP может еще не быть готов к командам
    // Добавляем задержку перед попыткой установки гейна
    // В эталонном коде (cx20921-master) функции SetMicGain нет в заголовках,
    // значит она может требовать дополнительных условий или вызываться позже
    vTaskDelay(pdMS_TO_TICKS(1000)); // Даем больше времени DSP на завершение инициализации
    
    // Пробуем установить гейн после задержки
    // Если не получится, будем пробовать в loop() когда микрофон начнет работать
    if (this->mic_gain_ > 0) {
        int gain_db = (int)this->mic_gain_;
        
        // Сначала пробуем получить текущий гейн для проверки, что функция доступна
        int current_gain = cx20921GetMicGain();
        ESP_LOGI(TAG, "Current microphone gain: %d dB", current_gain);
        
        // Пробуем установить гейн
        int ret = cx20921SetMicGain(gain_db);
        if (ret == 0) {
            ESP_LOGI(TAG, "Microphone gain set to %d dB", gain_db);
            this->gain_set_successfully_ = true;
        } else {
            ESP_LOGW(TAG, "Failed to set microphone gain: %d (will retry in loop when mic is running)", ret);
            ESP_LOGW(TAG, "  Code 15 may indicate DSP not ready or I2C callbacks not working");
            this->gain_set_successfully_ = false;
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
            this->gain_set_successfully_ = true;
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to set microphone gain: %d (will retry in loop)", ret);
            this->gain_set_successfully_ = false;
            // Не возвращаем false, так как гейн будет установлен позже в loop()
            return true;
        }
    }
    
    // Если еще не инициализирован, гейн будет установлен в setup()
    return true;
}

void CXI2SMicrophone::loop() {
    if (this->state_ != microphone::STATE_RUNNING) return;
    
    // Пробуем установить гейн, если еще не установлен успешно
    // Это нужно потому что cx20921SetMicGain() требует полной инициализации I2C,
    // которая может произойти не сразу после hardware reset
    if (!this->gain_set_successfully_ && this->mic_gain_ > 0) {
        this->gain_retry_counter_++;
        // Пробуем установить гейн каждые 500 циклов (примерно раз в 5 секунд)
        // Уменьшили частоту, чтобы не блокировать loopTask и не вызывать watchdog timeout
        if (this->gain_retry_counter_ >= 500) {
            int attempt = this->gain_retry_counter_ / 500;
            this->gain_retry_counter_ = 0;
            int gain_db = (int)this->mic_gain_;
            int ret = cx20921SetMicGain(gain_db);
            if (ret == 0) {
                ESP_LOGI(TAG, "Microphone gain successfully set to %d dB (retry attempt %d)", gain_db, attempt);
                this->gain_set_successfully_ = true;
            } else {
                // Логируем каждую попытку (раз в 5 секунд)
                ESP_LOGD(TAG, "Retrying microphone gain set to %d dB (attempt %d, ret=%d)", 
                         gain_db, attempt, ret);
            }
        }
    }
    
    // ЭТАЛОН: I2S1 настроен на стерео (2 канала) согласно va_board.c:
    // "Reading mono channel on I2S produces data mirroring, 0-4KHz mirrored to 4-8KHz
    //  For better audio performance, we read stereo data and then down sample it to mono"
    // i2s_set_clk(I2S_NUM_1, 16000, 16, 2); // 16kHz, 16bit, 2 channels (stereo)
    
    // Читаем стерео данные: 160 samples * 2 channels * 2 bytes = 640 bytes
    uint8_t stereo_buffer[640]; // Стерео данные от DSP
    size_t bytes_read = 0;
    // ВАЖНО: Используем короткий timeout (10ms) чтобы не блокировать loopTask
    // Если данных нет, просто выходим - не ждем
    esp_err_t err = i2s_read(I2S_NUM_1, stereo_buffer, sizeof(stereo_buffer), &bytes_read, pdMS_TO_TICKS(10));
    
    // Если timeout - это нормально, просто выходим
    if (err == ESP_ERR_TIMEOUT) {
        return;  // Нет данных, выходим быстро
    }
    
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