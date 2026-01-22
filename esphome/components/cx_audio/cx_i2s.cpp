#include "cx_i2s.h"
#include "cx_audio.h"
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
// Используем адаптеры вместо прямых вызовов SDK
void esphome_va_dsp_init(va_dsp_recognize_cb_t recognize_cb, va_dsp_record_cb_t record_cb,
                         va_dsp_notify_mute_cb_t mute_cb);
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

  // Используем адаптер esphome_va_dsp_init вместо прямого va_dsp_init
  // Адаптер гарантирует правильный порядок инициализации: очередь создается ДО va_boot_dsp_signal()
  esphome_va_dsp_init(nullptr, nullptr, nullptr);

  // Инициализируем сам DSP чип CX20921 после va_dsp_init()
  // На LyraTD V1.2: reset_pin=21 (GPIO21), int_pin=36, mute_pin=-1 (освобождаем 27 для LED)
  gpio_num_t reset_pin = GPIO_NUM_21;  // ЭТАЛОН: GPIO21 для V1.2
  int int_pin = 36;
  int mute_pin = -1;

  ESP_LOGI(TAG, "Initializing CX20921 DSP chip (reset=%d, int=%d, mute=%d)...", reset_pin, int_pin, mute_pin);

  // Настраиваем reset pin
  gpio_config_t io_conf = {.pin_bit_mask = (1ULL << reset_pin),
                           .mode = GPIO_MODE_OUTPUT,
                           .pull_up_en = GPIO_PULLUP_DISABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);

  // Hardware Reset Sequence
  ESP_LOGI(TAG, "Performing hardware reset of CX20921...");
  gpio_set_level(reset_pin, 0);
  vTaskDelay(pdMS_TO_TICKS(50));
  gpio_set_level(reset_pin, 1);
  vTaskDelay(pdMS_TO_TICKS(200));

  // Инициализируем DSP чип через cnx20921_init()
  // Эта функция определена в va_patch.cpp как wrapper, который вызывает
  // реальную функцию cnx20921_init_unused() из библиотеки
  // Полная инициализация включает:
  // 1. Hardware reset
  // 2. Инициализацию I2C через OpenI2cDevice()
  // 3. Установку I2C callbacks
  // 4. Инициализацию DSP через SendCmdV()
  SemaphoreHandle_t semph = cx_audio::CXAudio::get_i2c_semaphore();
  esp_err_t ret = cnx20921_init(semph, int_pin, mute_pin, NO_FLASH_FW);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "cnx20921_init failed: %s", esp_err_to_name(ret));
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "CX20921 DSP chip initialized successfully (I2C callbacks set)");

  // Даем время DSP на завершение инициализации перед установкой гейна
  vTaskDelay(pdMS_TO_TICKS(500));

  // Устанавливаем гейн микрофона после полной инициализации DSP
  int gain_db = (int) this->mic_gain_;
  int before_gain = cx20921GetMicGain();
  ESP_LOGI(TAG, "Microphone gain BEFORE setting: %d dB", before_gain);
  
  ESP_LOGI(TAG, "Setting microphone gain to %d dB...", gain_db);
  int ret_gain = cx20921SetMicGain(gain_db);
  
  int after_gain = cx20921GetMicGain();
  if (ret_gain == 0) {
    ESP_LOGI(TAG, "Microphone gain set to %d dB (confirmed: %d dB)", gain_db, after_gain);
  } else {
    ESP_LOGW(TAG, "Failed to set microphone gain: %d (I2C may not be ready). Current: %d dB", ret_gain, after_gain);
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
  // Расширяем диапазон: от ослабления (-50дБ) до мощного усиления (50дБ)
  this->mic_gain_ = clamp<float>(mic_gain, -50.0f, 50.0f);

  // Если микрофон уже инициализирован (не в состоянии STOPPED), применяем гейн сразу
  if (this->state_ != microphone::STATE_STOPPED && !this->is_failed()) {
    int gain_db = (int) this->mic_gain_;
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
  if (this->state_ != microphone::STATE_RUNNING)
    return;

  // Читаем стерео данные: 160 samples * 2 channels * 2 bytes = 640 bytes
  uint8_t stereo_buffer[640];
  size_t bytes_read = 0;
  esp_err_t err = i2s_read(I2S_NUM_1, stereo_buffer, sizeof(stereo_buffer), &bytes_read, pdMS_TO_TICKS(10));

  if (err == ESP_OK && bytes_read > 0) {
    // Конвертируем стерео в моно: берем только левый канал (первые 2 байта каждого сэмпла)
    size_t stereo_samples = bytes_read / 4;
    std::vector<uint8_t> mono_data;
    mono_data.reserve(stereo_samples * 2);

    for (size_t i = 0; i < stereo_samples; i++) {
      mono_data.push_back(stereo_buffer[i * 4 + 0]);  // Low byte
      mono_data.push_back(stereo_buffer[i * 4 + 1]);  // High byte
    }

    this->data_callbacks_.call(mono_data);

    static int log_counter = 0;
    if (++log_counter >= 100) {
      log_counter = 0;
      ESP_LOGD(TAG, "Microphone: read %zu stereo bytes -> %zu mono bytes from I2S1", bytes_read, mono_data.size());
    }
  }
}

// --- Speaker ---

void CXI2SSpeaker::setup() { ESP_LOGI(TAG, "Setting up CX I2S Speaker..."); }

void CXI2SSpeaker::start() { this->state_ = speaker::STATE_RUNNING; }

void CXI2SSpeaker::stop() { this->state_ = speaker::STATE_STOPPED; }

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