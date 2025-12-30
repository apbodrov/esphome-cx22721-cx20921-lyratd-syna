#include "cx_audio.h"
#include "esphome/core/log.h"

extern "C" {
#include <va_board.h>
#include <media_hal.h>

// Явное объявление функций для предотвращения C++ mangling
esp_err_t va_board_init(void);
media_hal_t *media_hal_get_handle(void);
}

namespace esphome {
namespace cx_audio {

static const char *const TAG = "cx_audio";

// Добавляем "защитный слой" вокруг указателя на семафор, чтобы защититься от 
// переполнения буферов в закрытой библиотеке SDK.
static uint8_t g_padding_before[256];
SemaphoreHandle_t CXAudio::i2c_sem_ = nullptr;
static uint8_t g_padding_after[256];

SemaphoreHandle_t CXAudio::get_i2c_semaphore() {
  if (i2c_sem_ == nullptr) {
    // Выделяем семафор в куче. Сам указатель i2c_sem_ защищен паддингом.
    i2c_sem_ = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "I2C Semaphore created at %p", i2c_sem_);
  }
  return i2c_sem_;
}

extern "C" SemaphoreHandle_t esphome_get_i2c_semaphore() {
  return esphome::cx_audio::CXAudio::get_i2c_semaphore();
}

void CXAudio::setup() {
  ESP_LOGI(TAG, "Setting up Synaptics Audio Component...");
  
  if (i2c_sem_ == nullptr) {
    i2c_sem_ = xSemaphoreCreateMutex();
  }

  // Вызовы теперь гарантированно идут к C-символам
  esp_err_t err = va_board_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "va_board_init failed: %d", err);
  }

  media_hal_t *hal = media_hal_get_handle();
  if (hal) {
    ESP_LOGI(TAG, "Media HAL handle obtained");
  }
}

void CXAudio::dump_config() { ESP_LOGCONFIG(TAG, "Synaptics CX20921 Audio"); }

}  // namespace cx_audio
}  // namespace esphome
