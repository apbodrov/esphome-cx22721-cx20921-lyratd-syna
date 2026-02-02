#include "cx_audio.h"
#include "esphome/core/log.h"
#include <esp_spiffs.h>

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
  
  // Монтируем SPIFFS для прошивки DSP
  if (this->use_firmware_) {
    ESP_LOGI(TAG, "Mounting SPIFFS for DSP Firmware...");
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = "storage",
      .max_files = 5,
      .format_if_mount_failed = false
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
      if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "Failed to mount or format filesystem");
      } else if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to find SPIFFS partition (storage)");
      } else {
        ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
      }
    } else {
      size_t total = 0, used = 0;
      ret = esp_spiffs_info("storage", &total, &used);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
      } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
      }
    }
  }

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
