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

void CXAudio::setup() {
  ESP_LOGI(TAG, "Setting up Synaptics Audio Component...");

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
