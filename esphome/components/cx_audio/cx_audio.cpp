#include "cx_audio.h"
#include "esphome/core/log.h"

extern "C" {
#include "va_board.h"
#include "media_hal.h"
}

namespace esphome {
namespace cx_audio {

static const char *const TAG = "cx_audio";

void CXAudio::setup() {
  ESP_LOGI(TAG, "Calling va_board_init() from SDK...");
  // Эта функция в SDK делает всё: I2C, I2S, Codec PWR, Registers.
  va_board_init();
  this->hal_ = media_hal_get_handle();
}

void CXAudio::dump_config() { ESP_LOGCONFIG(TAG, "CX Audio (Monolithic SDK Blob)"); }

}  // namespace cx_audio
}  // namespace esphome
