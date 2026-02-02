#pragma once
#include "esphome/core/component.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#ifndef xSemaphoreHandle
#define xSemaphoreHandle SemaphoreHandle_t
#endif

extern "C" {
#include <media_hal.h>
#include <audio_board.h>
#include <va_board.h>
}

namespace esphome {
namespace cx_audio {

class CXAudio : public Component {
 public:
  void setup() override;
  void dump_config() override;

  void set_use_firmware(bool use) { this->use_firmware_ = use; }
  bool is_use_firmware() const { return this->use_firmware_; }

  static SemaphoreHandle_t get_i2c_semaphore();

 private:
  media_hal_t *hal_{nullptr};
  static SemaphoreHandle_t i2c_sem_;
  bool use_firmware_{false};
};

}  // namespace cx_audio
}  // namespace esphome
