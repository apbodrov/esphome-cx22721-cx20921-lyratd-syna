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
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  media_hal_t *get_hal() { return this->hal_; }

 private:
  media_hal_t *hal_{nullptr};
};

}  // namespace cx_audio
}  // namespace esphome
