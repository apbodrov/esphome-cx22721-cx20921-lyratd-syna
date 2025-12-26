#include "cx_i2s.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"

extern "C" {
  #include <va_board.h>
  #include <va_dsp.h>
  
  // Используем ванильную функцию va_dsp_init, так как она есть в монолите
  void va_dsp_init(va_dsp_recognize_cb_t va_dsp_recognize_cb, 
                  va_dsp_record_cb_t va_dsp_record_cb, 
                  va_dsp_notify_mute_cb_t va_dsp_mute_notify_cb);
}

namespace esphome {
namespace cx_i2s {

static const char *const TAG = "cx_i2s";

// --- Microphone ---

void CXI2SMicrophone::setup() {
    ESP_LOGD(TAG, "CXI2SMicrophone setup");
    va_dsp_init(nullptr, nullptr, nullptr);
    ESP_LOGI(TAG, "DSP Initialized");
}

void CXI2SMicrophone::start() {
    this->state_ = microphone::STATE_RUNNING;
}

void CXI2SMicrophone::stop() {
    this->state_ = microphone::STATE_STOPPED;
}

void CXI2SMicrophone::loop() {
    if (this->state_ != microphone::STATE_RUNNING) return;
    
    static uint32_t last_send = 0;
    uint32_t now = esphome::millis();
    if (now - last_send > 100) {
        std::vector<uint8_t> data(320, 0);
        this->data_callbacks_.call(data);
        last_send = now;
    }
}

// --- Speaker ---

void CXI2SSpeaker::setup() {
    ESP_LOGD(TAG, "CXI2SSpeaker setup");
}

void CXI2SSpeaker::start() {
    this->state_ = speaker::STATE_RUNNING;
}

void CXI2SSpeaker::stop() {
    this->state_ = speaker::STATE_STOPPED;
}

void CXI2SSpeaker::loop() {}

size_t CXI2SSpeaker::play(const uint8_t *data, size_t length) {
    return length;
}

bool CXI2SSpeaker::has_buffered_data() const { return false; }

}  // namespace cx_i2s
}  // namespace esphome
