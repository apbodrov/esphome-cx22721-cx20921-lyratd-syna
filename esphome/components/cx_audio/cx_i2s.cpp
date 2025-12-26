#include "cx_i2s.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include <driver/i2s.h>

extern "C" {
  #include <va_board.h>
  #include <va_dsp.h>
  void va_dsp_init(va_dsp_recognize_cb_t va_dsp_recognize_cb, 
                  va_dsp_record_cb_t va_dsp_record_cb, 
                  va_dsp_notify_mute_cb_t va_dsp_mute_notify_cb);
}

namespace esphome {
namespace cx_i2s {

static const char *const TAG = "cx_i2s";

// --- Microphone ---

void CXI2SMicrophone::setup() {
    ESP_LOGI(TAG, "Setting up CX I2S Microphone...");
    va_dsp_init(nullptr, nullptr, nullptr);
    ESP_LOGI(TAG, "DSP Initialized");
}

void CXI2SMicrophone::start() {
    this->state_ = microphone::STATE_RUNNING;
    ESP_LOGI(TAG, "Microphone started");
}

void CXI2SMicrophone::stop() {
    this->state_ = microphone::STATE_STOPPED;
    ESP_LOGI(TAG, "Microphone stopped");
}

void CXI2SMicrophone::loop() {
    if (this->state_ != microphone::STATE_RUNNING) return;
    
    // Читаем данные из I2S1 (куда DSP шлет обработанный голос)
    // На LyraTD V1.2 это порт 1.
    
    uint8_t buffer[320]; // 160 samples * 2 bytes
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_NUM_1, buffer, sizeof(buffer), &bytes_read, pdMS_TO_TICKS(10));
    
    if (err == ESP_OK && bytes_read > 0) {
        std::vector<uint8_t> data(buffer, buffer + bytes_read);
        this->data_callbacks_.call(data);
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