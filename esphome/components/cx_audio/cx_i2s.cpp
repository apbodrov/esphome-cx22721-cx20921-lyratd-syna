#include "cx_i2s.h"
#include "esphome/core/log.h"

extern "C" {
#include "media_hal_playback.h"
#include "va_dsp.h"
#include "cnx20921_init.h"
}

namespace esphome {
namespace cx_i2s {

static const char *const TAG = "cx_i2s";
static CXI2SMicrophone *static_mic = nullptr;

static int va_dsp_record_callback(void *data, int len) {
  if (static_mic != nullptr && static_mic->is_running()) {
    // ESPHome expects std::vector<uint8_t> for raw audio data
    uint8_t *bytes = reinterpret_cast<uint8_t *>(data);
    std::vector<uint8_t> buffer(bytes, bytes + len);
    static_mic->publish_data(buffer);
  }
  return 0;
}

static int va_dsp_recognize_callback(int ww_length, enum initiator init_type) { return 0; }
static void va_dsp_mute_callback(bool mute) {}

void CXI2SMicrophone::setup() {
  static_mic = this;
  ESP_LOGI(TAG, "Initializing va_dsp for microphone capture...");
  va_dsp_init(va_dsp_recognize_callback, va_dsp_record_callback, va_dsp_mute_callback);
}

void CXI2SMicrophone::start() {
  this->state_ = microphone::STATE_RUNNING;
  ESP_LOGD(TAG, "Microphone capture started");
}

void CXI2SMicrophone::stop() {
  this->state_ = microphone::STATE_STOPPED;
  ESP_LOGD(TAG, "Microphone capture stopped");
}

void CXI2SMicrophone::loop() {}

void CXI2SMicrophone::publish_data(const std::vector<uint8_t> &data) { this->data_callbacks_.call(data); }

void CXI2SSpeaker::setup() {}
void CXI2SSpeaker::start() {
  this->state_ = speaker::STATE_RUNNING;
  this->partial_buffer_.clear();
}
void CXI2SSpeaker::stop() {
  this->state_ = speaker::STATE_STOPPED;
  this->partial_buffer_.clear();
}
void CXI2SSpeaker::loop() {}

size_t CXI2SSpeaker::play(const uint8_t *data, size_t length) {
  size_t total_len = length + this->partial_buffer_.size();
  size_t playable_len = (total_len / 4) * 4;
  size_t remainder = total_len % 4;

  media_hal_audio_info_t info = {
      .sample_rate = 48000,
      .channels = 2,
      .bits_per_sample = 16,
  };

  if (!this->partial_buffer_.empty()) {
    std::vector<uint8_t> temp_buf;
    temp_buf.reserve(total_len);
    temp_buf.insert(temp_buf.end(), this->partial_buffer_.begin(), this->partial_buffer_.end());
    temp_buf.insert(temp_buf.end(), data, data + length);

    media_hal_playback(&info, (void *) temp_buf.data(), (int) playable_len);

    if (remainder > 0) {
      this->partial_buffer_.assign(temp_buf.begin() + playable_len, temp_buf.end());
    } else {
      this->partial_buffer_.clear();
    }
  } else {
    media_hal_playback(&info, (void *) data, (int) playable_len);
    if (remainder > 0) {
      this->partial_buffer_.assign(data + playable_len, data + length);
    }
  }

  return length;
}

bool CXI2SSpeaker::has_buffered_data() const { return !this->partial_buffer_.empty(); }

}  // namespace cx_i2s
}  // namespace esphome