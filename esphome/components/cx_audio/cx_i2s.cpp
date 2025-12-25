#include "cx_i2s.h"
#include "esphome/core/log.h"

extern "C" {
#include "media_hal_playback.h"
}

namespace esphome {
namespace cx_i2s {

static const char *const TAG = "cx_i2s";

void CXI2SMicrophone::setup() {}
void CXI2SMicrophone::start() { this->state_ = microphone::STATE_RUNNING; }
void CXI2SMicrophone::stop() { this->state_ = microphone::STATE_STOPPED; }
void CXI2SMicrophone::loop() {
  // Микрофон пока не трогаем, чтобы не мешать спикеру
}

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
  // Соблюдаем требование SDK к выравниванию (4 байта для 16-бит стерео)
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
