#include "cx_i2s.h"
#include "cx_audio.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include <driver/i2s.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>

extern "C" {
  #include <va_dsp.h>
  #include <cnx20921_init.h>
  int cx20921SetMicGain(int gain_db);
}

namespace esphome {
namespace cx_i2s {

static const char *const TAG = "cx_i2s";

void CXI2SMicrophone::setup() {
    ESP_LOGI(TAG, "Setting up CX I2S Microphone...");
    
    // 1. Инициализируем HAL DSP
    va_dsp_init(nullptr, nullptr, nullptr);
    
    // 2. Hardware Reset DSP (GPIO21)
    gpio_num_t reset_pin = GPIO_NUM_21;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << reset_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(reset_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(reset_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 3. Инициализируем чип через SDK
    bool use_fw = this->parent_->is_use_firmware();
    int fw_mode = use_fw ? 1 : 0;
    
    ESP_LOGI(TAG, "Initializing DSP chip (mode: %s)...", use_fw ? "FLASH_FW" : "NO_FLASH_FW");
    
    SemaphoreHandle_t semph = xSemaphoreCreateMutex();
    esp_err_t ret = cnx20921_init(semph, 36, -1, (cnx_mode_t)fw_mode);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "cnx20921_init failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "CX20921 initialized successfully");
    }
    
    // 4. Установка начального гейна
    if (this->mic_gain_ != 0.0f) {
        cx20921SetMicGain((int)this->mic_gain_);
    }
    
    this->start();
}

void CXI2SMicrophone::start() {
    this->state_ = microphone::STATE_RUNNING;
    ESP_LOGI(TAG, "Microphone started");
}

void CXI2SMicrophone::stop() {
    this->state_ = microphone::STATE_STOPPED;
}

bool CXI2SMicrophone::set_mic_gain(float mic_gain) {
    this->mic_gain_ = clamp<float>(mic_gain, 0.0f, 30.0f);
    if (this->state_ == microphone::STATE_RUNNING) {
        cx20921SetMicGain((int)this->mic_gain_);
    }
    return true;
}

void CXI2SMicrophone::loop() {
    if (this->state_ != microphone::STATE_RUNNING) return;
    
    uint8_t stereo_buffer[640];
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_NUM_1, stereo_buffer, sizeof(stereo_buffer), &bytes_read, pdMS_TO_TICKS(10));
    
    if (err == ESP_OK && bytes_read > 0) {
        size_t samples = bytes_read / 4;
        std::vector<uint8_t> mono_data;
        mono_data.reserve(samples * 2);
        
        for (size_t i = 0; i < samples; i++) {
            mono_data.push_back(stereo_buffer[i * 4 + 0]);
            mono_data.push_back(stereo_buffer[i * 4 + 1]);
        }
        this->data_callbacks_.call(mono_data);
    }
}

// --- Speaker ---
void CXI2SSpeaker::setup() {}
void CXI2SSpeaker::start() { this->state_ = speaker::STATE_RUNNING; }
void CXI2SSpeaker::stop() { this->state_ = speaker::STATE_STOPPED; }
void CXI2SSpeaker::loop() {}
size_t CXI2SSpeaker::play(const uint8_t *data, size_t length) {
    size_t written = 0;
    i2s_write(I2S_NUM_0, data, length, &written, pdMS_TO_TICKS(10));
    return written;
}
bool CXI2SSpeaker::has_buffered_data() const { return false; }

}  // namespace cx_i2s
}  // namespace esphome