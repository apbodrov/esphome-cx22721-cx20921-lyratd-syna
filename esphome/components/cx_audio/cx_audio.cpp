#include "cx_audio.h"
#include "esphome/core/log.h"
#include <driver/gpio.h>
#include <driver/i2s.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

extern "C" {
#include "va_board.h"
#include "media_hal.h"
}

namespace esphome {
namespace cx_audio {

static const char *const TAG = "cx_audio";

void CXAudio::setup() {
  ESP_LOGI(TAG, "Starting advanced Soft-Reset Recovery...");

  // 1. Обесточиваем кодек (GPIO4)
  gpio_config_t io_conf;
  memset(&io_conf, 0, sizeof(io_conf));
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << 4);
  gpio_config(&io_conf);
  gpio_set_level(GPIO_NUM_4, 0);

  // 2. Принудительно очищаем периферию
  i2s_driver_uninstall(I2S_NUM_0);
  i2s_driver_uninstall(I2S_NUM_1);
  i2c_driver_delete(I2C_NUM_0);

  // 3. I2C Bus Recovery (SDA=18, SCL=23)
  ESP_LOGI(TAG, "Recovering I2C bus (clocking SCL)...");
  gpio_set_direction(GPIO_NUM_18, GPIO_MODE_INPUT);
  gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);
  for (int i = 0; i < 9; i++) {
    if (gpio_get_level(GPIO_NUM_18))
      break;
    gpio_set_level(GPIO_NUM_23, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(GPIO_NUM_23, 1);
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  // 4. Длительная пауза для разрядки конденсаторов
  vTaskDelay(pdMS_TO_TICKS(1000));

  // 5. Включаем кодек
  gpio_set_level(GPIO_NUM_4, 1);
  vTaskDelay(pdMS_TO_TICKS(500));

  ESP_LOGI(TAG, "Initializing SDK via va_board_init()...");
  va_board_init();
  this->hal_ = media_hal_get_handle();
}

void CXAudio::dump_config() { ESP_LOGCONFIG(TAG, "CX Audio (Monolithic SDK Blob)"); }

}  // namespace cx_audio
}  // namespace esphome
