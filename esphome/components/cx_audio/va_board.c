/*
 *
 * Copyright 2015-2018 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <string.h>
#include <esp_log.h>
#include <audio_board.h>
// LED и button драйверы не используются в ESPHome
// #include <led_pattern.h>
// #include <led_driver.h>
// #include <va_led.h>
// #include <button_driver.h>
#include <va_board.h>
#include <va_dsp.h>
#include <va_dsp_cnx.h>
#include <media_hal.h>
#include <media_hal_playback.h>
#include <va_dsp_hal.h>

#include <driver/i2c.h>

#define VA_TAG "AUDIO_BOARD"

#define I2C_BUS_NO I2C_NUM_0
#define SDA_PIN 18
#define SCL_PIN 23

bool ab_but_mute = false;

extern esp_err_t ab_button_gpio_init();

#define I2S_PORT_NUM I2S_NUM_0

// Forward declarations
esp_err_t va_board_button_init();
esp_err_t va_board_led_init();

int va_board_init() {
  int ret;
  // Use printf directly to ensure logs are flushed immediately
  printf("\n[AUDIO_BOARD] ===== va_board_init() STARTED =====\n");
  fflush(stdout);
  ESP_LOGI(VA_TAG, "=== Starting va_board_init() ===");
  fflush(stdout);

  // Initialize I2C first
  i2c_config_t i2c_conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = SDA_PIN,
      .scl_io_num = SCL_PIN,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = 100000,
  };
  ESP_LOGI(VA_TAG, "Configuring I2C (SDA:%d, SCL:%d)", SDA_PIN, SCL_PIN);
  i2c_param_config(I2C_BUS_NO, &i2c_conf);
  i2c_driver_install(I2C_BUS_NO, I2C_MODE_MASTER, 0, 0, 0);

  i2s_config_t i2s_cfg = {};
  ESP_LOGI(VA_TAG, "Calling audio_board_i2s_init_default()");
  fflush(stdout);
  audio_board_i2s_init_default(&i2s_cfg);
  ESP_LOGI(VA_TAG, "I2S config initialized");
  fflush(stdout);

  // Install I2S drivers first (before media_hal_init which needs I2S)
  ESP_LOGI(VA_TAG, "Installing I2S driver for port 0");
  fflush(stdout);
  ret = i2s_driver_install(I2S_PORT_NUM, &i2s_cfg, 0, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(VA_TAG, "Error installing i2s driver for port 0: %s", esp_err_to_name(ret));
    fflush(stdout);
    return ret;
  }
  ESP_LOGI(VA_TAG, "I2S driver installed for port 0");
  fflush(stdout);

  i2s_cfg.mode = I2S_MODE_SLAVE | I2S_MODE_RX;
  ESP_LOGI(VA_TAG, "Installing I2S driver for port 1");
  fflush(stdout);
  ret = i2s_driver_install(I2S_NUM_1, &i2s_cfg, 0, NULL);
  if (ret != ESP_OK) {
    ESP_LOGE(VA_TAG, "Error installing i2s driver for port 1: %s", esp_err_to_name(ret));
    fflush(stdout);
    return ret;
  }
  ESP_LOGI(VA_TAG, "I2S driver installed for port 1");
  fflush(stdout);

  i2s_pin_config_t pf_i2s_pin = {0};
  i2s_pin_config_t pf_i2s_pin_rcv = {0};
  ESP_LOGI(VA_TAG, "Configuring I2S pins for port 0");
  fflush(stdout);
  audio_board_i2s_pin_config(I2S_PORT_NUM, &pf_i2s_pin);
  i2s_set_pin(I2S_PORT_NUM, &pf_i2s_pin);
  ESP_LOGI(VA_TAG, "I2S pins configured for port 0");
  fflush(stdout);

  ESP_LOGI(VA_TAG, "Configuring I2S pins for port 1");
  fflush(stdout);
  audio_board_i2s_pin_config(I2S_NUM_1, &pf_i2s_pin_rcv);
  i2s_set_pin(I2S_NUM_1, &pf_i2s_pin_rcv);
  ESP_LOGI(VA_TAG, "I2S pins configured for port 1");
  fflush(stdout);

  /* XXX: Reading mono channel on I2S produces data mirroring, 0-4KHz mirrored to 4-8KHz
   * For better audio performance, we read stereo data and then down sample it to mono
   */
  ESP_LOGI(VA_TAG, "Setting I2S clock for port 1: 16000 Hz, 16 bits, 2 channels");
  fflush(stdout);
  ret = i2s_set_clk(I2S_NUM_1, 16000, 16, 2);
  if (ret != ESP_OK) {
    ESP_LOGE(VA_TAG, "Failed to set I2S clock: %s", esp_err_to_name(ret));
    fflush(stdout);
  } else {
    ESP_LOGI(VA_TAG, "I2S clock configured successfully");
    fflush(stdout);
  }

  ESP_LOGI(VA_TAG, "Zeroing I2S DMA buffers");
  ret = i2s_zero_dma_buffer(I2S_PORT_NUM);
  if (ret != ESP_OK) {
    ESP_LOGE(VA_TAG, "Failed to zero DMA buffer for port 0: %s", esp_err_to_name(ret));
  }
  ret |= i2s_zero_dma_buffer(I2S_NUM_1);
  if (ret != ESP_OK) {
    ESP_LOGE(VA_TAG, "Failed to zero DMA buffer for port 1: %s", esp_err_to_name(ret));
  }
  ESP_LOGI(VA_TAG, "I2S DMA buffers zeroed");
  fflush(stdout);  // Force flush logs

  // Initialize media_hal with codec configuration (after I2S is ready)
  ESP_LOGI(VA_TAG, "Preparing media_hal config structures");
  static media_hal_config_t media_hal_cfg = MEDIA_HAL_DEFAULT();
  static media_hal_playback_cfg_t media_hal_playback_cfg = DEFAULT_MEDIA_HAL_PLAYBACK_CONFIG();
  ESP_LOGI(VA_TAG, "Config structures prepared");
  fflush(stdout);

  ESP_LOGI(VA_TAG, "Calling media_hal_init() - this will initialize codec via I2C");
  fflush(stdout);

  // Initialize media_hal (this will also initialize the codec via I2C)
  media_hal_init(&media_hal_cfg, &media_hal_playback_cfg);

  ESP_LOGI(VA_TAG, "media_hal_init() returned");
  fflush(stdout);
  ESP_LOGI(VA_TAG, "media_hal initialized successfully");

  // CNX Interrupt Pin config
  ESP_LOGI(VA_TAG, "Configuring CNX interrupt pins");
  // GPIO27 используется для светодиодного кольца, поэтому убираем его из Mute
  cnx_pin_config(GPIO_NUM_36, -1);
  ESP_LOGI(VA_TAG, "CNX interrupt pins configured (Mute disabled to free GPIO27)");

  // Initialize buttons and LEDs
  ESP_LOGI(VA_TAG, "Initializing buttons");
  va_board_button_init();
  ESP_LOGI(VA_TAG, "Initializing LEDs");
  va_board_led_init();
  ESP_LOGI(VA_TAG, "Buttons and LEDs initialized");

  // Configure DSP HAL (for CNX20921, we can pass NULL or empty config)
  ESP_LOGI(VA_TAG, "Configuring DSP HAL");
  va_dsp_hal_configure(NULL);
  ESP_LOGI(VA_TAG, "DSP HAL configured");

  ESP_LOGI(VA_TAG, "va_board_init() completed successfully");
  return ret;
}

int but_cb_reg_handlr(int ui_but_evt) { return 1; }

esp_err_t va_board_button_init() {
  // TODO: Implement button initialization using button_driver API
  return ESP_OK;
}

esp_err_t va_board_led_init() {
  printf("Doing LED initialisation for this board\n");
  // va_led_config_t *ab_led_conf = NULL;
  // led_single_init - not implemented
  // ledc_cnx_init();
  // va_led_init - not implemented
  return ESP_OK;
}
