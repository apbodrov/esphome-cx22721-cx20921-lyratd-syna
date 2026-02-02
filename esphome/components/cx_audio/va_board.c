/*
 * Оригинальная инициализация платы LyraTD V1.2
 * Адаптировано для использования с libva_dsp.a и libcnx-ipc.a
 */

#include <string.h>
#include <esp_log.h>
#include <audio_board.h>
#include <va_board.h>
#include <va_dsp.h>
#include <va_dsp_cnx.h>
#include <media_hal.h>
#include <media_hal_playback.h>
#include <driver/i2c.h>
#include <driver/i2s.h>

#define VA_TAG "AUDIO_BOARD"

// Пины I2C для LyraTD V1.2
#define I2C_BUS_NO I2C_NUM_0
#define SDA_PIN 18
#define SCL_PIN 23

// Пины DSP
#define CNX_INTR_PIN 36
#define CNX_MUTE_PIN -1  // Освобождаем GPIO27 для LED

#define I2S_PORT_NUM I2S_NUM_0

// Объявления из SDK
extern void cnx_pin_config(int cnx_intr_pin, int cnx_mute_pin);

int va_board_init() {
  int ret = ESP_OK;
  ESP_LOGI(VA_TAG, "Starting va_board_init() for LyraTD V1.2");

  // 1. Инициализация I2C для кодека и DSP
  i2c_config_t i2c_conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = SDA_PIN,
      .scl_io_num = SCL_PIN,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = 100000,
  };
  i2c_param_config(I2C_BUS_NO, &i2c_conf);
  i2c_driver_install(I2C_BUS_NO, I2C_MODE_MASTER, 0, 0, 0);
  ESP_LOGI(VA_TAG, "I2C initialized (SDA:%d, SCL:%d)", SDA_PIN, SCL_PIN);

  // 2. Подготовка конфигурации I2S
  i2s_config_t i2s_cfg = {};
  audio_board_i2s_init_default(&i2s_cfg);

  // 3. Установка драйверов I2S
  // Порт 0 для воспроизведения (Кодек)
  ret = i2s_driver_install(I2S_PORT_NUM, &i2s_cfg, 0, NULL);
  if (ret != ESP_OK) return ret;

  // Порт 1 для записи (DSP)
  i2s_cfg.mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX);
  ret = i2s_driver_install(I2S_NUM_1, &i2s_cfg, 0, NULL);
  if (ret != ESP_OK) return ret;

  // 4. Настройка пинов I2S
  i2s_pin_config_t pf_i2s_pin = {0};
  audio_board_i2s_pin_config(I2S_PORT_NUM, &pf_i2s_pin);
  i2s_set_pin(I2S_PORT_NUM, &pf_i2s_pin);
  
  audio_board_i2s_pin_config(I2S_NUM_1, &pf_i2s_pin);
  i2s_set_pin(I2S_NUM_1, &pf_i2s_pin);

  // DSP требует стерео 16kHz для записи
  i2s_set_clk(I2S_NUM_1, 16000, 16, 2);

  i2s_zero_dma_buffer(I2S_PORT_NUM);
  i2s_zero_dma_buffer(I2S_NUM_1);

  // 5. Инициализация Media HAL (Кодек)
  static media_hal_config_t media_hal_cfg = MEDIA_HAL_DEFAULT();
  static media_hal_playback_cfg_t media_hal_playback_cfg = DEFAULT_MEDIA_HAL_PLAYBACK_CONFIG();
  media_hal_init(&media_hal_cfg, &media_hal_playback_cfg);
  ESP_LOGI(VA_TAG, "Media HAL initialized");

  // 6. Конфигурация пинов DSP в SDK
  cnx_pin_config(CNX_INTR_PIN, CNX_MUTE_PIN);
  ESP_LOGI(VA_TAG, "CNX pins configured: INT=%d, MUTE=%d", CNX_INTR_PIN, CNX_MUTE_PIN);

  return ret;
}

// Заглушки для неиспользуемых в ESPHome функций SDK
esp_err_t va_board_button_init() { return ESP_OK; }
esp_err_t va_board_led_init() { return ESP_OK; }
int but_cb_reg_handlr(int ui_but_evt) { return 1; }
esp_err_t ab_button_gpio_init() { return ESP_OK; }