#include <esp_err.h>
#include <stdbool.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/i2s.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include <string.h>

extern "C" {
#include <va_dsp.h>
#include <audio_board.h>
#include <media_hal.h>
#include <media_hal_playback.h>

// Forward declarations from the blob
esp_err_t cx20921Init(int flash_fw);

typedef enum {
    NO_FLASH_FW = 0,
    FLASH_FW = 1
} cnx_mode_t;

static const char *TAG = "va_monolith_custom";

// Replacement for cnx_pin_config which was lost when va_dsp.o was blacklisted
void cnx_pin_config(int cnx_intr_pin, int cnx_mute_pin) {
    ESP_LOGI(TAG, "cnx_pin_config: intr=%d, mute=%d", cnx_intr_pin, cnx_mute_pin);
}

// 1. Replacement for cnx20921_init - FORCING NO FLASH
esp_err_t cnx20921_init(SemaphoreHandle_t semph, int int_pin, int mute_pin, cnx_mode_t flash_fw) {
    ESP_LOGI(TAG, "FORCING NO_FLASH_FW mode for CX20921");
    // Ensure DSP is out of reset
    gpio_set_direction((gpio_num_t)19, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)19, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level((gpio_num_t)19, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    return cx20921Init(0); // 0 = NO_FLASH_FW
}

// 2. Replacement for va_dsp_init
void va_dsp_init(va_dsp_recognize_cb_t va_dsp_recognize_cb, va_dsp_record_cb_t va_dsp_record_cb, va_dsp_notify_mute_cb_t va_dsp_mute_notify_cb) {
    ESP_LOGI(TAG, "CUSTOM va_dsp_init (NO_FLASH)");
    cnx_pin_config(36, 27);
    cnx20921_init(NULL, 36, 27, NO_FLASH_FW);
}

// 3. Full replacement for va_board_init
int va_board_init() {
    ESP_LOGI(TAG, "CUSTOM va_board_init (NO_FLASH)...");
    
    gpio_set_direction((gpio_num_t)4, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)4, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    i2s_config_t i2s_cfg = {};
    audio_board_i2s_init_default(&i2s_cfg);

    media_hal_config_t mhal_conf = {
        .op_mode = MEDIA_HAL_MODE_SLAVE,
        .adc_input = MEDIA_HAL_ADC_INPUT_LINE1,
        .dac_output = MEDIA_HAL_DAC_OUTPUT_ALL,
        .codec_mode = MEDIA_HAL_CODEC_MODE_BOTH,
        .bit_length = MEDIA_HAL_BIT_LENGTH_16BITS,
        .format = MEDIA_HAL_I2S_NORMAL,
        .port_num = 0
    };

    media_hal_playback_cfg_t pbak_conf;
    memset(&pbak_conf, 0, sizeof(pbak_conf));
    pbak_conf.channels = 2;
    pbak_conf.sample_rate = 48000;
    pbak_conf.bits_per_sample = 16;
    pbak_conf.i2s_port_num = I2S_NUM_0;
    
    media_hal_init(&mhal_conf, &pbak_conf);

    i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL);
    i2s_cfg.mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX);
    i2s_driver_install(I2S_NUM_1, &i2s_cfg, 0, NULL);
    
    i2s_pin_config_t pf_i2s_pin;
    memset(&pf_i2s_pin, 0, sizeof(pf_i2s_pin));
    audio_board_i2s_pin_config(I2S_NUM_0, &pf_i2s_pin);
    i2s_set_pin(I2S_NUM_0, &pf_i2s_pin);
    audio_board_i2s_pin_config(I2S_NUM_1, &pf_i2s_pin);
    i2s_set_pin(I2S_NUM_1, &pf_i2s_pin);
    i2s_set_clk(I2S_NUM_1, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);

    ESP_LOGI(TAG, "CUSTOM va_board_init completed");
    return 0;
}

// 4. Mocks for missing symbols
void va_boot_dsp_signal() {}
void va_ui_set_state(int state) {}
void va_button_notify_mute(bool mute) {}
esp_err_t va_board_button_init() { return 0; }
esp_err_t va_board_led_init() { return 0; }
int but_cb_reg_handlr(int ui_but_evt) { return 1; }
esp_err_t va_nvs_set_i8(const char *nspace, const char *key, int8_t val) { return 0; }
esp_err_t va_nvs_get_i8(const char *nspace, const char *key, int8_t *val) { return -1; }
void va_set_state(int state) {}
esp_err_t ab_button_gpio_init() { return 0; }
esp_err_t va_dsp_hal_configure(void *config) { return 0; }
}
