#include <stdio.h>
#include <string.h>
#include <driver/i2c.h>
#include <driver/i2s.h>
#include <esp_err.h>
#include <esp_log.h>
#include <cx22721.h>
#include <rom/ets_sys.h>

#define HAL_TAG "CX22721"

static uint8_t volume_val = 60;

static void cx22721_i2c_init()
{
    // I2C driver is already installed in va_board_init()
    ESP_LOGI(HAL_TAG, "Using pre-installed I2C (SDA:%d, SCL:%d)", SDA_PIN, SCL_PIN);
}

extern SemaphoreHandle_t esphome_get_i2c_semaphore();

extern SemaphoreHandle_t esphome_get_i2c_semaphore();

static esp_err_t i2c_trans(i2c_port_t i2c_num, uint8_t addr, void *txdata, uint8_t txlen)
{
    esp_err_t rc = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    rc = i2c_master_start(cmd);
    rc = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    rc = i2c_master_write(cmd, txdata, txlen, true);
    rc = i2c_master_stop(cmd);
    
    // ПРОВЕРКА: Если драйвер не установлен, i2c_master_cmd_begin может паниковать
    rc = i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_PERIOD_MS);
    if (rc == ESP_ERR_INVALID_STATE) {
        static bool reported = false;
        if (!reported) {
            ESP_LOGE(HAL_TAG, "I2C driver NOT installed for port %d!", i2c_num);
            reported = true;
        }
    }
    
    i2c_cmd_link_delete(cmd);
    return rc;
}

static esp_err_t i2c_recv(i2c_port_t i2c_num, uint8_t addr, void *rxdata, uint8_t rxlen)
{
    esp_err_t rc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (rxlen > 1) {
        i2c_master_read(cmd, rxdata, rxlen - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, rxdata + rxlen - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    rc = i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return rc;
}

static esp_err_t write_register_i2c_d8(uint8_t slave_id, uint16_t reg_addr, uint8_t reg_val)
{
    uint8_t buff[3];
    buff[0] = (reg_addr >> 8) & 0xff;
    buff[1] = reg_addr & 0xff;
    buff[2] = reg_val;
    return i2c_trans(I2C_BUS_NO, slave_id, buff, 3);
}

static esp_err_t write_register_i2c_d16(uint8_t slave_id, uint16_t reg_addr, uint16_t reg_val)
{
    uint8_t buff[4];
    buff[0] = (reg_addr >> 8) & 0xff;
    buff[1] = reg_addr & 0xff;
    buff[2] = reg_val & 0xff;
    buff[3] = (reg_val >> 8) & 0xff;
    return i2c_trans(I2C_BUS_NO, slave_id, buff, 4);
}

esp_err_t cx22721_init(media_hal_config_t *media_hal_conf)
{
    esp_err_t ret = 0;
    (void)media_hal_conf;

    ESP_LOGI(HAL_TAG, "Initializing CX22721 codec...");

    // Power on the codec
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << CX22721_PWR_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(CX22721_PWR_PIN, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(CX22721_PWR_PIN, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    cx22721_i2c_init();

    // Register sequences from original SDK
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6D87, 0x03);

#ifdef CONFIG_SYNA_V1_2_BOARD
    ret |= write_register_i2c_d16(CODEC_ADDR, 0x7190, 0x040);
#else
    ret |= write_register_i2c_d16(CODEC_ADDR, 0x7190, 0x240);
#endif
    ret |= write_register_i2c_d16(CODEC_ADDR, 0x7194, 0x0000);
    ret |= write_register_i2c_d16(CODEC_ADDR, 0x7198, 0xbf8);
    ret |= write_register_i2c_d16(CODEC_ADDR, 0x719c, 0x193);
#ifdef CONFIG_SYNA_V1_2_BOARD
    ret |= write_register_i2c_d16(CODEC_ADDR, 0x71a0, 0x05f);
#else
    ret |= write_register_i2c_d16(CODEC_ADDR, 0x71a0, 0x017);
#endif
    ret |= write_register_i2c_d16(CODEC_ADDR, 0x71a8, 0x408);
    ret |= write_register_i2c_d16(CODEC_ADDR, 0x43c8, 0x11);
    ret |= write_register_i2c_d16(CODEC_ADDR, 0x47c8, 0x11);

    ret |= write_register_i2c_d8(CODEC_ADDR, 0x414, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x4014, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x4018, 0x10);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x4414, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x4418, 0x10);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e00, 0x3d);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e01, 0x46);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e02, 0x3d);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e03, 0x46);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e04, 0x30);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e05, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e06, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e07, 0x80); 
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e08, 0x0f);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e09, 0x10); 
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e0a, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e0b, 0x80); 
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e0c, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e0d, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e0e, 0x40); 
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e0f, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e10, 0x02); 
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6e11, 0x00); 
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6d85, 0x05);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6d18, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6d00, 0x0c);
#ifdef CONFIG_SYNA_V1_2_BOARD
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6d84, 0xec);
#else
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6d84, 0xac);
#endif
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x41c0, 0x4A);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x41e0, 0x4A);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x45c0, 0x4A);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x45e0, 0x4A);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x5c14, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x5c1c, 0x40);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x5c04, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x5814, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x581c, 0x40);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x5804, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x7414, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x7404, 0x00);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x741c, 0x40);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6d8c, 0x06);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6d8d, 0x01);

#ifdef CONFIG_SYNA_V1_2_BOARD
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6d84, 0xe4);
#else
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6d84, 0xa4);
#endif
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x6d92, 0x40);

    if (ret == ESP_OK) {
        ESP_LOGI(HAL_TAG, "Codec initialized successfully");
    } else {
        ESP_LOGE(HAL_TAG, "Codec initialization failed");
    }
    return ret;
}

esp_err_t cx22721_set_vol(uint8_t vol)
{
    int vol_to_set = 80 * (int)vol / 100;
    volume_val = vol;
    
    esp_err_t ret = 0;
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x41c0, vol_to_set);
    ret |= write_register_i2c_d8(CODEC_ADDR, 0x41e0, vol_to_set);
    return ret;
}

esp_err_t cx22721_set_mute(bool mute)
{
    esp_err_t ret = 0;
    if (mute) {
        ret |= write_register_i2c_d8(CODEC_ADDR, 0x41c0, 0x80 | volume_val);
        ret |= write_register_i2c_d8(CODEC_ADDR, 0x41e0, 0x80 | volume_val);
    } else {
        ret |= cx22721_set_vol(volume_val);
    }
    return ret;
}

esp_err_t cx22721_get_volume(uint8_t* vol)
{
    if (vol) *vol = volume_val;
    return ESP_OK;
}

esp_err_t cx22721_deinit(int port_num) { return ESP_OK; }
esp_err_t cx22721_powerup() { return ESP_OK; }
esp_err_t cx22721_powerdown() { return ESP_OK; }
esp_err_t cx22721_set_state(media_hal_codec_mode_t mode, media_hal_sel_state_t media_hal_state) { return ESP_OK; }
esp_err_t cx22721_config_format(media_hal_codec_mode_t mode, media_hal_format_t fmt) { return ESP_OK; }
esp_err_t cx22721_set_i2s_clk(media_hal_codec_mode_t mode, media_hal_bit_length_t media_hal_bit_length) { return ESP_OK; }
