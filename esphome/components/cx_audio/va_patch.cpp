#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/i2s.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" {

// --- NVS Utils Patch ---
esp_err_t va_nvs_set_i8(const char *key, int8_t val) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;
    err = nvs_set_i8(my_handle, key, val);
    nvs_commit(my_handle);
    nvs_close(my_handle);
    return err;
}

esp_err_t va_nvs_get_i8(const char *key, int8_t *val) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) return err;
    err = nvs_get_i8(my_handle, key, val);
    nvs_close(my_handle);
    return err;
}

// --- DSP Initialization (FORCE GPIO 21 FOR V1.2) ---
esp_err_t cnx20921_init(gpio_num_t reset_pin, gpio_num_t irq_pin) {
    // На LyraTD V1.2 Reset DSP - это всегда GPIO 21. 
    // Мы игнорируем reset_pin из SDK (который часто передает 36 или 227).
    gpio_num_t actual_reset = GPIO_NUM_21;
    
    ESP_LOGI("VA_PATCH", "cnx20921_init: FORCING Hardware Reset on GPIO %d", actual_reset);
    
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << actual_reset);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    gpio_set_level(actual_reset, 0);
    vTaskDelay(pdMS_TO_TICKS(100)); // Честный сброс
    gpio_set_level(actual_reset, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    return ESP_OK;
}

// --- Audio Resample Patch ---
int audio_resample_down_channel(short *src, short *dst, int src_rate, int dst_rate, int src_ch, int dst_size, int item_size, int mode) {
    return 0; 
}

// --- Board/DSP Signals Patch ---
void va_boot_dsp_signal(void) {
    ESP_LOGI("VA_PATCH", "DSP Boot Signal received");
}

esp_err_t va_dsp_hal_configure(void) {
    return ESP_OK;
}

void va_button_notify_mute(bool mute) {
    ESP_LOGI("VA_PATCH", "Mute notification: %s", mute ? "ON" : "OFF");
}

int i2s_read_bytes(i2s_port_t i2s_num, char *dest, size_t size, TickType_t ticks_to_wait) {
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(i2s_num, (void*)dest, size, &bytes_read, ticks_to_wait);
    if (err != ESP_OK) return -1;
    return (int)bytes_read;
}

} // extern "C"
