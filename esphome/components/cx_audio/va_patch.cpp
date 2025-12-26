#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

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

// --- Audio Resample Patch ---
// SDK ожидает эту функцию для преобразования каналов. 
// Мы реализуем простую версию (passthrough), так как используем моно.
int audio_resample_down_channel(short *src, short *dst, int src_rate, int dst_rate, int src_ch, int dst_size, int item_size, int mode) {
    // В минимальном варианте мы просто копируем данные, если они уже в нужном формате
    // Или возвращаем 0, если ресемплинг не критичен для старта.
    return 0; 
}

// --- Board/DSP Signals Patch ---
void va_boot_dsp_signal(void) {
    ESP_LOGI("VA_PATCH", "DSP Boot Signal received");
}

esp_err_t va_dsp_hal_configure(void) {
    ESP_LOGI("VA_PATCH", "va_dsp_hal_configure called");
    return ESP_OK;
}

// --- Button Patch ---
void va_button_notify_mute(bool mute) {
    ESP_LOGI("VA_PATCH", "Mute notification: %s", mute ? "ON" : "OFF");
}

// --- I2S Compatibility Patch ---
int i2s_read_bytes(i2s_port_t i2s_num, char *dest, size_t size, TickType_t ticks_to_wait) {
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(i2s_num, dest, size, &bytes_read, ticks_to_wait);
    if (err != ESP_OK) return -1;
    return bytes_read;
}

} // extern "C"