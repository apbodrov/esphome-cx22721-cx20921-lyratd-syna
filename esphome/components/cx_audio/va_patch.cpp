#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/i2s.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "cnx20921_init.h"

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
// ЭТАЛОН: Сигнатура должна совпадать с cnx20921_init.h из монолита
// ВАЖНО: Эта функция вызывается из va_dsp_init() в монолите
// 
// В библиотеке функция переименована в cnx20921_init_unused
// Создаем wrapper, который вызывает реальную функцию из библиотеки
extern "C" {
// Объявляем реальную функцию из библиотеки (переименованную)
extern esp_err_t cnx20921_init_unused(SemaphoreHandle_t semph, int int_pin, int mute_pin, cnx_mode_t flash_fw);

esp_err_t cnx20921_init(SemaphoreHandle_t semph, int int_pin, int mute_pin, cnx_mode_t flash_fw) {
    // ВАЖНО: Условия для работы полной инициализации:
    // - I2C driver должен быть установлен (ESPHome делает это)
    // - cnx_pin_config() должна быть вызвана ПЕРЕД va_dsp_init()
    // - va_dsp_hal_init() должна быть вызвана ПЕРЕД cnx20921_init()
    // - Функции i2c_write_imp и i2c_read_imp должны быть в библиотеке (они там есть)
    //
    // Полная версия cnx20921_init_unused() из libcnx-ipc.a:
    // 1. Делает hardware reset
    // 2. Вызывает OpenI2cDevice() для инициализации I2C
    // 3. Устанавливает I2C callbacks через SetupI2cWriteMemCallback/SetupI2cReadMemCallback
    // 4. Инициализирует DSP через SendCmdV()
    
    ESP_LOGI("VA_PATCH", "cnx20921_init: Calling REAL function from library (cnx20921_init_unused)");
    ESP_LOGI("VA_PATCH", "  Parameters: int_pin=%d, mute_pin=%d, flash_fw=%d", int_pin, mute_pin, flash_fw);
    
    // Вызываем реальную функцию из библиотеки
    esp_err_t ret = cnx20921_init_unused(semph, int_pin, mute_pin, flash_fw);
    
    if (ret != ESP_OK) {
        ESP_LOGE("VA_PATCH", "cnx20921_init_unused failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI("VA_PATCH", "cnx20921_init_unused completed successfully");
    }
    
    return ret;
}
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
