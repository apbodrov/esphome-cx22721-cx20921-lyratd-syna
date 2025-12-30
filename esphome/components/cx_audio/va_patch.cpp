// Патчи для SDK функций, которые требуют реализации в ESPHome
// ВАЖНО: Все weak символы и wrappers удалены - используем адаптеры из sdk_adapter/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/i2s.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stddef.h>

// Forward declarations для адаптеров
extern "C" {
typedef void* rb_handle_t;
void* esphome_esp_audio_mem_calloc(size_t n, size_t size);
void* esphome_esp_audio_mem_malloc(size_t size);
void esphome_esp_audio_mem_free(void* ptr);
void* esphome_esp_audio_mem_realloc(void* ptr, size_t size);
rb_handle_t esphome_rb_init(const char* name, uint32_t size);
int esphome_rb_read(rb_handle_t handle, uint8_t* buf, int len, uint32_t timeout);
int esphome_rb_write(rb_handle_t handle, uint8_t* buf, int len, uint32_t timeout);
ssize_t esphome_rb_available(rb_handle_t handle);
void esphome_rb_abort(rb_handle_t handle);
void esphome_rb_reset(rb_handle_t handle);
}

extern "C" {

// ============================================================================
// Алиасы для функций памяти - библиотека ожидает esp_audio_mem_*
// ============================================================================
void* esp_audio_mem_calloc(int n, int size) {
    return esphome_esp_audio_mem_calloc((size_t)n, (size_t)size);
}

void* esp_audio_mem_malloc(int size) {
    return esphome_esp_audio_mem_malloc((size_t)size);
}

void esp_audio_mem_free(void* ptr) {
    esphome_esp_audio_mem_free(ptr);
}

void* esp_audio_mem_realloc(void* old_ptr, int old_size, int new_size) {
    return esphome_esp_audio_mem_realloc(old_ptr, (size_t)new_size);
}

// ============================================================================
// Алиасы для ring buffer функций - библиотека ожидает rb_*
// ============================================================================
rb_handle_t rb_init(const char* name, uint32_t size) {
    return esphome_rb_init(name, size);
}

int rb_read(rb_handle_t handle, uint8_t* buf, int len, uint32_t timeout) {
    return esphome_rb_read(handle, buf, len, timeout);
}

int rb_write(rb_handle_t handle, uint8_t* buf, int len, uint32_t timeout) {
    return esphome_rb_write(handle, buf, len, timeout);
}

ssize_t rb_available(rb_handle_t handle) {
    return esphome_rb_available(handle);
}

void rb_abort(rb_handle_t handle) {
    esphome_rb_abort(handle);
}

void rb_reset(rb_handle_t handle) {
    esphome_rb_reset(handle);
}

// ============================================================================
// NVS Utils Patch
// ============================================================================
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

// ============================================================================
// Audio Resample Patch
// ============================================================================
int audio_resample_down_channel(short *src, short *dst, int src_rate, int dst_rate, int src_ch, int dst_size, int item_size, int mode) {
    (void)src; (void)dst; (void)src_rate; (void)dst_rate; (void)src_ch; (void)dst_size; (void)item_size; (void)mode;
    return 0; 
}

// ============================================================================
// DSP HAL Configure Patch
// ============================================================================
// Функция va_dsp_hal_configure теперь определена в va_dsp_hal.c
// Удалено отсюда, чтобы избежать множественного определения

// ============================================================================
// Button Mute Notification Patch
// ============================================================================
void va_button_notify_mute(bool mute) {
    ESP_LOGI("VA_PATCH", "Mute notification: %s", mute ? "ON" : "OFF");
}

// ============================================================================
// I2S Read Bytes Compatibility Patch
// ============================================================================
int i2s_read_bytes(i2s_port_t i2s_num, char *dest, size_t size, TickType_t ticks_to_wait) {
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(i2s_num, (void*)dest, size, &bytes_read, ticks_to_wait);
    if (err != ESP_OK) return -1;
    return (int)bytes_read;
}

// ============================================================================
// I2C Debug & Sync Wrapper
// ============================================================================
extern "C" SemaphoreHandle_t esphome_get_i2c_semaphore();
extern "C" esp_err_t __real_i2c_master_cmd_begin(i2c_port_t i2c_num, i2c_cmd_handle_t cmd_handle, TickType_t ticks_to_wait);

extern "C" esp_err_t __wrap_i2c_master_cmd_begin(i2c_port_t i2c_num, i2c_cmd_handle_t cmd_handle, TickType_t ticks_to_wait) {
    SemaphoreHandle_t sem = esphome_get_i2c_semaphore();
    
    // Проверка на валидность указателя. В ESP32 валидные адреса RAM обычно начинаются с 0x3F или 0x40.
    // Адрес 0x14... или 0x38... явно указывает на коррупцию данных из I2C.
    bool sem_valid = (sem != NULL && ((uint32_t)sem & 0xFF000000) == 0x3F000000);
    
    if (sem_valid) {
        xSemaphoreTake(sem, portMAX_DELAY);
    } else if (sem != NULL) {
        // Если указатель есть, но он подозрительный - это коррупция.
        // Не падаем, но и не берем семафор (рискуем коллизией, но это лучше чем паника).
        static uint32_t last_bad_sem = 0;
        if ((uint32_t)sem != last_bad_sem) {
            ESP_LOGE("I2C_WRAP", "Corrupted semaphore pointer detected: %p! Data leak from SDK suspected.", sem);
            last_bad_sem = (uint32_t)sem;
        }
    }
    
    esp_err_t ret = __real_i2c_master_cmd_begin(i2c_num, cmd_handle, ticks_to_wait);
    
    if (sem_valid) {
        xSemaphoreGive(sem);
    }
    return ret;
}

} // extern "C"
