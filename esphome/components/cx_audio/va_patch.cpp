// Glue-код для связи ESPHome с оригинальными библиотеками Synaptics SDK
// Реализует функции, которые библиотека libva_dsp.a ожидает от внешнего SDK фреймворка

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/i2s.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <stddef.h>

// Forward declarations для адаптеров из rb_adapter.c
extern "C" {
typedef void *rb_handle_t;
rb_handle_t esphome_rb_init(const char *name, uint32_t size);
int esphome_rb_read(rb_handle_t handle, uint8_t *buf, int len, uint32_t timeout);
int esphome_rb_write(rb_handle_t handle, uint8_t *buf, int len, uint32_t timeout);
ssize_t esphome_rb_available(rb_handle_t handle);
void esphome_rb_abort(rb_handle_t handle);
void esphome_rb_reset(rb_handle_t handle);
}

extern "C" {

// ============================================================================
// Memory Management Glue
// ============================================================================
void *esp_audio_mem_calloc(int n, int size) { return calloc(n, size); }

void *esp_audio_mem_malloc(int size) { return malloc(size); }

void esp_audio_mem_free(void *ptr) { free(ptr); }

void *esp_audio_mem_realloc(void *old_ptr, int old_size, int new_size) { return realloc(old_ptr, new_size); }

// ============================================================================
// Ring Buffer Glue
// ============================================================================
rb_handle_t rb_init(const char *name, uint32_t size) { return esphome_rb_init(name, size); }

int rb_read(rb_handle_t handle, uint8_t *buf, int len, uint32_t timeout) {
  return esphome_rb_read(handle, buf, len, timeout);
}

int rb_write(rb_handle_t handle, uint8_t *buf, int len, uint32_t timeout) {
  return esphome_rb_write(handle, buf, len, timeout);
}

ssize_t rb_available(rb_handle_t handle) { return esphome_rb_available(handle); }

void rb_abort(rb_handle_t handle) { esphome_rb_abort(handle); }

void rb_reset(rb_handle_t handle) { esphome_rb_reset(handle); }

// ============================================================================
// SDK State Signals Glue
// ============================================================================
void va_boot_dsp_signal(void) { ESP_LOGI("VA_PATCH", "DSP Boot Signal: DSP is ready"); }

void va_ui_set_state(int state) { (void) state; }

void va_button_notify_mute(bool mute) { (void) mute; }

void va_set_state(int state) { (void) state; }

// ============================================================================
// NVS Utils Patch
// ============================================================================
esp_err_t va_nvs_set_i8(const char *key, int8_t val) {
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage_nvs", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
    return err;
  err = nvs_set_i8(my_handle, key, val);
  nvs_commit(my_handle);
  nvs_close(my_handle);
  return err;
}

esp_err_t va_nvs_get_i8(const char *key, int8_t *val) {
  nvs_handle_t my_handle;
  esp_err_t err = nvs_open("storage_nvs", NVS_READONLY, &my_handle);
  if (err != ESP_OK)
    return err;
  err = nvs_get_i8(my_handle, key, val);
  nvs_close(my_handle);
  return err;
}

// ============================================================================
// Audio Utilities Patch
// ============================================================================
int audio_resample_down_channel(short *src, short *dst, int src_rate, int dst_rate, int src_ch, int dst_size,
                                int item_size, int mode) {
  return 0;
}

int i2s_read_bytes(i2s_port_t i2s_num, char *dest, size_t size, TickType_t ticks_to_wait) {
  size_t bytes_read = 0;
  esp_err_t err = i2s_read(i2s_num, (void *) dest, size, &bytes_read, ticks_to_wait);
  if (err != ESP_OK)
    return -1;
  return (int) bytes_read;
}

// ============================================================================
// DSP Initialization Override Wrapper
// Перехватываем вызов cnx20921_init из va_dsp_init, чтобы принудительно 
// выключить прошивку, если это задано в YAML.
// ============================================================================
typedef enum { NO_FLASH_FW = 0, FLASH_FW = 1 } cnx_mode_t;
extern "C" esp_err_t __real_cnx20921_init(SemaphoreHandle_t semph, int int_pin, int mute_pin, cnx_mode_t flash_fw);

// Глобальная переменная для хранения желаемого режима
static int g_dsp_fw_mode = -1; 

extern "C" void esphome_set_dsp_fw_mode(int mode) {
    g_dsp_fw_mode = mode;
}

extern "C" esp_err_t __wrap_cnx20921_init(SemaphoreHandle_t semph, int int_pin, int mute_pin, cnx_mode_t flash_fw) {
    cnx_mode_t mode_to_use = flash_fw;
    
    if (g_dsp_fw_mode != -1) {
        mode_to_use = (cnx_mode_t)g_dsp_fw_mode;
        ESP_LOGW("DSP_WRAP", "Overriding SDK flash mode: SDK asked %d, we use %d", flash_fw, mode_to_use);
    } else {
        ESP_LOGI("DSP_WRAP", "Using SDK default flash mode: %d", flash_fw);
    }

    return __real_cnx20921_init(semph, int_pin, mute_pin, mode_to_use);
}

// ============================================================================
// I2C Watchdog Feeder & Sync Wrapper
// Сбрасываем вочдог при каждой I2C транзакции, чтобы не упасть во время прошивки DSP
// ============================================================================
extern "C" SemaphoreHandle_t esphome_get_i2c_semaphore();
extern "C" esp_err_t __real_i2c_master_cmd_begin(i2c_port_t i2c_num, i2c_cmd_handle_t cmd_handle,
                                                 TickType_t ticks_to_wait);

extern "C" esp_err_t __wrap_i2c_master_cmd_begin(i2c_port_t i2c_num, i2c_cmd_handle_t cmd_handle,
                                                 TickType_t ticks_to_wait) {
  // 1. Сбрасываем вочдог для текущей задачи
  esp_task_wdt_reset();

  // 2. Берем семафор для синхронизации с ESPHome
  SemaphoreHandle_t sem = esphome_get_i2c_semaphore();
  bool sem_valid = (sem != NULL && ((uintptr_t) sem & 0xFF000000) == 0x3F000000);

  if (sem_valid) {
    xSemaphoreTake(sem, portMAX_DELAY);
  }

  // 3. Выполняем реальный I2C вызов
  esp_err_t ret = __real_i2c_master_cmd_begin(i2c_num, cmd_handle, ticks_to_wait);

  if (sem_valid) {
    xSemaphoreGive(sem);
  }
  return ret;
}

}  // extern "C"
