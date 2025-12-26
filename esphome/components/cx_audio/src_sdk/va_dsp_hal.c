// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/i2s.h>

#include <va_dsp_hal.h>
#include <va_dsp.h>
#include <cnx20921_init.h>
#include <freertos/queue.h>

#define TAG "va_dsp_hal_cnx"

static SemaphoreHandle_t dsp_sem = NULL;
static int dsp_int_pin = -1;
static int dsp_mute_pin = -1;

esp_err_t va_dsp_hal_configure(void *config) {
  (void) config;
  return ESP_OK;
}

esp_err_t va_dsp_hal_init(QueueHandle_t queue) {
  // CNX20921 initialization is done separately via cnx20921_init()
  // This function is called from va_dsp_init() which already initializes the DSP
  // Store queue handle if needed for event notifications
  (void) queue;
  return ESP_OK;
}

esp_err_t va_dsp_hal_reset() {
  // Reset handled by cnx20921_init
  ESP_LOGI(TAG, "DSP reset");
  return ESP_OK;
}

esp_err_t va_dsp_hal_start_capture() {
  cnx20921_start_speech();
  return ESP_OK;
}

esp_err_t va_dsp_hal_stop_capture() {
  cnx20921_stop_capture();
  return ESP_OK;
}

esp_err_t va_dsp_hal_mic_mute() {
  cnx20921_mic_mute();
  return ESP_OK;
}

esp_err_t va_dsp_hal_mic_unmute() {
  cnx20921_mic_unmute();
  return ESP_OK;
}

int va_dsp_hal_stream_audio(uint8_t *buffer, int size, int wait) {
  (void) wait;  // CNX20921 doesn't use wait parameter
  return cnx20921_stream_audio(I2S_NUM_1, buffer, size);
}

size_t va_dsp_hal_get_ww_len() {
  // Return wake word length in samples
  return PHRASE_LEN;
}

esp_err_t va_dsp_hal_get_preroll(void *data) {
  cnx20921_get_preroll(data);
  return ESP_OK;
}

esp_err_t va_dsp_hal_tap_to_talk() {
  cnx20921_tap_to_talk();
  return ESP_OK;
}

esp_err_t va_dsp_hal_enter_low_power() {
  // Low power mode not implemented for CNX20921
  return ESP_OK;
}

esp_err_t va_dsp_hal_exit_low_power() {
  // Low power mode not implemented for CNX20921
  return ESP_OK;
}

esp_err_t va_dsp_hal_stream_pause() { return ESP_OK; }

esp_err_t va_dsp_hal_stream_resume() { return ESP_OK; }
