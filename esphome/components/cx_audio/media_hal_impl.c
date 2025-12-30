#include <string.h>
#include <stdlib.h>
#include <esp_log.h>
#include <media_hal.h>
#include <media_hal_codec_init.h>
#include <media_hal_playback.h>
#include <cx22721.h>

#define HAL_TAG "MEDIA_HAL"

static media_hal_t *media_hal_handle = NULL;
static uint8_t volume_prv = MEDIA_HAL_VOL_DEFAULT;

media_hal_t* media_hal_init(media_hal_config_t *media_hal_cfg, media_hal_playback_cfg_t *media_hal_playback_cfg)
{
    if (!media_hal_handle) {
        ESP_LOGI(HAL_TAG, "Initializing Media HAL...");
        
        /* Initialize playback */
        if (media_hal_playback_cfg) {
            media_hal_init_playback(media_hal_playback_cfg);
        }

        /* Initialize codec structure */
        media_hal_t *media_hal = (media_hal_t *) calloc(1, sizeof(media_hal_t));
        if (media_hal == NULL) {
            ESP_LOGE(HAL_TAG, "Failed to allocate memory for media_hal");
            return NULL;
        }
        
        media_hal->media_hal_lock = xSemaphoreCreateMutex();
        if (media_hal->media_hal_lock == NULL) {
            ESP_LOGE(HAL_TAG, "Failed to create media_hal_lock");
            free(media_hal);
            return NULL;
        }

        /* Set up function pointers for cx22721 */
        media_hal->audio_codec_initialize = cx22721_init;
        media_hal->audio_codec_deinitialize = cx22721_deinit;
        media_hal->audio_codec_set_state = cx22721_set_state;
        media_hal->audio_codec_set_i2s_clk = cx22721_set_i2s_clk;
        media_hal->audio_codec_config_format = cx22721_config_format;
        media_hal->audio_codec_control_volume = cx22721_set_vol;
        media_hal->audio_codec_get_volume = cx22721_get_volume;
        media_hal->audio_codec_set_mute = cx22721_set_mute;
        media_hal->audio_codec_powerup = cx22721_powerup;
        media_hal->audio_codec_powerdown = cx22721_powerdown;

        xSemaphoreTake(media_hal->media_hal_lock, portMAX_DELAY);
        if (media_hal->audio_codec_initialize) {
            media_hal->audio_codec_initialize(media_hal_cfg);
        }
        xSemaphoreGive(media_hal->media_hal_lock);
        
        volume_prv = MEDIA_HAL_VOL_DEFAULT;
        media_hal_handle = media_hal;
        ESP_LOGI(HAL_TAG, "Media HAL initialized successfully");
    } else {
        ESP_LOGW(HAL_TAG, "Media HAL already initialized");
    }
    return media_hal_handle;
}

media_hal_t* media_hal_get_handle()
{
    if (!media_hal_handle) {
        ESP_LOGE(HAL_TAG, "Media HAL not initialized");
    }
    return media_hal_handle;
}

esp_err_t media_hal_control_volume(media_hal_t* media_hal, uint8_t volume)
{
    if (!media_hal) return ESP_FAIL;
    esp_err_t ret;
    xSemaphoreTake(media_hal->media_hal_lock, portMAX_DELAY);
    volume_prv = volume;
    if (volume == 0) {
        ret = media_hal->audio_codec_set_mute(true);
    } else {
        ret = media_hal->audio_codec_control_volume(volume);
    }
    xSemaphoreGive(media_hal->media_hal_lock);
    return ret;
}

esp_err_t media_hal_set_mute(media_hal_t* media_hal, bool mute)
{
    if (!media_hal) return ESP_FAIL;
    esp_err_t ret;
    xSemaphoreTake(media_hal->media_hal_lock, portMAX_DELAY);
    ret = media_hal->audio_codec_set_mute(mute);
    xSemaphoreGive(media_hal->media_hal_lock);
    return ret;
}

esp_err_t media_hal_get_volume(media_hal_t* media_hal, uint8_t *volume)
{
    if (!media_hal || !volume) return ESP_FAIL;
    *volume = volume_prv;
    return ESP_OK;
}

esp_err_t media_hal_deinit(media_hal_t* media_hal)
{
    if (!media_hal) return ESP_FAIL;
    xSemaphoreTake(media_hal->media_hal_lock, portMAX_DELAY);
    if (media_hal->audio_codec_deinitialize) {
        media_hal->audio_codec_deinitialize(0);
    }
    xSemaphoreGive(media_hal->media_hal_lock);
    vSemaphoreDelete(media_hal->media_hal_lock);
    free(media_hal);
    media_hal_handle = NULL;
    return ESP_OK;
}

esp_err_t media_hal_set_state(media_hal_t* media_hal, media_hal_codec_mode_t mode, media_hal_sel_state_t media_hal_state)
{
    if (!media_hal) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    xSemaphoreTake(media_hal->media_hal_lock, portMAX_DELAY);
    if (media_hal->audio_codec_set_state) {
        ret = media_hal->audio_codec_set_state(mode, media_hal_state);
    }
    xSemaphoreGive(media_hal->media_hal_lock);
    return ret;
}

esp_err_t media_hal_config_format(media_hal_t* media_hal, media_hal_codec_mode_t mode, media_hal_format_t fmt)
{
    if (!media_hal) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    xSemaphoreTake(media_hal->media_hal_lock, portMAX_DELAY);
    if (media_hal->audio_codec_config_format) {
        ret = media_hal->audio_codec_config_format(mode, fmt);
    }
    xSemaphoreGive(media_hal->media_hal_lock);
    return ret;
}

esp_err_t media_hal_set_clk(media_hal_t* media_hal, media_hal_codec_mode_t mode, uint32_t rate, media_hal_bit_length_t bits_per_sample)
{
    if (!media_hal) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    xSemaphoreTake(media_hal->media_hal_lock, portMAX_DELAY);
    if (media_hal->audio_codec_set_i2s_clk) {
        ret = media_hal->audio_codec_set_i2s_clk(mode, bits_per_sample);
    }
    xSemaphoreGive(media_hal->media_hal_lock);
    return ret;
}

esp_err_t media_hal_powerup(media_hal_t* media_hal)
{
    if (!media_hal) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    xSemaphoreTake(media_hal->media_hal_lock, portMAX_DELAY);
    if (media_hal->audio_codec_powerup) {
        ret = media_hal->audio_codec_powerup();
    }
    xSemaphoreGive(media_hal->media_hal_lock);
    return ret;
}

esp_err_t media_hal_powerdown(media_hal_t* media_hal)
{
    if (!media_hal) return ESP_FAIL;
    esp_err_t ret = ESP_OK;
    xSemaphoreTake(media_hal->media_hal_lock, portMAX_DELAY);
    if (media_hal->audio_codec_powerdown) {
        ret = media_hal->audio_codec_powerdown();
    }
    xSemaphoreGive(media_hal->media_hal_lock);
    return ret;
}

esp_err_t media_hal_register_volume_change_cb(media_hal_t *media_hal, void (*callback)(int volume))
{
    if (!media_hal) return ESP_FAIL;
    for (int i = 0; i < VOLUME_CHANGE_CB_MAX; i++) {
        if (media_hal->volume_change_notify_cb[i] == NULL) {
            media_hal->volume_change_notify_cb[i] = callback;
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

esp_err_t media_hal_deregister_volume_change_cb(media_hal_t *media_hal, void (*callback)(int volume))
{
    if (!media_hal) return ESP_FAIL;
    for (int i = 0; i < VOLUME_CHANGE_CB_MAX; i++) {
        if (media_hal->volume_change_notify_cb[i] == callback) {
            media_hal->volume_change_notify_cb[i] = NULL;
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

// Stub for playback initialization if not in monolith
void * __attribute__((weak)) media_hal_init_playback(media_hal_playback_cfg_t *cfg)
{
    ESP_LOGI(HAL_TAG, "media_hal_init_playback (stub) called");
    return NULL;
}
