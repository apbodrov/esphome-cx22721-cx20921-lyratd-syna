/*
 *      Copyright 2018, Espressif Systems (Shanghai) Pte Ltd.
 *  All rights regarding this code and its modifications reserved.
 *
 * This code contains confidential information of Espressif Systems
 * (Shanghai) Pte Ltd. No licenses or other rights express or implied,
 * by estoppel or otherwise are granted herein.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CNX20921_INIT_H
#define CNX20921_INIT_H

#include <freertos/FreeRTOS.h>
#include <esp_err.h>
#include "va_dsp.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PHRASE_LEN 32000 * 0.6   // 32000 * (no. of secs of wakeword)
#define PREROLL_LEN 32000 * 1.1  // 32000 * (no.of sec)  (500ms of Preroll + 600ms of Alexa wakeword)

typedef enum { NO_FLASH_FW = 0, FLASH_FW = 1 } cnx_mode_t;

esp_err_t cnx20921_init(SemaphoreHandle_t semph, int int_pin, int mute_pin, cnx_mode_t flash_fw);

void cnx20921_get_state(enum va_dsp_events *state);
void cnx20921_tap_to_talk();
void cnx20921_start_speech();
void cnx20921_get_preroll(void *data);
size_t cnx20921_stream_audio(int prt_num, void *data, size_t max_len);

void cnx20921_stop_capture();
void cnx20921_mic_mute();
void cnx20921_mic_unmute();

// Microphone gain control
int cx20921SetMicGain(int gain_db);
int cx20921GetMicGain(void);

#ifdef __cplusplus
}
#endif

#endif /* CNX20921_INIT_H */
