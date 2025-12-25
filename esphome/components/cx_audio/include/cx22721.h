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

#ifndef CX22721_H

#define CX22721_H

#include <media_hal.h>
#include <driver/i2c.h>
#include "esp_err.h"
#include "esp_log.h"

#define I2C_BUS_NO I2C_NUM_0

#define CX22721_PWR_PIN 4
#define SDA_PIN 18
#define SCL_PIN 23
#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

#define CODEC_ADDR 0x33

esp_err_t cx22721_init(media_hal_config_t *media_hal_conf);

esp_err_t cx22721_set_vol(int vol);

esp_err_t cx22721_set_mute(bool mute);

esp_err_t cx22721_get_volume(uint8_t *vol);

esp_err_t cx22721_deinit(int port_num);

esp_err_t cx22721_powerup();

esp_err_t cx22721_powerdown();

esp_err_t cx22721_set_state(media_hal_codec_mode_t mode, media_hal_sel_state_t media_hal_state);

esp_err_t cx22721_config_format(media_hal_codec_mode_t mode, media_hal_format_t fmt);

esp_err_t cx22721_set_i2s_clk(media_hal_codec_mode_t mode, media_hal_bit_length_t media_hal_bit_length);

#endif /* CX22721_H */
