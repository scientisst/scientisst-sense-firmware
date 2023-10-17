/** \file scientisst.h
    \brief Main header file for the Scientisst firmware.
*/

#ifndef SCIENTISST_H
#define SCIENTISST_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
#include "drivers/include/sci_wifi.h"
#include "esp_adc_cal.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdmmc_cmd.h"
#include "sys/time.h"
#include "time.h"

#include "sci_com.h"
#include "sci_version.h"

#define MAX_BUFFER_SIZE_SDCARD (1024 * 7)
#define NUM_BUFFERS_SDCARD (16)

typedef struct
{
    uint8_t api_mode;
    void (*select_ch_mask_func)();
} api_config_t;

typedef struct
{
    char device_name[17];                                //
    uint16_t battery_threshold;                          //
    uint8_t num_intern_active_chs;                       //
    uint8_t num_extern_active_chs;                       //
    uint8_t active_internal_chs[DEFAULT_ADC_CHANNELS];   ///< If all channels are active: = {5, 4, 3, 2, 1, 0}
    uint8_t active_ext_chs[EXT_ADC_CHANNELS];            ///< If all channels are active: = {7, 6}
    const uint8_t analog_channels[DEFAULT_ADC_CHANNELS]; //
    esp_adc_cal_characteristics_t adc_chars[2];          ///< Internal ADC characteristics
    uint8_t gpio_out_state[2];                           ///< Output of 01 & O2 (O0 & O1)
    api_config_t api_config;                             //
    op_settings_info_t op_settings;                      ///< Holds settings that should be saved on flash between reboots
    uint8_t op_mode;                                     ///< Flag that indicastes if op mode is on (idle, live or config)
    uint8_t send_busy;                                   ///< Flag that indicates if send task is busy
    uint32_t sample_rate;                                ///< Sample rate of the acquisition
    uint8_t is_op_settings_valid;                        ///< Indicates if op_settings has been loaded from flash
} scientisst_device_t;

typedef struct
{
    uint8_t *frame_buffer[NUM_BUFFERS];           ///< Buffer that holds the frames to be sent
    uint32_t frame_buffer_length_bytes;           ///< Length of each send buffer, set to optimal value depending on com mode
    uint16_t frame_buffer_write_idx[NUM_BUFFERS]; ///< The index of the first free element in for each buffer
    volatile uint16_t frame_buffer_ready_to_send[NUM_BUFFERS]; ///< If element 0 is set to 1, bt task has to send snd_buff[0]
    uint8_t tx_curr_buff;                                      ///< Index of the buffer that tx task is currently sending
    uint8_t acq_curr_buff;   ///< Index of the buffer that acquisition task is currently using
    uint8_t packet_size;     ///< Current packet size (dependent on number of channels used)
    uint32_t send_threshold; ///< Based on buffer and packet sizes, threshold that marks the buffer as ready to send and
                             ///< changes buffer used for acquisition
    FILE *sd_card_save_file; ///< File where data is saved in SD card mode
    cJSON *json;             ///< JSON object that is used in JSON api mode

} scientisst_buffers_t;

/*********************************************************
 * GLOBAL VARIABLES
 ********************************************************/
extern TaskHandle_t send_task;
extern TaskHandle_t abat_task;
extern TaskHandle_t rcv_task;
extern TaskHandle_t acq_adc1_task;

extern scientisst_device_t scientisst_device_settings;
extern scientisst_buffers_t scientisst_buffers;

void initScientisst(void);

#endif
