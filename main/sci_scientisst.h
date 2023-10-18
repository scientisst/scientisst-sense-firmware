/** \file scientisst.h
    \brief Main header file for the Scientisst firmware.
*/

#pragma once

#include "cJSON.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "sci_macros.h"
#include "sci_version.h"

typedef struct
{
    uint8_t api_mode;
    void (*select_ch_mask_func)(const uint8_t *buff);
} api_config_t;

typedef struct
{
    char ssid[32];
    char password[64];
    com_mode_t com_mode;
    char host_ip[16];
    char port_number[6];
    char bit_when[9];
    char sampling_rate[5];
    char no_channels[2];
    char channels[23];
    char bit_mode[10];
    char port_o1[5];
    char port_o2[5];
    uint8_t is_battery_threshold_inflated;
} op_settings_info_t;

typedef struct
{
    char device_name[17];                              //
    uint16_t battery_threshold;                        // When under this threshold, low battery LED is lit
    uint8_t num_intern_active_chs;                     //
    uint8_t num_extern_active_chs;                     //
    uint8_t active_internal_chs[DEFAULT_ADC_CHANNELS]; ///< If all channels are active: = {5, 4, 3, 2, 1, 0}
    uint8_t active_ext_chs[EXT_ADC_CHANNELS];          ///< If all channels are active: = {7, 6}
    esp_adc_cal_characteristics_t adc_chars[2];        ///< Internal ADC characteristics
    uint8_t gpio_out_state[2];                         ///< Output of 01 & O2 (O0 & O1)
    api_config_t api_config;                           //
    op_settings_info_t op_settings;                    ///< Holds settings that should be saved on flash between reboots
    volatile uint8_t op_mode;                          ///< Flag that indicates if op mode is on (idle, live or config)
    volatile uint8_t send_busy;                        ///< Flag that indicates if send task is busy
    uint32_t sample_rate;                              ///< Sample rate of the acquisition
    uint8_t is_op_settings_valid;                      ///< Indicates if op_settings has been loaded from flash
} scientisst_device_t;

typedef struct
{
    uint8_t *frame_buffer[NUM_BUFFERS]; ///< Buffer that holds the frames to be sent
    uint16_t frame_buffer_length_bytes; ///< Length of each send buffer, set to optimal value depending on com mode
    uint16_t frame_buffer_write_idx;    ///< The index of the first free element in for each buffer
    volatile uint16_t
        frame_buffer_ready_to_send[NUM_BUFFERS]; ///< If 0, buffer is not full/not ready to send. If != 0, then the buffer is
                                                 ///< ready to send and it holds the number of bytes to send
    uint8_t tx_curr_buff;                        ///< Index of the buffer that tx task is currently sending
    uint8_t acq_curr_buff;                       ///< Index of the buffer that acquisition task is currently using
    uint8_t packet_size;                         ///< Current packet size (dependent on number of channels used)
    uint16_t send_threshold; ///< Based on buffer and packet sizes, threshold that marks the buffer as ready to send and
                             ///< changes buffer used for acquisition
    FILE *sd_card_save_file; ///< File where data is saved in SD card mode
    cJSON *json;             ///< JSON object that is used in JSON api mode

} scientisst_buffers_t;

/*********************************************************
 * GLOBAL VARIABLES
 ********************************************************/
extern TaskHandle_t send_task;
extern TaskHandle_t battery_task;
extern TaskHandle_t rcv_task;
extern TaskHandle_t acq_adc1_task;

extern scientisst_device_t scientisst_device_settings;
extern scientisst_buffers_t scientisst_buffers;

void initScientisst(void);
void saveOpSettingsInfo(const op_settings_info_t *pOpSettingsInfo);
