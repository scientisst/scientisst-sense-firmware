/**
 * \file scientisst.h
 * \brief Central Header for the Scientisst Firmware.
 *
 * Includes necessary dependencies and declares structures and global variables like operational settings, buffer management,
 * and task handlers.
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
    uint8_t api_mode;                                 ///< Mode of the API being used (e.g., BITalino, Scientisst, JSON).
    void (*select_ch_mask_func)(const uint8_t *buff); ///< Function pointer to process channel masks.
} api_config_t;                                       ///< Structure holding the API configuration.

typedef struct
{
    // Network configuration parameters.
    char ssid[32];       // Network SSID for connectivity.
    char password[64];   // Password for network security.
    com_mode_t com_mode; // Communication mode (e.g., TCP, UDP, WS, BT, etc.).

    // Server or peripheral configurations.
    char host_ip[16];    // IP address of the host server.
    char port_number[6]; // Network port for communication.

    // Data acquisition and channel settings.
    char bit_when[9];         // Timing configuration for BITalino mode.
    char sampling_rate_hz[5]; // Rate of data sampling.
    char no_channels[2];      // Number of active channels.
    char channels[23];        // Configuration of individual channels.
    char bit_mode[10];        // BITalino operation mode.

    // Output port configurations.
    char port_o1[5]; // Configuration for output port O1.
    char port_o2[5]; // Configuration for output port O2.

    uint8_t is_battery_threshold_inflated; ///< Flag indicating if the battery threshold is inflated. Kept here so it can be
                                           ///< saved on flash.
} op_settings_info_t;                      ///< Structure holding the operational settings. Used to save/load from flash.

typedef struct
{
    // Device identification and configuration.
    char device_name[17];       ///< Name of the device.
    uint16_t battery_threshold; ///< Low battery indication threshold.

    // Channel configuration and ADC characteristics.
    uint8_t num_intern_active_chs;                     //< Number of active internal channels.
    uint8_t num_extern_active_chs;                     //< Number of active external channels.
    uint8_t active_internal_chs[DEFAULT_ADC_CHANNELS]; //< Active internal channels ID.
    uint8_t active_ext_chs[EXT_ADC_CHANNELS];          //< Active external channels ID.
    esp_adc_cal_characteristics_t adc_chars[2];        //< Calibration characteristics for the ADCs.

    // GPIO and operational settings.
    uint8_t gpio_out_state[2];      ///< States of GPIO outputs O1 and O2.
    api_config_t api_config;        ///< API configuration structure.
    op_settings_info_t op_settings; ///< Operational settings structure.

    // Operational flags and parameters.
    volatile uint8_t op_mode;     // Current operational mode (idle, live, config).
    volatile uint8_t send_busy;   ///< Flag indicating if the device is currently sending data, also used to pause calling
                                  ///< write functions when congested.
    uint32_t sample_rate_hz;      ///< Current sampling rate for data acquisition.
    uint8_t is_op_settings_valid; ///< True if valid operational settings were loaded from flash.
} scientisst_device_t;            ///< Structure holding the device configuration and settings.

typedef struct
{
    // Buffer management for data transmission.
    uint8_t *frame_buffer[NUM_BUFFERS];                        ///< Data frames buffers.
    uint16_t frame_buffer_length_bytes;                        ///< Size of each buffer.
    uint16_t frame_buffer_write_idx;                           ///< Write index for the current buffer.
    volatile uint16_t frame_buffer_ready_to_send[NUM_BUFFERS]; ///< Status array indicating ready-to-send buffers.

    // Transmission operational parameters.
    uint8_t tx_curr_buff;    ///< Index of the buffer currently being transmitted.
    uint8_t acq_curr_buff;   ///< Index of the buffer for data acquisition.
    uint8_t packet_size;     ///< Size of the frames (data packets).
    uint16_t send_threshold; ///< Threshold to trigger data transmission.
    FILE *sd_card_save_file; ///<  File pointer for data saving on the SD card.
    cJSON *json;             ///< JSON object for JSON API mode.
} scientisst_buffers_t;      ///< Structure holding the buffers and transmission parameters.

// Global task handlers.
extern TaskHandle_t send_task;     ///< Task handling the data transmission process.
extern TaskHandle_t battery_task;  ///< Task monitoring the battery status.
extern TaskHandle_t rcv_task;      ///< Task handling data reception.
extern TaskHandle_t acq_adc1_task; ///< Task managing data acquisition from ADC.

// Global configurations and buffers
extern scientisst_device_t scientisst_device_settings; ///< Central device configuration and settings.
extern scientisst_buffers_t scientisst_buffers;        ///< Buffers and transmission parameters.

void initScientisst(void);
void saveOpSettingsInfo(const op_settings_info_t *pOpSettingsInfo);
