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
#include "com.h"
#include "driver/spi_master.h"
#include "driver/timer.h"
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
#include "version.h"
#include "wifi.h"

#define FIRMWARE_BITALINO_VERSION "BITalino_v5.1\n"

extern TaskHandle_t send_task;
extern TaskHandle_t abat_task;
extern TaskHandle_t rcv_task;
extern TaskHandle_t acq_adc1_task;
extern TaskHandle_t acq_adc_ext_task;
extern int send_fd;
extern uint8_t* snd_buff[NUM_BUFFERS];
extern uint32_t send_buff_len;
extern uint8_t packet_size;
extern uint16_t snd_buff_idx[];
extern uint8_t bt_buffs_to_send[];
extern DRAM_ATTR const uint8_t crc_table[];
extern uint16_t crc_seq;
extern DRAM_ATTR const uint8_t analog_channels[];
extern uint8_t active_internal_chs[];
extern uint8_t num_intern_active_chs;
extern uint8_t op_mode;
extern uint32_t sample_rate;
// extern I2c_Sensor_State i2c_sensor_values;
extern spi_device_handle_t adc_ext_spi_handler;
extern esp_adc_cal_characteristics_t adc1_chars;
extern esp_adc_cal_characteristics_t adc2_chars;
extern char device_name[];
extern uint8_t send_busy;
extern SemaphoreHandle_t bt_buffs_to_send_mutex;
extern SemaphoreHandle_t spi_mutex;
extern sdmmc_host_t sd_spi_host;
extern uint16_t send_threshold;
extern uint8_t bt_curr_buff;
extern uint8_t acq_curr_buff;
extern Api_Config api_config;
extern uint8_t active_ext_chs[];
extern uint8_t num_extern_active_chs;
extern cJSON* json;
extern DRAM_ATTR const uint8_t sin10Hz[];
extern uint8_t sim_flag;
extern uint8_t sin_i;
extern uint8_t gpio_out_state[];
extern spi_transaction_t adc_ext_trans;
extern uint16_t battery_threshold;
extern uint8_t com_mode;
extern op_settings_info_t op_settings;
extern esp_err_t (*send_func)(uint32_t, int, uint8_t*);
extern uint8_t is_op_settings_valid;
extern esp_netif_t* netif_object;
extern uint8_t first_failed_send;
extern uint32_t ext_adc_raw_data[3];

void initScientisst(void);

#endif
