#ifndef _MAIN_H
#define _MAIN_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "time.h"
#include "sys/time.h"
#include "driver/timer.h"
#include "adc.h"
#include "com.h"
#include "i2c.h"
#include "spi.h"

#define FIRMWARE_VERSION_STR "ScientISST1.0\n"
#define FIRMWARE_BITALINO_VERSION_STR "BITalino_v5.1\n"

extern TaskHandle_t acquiring_1_task;
extern TaskHandle_t abat_task;
extern TaskHandle_t acquiring_i2c_task;
extern uint32_t bt_client;
extern uint8_t snd_buff[NUM_BUFFERS][MAX_BUFFER_SIZE];                                
extern uint8_t packet_size;
extern uint16_t snd_buff_idx[NUM_BUFFERS];
extern uint8_t bt_buffs_to_send[NUM_BUFFERS];                              
extern DRAM_ATTR const uint8_t crc_table[16];
extern uint8_t crc_seq;
extern const uint8_t packet_size_num_chs[DEFAULT_ADC_CHANNELS+1];        
extern DRAM_ATTR const uint8_t analog_channels[DEFAULT_ADC_CHANNELS];
extern uint8_t active_internal_chs[DEFAULT_ADC_CHANNELS];                                               
extern uint8_t num_intern_active_chs;
extern uint8_t live_mode;
extern uint32_t sample_rate;
extern I2c_Sensor_State i2c_sensor_values;
extern spi_device_handle_t adc_ext_spi_handler;
extern esp_adc_cal_characteristics_t adc1_chars;
extern esp_adc_cal_characteristics_t adc2_chars;
extern char bt_device_name[17];
extern uint8_t bt_write_busy;
extern SemaphoreHandle_t bt_buffs_to_send_mutex;
extern uint16_t send_threshold;
extern uint8_t bt_curr_buff;
extern uint8_t acq_curr_buff;
extern Api_Config api_config;
extern uint8_t active_ext_chs[EXT_ADC_CHANNELS];
extern uint8_t num_extern_active_chs;
extern cJSON *json;
extern DRAM_ATTR const uint8_t sin10Hz[100];
extern uint8_t sim_flag;
extern uint8_t sin_i;
extern uint8_t gpio_out_state[2];
extern spi_transaction_t adc_ext_trans;
extern uint16_t battery_threshold;

#endif
