/** \file adc.h
    \brief ADC interactions header file.

    This file is the ADC interactions header file. It contains relevant macros
   like pin and ADC channel definitions.
*/

#ifndef _ADC_H
#define _ADC_H

#include <stdio.h>

#include "cJSON.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "spi.h"

#define ADC_RESOLUTION ADC_WIDTH_BIT_12
#define ADC1_ATTENUATION ADC_ATTEN_DB_0
#define ADC_RES_VALUE \
    12  ///< ADC Resolution Value: Tem que ser o valor indicado por
        ///< ADC_RESOLUTION
#define ADC2_ATTENUATION ADC_ATTEN_DB_11

#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5
#define A6 6
#define A7 7

#define A0_ADC_CH ADC1_CHANNEL_0  ///< AI1, GPIO36
#define A1_ADC_CH ADC1_CHANNEL_3  ///< AI2, GPIO39
#define A2_ADC_CH ADC1_CHANNEL_4  ///< AI3, GPIO32
#define A3_ADC_CH ADC1_CHANNEL_5  ///< AI4, GPIO33
#define A4_ADC_CH ADC1_CHANNEL_6  ///< I5, GPIO34
#define A5_ADC_CH ADC1_CHANNEL_7  ///< AI6, GPIO35

#if HW_VERSION == HW_VERSION_CORE
#define ABAT_ADC_CH ADC2_CHANNEL_5
#elif HW_VERSION == HW_VERSION_CARDIO
#define ABAT_ADC_CH ADC1_CHANNEL_5
#elif HW_VERSION == HW_VERSION_NANO
#define ABAT_ADC_CH ADC1_CHANNEL_2  // GPIO38
#endif

#define A0_IO GPIO_NUM_36  ///< AI1
#define A1_IO GPIO_NUM_39  ///< AI2
#define A2_IO GPIO_NUM_32  ///< AI3
#define A3_IO GPIO_NUM_33  ///< AI4
#define A4_IO GPIO_NUM_34  ///< AI5
#define A5_IO GPIO_NUM_35  ///< AI6

#define DEFAULT_BATTERY_THRESHOLD 3500  ///< mV

#define DEFAULT_SAMPLE_RATE 1  // In Hz
#define MAX_LIVE_MODE_PACKET_SIZE 8

void configAdc(int adc_index, int adc_resolution, int adc_channel);
void initAdc(uint8_t adc_resolution, uint8_t adc1_en, uint8_t adc2_en);
void acquireAdc1Channels(uint8_t* frame);
void IRAM_ATTR acquireChannelsScientisst(uint8_t* frame);
void acquireChannelsJson(uint8_t* frame);

#define CALC_BYTE_CRC(_crc, _byte, _crc_table)        \
    ({                                                \
        (_crc) = _crc_table[(_crc)] ^ ((_byte) >> 4); \
        (_crc) = _crc_table[(_crc)] ^ ((_byte)&0x0F); \
    })

#define READ_ADC1(value, channel) ({ (value) = adc1_get_raw((channel)); })

#define READ_ADC2(value, channel) \
    ({ adc2_get_raw((channel), ADC_RESOLUTION, &(value)); })

#endif