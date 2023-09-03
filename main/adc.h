/** \file adc.h
    \brief ADC interactions header file.

    This file is the ADC interactions header file. It contains relevant macros
   like pin and ADC channel definitions.
*/

#ifndef _ADC_H
#define _ADC_H

#include "config.h"
#include "esp_attr.h"
#include "macros_conf.h"

#define ADC_RESOLUTION ADC_WIDTH_BIT_12
#define ADC1_ATTENUATION ADC_ATTEN_DB_0
#define ADC2_ATTENUATION ADC_ATTEN_DB_11

#if HW_VERSION == HW_VERSION_CORE
#define ABAT_ADC_CH ADC2_CHANNEL_5
#elif HW_VERSION == HW_VERSION_CARDIO
#define ABAT_ADC_CH ADC2_CHANNEL_5
#elif HW_VERSION == HW_VERSION_NANO
#define ABAT_ADC_CH ADC1_CHANNEL_2 // GPIO38
#endif

#define DEFAULT_BATTERY_THRESHOLD 3500 ///< mV

#define DEFAULT_SAMPLE_RATE 1 // In Hz
#define MAX_LIVE_MODE_PACKET_SIZE 8

void configAdc(int adc_index, int adc_resolution, int adc_channel);
void initAdc(uint8_t adc_resolution, uint8_t adc1_en, uint8_t adc2_en);
void acquireAdc1Channels(uint8_t *frame);
void IRAM_ATTR acquireChannelsScientisst(uint8_t *frame);
void acquireChannelsJson(uint8_t *frame);

#define CALC_BYTE_CRC(_crc, _byte, _crc_table)                                                                              \
    ({                                                                                                                      \
        (_crc) = _crc_table[(_crc)] ^ ((_byte) >> 4);                                                                       \
        (_crc) = _crc_table[(_crc)] ^ ((_byte)&0x0F);                                                                       \
    })

#endif