/** \file adc.h
    \brief ADC interactions header file.

    This file is the ADC interactions header file. It contains relevant macros
   like pin and ADC channel definitions.
*/

#ifndef _ADC_H
#define _ADC_H

#include <stdint.h>

#include "esp_attr.h"

#include "sci_config.h"
#include "sci_macros_conf.h"

#define ADC_RESOLUTION ADC_WIDTH_BIT_12
#define ADC1_ATTENUATION ADC_ATTEN_DB_0
#define ADC2_ATTENUATION ADC_ATTEN_DB_11

#if _HW_VERSION_ == HW_VERSION_CORE
#define ABAT_ADC_CH ADC2_CHANNEL_5
#elif _HW_VERSION_ == HW_VERSION_CARDIO
#define ABAT_ADC_CH ADC2_CHANNEL_5
#elif _HW_VERSION_ == HW_VERSION_NANO
#define ABAT_ADC_CH ADC1_CHANNEL_2 // GPIO38
#endif

#define DEFAULT_BATTERY_THRESHOLD 3500 ///< mV

#define DEFAULT_SAMPLE_RATE 1 // In Hz
#define MAX_LIVE_MODE_PACKET_SIZE 8

typedef enum
{
    ADC_INTERNAL_1,
    ADC_INTERNAL_2,
} adc_internal_id_t;

void configAdc(int adc_index, int adc_resolution, int adc_channel);
void initAdc(uint8_t adc_resolution, uint8_t adc1_en, uint8_t adc2_en);
uint16_t get_adc_internal_value(adc_internal_id_t adc_index, uint8_t adc_channel, uint8_t convert_to_mV_flag);

#endif
