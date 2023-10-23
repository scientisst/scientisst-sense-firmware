/**
 * \file sci_adc_internal.h
 * \brief Header file for sci_adc_internal.c
 */

#pragma once

#include <stdint.h>

#include "sci_scientisst.h"

// ADC2 channel that is used for battery voltage measurement
#ifdef CONFIG_HARDWARE_VERSION_CORE
#define BATTERY_ADC_CH ADC2_CHANNEL_5
#endif
#ifdef CONFIG_HARDWARE_VERSION_CARDIO
#define BATTERY_ADC_CH ADC2_CHANNEL_5
#endif
#ifdef CONFIG_HARDWARE_VERSION_NANO
#define BATTERY_ADC_CH ADC1_CHANNEL_2 // GPIO38
#endif

typedef enum
{
    ADC_INTERNAL_1 = 0,
    ADC_INTERNAL_2,
} adc_internal_id_t; ///< Internal ADC identifiers

void initAdc(uint8_t adc1_en, uint8_t adc2_en);
uint16_t getAdcInternalValue(adc_internal_id_t adc_index, uint8_t adc_channel, uint8_t convert_to_mV_flag);
