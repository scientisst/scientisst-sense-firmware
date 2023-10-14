/** \file adc.c
    \brief ADC interactions functions file.

    This file contains the declarations for the ADC interactions.
*/

#include "sci_adc_internal.h"

#include <stdio.h>

#include "cJSON.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "sci_adc_external.h"
#include "sci_com.h"
#include "sci_config.h"
#include "sci_gpio.h"
#include "sci_macros.h"
#include "sci_macros_conf.h"
#include "sci_scientisst.h"
#include "sci_task_imu.h"

#define DEFAULT_VREF 1100

/**
 * \brief configure Adc
 *
 * \param adc_index 1 or 2
 * \param adc_resolution resolution of the adc
 * \param adc_channel channel to be configured
 */
void configAdc(int adc_index, int adc_resolution, int adc_channel)
{
    int dummy;
    esp_err_t ret;

    if (adc_index == 1)
    {
        adc1_config_channel_atten(adc_channel, ADC1_ATTENUATION); // Atennuation to get a input
    }
    else if (adc_index == 2)
    {
        adc2_config_channel_atten(adc_channel, ADC2_ATTENUATION);
        ret = adc2_get_raw(adc_channel, ADC_RESOLUTION, &dummy);

        if (ret == ESP_ERR_TIMEOUT)
        {
            DEBUG_PRINT_E("configAdc", "CAN'T USE ADC2, WIFI IS ENABLED");
        }
    }
}

/**
 * \brief initialize Adc
 *
 * \param adc_resolution resolution of the adc
 * \param adc1_en enable adc1
 * \param adc2_en enable adc2
 */
void initAdc(uint8_t adc_resolution, uint8_t adc1_en, uint8_t adc2_en)
{
    uint8_t i;
    esp_adc_cal_value_t val_type;

    // Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        DEBUG_PRINT_I("eFuse Two Point: Supported\n");
    }
    else
    {
        DEBUG_PRINT_W("eFuse Two Point: NOT supported");
    }

    // Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        DEBUG_PRINT_I("eFuse Vref: Supported\n");
    }
    else
    {
        DEBUG_PRINT_W("eFuse Vref: NOT supported\n");
    }

    // If adc1 is enabled, configure it
    if (adc1_en)
    {
        // Configure only adc1 resolution (it's not done per channel)
        adc1_config_width(ADC_RESOLUTION);

        // Configure each adc channel
        for (i = 0; i < DEFAULT_ADC_CHANNELS; ++i)
        {
            configAdc(1, adc_resolution, analog_channels[i]);
        }

        // Characterize ADC
        val_type =
            esp_adc_cal_characterize(ADC_UNIT_1, ADC1_ATTENUATION, ADC_RESOLUTION, DEFAULT_VREF, &adc1_chars);

        if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
        {
            DEBUG_PRINT_W("ADC1 Calibration type: eFuse Vref");
        }
        else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
        {
            DEBUG_PRINT_W("ADC1 Calibration type: Two Point");
        }
        else
        {
            DEBUG_PRINT_E("ADC1 Calibration type: Default");
        }
    }

    // If adc2 is enabled, configure it
    if (adc2_en)
    {
        // Config ADC2 resolution
        configAdc(2, adc_resolution, ABAT_ADC_CH);

        val_type =
            esp_adc_cal_characterize(ADC_UNIT_2, ADC2_ATTENUATION, ADC_RESOLUTION, DEFAULT_VREF, &adc2_chars);

        if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
        {
            DEBUG_PRINT_W("ADC2 Calibration type: eFuse Vref");
        }
        else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
        {
            DEBUG_PRINT_W("ADC2 Calibration type: Two Point");
        }
        else
        {
            DEBUG_PRINT_E("ADC2 Calibration type: Default");
        }
    }
}
