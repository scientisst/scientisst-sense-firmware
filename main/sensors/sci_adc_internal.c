/** \file adc.c
    \brief ADC interactions functions file.

    This file contains the declarations for the ADC interactions.
*/

#include "sci_adc_internal.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_attr.h"
#include "esp_timer.h"

#define ADC_RESOLUTION ADC_WIDTH_BIT_12
#define ADC1_ATTENUATION ADC_ATTEN_DB_0
#define ADC2_ATTENUATION ADC_ATTEN_DB_11
#define DEFAULT_VREF 1100
#define BATTERY_DIVIDER_FACTOR 2

// ADC
DRAM_ATTR static const uint8_t analog_channels[DEFAULT_ADC_CHANNELS] = {ADC1_CHANNEL_0, ADC1_CHANNEL_3, ADC1_CHANNEL_4,
                                                                        ADC1_CHANNEL_5, ADC1_CHANNEL_6, ADC1_CHANNEL_7};

uint16_t IRAM_ATTR getAdcInternalValue(adc_internal_id_t adc_index, uint8_t adc_channel, uint8_t convert_to_mV_flag)
{
    uint16_t value = 0;
    esp_err_t res;

    if (adc_index == ADC_INTERNAL_1)
    {
        value = (uint16_t)adc1_get_raw(analog_channels[scientisst_device_settings.active_internal_chs[adc_channel]]);
    }
    else if (adc_index == ADC_INTERNAL_2)
    {
        res = adc2_get_raw(adc_channel, ADC_RESOLUTION, (int *)&value);
        if (res != ESP_OK)
        {
            DEBUG_PRINT_E("adc2_get_raw", "Error!");
            return 0;
        }
    }

    if (convert_to_mV_flag)
    {
        value = (uint16_t)esp_adc_cal_raw_to_voltage((uint32_t)value, &(scientisst_device_settings.adc_chars[adc_index])) *
                BATTERY_DIVIDER_FACTOR;
    }

    return value;
}

/**
 * \brief configure Adc
 *
 * \param adc_index 1 or 2
 * \param adc_resolution resolution of the adc
 * \param adc_channel channel to be configured
 */
static void configAdc(adc_internal_id_t adc_index, int adc_channel)
{
    int dummy;
    esp_err_t ret;

    if (adc_index == 1)
    {
        adc1_config_channel_atten(adc_channel, ADC1_ATTENUATION);
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
void initAdc(uint8_t adc1_en, uint8_t adc2_en)
{
    esp_adc_cal_value_t val_type;

    // Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        DEBUG_PRINT_I("eFuse Two Point: Supported");
    }
    else
    {
        DEBUG_PRINT_W("eFuse Two Point: NOT supported");
    }

    // Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        DEBUG_PRINT_I("eFuse Vref: Supported");
    }
    else
    {
        DEBUG_PRINT_W("eFuse Vref: NOT supported");
    }

    // If adc1 is enabled, configure it
    if (adc1_en)
    {
        // Configure only adc1 resolution (it's not done per channel)
        adc1_config_width(ADC_RESOLUTION);

        // Configure each adc channel
        for (int i = 0; i < DEFAULT_ADC_CHANNELS; ++i)
        {
            configAdc(ADC_INTERNAL_1, analog_channels[i]);
        }

        // Characterize ADC
        val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC1_ATTENUATION, ADC_RESOLUTION, DEFAULT_VREF,
                                            &(scientisst_device_settings.adc_chars[ADC_INTERNAL_1]));

        if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
        {
            DEBUG_PRINT_I("ADC1 Calibration type: eFuse Vref");
        }
        else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
        {
            DEBUG_PRINT_I("ADC1 Calibration type: Two Point");
        }
        else
        {
            DEBUG_PRINT_W("ADC1 Calibration type: Default");
        }
    }

    // If adc2 is enabled, configure it
    if (adc2_en)
    {
        // Config ADC2 resolution
        configAdc(ADC_INTERNAL_2, BATTERY_ADC_CH);

        val_type = esp_adc_cal_characterize(ADC_UNIT_2, ADC2_ATTENUATION, ADC_RESOLUTION, DEFAULT_VREF,
                                            &(scientisst_device_settings.adc_chars[ADC_INTERNAL_2]));

        if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
        {
            DEBUG_PRINT_I("ADC2 Calibration type: eFuse Vref");
        }
        else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
        {
            DEBUG_PRINT_I("ADC2 Calibration type: Two Point");
        }
        else
        {
            DEBUG_PRINT_W("ADC2 Calibration type: Default");
        }
    }
}
