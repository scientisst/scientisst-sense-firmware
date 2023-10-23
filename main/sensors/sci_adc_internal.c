/**
 * \file sci_adc_internal.c
 * \brief ADC interactions functions file.
 *
 * This file contains the declarations for the ADC interactions. It manages the interactions with both internal ADCs,
 * including reading raw data, converting data, and configuring ADC settings.
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
DRAM_ATTR static const uint8_t analog_channels[DEFAULT_ADC_CHANNELS] = {
    ADC1_CHANNEL_0, ADC1_CHANNEL_3, ADC1_CHANNEL_4,
    ADC1_CHANNEL_5, ADC1_CHANNEL_6, ADC1_CHANNEL_7}; ///< Analog channels used by the ADC.

/**
 * \brief Get a value from the internal ADC.
 *
 * This function reads a value from the specified channel of the specified ADC. It can optionally convert the
 * reading from raw data to millivolts.
 *
 * \param[in] adc_index Identifier of the ADC to be used (ADC_INTERNAL_1 or ADC_INTERNAL_2).
 * \param[in] adc_channel Channel number to read from.
 * \param[in] convert_to_mV_flag Flag indicating whether to convert the reading to millivolts.
 *
 * \return The ADC reading. If convert_to_mV_flag is set, the reading is in millivolts.
 */
uint16_t IRAM_ATTR getAdcInternalValue(adc_internal_id_t adc_index, uint8_t adc_channel, uint8_t convert_to_mV_flag)
{
    int value = 0;
    esp_err_t res;

    if (adc_index == ADC_INTERNAL_1)
    {
        value = (uint16_t)adc1_get_raw(analog_channels[scientisst_device_settings.active_internal_chs[adc_channel]]);
    }
    else if (adc_index == ADC_INTERNAL_2)
    {
        res = adc2_get_raw(adc_channel, ADC_RESOLUTION, &value);
        if (res != ESP_OK)
        {
            DEBUG_PRINT_E("adc2_get_raw", "Error!");
            return 0;
        }
    }

    if (convert_to_mV_flag)
    {
        value = esp_adc_cal_raw_to_voltage((uint32_t)value, &(scientisst_device_settings.adc_chars[adc_index])) *
                BATTERY_DIVIDER_FACTOR;
    }

    return (uint16_t)value;
}

/**
 * \brief Configure an ADC channel.
 *
 * This function configures a specific channel of an ADC for use.
 *
 * \param[in] adc_index Identifier of the ADC to be used (ADC_INTERNAL_1 or ADC_INTERNAL_2).
 * \param[in] adc_channel Channel number to configure.
 *
 * \return None.
 */
static void configAdc(adc_internal_id_t adc_index, int adc_channel)
{
    int dummy;
    esp_err_t ret;

    if (adc_index == ADC_INTERNAL_1)
    {
        adc1_config_channel_atten(adc_channel, ADC1_ATTENUATION);
    }
    else if (adc_index == ADC_INTERNAL_2)
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
 * \brief Initialize the ADCs.
 *
 * This function initializes the ADCs based on specified parameters. It sets up both ADC units, configures their
 * channels, and characterizes the ADCs for accurate readings. It also checks for calibration values stored in eFuse.
 *
 * \param[in] adc1_en Enable flag for ADC1. If set, ADC1 is configured and characterized.
 * \param[in] adc2_en Enable flag for ADC2. If set, ADC2 is configured and characterized.
 *
 * \return None.
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
