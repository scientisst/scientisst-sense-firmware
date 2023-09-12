/** \file adc.c
    \brief ADC interactions functions file.

    This file contains the declarations for the ADC interactions.
*/

#include <stdio.h>

#include "adc.h"
#include "cJSON.h"
#include "com.h"
#include "config.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "gpio.h"
#include "macros.h"
#include "macros_conf.h"
#include "scientisst.h"
#include "spi.h"

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

/**
 * \brief acquire adc1 channels
 *
 * Acquires and stores into frame the channel acquisitions of adc1. This is used
 * for Bitalino mode and API (Legacy).
 *
 * \param frame pointer of frame position in memory to store the adc1 channels
 */
void acquireAdc1Channels(uint8_t *frame)
{
    uint16_t adc_res[6] = {0, 0, 0, 0, 0, 0};
    uint8_t io_state = 0;
    uint8_t crc = 0;

    for (int i = 0; i < num_intern_active_chs; ++i)
    {
        // 2 lsl because adc resolution is 12bits
        adc_res[i] = adc1_get_raw(analog_channels[active_internal_chs[i]]) >> 2;
        DEBUG_PRINT_I("acquireAdc1Channels", "(adc_res)A%d=%d", active_internal_chs[i], adc_res[i]);
    }

    // Get the IO states
    io_state = gpio_get_level(I0_IO) << 7;
    io_state |= gpio_get_level(I1_IO) << 6;
    io_state |= gpio_out_state[0] << 5;
    io_state |= gpio_out_state[1] << 4;

    frame[packet_size - 2] = io_state;

    *(uint16_t *)(frame + packet_size - 3) |= adc_res[num_intern_active_chs - 1] << 2;
    if (num_intern_active_chs > 1)
    {
        *(uint16_t *)(frame + packet_size - 4) |= adc_res[num_intern_active_chs - 2];
    }
    if (num_intern_active_chs > 2)
    {
        *(uint16_t *)(frame + packet_size - 6) |= adc_res[num_intern_active_chs - 3] << 6;
    }
    if (num_intern_active_chs > 3)
    {
        *(uint16_t *)(frame + packet_size - 7) |= adc_res[num_intern_active_chs - 4] << 4;
    }
    if (num_intern_active_chs > 4)
    {
        *(uint16_t *)frame |= (adc_res[num_intern_active_chs - 5] & 0x3F0) << 2;
    } // Only the 6 upper bits of the 10-bit value are used
    if (num_intern_active_chs > 5)
    {
        frame[0] |= adc_res[num_intern_active_chs - 6] >> 4;
    } // Only the 6 upper bits of the 10-bit value are used

    // calculate CRC (except last byte (seq+CRC) )
    for (int i = 0; i < packet_size - 1; ++i)
    {
        // calculate CRC nibble by nibble
        CALC_BYTE_CRC(crc, frame[i], crc_table);
    }

    // calculate CRC for last byte (seq+CRC)
    crc = crc_table[crc] ^ (crc_seq & 0x0F);
    crc = (crc_seq << 4) | crc_table[crc];

    // store CRC and Seq in the last byte of the packet
    frame[packet_size - 1] = crc;

    ++crc_seq;
}

/**
 * \brief acquire adc1 channels
 *
 * Acquires and stores into frame the channel acquisitions of adc1. This is used
 * for ScientISST mode and API (Current).
 *
 * \param frame pointer of frame position in memory to store the adc1 channels.
 */
void IRAM_ATTR acquireChannelsScientisst(uint8_t *frame)
{
    uint8_t io_state = 0;
    uint8_t crc = 0;
    uint8_t frame_next_wr = 0;
    uint8_t wr_mid_byte_flag = 0;
    uint16_t adc_internal_res[6] = {0, 0, 0, 0, 0, 0};

    // Get the IO states
    io_state = gpio_get_level(I0_IO) << 7;
    io_state |= gpio_get_level(I1_IO) << 6;
    io_state |= (gpio_out_state[0] & 0b1) << 5;
    io_state |= (gpio_out_state[1] & 0b1) << 4;

    // Get raw values from A1 to A6 (A1 to A6)
    for (int i = 0; i < num_intern_active_chs; ++i)
    {
        adc_internal_res[i] = adc1_get_raw(analog_channels[active_internal_chs[i]]);
    }

#if _ADC_EXT_ != NO_EXT_ADC
    if (num_extern_active_chs > 0)
    {
        // Get raw values from AX1 & AX2 (A6 and A7)
        mcpReadADCValues(REG_ADCDATA, 4 * num_extern_active_chs);

        // Store values of external channels into frame
        for (int i = 0; i < num_extern_active_chs; ++i)
        {
            uint32_t adc_external_res = 1; // If the raw value is not found, it stays 1. Usefull for debugging
            for (int j = 0; j < 3; ++j)
            {
                if ((ext_adc_raw_data[j] >> 28) ==
                    (active_ext_chs[i] - 6)) // Check if the channel is the one we want
                {
                    // If the value is negative, round it to 0
                    adc_external_res =
                        ((ext_adc_raw_data[j] >> 24) & 0x01) ? 0 : (ext_adc_raw_data[j] & 0x00FFFFFF);
                    break;
                }
            }

            *(uint32_t *)(frame + frame_next_wr) |= adc_external_res;
            frame_next_wr += 3;
        }
    }
#elif _TIMESTAMP_ == TIMESTAMP_ENABLED
    if (num_extern_active_chs == 2)
    {
        *(uint64_t *)(frame + frame_next_wr) |= (int64_t)(esp_timer_get_time() & 0xFFFFFFFFFFFF);
        frame_next_wr += 6;
    }
#endif

    // Store values of internal channels into frame
    for (int i = 0; i < num_intern_active_chs; ++i)
    {
        if (!wr_mid_byte_flag)
        {
            *(uint16_t *)(frame + frame_next_wr) |= adc_internal_res[i];
            ++frame_next_wr;
            wr_mid_byte_flag = 1;
        }
        else
        {
            *(uint16_t *)(frame + frame_next_wr) |= adc_internal_res[i] << 4;
            frame_next_wr += 2;
            wr_mid_byte_flag = 0;
        }
    }

    // Store IO states into frame
    frame[packet_size - 3] = io_state;

    // Calculate CRC & SEQ Number---------------------------------------------------------
    // Store seq number
    *(uint16_t *)(frame + packet_size - 2) = crc_seq << 4;

    // calculate CRC (except last byte (seq+CRC) )
    for (int i = 0; i < packet_size - 2; ++i)
    {
        // calculate CRC nibble by nibble
        CALC_BYTE_CRC(crc, frame[i], crc_table);
    }

    // calculate CRC for seq
    crc = crc_table[crc] ^ (frame[packet_size - 2] >> 4);  // Calculate CRC for first 4 bits of seq
    CALC_BYTE_CRC(crc, frame[packet_size - 1], crc_table); // Calcultate CRC for last byte of seq

    crc = crc_table[crc];

    frame[packet_size - 2] |= crc;

    ++crc_seq;
}

/**
 * \brief acquire adc1 channels
 *
 * Acquires and stores into frame the channel acquisitions of adc1. This is used
 * for JSON mode.
 *
 * \param frame pointer of frame position in memory to store the adc1 channels
 */
void acquireChannelsJson(uint8_t *frame)
{
    uint16_t adc_internal_res[6] = {0, 0, 0, 0, 0, 0};
    uint32_t adc_external_res[2] = {0, 0};
    spi_transaction_t *ads_rtrans;
    uint8_t *recv_ads;
    char ch_str[10];
    char value_str[10];
    cJSON *item;

    for (int i = 0; i < num_intern_active_chs; ++i)
    {
        adc_internal_res[i] = adc1_get_raw(analog_channels[active_internal_chs[i]]);
        DEBUG_PRINT_I("acquireAdc1Channels", "(adc_internal_res)A%d=%d", active_internal_chs[i],
                      adc_internal_res[i]);
    }

    // Get and store the IO states into json
    sprintf(value_str, "%01d", gpio_get_level(I0_IO));
    item = cJSON_GetObjectItemCaseSensitive(json, "I1");
    strcpy(item->valuestring, value_str);

    sprintf(value_str, "%01d", gpio_get_level(I1_IO));
    item = cJSON_GetObjectItemCaseSensitive(json, "I2");
    strcpy(item->valuestring, value_str);

    sprintf(value_str, "%01d", gpio_out_state[0]);
    item = cJSON_GetObjectItemCaseSensitive(json, "O1");
    strcpy(item->valuestring, value_str);

    sprintf(value_str, "%01d", gpio_out_state[1]);
    item = cJSON_GetObjectItemCaseSensitive(json, "O2");
    strcpy(item->valuestring, value_str);

    if (num_extern_active_chs > 0)
    {
        // Get external adc values - Can be a huge bottleneck
        spi_device_get_trans_result(adc_ext_spi_handler, &ads_rtrans, portMAX_DELAY);
        recv_ads = ads_rtrans->rx_buffer;

        // Get raw values from AX1 & AX2 (A6 and A7), store them in the frame
        for (int i = 0; i < num_extern_active_chs; ++i)
        {
            adc_external_res[i] = (*(uint32_t *)(recv_ads + 3 + (3 * (active_ext_chs[i] - 6)))) & 0x00FFFFFF;
        }
    }

    // Store values of channels into JSON object
    for (int i = num_intern_active_chs - 1; i >= 0; --i)
    {
        sprintf(value_str, "%04d", adc_internal_res[i]);
        sprintf(ch_str, "AI%d", active_internal_chs[i] + 1);
        item = cJSON_GetObjectItemCaseSensitive(json, ch_str);
        strcpy(item->valuestring, value_str);
    }

    if (num_extern_active_chs > 0)
    {
        for (int i = num_extern_active_chs - 1; i >= 0; --i)
        {
            sprintf(value_str, "%08d", adc_external_res[i]);
            sprintf(ch_str, "AX%d", active_ext_chs[i] + 1 - 6);
            item = cJSON_GetObjectItemCaseSensitive(json, ch_str);
            strcpy(item->valuestring, value_str);
        }
    }
}