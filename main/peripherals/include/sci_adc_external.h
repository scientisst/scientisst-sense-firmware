/**
 * \file sci_adc_external.h
 * \brief Header file for sci_adc_external.c
 */

#pragma once

#include <stdio.h>

#include "driver/sdmmc_types.h"

#include "sci_scientisst.h"

// Define 2 different clock frequen// Define 2 different clock frequencies because when using 2 channels the data gets corrupted if the clock is too fast.
// #ifdef CONFIG_SD_CARD
// #define ADC_EXT_SLCK_HZ_1_EXT_CH (SDMMC_FREQ_DEFAULT / 2)  ///< ADC external clock frequency when using 1 external channel
// #define ADC_EXT_SLCK_HZ_2_EXT_CH (SDMMC_FREQ_DEFAULT / 16) ///< ADC external clock frequency when using 2 external channels
// #else
#define ADC_EXT_SLCK_HZ_1_EXT_CH (APB_CLK_FREQ / 8)  ///< ADC external clock frequency when using 1 external channel
#define ADC_EXT_SLCK_HZ_2_EXT_CH (APB_CLK_FREQ / 64) ///< ADC external clock frequency when using 2 external channelscies because when using 2 channels the data gets corrupted if the clock is too fast.
//#endif

void adcExtInit(const sdmmc_host_t *spi_host);
void adcExtStart(void);
void adcExtStop(void);
void mcpSetupRoutine(uint8_t channel_mask);
esp_err_t getAdcExtValuesRaw(uint8_t channels_mask, uint32_t values[2]);
