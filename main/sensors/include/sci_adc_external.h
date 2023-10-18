/** \file spi.h
    \brief This file contains the definitions of the functions used to
   communicate with the external ADCs.
*/

#pragma once

#include <stdio.h>

#include "driver/sdmmc_types.h"

#include "sci_scientisst.h"

#define ADC_EXT_SLCK_HZ_1_EXT_CH (APB_CLK_FREQ / 8)
#define ADC_EXT_SLCK_HZ_2_EXT_CH (APB_CLK_FREQ / 64)

void adcExtInit(const sdmmc_host_t *spi_host);
void adcExtStart(void);
void adcExtStop(void);
void mcpSetupRoutine(uint8_t channel_mask);
esp_err_t getAdcExtValuesRaw(uint8_t channels_mask, uint32_t values[2]);
