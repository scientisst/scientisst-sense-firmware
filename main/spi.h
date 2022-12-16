#ifndef _SPI_H
#define _SPI_H

#include <stdio.h>
#include "driver/spi_master.h"
#include "config.h"

void adcExtInit(void);
void adcExtStart(void);
void adcExtStop(void);

#if _ADC_EXT_ == ADC_ADS
#define ADS_CONTINUOUS_TRANS 1

void adsSetSamplingRate(uint16_t sampling_rate);
void adsSetupRoutine();
void adsConfigureChannels(uint8_t ads_channel_mask);
void adsStart();
void adsStop();
void IRAM_ATTR adsEndTransCb(spi_transaction_t* trans);
void adsSendCmd(uint8_t cmd);

#elif _ADC_EXT_ == ADC_MCP
void mcpSetupRoutine(uint8_t channel_mask);
uint32_t IRAM_ATTR mcpReadRegister(uint8_t address, uint8_t rx_data_bytes);
void mcpStart();
void mcpStop();
void decodeSample(int32_t sample);
#endif

#endif