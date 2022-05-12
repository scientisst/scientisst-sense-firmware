#ifndef _SPI_H
#define _SPI_H

#include <stdio.h>
#include "driver/spi_master.h"
#include "config.h"

#define SPI3_MISO_IO    GPIO_NUM_19
#define SPI3_MOSI_IO    GPIO_NUM_23
#define SPI3_SCLK_IO    GPIO_NUM_18
#if _ADC_EXT_ == ADC_MCP
#define MCP_DRDY_IO     I1_IO
#elif _ADC_EXT_ == ADC_ADS
#define ADS_DRDY_IO     GPIO_NUM_16
#endif

#define SPI3_CS0_IO     GPIO_NUM_5
#define SPI3_CS1_IO     GPIO_NUM_4

void adcExtInit(uint32_t cs_pin);
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
void mcpSetupRoutine(void);
uint32_t IRAM_ATTR mcpReadRegister(uint8_t address, uint8_t rx_data_bytes);
void mcpStart();
void mcpStop();
#endif

#endif