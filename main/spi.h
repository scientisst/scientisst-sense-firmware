#ifndef _SPI_H
#define _SPI_H

#include <stdio.h>
#include "driver/spi_master.h"

#define SPI3_MISO_IO    GPIO_NUM_19
#define SPI3_MOSI_IO    GPIO_NUM_23
#define SPI3_SCLK_IO    GPIO_NUM_18
//#define SPI3_QUADWP_IO  GPIO_NUM_22
//#define SPI3_QUADHD_IO  GPIO_NUM_21
#define SPI3_CS0_IO     GPIO_NUM_5

void adsInit();
void adsSetSamplingRate(uint16_t sampling_rate);
void adsSetupRoutine();
void adsConfigureChannels(uint8_t ads_channel_mask);
void adsStart();
void adsStop();

#endif