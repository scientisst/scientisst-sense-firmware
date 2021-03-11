#ifndef _SPI_H
#define _SPI_H

#include <stdio.h>
#include "driver/spi_master.h"

void adsInit();
void adsSetSamplingRate(uint16_t sampling_rate);
void adsSetupRoutine();
void adsConfigureChannels(uint8_t ads_channel_mask);
void adsStart();
void adsStop();

#endif