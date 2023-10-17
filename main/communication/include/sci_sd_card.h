#include <sys/cdefs.h>
/** \file sd_card.h
    \brief SD card driver.
    This file implements the SD card driver.
*/

#pragma once

#include <stdio.h>

#include "driver/sdmmc_types.h"
#include "esp_attr.h"
#include "esp_err.h"

#include "sci_scientisst.h"

#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK GPIO_NUM_18
#define PIN_NUM_CS GPIO_NUM_4

sdmmc_host_t *initSdCardSpiBus(void);
esp_err_t initSDCard(void);
void writeFileHeader(void);
