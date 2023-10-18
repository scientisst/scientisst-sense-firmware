#include <sys/cdefs.h>
/** \file sd_card.h
    \brief SD card driver.
    This file implements the SD card driver.
*/

#pragma once

#include <stdio.h>

#include "driver/sdmmc_types.h"
#include "esp_err.h"

#include "sci_scientisst.h"

sdmmc_host_t *initSdCardSpiBus(void);
esp_err_t initSDCard(void);
void writeFileHeader(void);
