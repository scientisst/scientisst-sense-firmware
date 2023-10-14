#include <sys/cdefs.h>
/** \file sd_card.h
    \brief SD card driver.
    This file implements the SD card driver.
*/

#ifndef SCIENTISST_SENSE_FIRMWARE_SD_CARD_H
#define SCIENTISST_SENSE_FIRMWARE_SD_CARD_H

#include <stdio.h>

#include "driver/sdmmc_types.h"
#include "esp_attr.h"
#include "esp_err.h"

#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK GPIO_NUM_18
#define PIN_NUM_CS GPIO_NUM_4

sdmmc_host_t *init_sd_card_spi_bus(void);
esp_err_t initSDCard(uint8_t num_active_channels_ext_adc, FILE **save_file);

#endif // SCIENTISST_SENSE_FIRMWARE_SD_CARD_H
