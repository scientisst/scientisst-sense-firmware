/** \file sd_card.h
    \brief SD card driver.
    This file implements the SD card driver.
*/

#ifndef SCIENTISST_SENSE_FIRMWARE_SD_CARD_H
#define SCIENTISST_SENSE_FIRMWARE_SD_CARD_H

#include "esp_attr.h"
#include "esp_err.h"

#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK GPIO_NUM_18
#define PIN_NUM_CS GPIO_NUM_4

extern FILE* save_file;
extern DRAM_ATTR char full_file_name[100];

esp_err_t initSDCard(void);
void unmountSDCard(void);
esp_err_t createFile(void);
void startAcquisitionSDCard(void);
void IRAM_ATTR acquisitionSDCard(void* not_used);

/**
 * \brief Close the file on the SD card.
 *
 * \return ESP_OK if successful, ESP_FAIL otherwise.
 */
FORCE_INLINE_ATTR void closeSDCard(void) { fclose(save_file); }

/**
 * \brief Open the current file on the SD card for appending.
 *
 * \return ESP_OK if successful, ESP_FAIL otherwise.
 */
FORCE_INLINE_ATTR void openFile(void) {
    save_file = fopen(full_file_name, "a");  // Open file
}

#endif  // SCIENTISST_SENSE_FIRMWARE_SD_CARD_H
