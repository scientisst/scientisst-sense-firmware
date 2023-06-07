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

esp_err_t IRAM_ATTR saveToSDCardSend(uint32_t fd, int len, uint8_t* buff);
esp_err_t initSDCard(void);
void unmountSDCard(void);
void closeSDCard(void);
esp_err_t openSDCard(void);

#endif  // SCIENTISST_SENSE_FIRMWARE_SD_CARD_H
