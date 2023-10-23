/**
 * \file sci_ble.h
 * \brief Header file for sci_ble.c
 */

#pragma once

#include "esp_err.h"

#include "sci_scientisst.h"

void initBle(void);
esp_err_t sendBle(uint32_t fd, int len, const uint8_t *buff);
