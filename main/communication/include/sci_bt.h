/**
 * \file sci_bt.h
 * \brief Header file for sci_bt.c
 */

#pragma once

#include "esp_err.h"

#include "sci_scientisst.h"

void initBt(void);
void sendDataBluetooth(esp_err_t (*tx_write_func)(uint32_t, int, const uint8_t *));
void getDeviceName(void);
