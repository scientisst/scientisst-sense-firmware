/** \file bt.h
    \brief Bluetooth header file

    This file contains the bluetooth header file and relevant macros.
*/

#pragma once

#include <stdio.h>

#include "esp_attr.h"
#include "esp_err.h"

#include "sci_scientisst.h"

void initBt(void);
void sendDataBluetooth(esp_err_t (*tx_write_func)(uint32_t, int, const uint8_t *));
void getDeviceName(void);
