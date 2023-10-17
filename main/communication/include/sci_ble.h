/** \file ble.h
    \brief BLE mode functions header file.

    This file contains the functions to initialize and send data in BLE mode. It
   also contains the maximum length of the buffer in BLE mode.
*/

#pragma once

#include "esp_attr.h"
#include "esp_err.h"

#include "sci_scientisst.h"

void initBle(void);
esp_err_t sendBle(uint32_t fd, int len, const uint8_t *buff);
