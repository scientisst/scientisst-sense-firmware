/** \file bt.h
    \brief Bluetooth header file

    This file contains the bluetooth header file and relevant macros.
*/

#ifndef _BT_H
#define _BT_H

#include <stdio.h>

#include "esp_attr.h"
#include "esp_err.h"

#define SPP_SERVER_NAME "SPP_SERVER"
#define BT_DEFAULT_DEVICE_NAME "ScientISST\0"

#define SEND_AFTER_C0NG 2

void initBt(void);
void IRAM_ATTR sendDataBluetooth(esp_err_t (*tx_write_func)(uint32_t, int, uint8_t *));
void getDeviceName(void);

#endif
