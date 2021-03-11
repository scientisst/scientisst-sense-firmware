#ifndef _BT_H
#define _BT_H

#include <stdio.h>
#include "esp_err.h"

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
#define BT_DEFAULT_DEVICE_NAME "ScientISST\0"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/

void IRAM_ATTR sendData();
void initBt();
void getDeviceName();

#endif