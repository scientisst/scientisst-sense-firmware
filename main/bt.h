#ifndef _BT_H
#define _BT_H

#include <stdio.h>
#include "esp_err.h"

#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"
//#define BT_DEFAULT_DEVICE_NAME "eFORTO\0"
#define BT_DEFAULT_DEVICE_NAME "ScientISST\0"


void IRAM_ATTR sendData();
void IRAM_ATTR finalizeSend();
void initBt();
void getDeviceName();

#endif