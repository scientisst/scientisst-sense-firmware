/** \file wifi.h
    \brief Wifi functions definitions and relevant defines
*/

#pragma once

#include <stdint.h>

#include "sci_scientisst.h"

int wifiInit(uint8_t force_ap);
void wifiInitSoftap(void);
int wifiInitSta(void);
