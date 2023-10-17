/** \file wifi.h
    \brief Wifi functions definitions and relevant defines
*/

#pragma once

#include <stdint.h>

#include "sci_scientisst.h"

int wifiInit(uint8_t force_ap);
void wifi_init_softap(void);
int wifi_init_sta(void);
