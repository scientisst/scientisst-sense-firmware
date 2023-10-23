/**
 * \file sci_wifi.h
 * \brief Header file for sci_wifi.c
 */

#pragma once

#include <stdint.h>

#include "sci_scientisst.h"

int wifiInit(uint8_t force_ap);
void wifiInitSoftap(void);
int wifiInitSta(void);
