/** \file com.h
    \brief This file contains the definitions of the functions used to process
   the received data.
*/

#pragma once

#include <stdio.h>

#include "sdkconfig.h"

#include "sci_scientisst.h"

extern int send_fd;

void processPacket(uint8_t *buff);
void selectChsFromMaskBitalino(const uint8_t *buff);
void stopAcquisition(void);
void sendStatusPacket();
void sendFirmwareVersionPacket();
void changeAPI(uint8_t mode);
void selectChsFromMaskScientisstAndJson(const uint8_t *buff);
void setSampleRate(uint8_t *buff);
void triggerGpio(const uint8_t *buff);
void triggerDAC(const uint8_t *buff);
void startAcquisition(void);
uint8_t getPacketSize(void);
