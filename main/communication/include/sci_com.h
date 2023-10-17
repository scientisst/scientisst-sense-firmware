/** \file com.h
    \brief This file contains the definitions of the functions used to process
   the received data.
*/

#pragma once

#include <stdio.h>

#include "sdkconfig.h"

#include "sci_scientisst.h"

extern int send_fd;

void processRcv(uint8_t *buff, int buff_size);
void selectChsFromMask(uint8_t *buff);
void stopAcquisition(void);
void sendStatusPacket();
void sendFirmwareVersionPacket();
void changeAPI(uint8_t mode);
void selectChsFromMaskScientisstJson(uint8_t *buff);
void setSampleRate(uint8_t *buff);
void triggerGpio(uint8_t *buff);
void triggerDAC(uint8_t *buff);
void startAcquisition(uint8_t *buff, uint8_t cmd);
uint8_t getPacketSize(void);
