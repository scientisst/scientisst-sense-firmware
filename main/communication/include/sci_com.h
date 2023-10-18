/** \file com.h
    \brief This file contains the definitions of the functions used to process
   the received data.
*/

#pragma once

#include <stdio.h>

#include "sci_scientisst.h"

extern int send_fd;

void processPacket(uint8_t *buff);
void selectChsFromMaskBitalino(const uint8_t *buff);
void stopAcquisition(void);
