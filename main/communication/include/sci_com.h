/**
 * \file sci_com.h
 * \brief Header file for sci_com.c
 */

#pragma once

#include <stdio.h>

#include "sci_scientisst.h"

extern int send_fd;

void processPacket(uint8_t *buff);
void selectChsFromMaskBitalino(const uint8_t *buff);
void stopAcquisition(void);
