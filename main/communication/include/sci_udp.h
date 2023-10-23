/**
 * \file sci_udp.h
 * \brief Header file for sci_udp.c
 */

#pragma once

#include "esp_err.h"

#include "sci_scientisst.h"

int initUdpClient(const char *ip, const char *port);
esp_err_t udpSend(uint32_t fd, int len, const uint8_t *buff);
