/** \file udp.h
    \brief UDP client functions

    This file contains the definitions of functions to create a UDP client and
   send data to a server.
*/

#pragma once

#include "esp_attr.h"
#include "esp_err.h"

#include "sci_scientisst.h"

int initUdpClient(const char *ip, const char *port);
esp_err_t udpSend(uint32_t fd, int len, const uint8_t *buff);
