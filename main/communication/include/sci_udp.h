/** \file udp.h
    \brief UDP client functions

    This file contains the definitions of functions to create a UDP client and
   send data to a server.
*/
#ifndef _UDP_H
#define _UDP_H

#include "esp_attr.h"
#include "esp_err.h"

int initUdpClient(char *ip, char *port);
esp_err_t udpSend(uint32_t fd, int len, uint8_t *buff);

#endif
