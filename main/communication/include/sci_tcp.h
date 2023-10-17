/** \file tcp.h
    \brief Header file for the TCP server and client functions.

    This file contains the declarations for the TCP server and client functions.
*/

#pragma once

#include "esp_attr.h"
#include "esp_err.h"

#include "sci_scientisst.h"

int initTcpServer(const char *port_str);
int initTcpConnection(void);
int initTcpClient(char *ip, char *port);
esp_err_t tcpSerialSend(uint32_t fd, int len, uint8_t *buff);
