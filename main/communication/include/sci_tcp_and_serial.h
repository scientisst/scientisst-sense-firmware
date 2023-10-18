/** \file tcp.h
    \brief Header file for the TCP server and client functions.

    This file contains the declarations for the TCP server and client functions.
*/

#pragma once

#include "esp_err.h"

#include "sci_scientisst.h"

int initTcpServer(const char *port_str);
int initTcpConnection(void);
int initTcpClient(const char *ip, const char *port);
esp_err_t tcpSerialSend(uint32_t fd, int len, const uint8_t *buff);
