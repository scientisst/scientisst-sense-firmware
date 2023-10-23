/**
 * \file sci_tcp_and_serial.h
 * \brief Header file for sci_tcp_and_serial.c
 */

#pragma once

#include "esp_err.h"

#include "sci_scientisst.h"

int initTcpServer(const char *port_str);
int initTcpConnection(void);
int initTcpClient(const char *ip, const char *port);
esp_err_t tcpSerialSend(uint32_t fd, int len, const uint8_t *buff);
