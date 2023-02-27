/** \file tcp.h
    \brief Header file for the TCP server and client functions.

    This file contains the declarations for the TCP server and client functions.
*/

#ifndef _TCP_H
#define _TCP_H

int initTcpServer(char *port_str);
int initTcpConnection(int listen_fd);
int initTcpClient(char *ip, char *port);
esp_err_t IRAM_ATTR tcpSerialSend(uint32_t fd, int len, uint8_t *buff);
void wifiSerialRcv(void);

#endif