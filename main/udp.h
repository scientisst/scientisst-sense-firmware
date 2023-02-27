/** \file udp.h
    \brief UDP client functions
    
    This file contains the definitions of functions to create a UDP client and send data to a server.
*/
#ifndef _UDP_H
#define _UDP_H

int initUdpClient(char* ip, char* port);
esp_err_t IRAM_ATTR udpSend(uint32_t fd, int len, uint8_t *buff);

#endif