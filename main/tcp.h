#ifndef _TCP_H
#define _TCP_H

int initTcpServer(char* port_str);
int initTcpClient(char* ip, char* port);
esp_err_t IRAM_ATTR tcpSend(uint32_t fd, int len, uint8_t *buff);
void wifiRcv();

#endif