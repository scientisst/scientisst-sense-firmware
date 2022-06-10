#ifndef _WS_H
#define _WS_H

#include <esp_http_server.h>

esp_err_t IRAM_ATTR wsSerialSend(uint32_t fd, int len, uint8_t *buff);
httpd_handle_t start_webserver(void);

#endif