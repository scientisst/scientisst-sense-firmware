#ifndef _WS_H
#define _WS_H

#include "sdkconfig.h"
#include <esp_http_server.h>

#if !CONFIG_HTTPD_WS_SUPPORT
#error This example cannot be used unless HTTPD_WS_SUPPORT is enabled in esp-http-server component configuration
#endif

esp_err_t IRAM_ATTR wsSerialSend(uint32_t fd, int len, uint8_t *buff);
httpd_handle_t start_webserver(void);

#endif