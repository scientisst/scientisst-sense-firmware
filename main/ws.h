/** \file ws.h
    \brief Websocket server

    This file contains the websocket server code. It is used to send data to the
   web interface.
    //TODO: Confirm it is imcomplete and undestand what it's supposed to do.
*/
#ifndef _WS_H
#define _WS_H

#include <esp_http_server.h>

#include "sdkconfig.h"

#if !CONFIG_HTTPD_WS_SUPPORT
#error This example cannot be used unless HTTPD_WS_SUPPORT is enabled in esp-http-server component configuration
#endif

esp_err_t wsSerialSend(uint32_t fd, int len, uint8_t* buff);
httpd_handle_t start_webserver(void);

#endif
