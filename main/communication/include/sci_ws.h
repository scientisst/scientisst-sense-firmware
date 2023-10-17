/** \file ws.h
    \brief Websocket server

    This file contains the websocket server code. It is used to send data to the
   web interface.
    //TODO: Confirm it is imcomplete and undestand what it's supposed to do.
*/

#pragma once

#include "esp_http_server.h"
#include "sdkconfig.h"

#include "sci_scientisst.h"

#if !CONFIG_HTTPD_WS_SUPPORT
#error This mode cannot be used unless HTTPD_WS_SUPPORT is enabled in esp-http-server component configuration
#endif

esp_err_t wsSerialSend(uint32_t fd, int len, uint8_t *buff);
httpd_handle_t startWebserver(void);
