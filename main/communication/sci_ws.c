/** \file ws.c
    \brief Websocket server

    This file contains the websocket server code. It is used to send data to the
   web interface.
    //TODO: Confirm it is imcomplete and undestand what it's supposed to do.
*/
#include "sci_ws.h"

#include "esp_https_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "sci_bt.h"
#include "sci_com.h"
#include "sci_macros.h"
#include "sci_macros_conf.h"
#include "sci_scientisst.h"
#include "sys/param.h"

static const size_t max_clients = 1;

httpd_handle_t ws_hd;
int ws_fd;

/**
 * \brief Send data to the websocket client
 *
 * This function sends data to the websocket client. It is called by the
 * sendData function when in websocket mode.
 *
 * \param fd The file descriptor of the TCP connection.
 * \param len The length of the data to send.
 * \param buff The data to send.
 *
 * \return:
 *      - ESP_OK if the data was sent successfully
 *      - ESP_FAIL otherwise
 */
esp_err_t IRAM_ATTR wsSerialSend(uint32_t fd, int len, uint8_t *buff)
{
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = buff;
    ws_pkt.len = len;
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;

    esp_err_t ret = httpd_ws_send_frame_async(ws_hd, ws_fd, &ws_pkt);
    if (ret != ESP_OK)
    {
        DEBUG_PRINT_E("ws_server", "httpd_ws_send_frame_async failed with %d", ret);
        return ret;
    }

    return ESP_OK;
}

/**
 * \brief Receive data from the websocket client and process it
 *
 * \return:
 *      - ESP_OK if the server was started successfully
 *      - ESP_FAIL otherwise
 */
esp_err_t rcv_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        DEBUG_PRINT_I("ws_server", "Handshake done, the new connection was opened");
        ws_fd = httpd_req_to_sockfd(req);
        ws_hd = req->handle;
        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK)
    {
        DEBUG_PRINT_E("ws_server", "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    if (ws_pkt.len)
    {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string
         */
        if ((buf = calloc(1, ws_pkt.len + 1)) == NULL)
        {
            DEBUG_PRINT_E("ws_server", "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            DEBUG_PRINT_E("ws_server", "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        processRcv(ws_pkt.payload, ws_pkt.len);
    }
    free(buf);
    return ret;
}

/**
 * \brief Send the ScientISST Sense self-signed certificate to the client
 *
 * \return:
 *      - ESP_OK always
 */
esp_err_t get_handler(httpd_req_t *req)
{
    const char resp[] = "<!DOCTYPE html><html><body><h1>Success!</h1><p>Authorized ScientISST "
                        "Sense self-signed "
                        "certificate</p><script>setTimeout(()=>{history.back();},1000);</"
                        "script></body></html>";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

const httpd_uri_t ws = {
    .uri = "/", .method = HTTP_GET, .handler = rcv_handler, .user_ctx = NULL, .is_websocket = true};

const httpd_uri_t cert_get = {.uri = "/cert", .method = HTTP_GET, .handler = get_handler, .user_ctx = NULL};

/**
 * \brief Start the websocket server
 *
 * \return:
 *      - The handle of the server if it was started successfully
 *      - NULL otherwise
 */
httpd_handle_t start_webserver(void)
{
    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
    conf.httpd.max_open_sockets = max_clients;

    httpd_handle_t server = NULL;

    extern const unsigned char cacert_start[] asm("_binary_cacert_pem_start");
    extern const unsigned char cacert_end[] asm("_binary_cacert_pem_end");
    conf.cacert_pem = cacert_start;
    conf.cacert_len = cacert_end - cacert_start;

    extern const unsigned char prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
    extern const unsigned char prvtkey_pem_end[] asm("_binary_prvtkey_pem_end");
    conf.prvtkey_pem = prvtkey_pem_start;
    conf.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;

    // Start the httpd server
    if (httpd_ssl_start(&server, &conf) == ESP_OK)
    {
        // Registering the ws handler
        DEBUG_PRINT_I("ws_server", "Registering URI handlers");
        httpd_register_uri_handler(server, &ws);
        httpd_register_uri_handler(server, &cert_get);
        return server;
    }

    DEBUG_PRINT_I("ws_server", "Error starting server!");
    exit(-1);
}
