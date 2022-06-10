#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"

#include "com.h"
#include "main.h"
#include "macros.h"
#include "bt.h"
#include "ws.h"

static const char *TAG = "ws_server";

httpd_handle_t ws_hd;
int ws_fd;

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
        ESP_LOGE(TAG, "httpd_ws_send_frame_async failed with %d", ret);
        return ret;
    }

    finalizeSend();

    // Try to send next buff
    sendData();
    return ESP_OK;
}

esp_err_t rcv_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
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
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    if (ws_pkt.len)
    {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL)
        {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        processRcv(ws_pkt.payload, ws_pkt.len);
    }
    free(buf);
    return ret;
}

const httpd_uri_t ws = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = rcv_handler,
    .user_ctx = NULL,
    .is_websocket = true};

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Registering the ws handler
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &ws);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}
