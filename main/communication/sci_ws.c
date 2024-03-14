/**
 * \file sci_ws.c
 * \brief Websocket server implementation.
 *
 * This file contains the implementation of the websocket server used for real-time communication between the device and the
 * web interface. It handles sending and receiving data through websockets.
 *
 * \warning WS_AP mode is not fully working yet.
 */

// #include "sci_ws.h"
//
// #include "esp_https_server.h"
// #include "esp_log.h"
// #include "esp_system.h"
// #include "sys/param.h"
//
// #include "sci_com.h"
//
// static const size_t max_clients = 1; ///< Maximum number of websocket clients that can be connected simultaneously.
// static httpd_handle_t ws_hd;         ///< Handle to the websocket server.
// static int ws_fd;                    ///< File descriptor for the websocket server.
//
// /**
//  * \brief Sends data to a connected websocket client.
//  *
//  * This function is responsible for sending binary data over an established websocket connection. It prepares a websocket
//  * frame with the provided data and sends it asynchronously.
//  *
//  * \param[in] fd File descriptor for the websocket connection.
//  * \param[in] len Length of the data to be sent.
//  * \param[in] buff Pointer to the data buffer.
//  *
//  * \return ESP_OK if the data was sent successfully, or an error code indicating the type of failure.
//  */
// esp_err_t wsSerialSend(uint32_t fd, int len, const uint8_t *buff)
// {
//     httpd_ws_frame_t ws_pkt;
//     memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
//     ws_pkt.payload = (uint8_t *)buff;
//     ws_pkt.len = len;
//     ws_pkt.type = HTTPD_WS_TYPE_BINARY;
//
//     esp_err_t ret = httpd_ws_send_frame_async(ws_hd, ws_fd, &ws_pkt);
//     if (ret != ESP_OK)
//     {
//         DEBUG_PRINT_E("ws_server", "httpd_ws_send_frame_async failed with %d", ret);
//         return ret;
//     }
//
//     return ESP_OK;
// }
//
// /**
//  * \brief Handles incoming data from a websocket client.
//  *
//  * This function is a callback for the server to handle incoming websocket frames. It reads the data from the frame,
//  * processes it if necessary, and performs any required responses or actions based on the data.
//  *
//  * \param[in] req The request structure containing client information.
//  *
//  * \return ESP_OK - Success, ESP Error Code - Failure.
//  */
// esp_err_t rxHandler(httpd_req_t *req)
// {
//     if (req->method == HTTP_GET)
//     {
//         DEBUG_PRINT_I("ws_server", "Handshake done, the new connection was opened");
//         ws_fd = httpd_req_to_sockfd(req);
//         ws_hd = req->handle;
//         return ESP_OK;
//     }
//     httpd_ws_frame_t ws_pkt;
//     uint8_t *buf = NULL;
//     memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
//     ws_pkt.type = HTTPD_WS_TYPE_BINARY;
//     /* Set max_len = 0 to get the frame len */
//     esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
//     if (ret != ESP_OK)
//     {
//         DEBUG_PRINT_E("ws_server", "httpd_ws_recv_frame failed to get frame len with %d", ret);
//         return ret;
//     }
//     if (ws_pkt.len)
//     {
//         /* ws_pkt.len + 1 is for NULL termination as we are expecting a string
//          */
//         if ((buf = calloc(1, ws_pkt.len + 1)) == NULL)
//         {
//             DEBUG_PRINT_E("ws_server", "Failed to calloc memory for buf");
//             return ESP_ERR_NO_MEM;
//         }
//         ws_pkt.payload = buf;
//         /* Set max_len = ws_pkt.len to get the frame payload */
//         ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
//         if (ret != ESP_OK)
//         {
//             DEBUG_PRINT_E("ws_server", "httpd_ws_recv_frame failed with %d", ret);
//             free(buf);
//             return ret;
//         }
//         processPacket(ws_pkt.payload);
//     }
//     free(buf);
//     return ret;
// }
//
// /**
//  * \brief Sends a confirmation response to the client.
//  *
//  * This function handles GET requests to a specific endpoint, responding with a confirmation message.
//  *
//  * \param[in] req The request structure containing the HTTP request.
//  *
//  * \return ESP_OK - response was sent successfully, ESP_FAIL - Failure.
//  */
// esp_err_t getHandler(httpd_req_t *req)
// {
//     esp_err_t ret = ESP_OK;
//     const char resp[] = "<!DOCTYPE html><html><body><h1>Success!</h1><p>Authorized ScientISST "
//                         "Sense self-signed "
//                         "certificate</p><script>setTimeout(()=>{history.back();},1000);</"
//                         "script></body></html>";
//     ret = httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
//     return ret;
// }
//
// const httpd_uri_t ws = {.uri = "/", .method = HTTP_GET, .handler = rxHandler, .user_ctx = NULL, .is_websocket = true};
//
// const httpd_uri_t cert_get = {.uri = "/cert", .method = HTTP_GET, .handler = getHandler, .user_ctx = NULL};
//
// /**
//  * \brief Starts the secure websocket server.
//  *
//  * This function initializes the configuration for the server, loads SSL certificates, and starts the server. It also
//  * registers URI handlers for handling websocket connections and other HTTP requests.
//  *
//  * \return A handle to the started server, or NULL if the server failed to start.
//  */
// httpd_handle_t startWebserver(void)
// {
//     httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
//     conf.httpd.max_open_sockets = max_clients;
//
//     httpd_handle_t server = NULL;
//
//     extern const unsigned char cacert_start[] asm("_binary_cacert_pem_start");
//     extern const unsigned char cacert_end[] asm("_binary_cacert_pem_end");
//     conf.cacert_pem = cacert_start;
//     conf.cacert_len = cacert_end - cacert_start;
//
//     extern const unsigned char prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
//     extern const unsigned char prvtkey_pem_end[] asm("_binary_prvtkey_pem_end");
//     conf.prvtkey_pem = prvtkey_pem_start;
//     conf.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;
//
//     // Start the httpd server
//     if (httpd_ssl_start(&server, &conf) == ESP_OK)
//     {
//         // Registering the ws handler
//         DEBUG_PRINT_I("ws_server", "Registering URI handlers");
//         httpd_register_uri_handler(server, &ws);
//         httpd_register_uri_handler(server, &cert_get);
//         return server;
//     }
//
//     DEBUG_PRINT_I("ws_server", "Error starting server!");
//     exit(-1);
// }

