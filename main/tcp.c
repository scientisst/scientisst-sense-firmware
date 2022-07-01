#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "scientisst.h"
#include "macros.h"
#include "tcp.h"
#include "bt.h"

int bind_err;

int initTcpServer(char *port_str){
    int port;
    struct sockaddr_in listen_addr;
    int listen_fd;

    sscanf(port_str, "%d", &port); // Transform port string to int

    if((listen_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0){ // Verificar se não houve erro a criar a socket
        DEBUG_PRINT_E("initTcpServer", "socket error");
    }

    listen_addr.sin_family = AF_INET;
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    listen_addr.sin_port = htons(port);

    bind_err = bind(listen_fd, (struct sockaddr *)&listen_addr, sizeof(listen_addr));
    if (bind_err != 0)
    { // Verificar se não houve erro a fazer bind
        DEBUG_PRINT_E("initTcpServer", "bind error %d", bind_err);
    }

    if (listen(listen_fd, 2) == -1)
    { // Verificar se não houve erro a fazer listen
        DEBUG_PRINT_E("initTcpServer", "listen error");
    }

    return listen_fd;
}

int initTcpConnection(int listen_fd){
    struct sockaddr_in client_addr;
    socklen_t client_addr_len;
    int client_fd;
    if ((client_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &client_addr_len)) < 0){
        DEBUG_PRINT_E("initTcpServer", "accept error");
        return -1;
    }
    return client_fd;
}

int initTcpClient(char *ip, char *port)
{
    struct addrinfo hints, *res;
    int server_fd;

    server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP); // TCP socket
    if (server_fd == -1)
    {
        DEBUG_PRINT_E("initTcpClient", "ERROR: SOCKET CREATION FAILED");
        return -1;
    }

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;       // IPv4
    hints.ai_socktype = SOCK_STREAM; // TCP socket

    if (getaddrinfo(ip, port, &hints, &res) != 0)
    {
        DEBUG_PRINT_E("initTcpClient", "ERROR: GETADDRINFO FAILED");
        free(res);
        shutdown(server_fd, 0);
        close(server_fd);
        server_fd = 0;
        return -1;
    }
    if (connect(server_fd, res->ai_addr, res->ai_addrlen) == -1)
    {
        DEBUG_PRINT_E("initTcpClient", "ERROR: CONNECT FAILED");
        free(res);
        shutdown(server_fd, 0);
        close(server_fd);
        server_fd = 0;
        return -1;
    }

    DEBUG_PRINT_I("initTcpClient", "Client successfully connected");
    return server_fd;
}

esp_err_t IRAM_ATTR tcpSerialSend(uint32_t fd, int len, uint8_t *buff)
{
    uint8_t *ptr;
    ssize_t n_left = len;
    ssize_t n_written;

    ptr = buff;

    while (n_left > 0)
    {
        n_written = write(fd, ptr, n_left);
        if (n_written == -1)
        {
            DEBUG_PRINT_E("tcpSerialSend", "ERROR: WRITE FAILED\n");
            return ESP_FAIL;
        }
        n_left -= n_written;
        ptr += n_written;
    }
    finalizeSend();

    // Try to send next buff
    sendData();
    return ESP_OK;
}

void wifiSerialRcv()
{
    uint8_t buff[CMD_MAX_BYTES];
    int len;
    int read_bytes;

    while (1)
    {
        // Wait until a connection is made
        while (send_fd <= 0)
        {
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        while (1)
        {
            if ((read_bytes = read(send_fd, buff, CMD_MAX_BYTES)) == 0)
            {
                DEBUG_PRINT_W("wifiSerialRcv", "Connection closed gracefully");
                buff[0] = 0;
                len = 1;
            }
            else if (read_bytes < 0)
            {
                DEBUG_PRINT_W("wifiSerialRcv", "Connection closed with errno %d", errno);
                buff[0] = 0;
                len = 1;
            }
            else
            {
                // TODO: implement walk around (if a message is 3bytes, it may happen that it is divided into a read of 2bytes and a read of 1byte, thus the check below will give a false positive error)
                if (read_bytes != CMD_MAX_BYTES)
                {
                    DEBUG_PRINT_E("wifiSerialRcv", "Recieved %d bytes and the acceptable amount is %d", read_bytes, CMD_MAX_BYTES);
                }
                len = CMD_MAX_BYTES;
            }
            processRcv(buff, len);

            // If connection was broken, close socket and return
            if (read_bytes <= 0)
            {
                shutdown(send_fd, 0);
                close(send_fd);
                send_fd = 0;

                stopAcquisition();
                return;
            }
        }
    }
}