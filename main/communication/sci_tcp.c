/** \file tcp.c
    \brief Header file for the TCP server and client functions.

    This file contains the declarations for the TCP server and communications
   functions.
*/

#include "sci_tcp.h"

#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "sci_bt.h"
#include "sci_macros.h"
#include "sci_scientisst.h"

int listen_fd = 0; ///< Used to listen for connections when used as a TCP server

/**
 * \brief Initializes a TCP server.
 *
 * \param port_str The port to listen to.
 *
 * \return The file descriptor of the TCP server.
 */
esp_err_t initTcpServer(const char *port_str)
{
    esp_err_t res = ESP_OK;
    int bind_err;
    int port;
    struct sockaddr_in listen_addr;

    sscanf(port_str, "%d", &port); // Transform port string to int

    // Verificar se não houve erro a criar a socket
    if ((listen_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        DEBUG_PRINT_E("initTcpServer", "socket error");
        res = ESP_FAIL;
    }

    listen_addr.sin_family = AF_INET;
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    listen_addr.sin_port = htons(port);

    bind_err = bind(listen_fd, (struct sockaddr *)&listen_addr, sizeof(listen_addr));
    if (bind_err != 0) // Verificar se não houve erro a fazer bind
    {
        DEBUG_PRINT_E("initTcpServer", "bind error %d", bind_err);
        res = ESP_FAIL;
    }

    if (listen(listen_fd, 2) == -1) // Verificar se não houve erro a fazer listen
    {
        DEBUG_PRINT_E("initTcpServer", "listen error");
        res = ESP_FAIL;
    }

    return res;
}

/**
 * \brief Initializes a TCP connection.
 *
 * \param listen_fd The file descriptor of the TCP server.
 *
 * \return The file descriptor of the TCP connection.
 */
int initTcpConnection(void)
{
    struct sockaddr_in client_addr;
    socklen_t client_addr_len;
    int client_fd;
    if ((client_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &client_addr_len)) < 0)
    {
        DEBUG_PRINT_E("initTcpServer", "accept error");
        return -1;
    }
    return client_fd;
}

/**
 * \brief Initializes a TCP client.
 *
 * \param ip The IP address of the server.
 * \param port The port of the server.
 *
 * \return The file descriptor of the TCP client.
 */
int initTcpClient(char *ip, char *port)
{
    struct addrinfo hints;
    struct addrinfo *res;
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

/**
 * \brief Sends data through a TCP connection.
 *
 * This function sends data through a TCP connection. It is called by the
 * sendData() function when in TCP mode.
 *
 * \param fd The file descriptor of the TCP connection.
 * \param len The length of the data to send.
 * \param buff The data to send.
 *
 * \return:
 *      - ESP_OK if the data was sent successfully
 *      - ESP_FAIL otherwise
 */
esp_err_t IRAM_ATTR tcpSerialSend(uint32_t fd, int len, uint8_t *buff)
{
    uint8_t *ptr;
    ssize_t n_left = len;
    ssize_t n_written;

    ptr = buff;

    if (fd <= 0)
    {
        DEBUG_PRINT_E("tcpSerialSend", "ERROR: INVALID FD");
        return ESP_FAIL;
    }

    while (n_left > 0)
    {
        n_written = write(fd, ptr, n_left);
        if (n_written == -1)
        {
            DEBUG_PRINT_E("tcpSerialSend", "ERROR: WRITE FAILED");
            return ESP_FAIL;
        }
        n_left -= n_written;
        ptr += n_written;
    }

    return ESP_OK;
}
