/**
 * \file sci_tcp_and_serial.c
 * \brief Implementation of the TCP server and client. Some functions are also used for the serial communication to save code
 * size.
 *
 * This file contains the implementation of the functions necessary for setting up and communicating over a TCP server/client
 * architecture. Some functions are also used for the serial communication to save code size. It includes functions to
 * initialize a TCP server, accept connections, initialize a TCP client, and send data over a TCP connection.
 */

#include "sci_tcp_and_serial.h"

#include "esp_attr.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

DRAM_ATTR static int listen_fd = 0; ///< Used to listen for connections when used as a TCP server

/**
 * \brief Initializes a TCP server.
 *
 * This function sets up a TCP server by creating a socket, binding it to an address and port, and then starting to listen
 * for incoming connections. It returns the file descriptor for the listening socket or a negative value indicating an error.
 *
 * \param[in] port_str String representing the port number on which the server will listen for connections.
 *
 * \return File descriptor - Success, ESP_FAIL - Failure, along with relevant debug messages.
 */
esp_err_t initTcpServer(const char *port_str)
{
    esp_err_t res = ESP_OK;
    int bind_err;
    int port;
    struct sockaddr_in listen_addr;

    sscanf(port_str, "%d", &port); // Transform port string to int

    // Check if socket creation was successful
    if ((listen_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        DEBUG_PRINT_E("initTcpServer", "socket error");
        res = ESP_FAIL;
    }

    listen_addr.sin_family = AF_INET;
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    listen_addr.sin_port = htons(port);

    bind_err = bind(listen_fd, (struct sockaddr *)&listen_addr, sizeof(listen_addr));

    if (bind_err != 0) // Check if bind was successful
    {
        DEBUG_PRINT_E("initTcpServer", "bind error %d", bind_err);
        res = ESP_FAIL;
    }

    if (listen(listen_fd, 2) == -1) // Check if listening was successful
    {
        DEBUG_PRINT_E("initTcpServer", "listen error");
        res = ESP_FAIL;
    }

    return res;
}

/**
 * \brief Accepts a new TCP connection on a server.
 *
 * This function waits for a new connection to the server and initializes it. The function returns a new file descriptor for
 * the accepted connection.
 *
 * \return The file descriptor for the new client connection, ESP_FAIL1 - connection failed.
 */
int initTcpConnection(void)
{
    struct sockaddr_in client_addr;
    socklen_t client_addr_len;
    int client_fd;
    if ((client_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &client_addr_len)) < 0)
    {
        DEBUG_PRINT_E("initTcpServer", "accept error");
        return ESP_FAIL;
    }
    return client_fd;
}

/**
 * \brief Initializes a TCP client and connects it to a server.
 *
 * This function sets up a TCP client by resolving the server address and establishing a connection to it. It returns a file
 * descriptor that can be used for subsequent communication.
 *
 * \param[in] ip String representing the IP address of the server.
 * \param[in] port String representing the port number of the server.
 *
 * \return The file descriptor of the client socket, ESP_FAIL - connection failed.
 */
int initTcpClient(const char *ip, const char *port)
{
    struct addrinfo hints;
    struct addrinfo *res;
    int server_fd;

    server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP); // TCP socket
    if (server_fd == -1)
    {
        DEBUG_PRINT_E("initTcpClient", "ERROR: SOCKET CREATION FAILED");
        return ESP_FAIL;
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
        return ESP_FAIL;
    }
    if (connect(server_fd, res->ai_addr, res->ai_addrlen) == -1)
    {
        DEBUG_PRINT_E("initTcpClient", "ERROR: CONNECT FAILED");
        free(res);
        shutdown(server_fd, 0);
        close(server_fd);
        server_fd = 0;
        return ESP_FAIL;
    }

    DEBUG_PRINT_I("initTcpClient", "Client successfully connected");
    return server_fd;
}

/**
 * \brief Sends data through a TCP connection or serial port.
 *
 * This function is responsible for sending data over an established TCP connection or serial connection. It continues to
 * send all data in the buffer, handling partial writes as needed.
 *
 * \param[in] fd The file descriptor of the established TCP connection.
 * \param[in] len The length of the data in bytes.
 * \param[in] buff Pointer to the data buffer.
 *
 * \return ESP_OK - data was sent successfully, ESP_FAIL - Error.
 */
esp_err_t IRAM_ATTR tcpSerialSend(uint32_t fd, int len, const uint8_t *buff)
{
    const uint8_t *ptr;
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
