/** \file udp.c
    \brief Contains functions to send data over UDP

    This file contains functions to send data over UDP. The data is sent to the
   server specified in the config file.
*/
#include "udp.h"

#include <lwip/netdb.h>

#include "bt.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "macros.h"
#include "macros_conf.h"
#include "scientisst.h"

struct addrinfo *udp_server_addr;

/**
 * \brief Initializes the UDP client
 *
 * This function initializes the UDP client. It creates a socket and performs a
 * handshake with the server.
 *
 * \param ip The IP address of the server
 * \param port The port of the server
 *
 * \return:
 *      - The file descriptor of the socket if successful
 *      - -1 if an error occurred
 */
int initUdpClient(char *ip, char *port)
{
    struct addrinfo hints;
    int server_fd;

    server_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (server_fd == -1)
    {
        DEBUG_PRINT_E("initUdpClient", "ERROR: SOCKET CREATION FAILED");
        return -1;
    }

    // getaddrinfo() returns a list of sockaddr given the ip/host and port. This list is stored in
    // udp_server_addr
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET; // IPv4
    hints.ai_socktype = SOCK_DGRAM;

    if (getaddrinfo(ip, port, &hints, &udp_server_addr) != 0)
    {
        DEBUG_PRINT_E("initUdpClient", "ERROR: GETADDRINFO FAILED");
        freeaddrinfo(udp_server_addr);
        shutdown(server_fd, 0);
        close(server_fd);
        server_fd = 0;
        return -1;
    }

    // Perform handshake, so that the server has our address
    if (sendto(server_fd, "handshake", strlen("handshake") + 1, 0, udp_server_addr->ai_addr,
               udp_server_addr->ai_addrlen) < 0)
    {
        DEBUG_PRINT_E("initUdpClient", "ERROR: sendto FAILED");
        freeaddrinfo(udp_server_addr);
        shutdown(server_fd, 0);
        close(server_fd);
        server_fd = 0;
        return -1;
    }

    DEBUG_PRINT_I("initUdpClient", "Client successfully created");
    return server_fd;
}

/**
 * \brief Sends data over UDP
 *
 * This function sends data over UDP. It is called by sendData() when in UDP
 * mpde.
 *
 * \param fd The file descriptor of the socket
 * \param len The length of the data to be sent
 * \param buff The data to be sent
 *
 * \return:
 *      - ESP_OK if successful
 *      - ESP_FAIL if an error occurred
 */
esp_err_t IRAM_ATTR udpSend(uint32_t fd, int len, uint8_t *buff)
{
    int sent_bytes;
    if ((sent_bytes = sendto(fd, buff, len, 0, udp_server_addr->ai_addr, udp_server_addr->ai_addrlen)) != len)
    {
        DEBUG_PRINT_E("udpSend", "ERROR: WRITE FAILED, sent %dbytes of %d bytes, errno:%d\n", sent_bytes, len,
                      errno);
        ESP_ERROR_CHECK(errno);
        return ESP_FAIL;
    }

    finalizeSend();
    DEBUG_PRINT_I("udpSend", "[UDP] Sent: %d bytes", len);

    // Try to send next buff
    sendData();
    return ESP_OK;
}