/**
 * \file sci_udp.c
 * \brief UDP Communication Implementation.
 *
 * This file contains the implementation of functions used for UDP communication.
 */
#include "sci_udp.h"

#include "esp_attr.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

static struct addrinfo *udp_server_addr;

/**
 * \brief Initializes the UDP client.
 *
 * This function sets up the UDP client by establishing a socket connection and performing a handshake with the server.
 *
 * \param[in] ip The IP address of the server.
 * \param[in] port The port on the server to connect to.
 *
 * \return The file descriptor for the socket connected to the server, or ESP_FAIL if an error occurs.
 */
int initUdpClient(const char *ip, const char *port)
{
    struct addrinfo hints;
    int server_fd;

    server_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (server_fd == -1)
    {
        DEBUG_PRINT_E("initUdpClient", "ERROR: SOCKET CREATION FAILED");
        return ESP_FAIL;
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
        return ESP_FAIL;
    }

    // Perform handshake, so that the server has our address
    if (sendto(server_fd, "handshake", strlen("handshake") + 1, 0, udp_server_addr->ai_addr, udp_server_addr->ai_addrlen) <
        0)
    {
        DEBUG_PRINT_E("initUdpClient", "ERROR: sendto FAILED");
        freeaddrinfo(udp_server_addr);
        shutdown(server_fd, 0);
        close(server_fd);
        server_fd = 0;
        return ESP_FAIL;
    }

    DEBUG_PRINT_I("initUdpClient", "Client successfully created");
    return server_fd;
}

/**
 * \brief Sends data to a server via UDP.
 *
 * This function is responsible for sending data packets over a UDP connection.
 *
 * \param[in] fd The file descriptor of the socket.
 * \param[in] len The length of the data in bytes.
 * \param[in] buff The buffer containing the data to be sent.
 *
 * \return ESP_OK - the data was sent successfully,  ESP_FAIL - an error occurred during transmission.
 */
esp_err_t udpSend(uint32_t fd, int len, const uint8_t *buff)
{
    int sent_bytes;
    if ((sent_bytes = sendto(fd, buff, len, 0, udp_server_addr->ai_addr, udp_server_addr->ai_addrlen)) != len)
    {
        DEBUG_PRINT_E("udpSend", "ERROR: WRITE FAILED, sent %dbytes of %d bytes, errno:%d", sent_bytes, len, errno);
        ESP_ERROR_CHECK(errno);
        return ESP_FAIL;
    }

    return ESP_OK;
}
