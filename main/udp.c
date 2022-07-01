#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "scientisst.h"
#include "macros.h"
#include "udp.h"
#include "bt.h"

struct addrinfo *udp_server_addr;

int initUdpClient(char* ip, char* port){
	struct addrinfo hints;
    int server_fd;

    server_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (server_fd == -1){ 
        DEBUG_PRINT_E("initUdpClient", "ERROR: SOCKET CREATION FAILED"); 
        return -1;
    }
    
    //getaddrinfo() returns a list of sockaddr given the ip/host and port. This list is stored in udp_server_addr
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;			//IPv4
    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(ip, port, &hints, &udp_server_addr) != 0){
        DEBUG_PRINT_E("initUdpClient", "ERROR: GETADDRINFO FAILED");
        freeaddrinfo(udp_server_addr);
        shutdown(server_fd, 0);
        close(server_fd);
        server_fd = 0;
        return -1;
    }

    //Perform handshake, so that the server has our address
    if(sendto(server_fd, "handshake", strlen("handshake")+1, 0, udp_server_addr->ai_addr, udp_server_addr->ai_addrlen) < 0){
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

esp_err_t IRAM_ATTR udpSend(uint32_t fd, int len, uint8_t *buff){
    int sent_bytes;
    if((sent_bytes = sendto(fd, buff, len, 0, udp_server_addr->ai_addr, udp_server_addr->ai_addrlen)) != len){
        DEBUG_PRINT_E("udpSend", "ERROR: WRITE FAILED, sent %dbytes of %d bytes, errno:%d\n", sent_bytes, len, errno);
        ESP_ERROR_CHECK(errno);
        return ESP_FAIL;
    }

    finalizeSend();
    DEBUG_PRINT_I("udpSend", "[UDP] Sent: %d bytes", len);

    //Try to send next buff
    sendData(); 
    return ESP_OK;
}