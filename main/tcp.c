#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "main.h"
#include "macros.h"
#include "tcp.h"
#include "bt.h"


int initTcpServer(char* port_str){
    int port;
    struct sockaddr_in local_addr;
    int server_fd;

    sscanf(port_str, "%d", &port);  //Transform port string to int
	
	if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0){										//Verificar se não houve erro a criar a socket
		DEBUG_PRINT_E("initTcpServer", "socket error");
	}

	local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);                     
    local_addr.sin_port = htons(port); 

	if(bind(server_fd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0){					//Verificar se não houve erro a fazer bind
		DEBUG_PRINT_E("initTcpServer", "bind error");
	}

	if(listen(server_fd, 2) == -1){																//Verificar se não houve erro a fazer listen
		DEBUG_PRINT_E("initTcpServer", "listen error");
	}
	return server_fd;
}

int initTcpClient(char* ip, char* port){
	struct addrinfo hints,*res;
    int client_fd;

    client_fd = socket(AF_INET, SOCK_STREAM, 0);    // TCP socket
    if (client_fd == -1){ 
        DEBUG_PRINT_E("initTcpClient", "ERROR: SOCKET CREATION FAILED"); 
        return -1;
    }
    
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;			//IPv4
    hints.ai_socktype = SOCK_STREAM;	//TCP socket

    if (getaddrinfo(ip, port, &hints, &res) != 0){
        DEBUG_PRINT_E("initTcpClient", "ERROR: GETADDRINFO FAILED");
        shutdown(client_fd, 0);
        close(client_fd);
        client_fd = 0;
        return -1;
    }
    if (connect(client_fd, res->ai_addr, res->ai_addrlen) == -1){
        DEBUG_PRINT_E("initTcpClient", "ERROR: CONNECT FAILED");
        shutdown(client_fd, 0);
        close(client_fd);
        client_fd = 0;
        return -1;
    }

    DEBUG_PRINT_I("initTcpClient", "Client successfully connected");
    return client_fd;
}

esp_err_t IRAM_ATTR tcpSend(uint32_t fd, int len, uint8_t *buff){
    uint8_t *ptr;
    ssize_t n_left = len;
    ssize_t n_written;

    ptr = buff;

    while (n_left > 0){
        n_written = write(fd, ptr, n_left);
        if (n_written == -1) {
            DEBUG_PRINT_E("tcpSend", "ERROR: WRITE FAILED\n");
            return ESP_FAIL;
        }
        n_left -= n_written;
        ptr += n_written;
    }
    finalizeSend();
    DEBUG_PRINT_I("tcpSend", "[TCP] Sent: %d bytes", len);

    //Try to send next buff
    sendData(); 
    return ESP_OK;
}

void tcpRcv(){
    uint8_t buff[CMD_MAX_BYTES];
    int len;
    int read_bytes;

    while(1){
        //Wait until a connection is made
        while(send_fd <= 0){
            vTaskDelay(500/portTICK_PERIOD_MS);
        }

        while(1){
            if((read_bytes = read(send_fd, buff, CMD_MAX_BYTES)) == 0){
                DEBUG_PRINT_W("wifiRcvTask", "Connection closed gracefully");
                buff[0] = 0;
                len = 1;
            }else if(read_bytes < 0){
                DEBUG_PRINT_W("wifiRcvTask", "Connection closed with errno %d", errno);
                buff[0] = 0;
                len = 1;
            }else{
                //TODO: implement walk around (if a message is 3bytes, it may happen that it is divided into a read of 2bytes and a read of 1byte, thus the check below will give a false positive error)
                if(read_bytes != CMD_MAX_BYTES){
                    DEBUG_PRINT_E("wifiRcvTask", "Recieved %d bytes and the acceptable amount is %d", read_bytes, CMD_MAX_BYTES);
                }
                len = CMD_MAX_BYTES;
            }
            processRcv(buff, len);

            //If connection was broken, close socket and return
            if(read_bytes <= 0){
                shutdown(send_fd, 0);
                close(send_fd);
                send_fd = 0;
                return;
            }
        }
    }
}