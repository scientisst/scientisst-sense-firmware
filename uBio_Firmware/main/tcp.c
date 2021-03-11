#include "freertos/FreeRTOS.h"
#include <lwip/sockets.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "tcp.h"
#include "timer.h"
#include "gpio.h"
#include "spi_adc.h"
#include "pwm.h"
#include "tasks.h"
#include "bitalino-functions.h"
#include <freertos/ringbuf.h>


#define PORT_NUMBER (8001)
#define MAX_PENDING_CONNECTIONS (1)

#define RECV_PKTSIZE (1)
// assuming the computer issues 1-byte commands only

extern RingbufHandle_t framesRingBuffer;
extern TaskHandle_t xHandleManageClient;
extern TaskHandle_t xHandleSocketSend;

int serverSocket;
int clientSocket;

uint16_t sampling_rate = 0;
uint8_t set_sampling_rate = 0;
uint8_t countIterations = 0;
uint16_t interruptIterations = 0;
uint16_t no_interrupt_iterations = 0;
uint8_t incomingCHcmd = 0;
uint8_t incomingPWMcmd = 0;
uint16_t bat_threshold = BAT_THRESHOLD_BASE;
uint8_t simulated;
uint8_t simulated_seq;
uint8_t sequence_no;
uint8_t no_channels_bit;
uint8_t no_channels_ads;
uint8_t channel_table[6];
uint8_t start_position;
uint8_t ads_channel_mask;
uint8_t no_bytes;
uint8_t no_bytes_bit;
uint8_t no_bytes_ads;
uint8_t map_analog_channels[6] = {0, 3, 6, 7, 4, 5};

// forward declarations 
void tcpSendTask(void *parameter); 


int socketWaitClient(int serverSocket) {
	static struct sockaddr_in clientAddress;
	socklen_t clientAddressLength = sizeof(clientAddress);

	// listen for a new client connection
	printf(">> OK: server socket is listening for clients ...\n");
	int clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddress, &clientAddressLength);
	if (clientSocket < 0) {
		printf(">> ERROR: failed to accept a client\n");
		close(serverSocket);
		return ESP_FAIL;
	}

	// connection established
    printf(">> OK: TCP connection ESTABLISHED\n");

    return clientSocket;;
}


int createTCPserver() {
	// create a socket that we will listen upon
	int serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (serverSocket < 0) {
		printf(">> ERROR: failed to create the server socket\n");
		return ESP_FAIL;
	}
	else {
		printf(">> OK: server socket created\n");
	}

	// bind the server socket to a port
	struct sockaddr_in serverAddress;
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_port = htons(PORT_NUMBER);
	serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
	if (bind(serverSocket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0) {
		printf(">> ERROR: failed to bind the server socket\n");
		close(serverSocket);
		return ESP_FAIL;
	}
	printf(">> OK: server socket binded\n");

	// flag the socket as listening for new connections
	if (listen(serverSocket, MAX_PENDING_CONNECTIONS) < 0) {
		printf(">> ERROR: failed to listen on server socket\n");
		close(serverSocket);
		return ESP_FAIL;
	}

	return serverSocket;
}


void socketWrite(int clientSocket, uint8_t *dataBuffer, size_t size) {
	int ret = write(clientSocket, dataBuffer, size);
	if (ret < 0)
		printf(">> ERROR: %s\n", strerror(errno));
}


void waitClientTask(void *parameter) {
	extern TaskHandle_t xHandleSocketRecv;

	while (1) {
		printf(">> OK: idle task running ...\n");

		clientSocket = socketWaitClient(serverSocket); // accept client connections on the server socket

		if (clientSocket != ESP_FAIL) {
			vTaskResume(xHandleSocketRecv)
;			vTaskSuspend(NULL);
		}
	}
}


void tcpRecvTask(void *parameter) {
	printf(">> OK: socket receiving task running ...\n");

	extern uint8_t gpioInterruptRunning;

	char *dataBuffer = (char *)malloc(RECV_PKTSIZE * sizeof(char));
	ssize_t bytesRead = 0;

	while(1) {
		bytesRead = recv(clientSocket, dataBuffer, RECV_PKTSIZE, 0);
		if (bytesRead > 0) {
			printf(">> OK: via SOCKET received %i byte(s) with the message \"0x%02X\"\n", bytesRead, dataBuffer[0]);
			// parse received command

			if (incomingCHcmd == 1) { 
				incomingCHcmd = 0;
				sequence_no = 0;
			   
			    // BITalino channels
			    no_channels_bit = 0;
			   	for (uint8_t i = 0; i < 6; i++) { 
					if (dataBuffer[0] & 0x01)
                		channel_table[no_channels_bit++] = simulated ? i : map_analog_channels[i];
               		dataBuffer[0] >>= 1;
	            }
		        start_position = 6 - no_channels_bit;
				if (no_channels_bit >= 3 && no_channels_bit <= 5) 
					start_position--;
			    if (no_channels_bit <= 4)
					no_bytes_bit = (int)(ceil((12. + 10. * no_channels_bit) / 8.));
				else
					no_bytes_bit = (int)(ceil((52. + 6. * (no_channels_bit - 4)) / 8.));

				// ADS channels
				no_channels_ads = 0;
				ads_channel_mask = 0;
				for (uint8_t i = 1; i < 3; i++) { // BITalino channels
					if (dataBuffer[0] & 0x01) {
                		ads_channel_mask |= i;
                		no_channels_ads++;
                	}
               		dataBuffer[0] >>= 1;
	            }
	            adsConfigureChannels(ads_channel_mask);
	            no_bytes_ads = 0;
	            if (ads_channel_mask == 3)
	            	no_bytes_ads = 6;
	           	if (ads_channel_mask == 1 || ads_channel_mask == 2)
	           		no_bytes_ads = 3;

				no_bytes = no_bytes_bit + no_bytes_ads;

				createFramesRingBuffer(no_bytes);
				createSocketSendTask(&tcpSendTask);
				gpioInterruptStart(); 
				continue;
			}

			if (incomingPWMcmd == 1) {
	      		incomingPWMcmd = 0;
			    setDCpwm(dataBuffer[0]); // set PWM output duty cycle
				continue;
			}

			if ((dataBuffer[0] & 0xA3) == 0xA3 && dataBuffer[0] != 0xFF) {
				if (dataBuffer[0] == 0xA3) // set analog output
		    		incomingPWMcmd = 1;
		    	else if ((dataBuffer[0] & 0xF3) == 0xB3) // set digital outputs
					setOutputsLevel(dataBuffer[0]); // update O1 and O2
				else if ((dataBuffer[0] & 0xF3) == 0xE3) // set sampling rate at 8000 Hz
					adsSetSamplingRate(8000);
				continue;
			}

			if (gpioInterruptRunning == 0) { // idle mode
      			switch (dataBuffer[0] & 3) {
         			case 0: // battery threshold definition
            			bat_threshold = BAT_THRESHOLD_BASE + (dataBuffer[0] >> 2);  // dataBuffer[0] = 0 -> 527 = 3.4 V ; dataBuffer[0] = 63 -> 527+63 = 590 = 3.8 V
            			clearLedBat(); // battery LED may turn off with new threshold value
        				break;

     				case 1: // start acquisition mode (live)

         			case 2: // start acquisition mode (simulated)
        				if ((dataBuffer[0] & 3) == 2) { // simulated mode
        					printf(">> OK: simulated mode\n");
		               		simulated = 1;
		               		simulated_seq = 0;
            			}
            			else {
            				printf(">> OK: live mode\n");
               				simulated = 0;
            			}

            			incomingCHcmd = 1;
            			break;
			            
         			case 3:
            			switch (dataBuffer[0]) {
               				case 0x07: // send version string
               				case 0x0F:
		                  		sendVersion();
		                  		break;
               				case 0x0B: // send device status
                  				sendStatus();
                  				break;
							case 0x03: // set sampling rate at 1 Hz
								sampling_rate = 1; // count 1000 iterations
								set_sampling_rate = 1;
								countIterations = 1;
					  			break;
					  		case 0x23: // set sampling rate at 10 Hz
								sampling_rate = 10; // count 100 iterations
								set_sampling_rate = 1;
								countIterations = 1;
					  			break;
				  			case 0x43: // set sampling rate at 100 Hz
								sampling_rate = 100; // count 10 iterations
								set_sampling_rate = 1;
								countIterations = 1;
					  			break;
				  			case 0x63: // set sampling rate at 1000 Hz
								sampling_rate = 1000;
								set_sampling_rate = 1;
					  			break;
							case 0x83: // set sampling rate at 2000 Hz
								sampling_rate = 2000;
								set_sampling_rate = 1;
								break;
							case 0xC3: // set sampling rate at 4000 Hz
								sampling_rate = 4000;
								set_sampling_rate = 1;
								break;
							// command 0xFF (stop) is ignored in idle mode
            			}
            			if (set_sampling_rate == 1) {
            				set_sampling_rate = 0;
            				if (countIterations == 1) {
            					adsSetSamplingRate(1000);
            					no_interrupt_iterations = 1000/sampling_rate;
            				}
            				else {
            					adsSetSamplingRate(sampling_rate);
            				}
            			}
            			break;
      			}
      		}
   			else { // live mode
      			if (dataBuffer[0] == 0 || dataBuffer[0] == 0xFF) { // stop live/simulated mode
      				if (gpioInterruptRunning == 1) {
						gpioInterruptStop();
						countIterations = 0;
						interruptIterations = 0;
      				}
				}
   			}
   			continue;
		}
		else if (bytesRead == 0) {
			printf(">> OK: connection DISCONNECTED\n");
			if (gpioInterruptRunning == 1) { // just in case the user doesn't call stop before closing
				gpioInterruptStop();
				countIterations = 0;
				interruptIterations = 0;
			}
			shutdown(clientSocket, SHUT_RDWR);
			close(clientSocket);
			vTaskDelete(xHandleSocketSend);
			vRingbufferDelete(framesRingBuffer);
			printf(">> OK: client socket closed\n");
			vTaskResume(xHandleManageClient);
			vTaskSuspend(NULL);
		}
	}
}


void tcpSendTask(void *parameter) {
	printf(">> OK: socket sending task running ...\n");

	uint8_t *frame;
	size_t item_length;

	while (1) {
		frame = xRingbufferReceive(framesRingBuffer, &item_length, portMAX_DELAY);
		if (item_length == no_bytes) {
			vRingbufferReturnItem(framesRingBuffer, frame);
			socketWrite(clientSocket, frame, no_bytes);
		}
	}
}
