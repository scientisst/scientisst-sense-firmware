#ifndef MAIN_TCP_H_
#define MAIN_TCP_H_

// #define BAT_THRESHOLD_BASE (527)
#define BAT_THRESHOLD_BASE (867)

int socketWaitClient(int serverSocket);
int createTCPserver();
void socketWrite(int clientSocket, uint8_t *dataBuffer, size_t size);
void waitClientTask(void *parameter);
void tcpRecvTask(void *parameter);
void tcpSendTask(void *parameter);

#endif /* MAIN_TCP_H_ */