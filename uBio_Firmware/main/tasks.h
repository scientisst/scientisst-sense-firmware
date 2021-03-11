#ifndef MAIN_TASKS_H_
#define MAIN_TASKS_H_

void createFramesRingBuffer(uint8_t no_bytes);
void createPulseLEDTask(void (*taskCode)(void *));
void createManageClientTask(void (*taskCode)(void *));
void createSocketRecvTask(void (*taskCode)(void *));
void createSocketSendTask(void (*taskCode)(void *));
void createGpioInterruptTask(void (*taskCode)(void *));

#endif /* MAIN_TASKS_H_ */