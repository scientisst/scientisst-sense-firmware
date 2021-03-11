#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>

#define RINGBUF_SIZE 4096

RingbufHandle_t framesRingBuffer;

TaskHandle_t xHandleManageClient;
TaskHandle_t xHandleSocketRecv;
TaskHandle_t xHandleSocketSend;


void createFramesRingBuffer(uint8_t no_bytes) {
	framesRingBuffer = xRingbufferCreateNoSplit(no_bytes, RINGBUF_SIZE); // each item has no_bytes size
	printf(">> OK: ringbuffer created\n");
}


void createPulseLEDTask(void (*taskCode)(void *)) {
	xTaskCreatePinnedToCore(taskCode, "pulseLEDTask", 1024, NULL, 3, NULL, 0);
	printf(">> OK: pulseLEDTask created\n");
}


void createManageClientTask(void (*taskCode)(void *)) {
	xTaskCreatePinnedToCore(taskCode, "manageClientTask", 2048, NULL, 3, &xHandleManageClient, 1);
	printf(">> OK: manageClientTask created\n");
}


void createSocketRecvTask(void (*taskCode)(void *)) {
	xTaskCreatePinnedToCore(taskCode, "socketRecvTask", 2048, NULL, 7, &xHandleSocketRecv, 0);
	vTaskSuspend(xHandleSocketRecv);
	printf(">> OK: socketRecvTask created\n");
}


void createSocketSendTask(void (*taskCode)(void *)) {
	xTaskCreatePinnedToCore(taskCode, "socketSendTask", 2048, NULL, 5, &xHandleSocketSend, 0);
	printf(">> OK: socketSendTask created\n");
}


void createGpioInterruptTask(void (*taskCode)(void *)) {
	xTaskCreatePinnedToCore(taskCode, "gpioInterruptCreationTask", 1024, NULL, 3, NULL, 1);
	// when generating an interrupt, it is hard-wired to its associated core
	// WiFi task runs on core 0 and so the gpio ISR must execute on core 1
	printf(">> OK: gpioInterruptTask created\n");
}
