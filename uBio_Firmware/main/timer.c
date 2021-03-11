#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h> 
#include <esp_system.h>
#include "driver/gpio.h"
#include <string.h>
#include "tasks.h"
#include "spi_adc.h"
#include "adc.h"
#include "gpio.h"
#include "bitalino-functions.h"
#include <freertos/ringbuf.h>

#define ESP_INTR_FLAG_DEFAULT 0

uint8_t gpioInterruptRunning = 0;
extern uint8_t led_step;

extern RingbufHandle_t framesRingBuffer;
BaseType_t xHigherPriorityTaskWoken;

extern uint8_t countIterations;
extern uint16_t interruptIterations;
extern uint16_t no_interrupt_iterations;
extern uint16_t bat_threshold;
extern uint8_t simulated;
extern uint8_t simulated_seq;
extern uint8_t sequence_no;
extern uint8_t no_channels_bit;
extern uint8_t no_channels_ads;
extern uint8_t channel_table[6];
extern uint8_t start_position;
extern uint8_t ads_channel_mask;
extern uint8_t no_bytes;
extern uint8_t no_bytes_bit;
extern uint8_t no_bytes_ads;
uint8_t s50 = 0;
uint8_t s25 = 0;
uint8_t s20 = 0;
uint8_t frame[14] = {0};
uint8_t bitFrame[8] = {0};
uint8_t adsFullFrame[9] = {0}; // 24 status bits + 24 bits x 2 channels
uint8_t adsFrame[6] = {0};


void createDataPacket() {
	gpio_set_level(GPIO_NUM_4, 0);

	if (!simulated) { // live mode
		memset(bitFrame, 0, 7);

		if (isI1high())
 			bitFrame[6] |= 0x80;
	 	if (isI2high())
	 		bitFrame[6] |= 0x40;
	 	if (isO1high())
	 		bitFrame[6] |= 0x20;
	 	if (isO2high())
	 		bitFrame[6] |= 0x10;

		if (no_channels_bit > 0)
			*(uint16_t*)(bitFrame+5) |= adcRead(channel_table[0]) << 2;
		if (no_channels_bit > 1)
			*(uint16_t*)(bitFrame+4) |= adcRead(channel_table[1]);
		if (no_channels_bit > 2)
		 	*(uint16_t*)(bitFrame+2) |= adcRead(channel_table[2]) << 6;
		if (no_channels_bit > 3)
	 		*(uint16_t*)(bitFrame+1) |= adcRead(channel_table[3]) << 4;
		if (no_channels_bit > 4)
			*(uint16_t*)bitFrame |= (adcRead(channel_table[4]) & 0x3F0) << 2; // only the 6 upper bits of the 10-bit value are used
		if (no_channels_bit > 5)
			bitFrame[0] |= adcRead(channel_table[5]) >> 4; // only the 6 upper bits of the 10-bit value are used

		if (no_channels_ads < 2)
			memcpy(adsFrame, &adsFullFrame[3*ads_channel_mask], no_bytes_ads);
		else
			memcpy(adsFrame, &adsFullFrame[3], no_bytes_ads);

		if (!isLedBatHigh()) { // check battery voltage if battery LED not already on
			if (adcRead(channel_table[0]) < bat_threshold) // !! REPLACE channel_table[0] with the ADC pin for ABAT input !!
				setLedBat();
		}
	}
	else { // simulated mode
		memset(bitFrame, 0, 6);

		bitFrame[6] = sequence_no << 4;

		s50 = (simulated_seq >= 50) ? simulated_seq-50 : simulated_seq;
		s25 = (s50 >= 25) ? s50-25 : s50;
		s20 = simulated_seq % 20;

		*(uint16_t*)(bitFrame+5) |= simulateData(0, s50, s25, s20) << 2;
		if (no_channels_bit > 1)
		  *(uint16_t*)(bitFrame+4) |= simulateData(1, s50, s25, s20);
		if (no_channels_bit > 2)
		  *(uint16_t*)(bitFrame+2) |= simulateData(2, s50, s25, s20) << 6;
		if (no_channels_bit > 3)
		  *(uint16_t*)(bitFrame+1) |= simulateData(3, s50, s25, s20) << 4;
		if (no_channels_bit > 4)
		  *(uint16_t*)bitFrame |= simulateData(4, s50, s25, s20) << 6;
		if (no_channels_bit > 5)
			bitFrame[0] |= simulateData(5, s50, s25, s20);

		simulated_seq++;
		if (simulated_seq == 100)
			simulated_seq = 0;
	}

	bitFrame[7] = calculateSeqNoCRC(bitFrame);

	memcpy(frame, adsFrame, no_bytes_ads);

	memcpy(&frame[no_bytes_ads], &bitFrame[start_position], no_bytes_bit);

	sequence_no++;

	xRingbufferSendFromISR(framesRingBuffer, frame, no_bytes, &xHigherPriorityTaskWoken);

	gpio_set_level(GPIO_NUM_4, 1);
} 


void IRAM_ATTR drdyISR() {
	if (countIterations == 1) {
		interruptIterations++;
		if (interruptIterations == no_interrupt_iterations) {
			interruptIterations = 0;
			if (no_bytes_ads > 0)
				adsReadDataContinuous(adsFullFrame);
    		createDataPacket();
		}
	}
	else { // acquire at each interrupt
		if (no_bytes_ads > 0)
			adsReadDataContinuous(adsFullFrame);
    	createDataPacket();
	}
}


void gpioInterruptEnable() {
	gpio_intr_enable(DRDY_GPIO);
}


void gpioInterruptDisable() {
	gpio_intr_disable(DRDY_GPIO);
}


void gpioInterruptStop() {
	led_step = 10;
	gpioInterruptRunning = 0;

	adsStop();

	printf(">> OK: GPIO interrupt stopped\n");
}


void gpioInterruptStart() {
	led_step = 100;
	gpioInterruptRunning = 1;

	adsStart();

	printf(">> OK: GPIO interrupt started\n");
}


void gpioInterruptConfig() {
	adsSetupRoutine(); // configure external ADC that with a GPIO interrupt controls sampling rate

	gpio_set_intr_type(DRDY_GPIO, GPIO_INTR_NEGEDGE);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM); // install GPIO ISR service
    gpio_isr_handler_add(DRDY_GPIO, drdyISR, NULL); // hook ISR handler for DRDY pin

    gpioInterruptEnable();

    printf(">> OK: GPIO interrupt configured\n");
}


void gpioInterruptTask(void *parameter) {
	printf(">> OK: gpio interrupt task running ...\n");

	gpioInterruptConfig(); // configure GPIO interrupt

	gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT_OUTPUT);

	vTaskDelete(NULL);
}
