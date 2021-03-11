#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include "driver/gpio.h"


#define LED_BAT_GPIO	GPIO_NUM_25

#define I1_GPIO 		GPIO_NUM_26
#define I2_GPIO 		GPIO_NUM_27
#define O1_GPIO 		GPIO_NUM_14
#define O2_GPIO 		GPIO_NUM_12


void ledBatConfig() {
	gpio_set_direction(LED_BAT_GPIO, GPIO_MODE_INPUT_OUTPUT);
}


void setLedBat() {
	gpio_set_level(LED_BAT_GPIO, 1);
}


void clearLedBat() {
	gpio_set_level(LED_BAT_GPIO, 0);
}


void inputConfig() {
	gpio_set_direction(I1_GPIO, GPIO_MODE_INPUT);
	gpio_set_direction(I2_GPIO, GPIO_MODE_INPUT);
	gpio_set_pull_mode(I1_GPIO, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(I2_GPIO, GPIO_PULLUP_ONLY);
}


void outputConfig() {
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[O1_GPIO], PIN_FUNC_GPIO);
	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[O2_GPIO], PIN_FUNC_GPIO);
	gpio_set_direction(O1_GPIO, GPIO_MODE_INPUT_OUTPUT);
	gpio_set_direction(O2_GPIO, GPIO_MODE_INPUT_OUTPUT);
}


void setOutputsLevel(uint8_t dataBuffer) {
	uint8_t o1_level = (dataBuffer >> 2) & 1;
	uint8_t o2_level = (dataBuffer >> 3) & 1;

	gpio_set_level(O1_GPIO, o1_level);
	gpio_set_level(O2_GPIO, o2_level);
}


int isI1high() {
	return gpio_get_level(I1_GPIO);
}


int isI2high() {
	return gpio_get_level(I2_GPIO);
}


int isO1high() {
	return gpio_get_level(O1_GPIO);
}


int isO2high() {
	return gpio_get_level(O2_GPIO);
}


int isLedBatHigh() {
	return gpio_get_level(LED_BAT_GPIO);
} 
