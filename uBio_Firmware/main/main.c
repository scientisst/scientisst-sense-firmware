#include <stdio.h>
#include <stdint.h>
#include "pwm.h"
#include "gpio.h"
#include "spi_adc.h"
#include "adc.h"
#include "timer.h"
#include "bootwifi.h"
#include "tcp.h"
#include "tasks.h"


void app_main() {
    printf(">> hello world!\n");

    ledStatConfig(); // configure STAT LED

    ledBatConfig(); // configure battery LED

    spiConfig(); // configure SPI 

    adcConfig(); // configure ADC

    pwmConfig(); // configure PWM

    inputConfig(); // configure input pins
    outputConfig(); // configure output pins

    createGpioInterruptTask(&gpioInterruptTask); 

    uint8_t op_mode = 1;
    // check operation mode (WiFi or Bluetooth)
    if (op_mode == 1) { // WiFi - softAP mode
		bootWiFi(); 
		extern int serverSocket;
		serverSocket = createTCPserver();
        createManageClientTask(&waitClientTask);
    	createSocketRecvTask(&tcpRecvTask);
    }
}
