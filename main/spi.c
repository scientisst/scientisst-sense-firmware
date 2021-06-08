#include "spi.h"
#include "adc.h"
#include "macros.h"
#include "com.h"
#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include <math.h>

#define MAX_TRANSFER_SIZE 0

#define DMA_CHAN        0		//No DMA

#define ADC_EXT_SLCK_HZ 1*1000*1000           //Clock out at 1 MHz, divisors of 80MHz

// register address
#define ADS1292_REG_ID			0x00
#define ADS1292_REG_CONFIG1		0x01
#define ADS1292_REG_CONFIG2		0x02
#define ADS1292_REG_LOFF		0x03
#define ADS1292_REG_CH1SET		0x04
#define ADS1292_REG_CH2SET		0x05
#define ADS1292_REG_RLDSENS		0x06
#define ADS1292_REG_LOFFSENS	0x07
#define ADS1292_REG_LOFFSTAT    0x08
#define ADS1292_REG_RESP1	    0x09
#define ADS1292_REG_RESP2	    0x0A

#define START	0x08 	//start/restart (synchronize) conversions
#define STOP	0x0A 	//stop conversion
#define RDATAC  0x10 	//enable Read Data Continuous mode - the default at power-up
#define SDATAC	0x11 	//stop Read Data Continuously mode
#define RESET	0x06	//Reset ADS

#define RREG	0x20;	//read n nnnn registers starting at address r rrrr |||| first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define WREG 	0x40; 	//write n nnnn registers starting at address r rrrr |||| first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)

void adsInit(){
    esp_err_t ret;

    spi_bus_config_t buscfg={
        .miso_io_num = SPI3_MISO_IO,
        .mosi_io_num = SPI3_MOSI_IO,
        .sclk_io_num = SPI3_SCLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_TRANSFER_SIZE
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz = ADC_EXT_SLCK_HZ,
        .mode = 1,                                //SPI mode 1: (CPOL) = 0 and the clock phase (CPHA) = 1. 
        .spics_io_num = SPI3_CS0_IO,               //CS pin
        .queue_size = 1,                          //We want to be able to queue 1 transactions at a time
        //.pre_cb = lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(SPI3_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(SPI3_HOST, &devcfg, &ads_spi_handler);
    ESP_ERROR_CHECK(ret);
}

void adsSendCmd(uint8_t cmd){
	spi_transaction_t transaction;

    memset(&transaction, 0, sizeof(transaction)); // zero out the transaction
    transaction.flags = SPI_TRANS_USE_TXDATA;
    transaction.length = 8; 
    transaction.tx_data[0] = cmd;
    transaction.rx_buffer = NULL; // skip read phase
	spi_device_polling_transmit(ads_spi_handler, &transaction);
    
    vTaskDelay(10/portTICK_PERIOD_MS);
}

void adsWriteRegister(uint8_t address, uint8_t data) {
	uint8_t first_opcode = address | WREG;
	uint8_t second_opcode = 0x00;

	switch (address){
		case 1:
        	data = data & 0x87;
	    	break;
    	case 2:
     		data = data & 0xFB;
	    	data |= 0x80;		
	    	break;
	    case 3:
		    data = data & 0xFD;
		    data |= 0x10;
		    break;
	    case 7:
		    data = data & 0x3F;
		    break;
	    case 8:
    	    data = data & 0x5F;
		    break;
	    case 9:
		    data |= 0x02;
		    break;
	    case 10:
		    data = data & 0x87;
		    data |= 0x01;
		    break;
	    case 11:
		    data = data & 0x0F;
		    break;
	    default:
		    break;		
  	}

	spi_transaction_t transaction;
   
	memset(&transaction, 0, sizeof(transaction)); // zero out the transaction
   	transaction.flags = SPI_TRANS_USE_TXDATA;
   	transaction.length = 24;
    transaction.tx_data[0] = first_opcode;
    transaction.tx_data[1] = second_opcode;
    transaction.tx_data[2] = data;
    transaction.rx_buffer = NULL; // skip read phase
    spi_device_polling_transmit(ads_spi_handler, &transaction);  //Transmit!

	//vTaskDelay(10/portTICK_PERIOD_MS);
	vTaskDelay(40/portTICK_PERIOD_MS);
}

void adsSetupRoutine(){
	adsSendCmd(RESET);
    vTaskDelay(100/portTICK_PERIOD_MS);

	adsSendCmd(SDATAC);
	vTaskDelay(100/portTICK_PERIOD_MS);

	// adsReadRegister(ADS1292_REG_ID);
	// adsReadRegister(ADS1292_REG_CONFIG1);
	// adsReadRegister(ADS1292_REG_CONFIG2);

	adsWriteRegister(ADS1292_REG_CONFIG2, 0x80); // disable reference buffer, Lead-off comparator, etc
	adsWriteRegister(ADS1292_REG_LOFF, 0x10); // lead-off defaults

	adsWriteRegister(ADS1292_REG_CH1SET, 0x10); // Unit gain, normal electrode input
	adsWriteRegister(ADS1292_REG_CH2SET, 0x10); // Unit gain, normal electrode input

	adsWriteRegister(ADS1292_REG_RLDSENS, 0x00); // RLD settings defaults
	adsWriteRegister(ADS1292_REG_LOFFSENS, 0x00); // LOFF settings defaults
	adsWriteRegister(ADS1292_REG_RESP1, 0x02); // respiration defaults
	adsWriteRegister(ADS1292_REG_RESP2, 0x03); // respiration defaults
}

//_sampling_rate (Hz) needs to be either 16kHz | 8kHz | 4kHz | 2kHz | 1kHz | 500Hz | 250Hz
void adsSetSamplingRate(uint16_t _sampling_rate){
	//value to right in register: 0 - 16kHz || 1 - 8kHz || 2 - 4kHz || 3 - 2kHz || 4 - 1kHz || 5 - 500Hz || 6 - 250Hz
	adsWriteRegister(ADS1292_REG_CONFIG1, (uint8_t)(7-log2(_sampling_rate/125)));
}

void adsConfigureChannels(uint8_t ads_channel_mask){
	switch (ads_channel_mask){
		case 0: // no ADS channels
		case 1: // CH1
			adsWriteRegister(ADS1292_REG_CH1SET, 0x10); // CH1 enabled, gain 1, connected to electrode in
			adsWriteRegister(ADS1292_REG_CH2SET, 0x80); // CH2 disabled
			break;
		case 2: // CH2
			adsWriteRegister(ADS1292_REG_CH1SET, 0x80); // CH1 disabled
			adsWriteRegister(ADS1292_REG_CH2SET, 0x10); // CH2 enabled, gain 1, connected to electrode in
			break;
		case 3: // CH1 + CH2
			adsWriteRegister(ADS1292_REG_CH1SET, 0x10); // CH1 enabled, gain 1, connected to electrode in
			adsWriteRegister(ADS1292_REG_CH2SET, 0x10); // CH2 enabled, gain 1, connected to electrode in
			break;
	}
}

void adsStart(){
	adsSendCmd(RDATAC);
	adsSendCmd(START);
}

void adsStop(){
	adsSendCmd(STOP);
}
