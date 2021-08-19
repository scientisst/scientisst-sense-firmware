#include "spi.h"
#include "adc.h"
#include "macros.h"
#include "com.h"
#include "gpio.h"
#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include <math.h>

#define MAX_TRANSFER_SIZE 0

#if _ADC_EXT_ == ADC_ADS
	#define DMA_CHAN        2
	#define SPI_MODE		1		
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

#elif _ADC_EXT_ == ADC_MCP
	#define DMA_CHAN	0
	#define SPI_MODE	2			//0 or 2, MCP Only supports these two modes
	#define ADC_EXT_SLCK_HZ 1*1000*1000           //Clock out at 1 MHz, divisors of 80MHz

	#define MCP_ADDR	0b01

	//Commands
	#define RREG 0b01
	#define WREG 0b10
	#define RESET 0b111000
	#define START 0b101000
	#define FSHUTDOWN 0b110100				//Full Shutdown; the internal config registers keep the values

	//Register Addresses
	#define REG_ADCDATA 	0x00	//R
	#define REG_CONFIG0 	0x01	//R/W
	#define REG_CONFIG1 	0x02	//R/W
	#define REG_CONFIG2 	0x03	//R/W
	#define REG_CONFIG3 	0x04	//R/W
	#define REG_IRQ			0x05	//R/W
	#define REG_MUX 		0x06	//R/W
	#define REG_SCAN 		0x07	//R/W
	#define REG_TIMER 		0x08	//R/W
	#define REG_OFFSETCAL 	0x09	//R/W
	#define REG_GAINCAL 	0x0A	//R/W
	#define REG_LOCK		0x0D	//R/W
	#define REG_CRCCFG		0X0F	//R

#endif

void adcExtInit(uint32_t cs_pin){
    esp_err_t ret;

    spi_bus_config_t buscfg={
        .miso_io_num = SPI3_MISO_IO,
        .mosi_io_num = SPI3_MOSI_IO,
        .sclk_io_num = SPI3_SCLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_TRANSFER_SIZE,
		.intr_flags = ESP_INTR_FLAG_IRAM
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz = ADC_EXT_SLCK_HZ,
        .mode = SPI_MODE,                         //SPI mode 1: (CPOL) = 0 and the clock phase (CPHA) = 1. 
        .spics_io_num = cs_pin,              //CS pin
        .queue_size = 1                          //We want to be able to queue 1 transactions at a time
        //.pre_cb = lcd_spi_pre_transfer_callback  //Specify pre-transfer callback to handle D/C line
		//.post_cb = adsEndTransCb
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(SPI3_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    //Attach the device to the SPI bus
    ret=spi_bus_add_device(SPI3_HOST, &devcfg, &adc_ext_spi_handler);
    ESP_ERROR_CHECK(ret);
}

void adcExtStart(void){
	#if (_ADC_EXT_ == ADC_MCP)
		mcpStart();
	#elif (_ADC_EXT_ == ADC_ADS)
		adsStart();
	#endif
}

void adcExtStop(void){
	#if (_ADC_EXT_ == ADC_MCP)
		mcpStop();
	#elif (_ADC_EXT_ == ADC_ADS)
		adsStop();
	#endif
}

#if (_ADC_EXT_ == ADC_MCP)

void mcpSendCmd(uint8_t cmd){
	spi_transaction_t transaction;
	uint8_t first_opcode = 0;

	first_opcode = (uint8_t)MCP_ADDR << 6;
	first_opcode |= cmd;

    memset(&transaction, 0, sizeof(transaction)); // zero out the transaction
    transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    transaction.length = 8; 
	transaction.rxlength = 8;
    transaction.tx_data[0] = first_opcode;

	vTaskDelay(10/portTICK_PERIOD_MS);
	spi_device_polling_transmit(adc_ext_spi_handler, &transaction);

	#if (_DEBUG_ > 0)
	printf("STATUS recieved for mcpSendCmd(): %u \n", transaction.rx_data[0]);
	#endif
}

void mcpWriteRegister(uint8_t address, uint32_t tx_data, uint8_t tx_data_bytes){
	spi_transaction_t transaction;
	uint8_t first_opcode = 0;

	first_opcode = (uint8_t)MCP_ADDR << 6;
	first_opcode |= address << 2;
	first_opcode |= WREG;
	
	if(tx_data_bytes > 1){
		tx_data = SPI_SWAP_DATA_TX((tx_data), (tx_data_bytes*8));
	}
   
	memset(&transaction, 0, sizeof(transaction)); // zero out the transaction
   	transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
   	transaction.length = (tx_data_bytes+1)*8;				// length is MAX(in_bits, out_bits)
	transaction.rxlength = 8;								//Recieve status byte
    transaction.tx_data[0] = first_opcode;
	*(uint32_t*)(transaction.tx_data) |= tx_data << 8;
	printf("Register write: %u %u %u %u\n", transaction.tx_data[0], transaction.tx_data[1], transaction.tx_data[2], transaction.tx_data[3]);

	vTaskDelay(10/portTICK_PERIOD_MS);
    spi_device_polling_transmit(adc_ext_spi_handler, &transaction);  //Transmit!

	#if (_DEBUG_ > 0)
	printf("STATUS recieved for mcpWriteRegister(): %u \n", transaction.rx_data[0]);
	#endif
}

uint32_t IRAM_ATTR mcpReadRegister(uint8_t address, uint8_t rx_data_bytes){
	spi_transaction_t transaction;
	uint8_t first_opcode;

	first_opcode = MCP_ADDR << 6;
	first_opcode |= address << 2;
	first_opcode |= RREG;
   
	memset(&transaction, 0, sizeof(transaction)); // zero out the transaction
   	transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
   	transaction.length = rx_data_bytes*8;				// length is MAX(in_bits, out_bits)
	transaction.rxlength = rx_data_bytes*8;								//Recieve status byte
    transaction.tx_data[0] = first_opcode;
	vTaskDelay(10/portTICK_PERIOD_MS);
    spi_device_polling_transmit(adc_ext_spi_handler, &transaction);  //Transmit!

	if(rx_data_bytes > 1){
		*(uint32_t*)(transaction.rx_data) = SPI_SWAP_DATA_RX((*(uint32_t*)(transaction.rx_data)), (rx_data_bytes*8));
	}

	//Check response
	#if (_DEBUG_ > 0)
		printf("Register read: %u %u %u %u\n", transaction.rx_data[0], transaction.rx_data[1], transaction.rx_data[2], transaction.rx_data[3]);
	#endif

	return (*(uint32_t*)(transaction.rx_data));
}

void mcpSetupRoutine(void){
	mcpSendCmd(RESET);

	//Config Drdy GPIO and interrupt
	mcpConfigDrdyGpio();

	mcpWriteRegister(REG_CONFIG0, (0b01 << 6) | (0b10 << 4) | (0b00 << 2) | (0b00), 1);
	mcpWriteRegister(REG_CONFIG1, 0, 1);
	mcpWriteRegister(REG_CONFIG2, (0b10 << 6) | (0b001 << 3) | (0b0 << 2) | (0b11), 1);
	mcpWriteRegister(REG_CONFIG3, (0b11 << 6) | (0b00 << 4) | (0b0 << 3) | (0b0 << 2) | (0b0 << 1) | (0b0), 1);
	mcpWriteRegister(REG_IRQ, (0b0 << 7) | (0b111 << 4) | (0b01 << 2) | (0b1 << 1) | 0b0, 1);
	mcpWriteRegister(REG_MUX, 0, 1);																				
	mcpWriteRegister(REG_SCAN, (0b000 << 21) | (0b00000 << 16) | 0b1, 3);		//Delay time between conversions of each channel & Channels selection
	mcpWriteRegister(REG_TIMER, 1000, 3);										//(Delay) number of DMCLK periods between SCAN cycles (aka sample rate)
	mcpWriteRegister(REG_OFFSETCAL, 0, 3);
	mcpWriteRegister(REG_GAINCAL, 0, 3);
}


void mcpStart(){
	mcpSendCmd(START);
}

void mcpStop(){
	mcpSendCmd(FSHUTDOWN);
}

#elif (_ADC_EXT_ == ADC_ADS)
void adsSendCmd(uint8_t cmd){
	spi_transaction_t transaction;

    memset(&transaction, 0, sizeof(transaction)); // zero out the transaction
    transaction.flags = SPI_TRANS_USE_TXDATA;
    transaction.length = 8; 
    transaction.tx_data[0] = cmd;
    transaction.rx_buffer = NULL; // skip read phase
	spi_device_polling_transmit(adc_ext_spi_handler, &transaction);
    
    vTaskDelay(10/portTICK_PERIOD_MS);
}

void adsWriteRegister(uint8_t address, uint8_t data){
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
    spi_device_polling_transmit(adc_ext_spi_handler, &transaction);  //Transmit!

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
	if(_sampling_rate != 16000 && _sampling_rate != 8000 && _sampling_rate != 4000 && _sampling_rate != 2000 && _sampling_rate != 1000 && _sampling_rate != 500 && _sampling_rate != 250){
		DEBUG_PRINT_E("adsSetSamplingRate", "ADS sample rate not valid. Defaulting to 16kHz");
		_sampling_rate = 16000;
	}
	//value to right in register: 0 - 16kHz || 1 - 8kHz || 2 - 4kHz || 3 - 2kHz || 4 - 1kHz || 5 - 500Hz || 6 - 250Hz
	adsWriteRegister(ADS1292_REG_CONFIG1, (uint8_t)(7-log2(_sampling_rate/125)));
	vTaskDelay(10/portTICK_PERIOD_MS);
}

//Configure ads with the channels chosen. Configure SPI transaction in function of the CHs chosen
void adsConfigureChannels(uint8_t ads_channel_mask){
	//24 status bits + 24 bits x 2 channels = 9bytes required. But DMA buffer needs to be 32bit aligned
    uint8_t *rx_data_ads = heap_caps_malloc(3*sizeof(uint32_t), MALLOC_CAP_DMA);
    memset(rx_data_ads, 0, 3*sizeof(uint32_t));

	memset(&adc_ext_trans, 0, sizeof(adc_ext_trans));   //zero out the transaction

	switch (ads_channel_mask){
		case 0: // no ADS channels
		case 1: // CH1
			adsWriteRegister(ADS1292_REG_CH1SET, 0x10); // CH1 enabled, gain 1, connected to electrode in
			adsWriteRegister(ADS1292_REG_CH2SET, 0x80); // CH2 disabled
			adc_ext_trans.length = 48;                  //length is MAX(sending length, recieving length)
    		adc_ext_trans.rxlength = 48;                //24 status bits + 24 bits x 1 channels
			break;
		case 2: // CH2
			adsWriteRegister(ADS1292_REG_CH1SET, 0x80); // CH1 disabled
			adsWriteRegister(ADS1292_REG_CH2SET, 0x10); // CH2 enabled, gain 1, connected to electrode in
			adc_ext_trans.length = 48;                  //length is MAX(sending length, recieving length)
    		adc_ext_trans.rxlength = 48;                //24 status bits + 24 bits x 1 channels
			break;
		case 3: // CH1 + CH2
			adsWriteRegister(ADS1292_REG_CH1SET, 0x10); // CH1 enabled, gain 1, connected to electrode in
			adsWriteRegister(ADS1292_REG_CH2SET, 0x10); // CH2 enabled, gain 1, connected to electrode in
			adc_ext_trans.length = 72;                  //length is MAX(sending length, recieving length)
    		adc_ext_trans.rxlength = 72;                //24 status bits + 24 bits x 2 channels
			break;
	}
    adc_ext_trans.tx_buffer = NULL;             //skip write phase
    adc_ext_trans.rx_buffer = rx_data_ads;
}

void adsStart(){
	adsSetSamplingRate(sample_rate);
	//Update ads continous transaction to indicate flag for post call back of spi transfer
	adc_ext_trans.user = (void*)ADS_CONTINUOUS_TRANS;
	adsSendCmd(RDATAC);
	adsSendCmd(START);
	//spi_device_queue_trans(adc_ext_spi_handler, &adc_ext_trans, portMAX_DELAY);
}

void adsStop(){
	//spi_transaction_t *ads_rtrans;

	//Update ads continous transaction to indicate flag for post call back of spi transfer
	adc_ext_trans.user = NULL;
	//Get the trans result of the last queued trans (by adsEndTransCb)
	//spi_device_get_trans_result(adc_ext_spi_handler, &ads_rtrans, portMAX_DELAY);
	adsSendCmd(STOP);
}


//Call back in the end of a transaction made in live mode (when ads is in continous mode)
void IRAM_ATTR adsEndTransCb(spi_transaction_t* trans){
	if((uint8_t)(trans->user) == ADS_CONTINUOUS_TRANS){
		spi_device_queue_trans(adc_ext_spi_handler, &adc_ext_trans, portMAX_DELAY);
	}
}

#endif