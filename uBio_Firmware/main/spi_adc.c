#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <string.h>
#include <math.h>
#include "spi_master_nodma.h"
#include "spi_adc.h"


static spi_nodma_device_handle_t adsHandler;


void spiConfig() {
	gpio_set_direction(CS_GPIO, GPIO_MODE_OUTPUT);
	
	// initialize non-SPI GPIOs
	gpio_set_direction(DRDY_GPIO, GPIO_MODE_INPUT);
	gpio_set_direction(PWDN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(START_GPIO, GPIO_MODE_OUTPUT);

    // spi_bus_config_t spi_bus_config = {
	spi_nodma_bus_config_t spi_bus_config = {
        .miso_io_num = MISO_GPIO,
        .mosi_io_num = MOSI_GPIO,
        .sclk_io_num = SCLK_GPIO,
        .quadwp_io_num = -1, // not used
        .quadhd_io_num = -1, // not used
    };

    // spi_device_interface_config_t spi_device_config = {0};
    spi_nodma_device_interface_config_t spi_device_config = {0};
    spi_device_config.mode = 1; // SPI mode 1 : CPOL = 0, CPHA = 1
    spi_device_config.spics_io_num = -1;
    spi_device_config.spics_ext_io_num = CS_GPIO;
    spi_device_config.clock_speed_hz = 1000000; // clock out at 1MHz
    spi_device_config.queue_size = 1;

	// initialize the SPI bus and attach the 24-bit ADC to it
	spi_nodma_bus_add_device(VSPI_HOST, &spi_bus_config, &spi_device_config, &adsHandler);

	printf(">> OK: SPI configured\n");
}


void adsSendCmd(uint8_t cmd) {
	spi_nodma_transaction_t transaction;

	spi_nodma_device_select(adsHandler, 0);

    memset(&transaction, 0, sizeof(transaction)); // zero out the transaction
    transaction.flags = SPI_TRANS_USE_TXDATA;
    transaction.length = 8; 
    transaction.tx_data[0] = cmd;
    transaction.rx_buffer = NULL; // skip read phase
    spi_nodma_transfer_data(adsHandler, &transaction);

    spi_nodma_device_deselect(adsHandler);
    
    vTaskDelay(10/portTICK_PERIOD_MS);
}


void adsReadRegister(uint8_t address) {
	uint8_t first_opcode = address | RREG;
	uint8_t second_opcode = 0x00;

	uint8_t rx_data[0];

	spi_nodma_transaction_t transaction;
   
   	spi_nodma_device_select(adsHandler, 0);
   	
   	memset(&transaction, 0, sizeof(transaction)); // zero out the transaction
   	transaction.flags = SPI_TRANS_USE_TXDATA;
   	transaction.length = 16;
    transaction.tx_data[0] = first_opcode;
    transaction.tx_data[1] = second_opcode;
    transaction.rx_buffer = NULL; // skip read phase
    spi_nodma_transfer_data(adsHandler, &transaction);

   	memset(&transaction, 0, sizeof(transaction)); // zero out the transaction
    transaction.length = 0; // send nothing
    transaction.rxlength = 8;
    transaction.tx_buffer = NULL; // skip write phase
    transaction.rx_buffer = rx_data;
    spi_nodma_transfer_data(adsHandler, &transaction);
    
    spi_nodma_device_deselect(adsHandler);

    vTaskDelay(10/portTICK_PERIOD_MS);

	printf("%d\n", rx_data[0]);
}


void adsWriteRegister(uint8_t address, uint8_t data) {
	switch (address) {
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
  
	uint8_t first_opcode = address | WREG;
	uint8_t second_opcode = 0x00;

	spi_nodma_transaction_t transaction;
   
   	spi_nodma_device_select(adsHandler, 0);

	memset(&transaction, 0, sizeof(transaction)); // zero out the transaction
   	transaction.flags = SPI_TRANS_USE_TXDATA;
   	transaction.length = 24;
    transaction.tx_data[0] = first_opcode;
    transaction.tx_data[1] = second_opcode;
    transaction.tx_data[2] = data;
    transaction.rx_buffer = NULL; // skip read phase
    spi_nodma_transfer_data(adsHandler, &transaction);

 	spi_nodma_device_deselect(adsHandler);

	vTaskDelay(10/portTICK_PERIOD_MS);
}


void IRAM_ATTR adsReadDataContinuous(uint8_t *rx_data) {
	spi_nodma_transaction_t transaction;

	spi_nodma_device_select(adsHandler, 0);

    memset(&transaction, 0, sizeof(transaction)); // zero out the transaction
    transaction.length = 0; // send nothing
    transaction.rxlength = 72;
    transaction.tx_buffer = NULL; // skip write phase
    transaction.rx_buffer = rx_data;
    spi_nodma_transfer_data(adsHandler, &transaction);

    spi_nodma_device_deselect(adsHandler);
}


void adsReset() {
 	gpio_set_level(PWDN_GPIO, 1);
  	vTaskDelay(100/portTICK_PERIOD_MS);
  	gpio_set_level(PWDN_GPIO, 0);
  	vTaskDelay(100/portTICK_PERIOD_MS);
  	gpio_set_level(PWDN_GPIO, 1);
	vTaskDelay(100/portTICK_PERIOD_MS);
}

void adsDisableStart() {
	gpio_set_level(START_GPIO, 0);
	vTaskDelay(20/portTICK_PERIOD_MS);
}


void adsEnableStart() {
	gpio_set_level(START_GPIO, 1);
	vTaskDelay(20/portTICK_PERIOD_MS);
}


void adsHardStop() {
	gpio_set_level(START_GPIO, 0);
	vTaskDelay(100/portTICK_PERIOD_MS);
}


void adsStartDavaConversion() {
	adsSendCmd(START);
}


void adsSoftStop() {
	adsSendCmd(STOP);
}


void adsStartReadDataContinuous() {
	adsSendCmd(RDATAC);
}


void adsStopReadDataContinuous() {
    adsSendCmd(SDATAC);
}


void adsSetupRoutine() {
	adsReset();
    vTaskDelay(100/portTICK_PERIOD_MS);

	adsStopReadDataContinuous();
	vTaskDelay(100/portTICK_PERIOD_MS);

	// adsReadRegister(ADS1292_REG_ID);
	// adsReadRegister(ADS1292_REG_CONFIG1);
	// adsReadRegister(ADS1292_REG_CONFIG2);

	adsWriteRegister(ADS1292_REG_CONFIG2, 0x80); // disable reference buffer
	adsWriteRegister(ADS1292_REG_LOFF, 0x10); // lead-off defaults

	adsWriteRegister(ADS1292_REG_RLDSENS, 0x00); // RLD settings defaults
	adsWriteRegister(ADS1292_REG_LOFFSENS, 0x00); // LOFF settings defaults
	adsWriteRegister(ADS1292_REG_RESP1, 0x02); // respiration defaults
	adsWriteRegister(ADS1292_REG_RESP2, 0x03); // respiration defaults
}


void adsConfigureChannels(uint8_t ads_channel_mask) {
	switch (ads_channel_mask) {
		case 0: // no ADS channels - at least 1 channel must be enabled to generate DRDY interrupt
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


void adsSetSamplingRate(uint16_t sampling_rate) {
	adsWriteRegister(ADS1292_REG_CONFIG1, (int)log2(sampling_rate/125));
}


void adsStart() {
	adsStartReadDataContinuous();
	adsEnableStart();
} 


void adsStop() {
	adsHardStop();
} 
