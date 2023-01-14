#ifndef _SPI_H
#define _SPI_H

#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "config.h"

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
	#define SPI_MODE	0			//0 or 3, MCP Only supports these two modes
	#define ADC_EXT_SLCK_HZ (APB_CLK_FREQ / 800)

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

void adcExtInit(void);
void adcExtStart(void);
void adcExtStop(void);

#if _ADC_EXT_ == ADC_ADS
#define ADS_CONTINUOUS_TRANS 1

void adsSetSamplingRate(uint16_t sampling_rate);
void adsSetupRoutine();
void adsConfigureChannels(uint8_t ads_channel_mask);
void adsStart();
void adsStop();
void IRAM_ATTR adsEndTransCb(spi_transaction_t* trans);
void adsSendCmd(uint8_t cmd);

#elif _ADC_EXT_ == ADC_MCP
void mcpSetupRoutine(uint8_t channel_mask);
uint32_t IRAM_ATTR mcpReadRegister(uint8_t address, uint8_t rx_data_bytes);
void mcpStart();
void mcpStop();
void decodeSample(int32_t sample);
#endif

#endif