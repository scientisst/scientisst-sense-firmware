#ifndef MAIN_SPI_ADC_H_
#define MAIN_SPI_ADC_H_

#define CS_GPIO 	GPIO_NUM_5
#define MISO_GPIO   GPIO_NUM_19
#define MOSI_GPIO   GPIO_NUM_23
#define SCLK_GPIO   GPIO_NUM_18

#define PWDN_GPIO	GPIO_NUM_22
#define START_GPIO	GPIO_NUM_21
#define DRDY_GPIO	GPIO_NUM_17

// read n nnnn registers starting at address r rrrr
// first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define RREG	0x20;

// write n nnnn registers starting at address r rrrr
//first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)
#define WREG 	0x40; 

#define START	0x08 // start/restart (synchronize) conversions
#define STOP	0x0A // stop conversion
#define RDATAC  0x10 // enable Read Data Continuous mode - the default at power-up
#define SDATAC	0x11 // stop Read Data Continuously mode

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

void spiConfig();
void adsReadDataContinuous(uint8_t *rx_data);
void adsSetupRoutine();
void adsConfigureChannels(uint8_t ads_channel_mask);
void adsSetSamplingRate(uint16_t sampling_rate);
void adsStart();
void adsStop();

#endif /* MAIN_SPI_ADC_H_ */
