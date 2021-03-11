#include <esp_system.h>
#include <string.h>
#include "tcp.h"
#include "adc.h"
#include "gpio.h"
#include "spi_adc.h"
#include "timer.h"

extern int clientSocket;

uint8_t crc_table[16] = {0, 3, 6, 5, 12, 15, 10, 9, 11, 8, 13, 14, 7, 4, 1, 2};

static const uint16_t ecg[50] = {440, 440, 440, 443, 456, 468, 477, 480, 472, 451, 440, 440, 440, 440, 437, 415, 579, 703, 484, 392, 439, 440, 440, 440, 440, 440, 440, 440, 440, 446, 456, 466, 474, 482, 488, 494, 497, 499, 499, 493, 482, 467, 448, 440, 440, 441, 450, 450, 441, 440};

static const uint16_t random_signal[50] = {437, 977, 741, 594, 553, 722, 5, 801, 948, 8, 844, 785, 1020, 233, 941, 657, 108, 274, 781, 824, 107, 481, 224, 944, 328, 877, 266, 898, 193, 777, 32, 657, 580, 385, 217, 810, 149, 500, 13, 191, 496, 858, 144, 749, 707, 35, 500, 994, 115, 760};

static const uint16_t sin50Hz[20] =  {511, 669, 811, 924, 997, 1022, 997, 924, 811, 669, 511, 353, 211, 98, 25, 0, 25, 98, 211, 353};

static const uint16_t sin40Hz[25] = {511, 638, 757, 861, 942, 997, 1021, 1013, 973, 905, 811, 699, 575, 447, 323, 211, 117, 49, 9, 1, 25, 80, 161, 265, 384};

static const uint8_t sin20Hz[50] =  {31, 35, 39, 42, 46, 49, 52, 55, 57, 59, 60, 61, 62, 62, 61, 60, 59, 57, 55, 52, 49, 46, 42, 39, 35, 31, 27, 23, 20, 16, 13, 10, 7, 5, 3, 2, 1, 0, 0, 1, 2, 3, 5, 7, 10, 13, 16, 20, 23, 27};

static const uint8_t sin10Hz[100] =  {31, 33, 35, 37, 39, 41, 42, 44, 46, 48, 49, 51, 52, 54, 55, 56, 57, 58, 59, 60, 60, 61, 61, 62, 62, 62, 62, 62, 61, 61, 60, 60, 59, 58, 57, 56, 55, 54, 52, 51, 49, 48, 46, 44, 42, 41, 39, 37, 35, 33, 31, 29, 27, 25, 23, 21, 20, 18, 16, 14, 13, 11, 10, 8, 7, 6, 5, 4, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 5, 6, 7, 8, 10, 11, 13, 14, 16, 18, 20, 21, 23, 25, 27, 29};


uint8_t calculateByteCRC(uint8_t crc, uint8_t byte_value) {
	crc = crc_table[crc] ^ (byte_value >> 4);
	crc = crc_table[crc] ^ (byte_value & 0x0F);
	return crc;
}


uint16_t calculateWordCRC(uint8_t crc, uint16_t word_value) {
	crc = calculateByteCRC(crc, word_value);
	crc = calculateByteCRC(crc, word_value >> 8);
	return crc;
}


void sendVersion() {
	const char *versionStr = "microbio_v0.1\n";
	socketWrite(clientSocket, (uint8_t *)versionStr, strlen(versionStr));
}
	

void sendStatus() {
	int sample = 0;
	uint8_t crc = 0;
	
	uint8_t no_bytes = 16;
	uint8_t *frame = (uint8_t *)malloc(no_bytes * sizeof(uint8_t));
	memset(frame, 0, no_bytes);

	// ADC values for A1-A6 input channels
	extern uint8_t map_analog_channels[6];
	for (uint8_t i = 0; i < 6; i++) {
		sample = adcRead(map_analog_channels[i]);
		*(uint16_t*)(frame+(i*2)) |= sample;
	  	crc = calculateWordCRC(crc, sample);
	}

	// ADC value fot ABAT input channel
	extern uint8_t channel_table[6]; // !! to be REPLACED, delete adterwards
	sample = adcRead(channel_table[0]); // !! REPLACE channel_table[0] with the ADC pin for ABAT input !!
	*(uint16_t*)(frame+12) |= sample;
  	crc = calculateWordCRC(crc, sample);

	// send battery threshold and calculate CRC
	extern uint16_t bat_threshold;
	uint8_t data = bat_threshold - BAT_THRESHOLD_BASE;
	frame[14] = data;
	crc = calculateByteCRC(crc, data);

	// send digital ports and calculate and send CRC
	data = 0;
	if (isI1high())
		data |= 0x08;
 	if (isI2high())
 		data |= 0x04;
 	if (isO1high())
 		data |= 0x02;
 	if (isO2high())
 		data |= 0x01;
	crc = crc_table[crc] ^ data;
	data = (data << 4) | crc_table[crc];
	frame[15] = data;

  	socketWrite(clientSocket, frame, no_bytes);
}


uint8_t calculateSeqNoCRC(uint8_t *frame) {
	extern uint8_t sequence_no;
	extern uint8_t start_position;
	
	// calculate CRC (except last byte (sequence # + CRC))
	uint8_t crc = 0;
	for (uint8_t i = start_position; i < 7; i++) {
		crc = calculateByteCRC(crc, frame[i]);
	}

	// calculate CRC for last byte (sequence # + CRC)
	crc = crc_table[crc] ^ (sequence_no & 0x0F);
	return ((sequence_no << 4) | crc_table[crc]);
}


uint16_t simulateData(uint8_t channel_no, uint8_t s50, uint8_t s25, uint8_t s20) {
	extern uint8_t channel_table[6];
	extern uint8_t simulated_seq;

	switch(channel_table[channel_no]) {
		case 0:
			return ecg[s50];
		 
		case 1:
			return random_signal[s50];
		 
		case 2:
			return sin50Hz[s20];
		 
		case 3:
			return sin40Hz[s25];
		 
		case 4:
			return sin20Hz[s50];
		 
		case 5:
			return sin10Hz[simulated_seq];
	}

	return 0; // just to avoid a warning
}