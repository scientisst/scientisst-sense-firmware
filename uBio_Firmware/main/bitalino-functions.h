#ifndef MAIN_BITALINO_FUNCTIONS_H_
#define MAIN_BITALINO_FUNCTIONS_H_

uint8_t calculateByteCRC(uint8_t crc, uint8_t byte_value);
uint16_t calculateWordCRC(uint8_t crc, uint16_t word_value);
void sendVersion();
void sendStatus();
uint8_t calculateSeqNoCRC(uint8_t *frame);
uint16_t simulateData(uint8_t channel_no, uint8_t s50, uint8_t s25, uint8_t s20);

#endif /* MAIN_BITALINO_FUNCTIONS_H_ */
