#ifndef MAIN_GPIO_H_
#define MAIN_GPIO_H_

void ledBatConfig();
void setLedBat();
void clearLedBat();
void inputConfig();
void outputConfig();
void setOutputsLevel(uint8_t dataBuffer);
int isI1high();
int isI2high();
int isO1high();
int isO2high();
int isLedBatHigh();

#endif /* MAIN_GPIO_H_ */
