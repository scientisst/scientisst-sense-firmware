/** \file com.h
    \brief This file contains the definitions of the functions used to process
   the received data.
*/

#ifndef _COM_H
#define _COM_H

#include <stdio.h>

#include "sdkconfig.h"

#define DEFAULT_ADC_CHANNELS 6        // Default number of active adc channels
#define EXT_ADC_CHANNELS 2            // Num of external adc channels
#define NUM_UNUSED_BITS_FOR_CH_MASK 2 // It's the number explained below

#define STATUS_PACKET_SIZE 16

#define API_MODE_BITALINO 1
#define API_MODE_SCIENTISST 2
#define API_MODE_JSON 3

#define OP_MODE_IDLE 0
#define OP_MODE_LIVE 1
#define OP_MODE_CONFIG 2

#define CMD_MAX_BYTES 4
#define NUM_BUFFERS 50
#define MAX_BUFFER_SIZE (ESP_SPP_MAX_MTU) // If changed, change in API

typedef struct
{
    uint8_t api_mode;
    void (*select_ch_mask_func)();
} api_config_t;

extern int send_fd;

void processRcv(uint8_t *buff, int buff_size);
void selectChsFromMask(uint8_t *buff);
void stopAcquisition(void);
void sendStatusPacket();
void sendFirmwareVersionPacket();
void changeAPI(uint8_t mode);
void selectChsFromMaskScientisstJson(uint8_t *buff);
void setSampleRate(uint8_t *buff);
void triggerGpio(uint8_t *buff);
void triggerDAC(uint8_t *buff);
void startAcquisition(uint8_t *buff, uint8_t cmd);
uint8_t getPacketSize(void);

#endif
