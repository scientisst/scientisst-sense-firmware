#ifndef _COM_H
#define _COM_H

#include "sdkconfig.h"
#include <stdio.h>

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
#define NUM_BUFFERS 4
#define MAX_BUFFER_SIZE (ESP_SPP_MAX_MTU) // If changed, change in API
// #define MAX_BUFFER_SIZE CONFIG_LWIP_TCP_SND_BUF_DEFAULT

typedef struct {
    uint8_t api_mode;
    void (*aquire_func)();
    void (*select_ch_mask_func)();
} Api_Config;

#define ADC1_CFG_IDX 0
#define ADC2_CFG_IDX 1

#define GET_NEXT_POS(curr_pos, dist)                                                                                                                                                                   \
    ({                                                                                                                                                                                                 \
        curr_pos.start_byte += (curr_pos.start_bit + dist) / 8;                                                                                                                                        \
        curr_pos.start_bit = (curr_pos.start_bit + dist) % 8;                                                                                                                                          \
    })

void processRcv2(uint8_t *buff, int buff_size);
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

/*          MASK CONFIGURATION EXAMPLE
 *                       bits
 *           7   6   5   4   3   2   1   0
 *           A6  A5  A4  A3  A2  A1  X   X
 *           <--------------------><------>
 *           DEFAULT_ADC_CHANNELS
 *                        NUM_UNUSED_BITS_FOR_CH_MASK
 */

#endif