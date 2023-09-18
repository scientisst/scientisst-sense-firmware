/** \file spi.h
    \brief This file contains the definitions of the functions used to
   communicate with the external ADCs.

    //TODO: Add more info

*/
#ifndef _SPI_H
#define _SPI_H

#include <stdio.h>

#include "config.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "macros_conf.h"

#define DMA_CHAN 0
#define SPI_MODE 0 // 0 or 3, MCP Only supports these two modes
#define ADC_EXT_SLCK_HZ_1_EXT_CH (APB_CLK_FREQ / 8)
#define ADC_EXT_SLCK_HZ_2_EXT_CH (APB_CLK_FREQ / 64)

#define MCP_ADDR 0b01

// Commands
#define RREG 0b01
#define WREG 0b10
#define RESET 0b111000
#define START 0b101000
#define FSHUTDOWN 0b110100 // Full Shutdown; the internal config registers keep the values

// Register Addresses
#define REG_ADCDATA 0x00   // R
#define REG_CONFIG0 0x01   // R/W
#define REG_CONFIG1 0x02   // R/W
#define REG_CONFIG2 0x03   // R/W
#define REG_CONFIG3 0x04   // R/W
#define REG_IRQ 0x05       // R/W
#define REG_MUX 0x06       // R/W
#define REG_SCAN 0x07      // R/W
#define REG_TIMER 0x08     // R/W
#define REG_OFFSETCAL 0x09 // R/W
#define REG_GAINCAL 0x0A   // R/W
#define REG_LOCK 0x0D      // R/W
#define REG_CRCCFG 0X0F    // R

void adcExtInit(void);
void adcExtStart(void);
void adcExtStop(void);
void mcpSetupRoutine(uint8_t channel_mask);
void IRAM_ATTR mcpReadADCValues(uint8_t address, uint8_t rx_data_bytes);

#endif