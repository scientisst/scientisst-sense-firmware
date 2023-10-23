/**
 * \file sci_gpio.h
 * \brief Header file for sci_gpio.c
 *
 * This file includes all MACROs relative to the ScientISST board's GPIOs.
 */

#pragma once

#include <stdio.h>

#include "driver/dac.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "sci_scientisst.h"

// LEDs
#ifdef CONFIG_HARDWARE_VERSION_CORE
#define STATE_LED_R_IO GPIO_NUM_22
#define STATE_LED_B_IO GPIO_NUM_13
#define STATE_LED_G_IO GPIO_NUM_17
#endif
#ifdef CONFIG_HARDWARE_VERSION_NANO
#define STATE_LED_R_IO GPIO_NUM_13
#define STATE_LED_B_IO GPIO_NUM_22
#define STATE_LED_G_IO GPIO_NUM_10
#endif
#ifdef CONFIG_HARDWARE_VERSION_CARDIO
#define STATE_LED_R_IO GPIO_NUM_22
#define STATE_LED_B_IO GPIO_NUM_13
#define STATE_LED_G_IO GPIO_NUM_17
#endif

#define BAT_LED_STATUS_IO GPIO_NUM_21

// Digital channels
#define O0_IO GPIO_NUM_0
#define O1_IO GPIO_NUM_2
#define I0_IO GPIO_NUM_15
#define I1_IO GPIO_NUM_14

// Buttons
#define CONFIG_BTN_IO I1_IO

// MCP - SPI
#define MCP_MISO GPIO_NUM_19
#define MCP_MOSI GPIO_NUM_23
#define MCP_SCLK GPIO_NUM_18
#define MCP_CS GPIO_NUM_5
#ifdef CONFIG_HARDWARE_VERSION_CORE
#define MCP_DRDY_IO GPIO_NUM_16
#endif
#ifdef CONFIG_HARDWARE_VERSION_CARDIO
#define MCP_DRDY_IO GPIO_NUM_16
#endif
#ifdef CONFIG_HARDWARE_VERSION_NANO
#define MCP_DRDY_IO GPIO_NUM_9
#endif
// SD Card - SPI
#define SD_CARD_MISO GPIO_NUM_19
#define SD_CARD_MOSI GPIO_NUM_23
#define SD_CARD_CLK GPIO_NUM_18
#define SD_CARD_CS GPIO_NUM_4

// I2C
#define SDA_IO GPIO_NUM_26
#define SCL_IO GPIO_NUM_27

// IMU
#define IMU_SDA_PIN SDA_IO
#define IMU_SCL_PIN SCL_IO

// Timers
#define LEDC_LS_TIMER LEDC_TIMER_1 // Low speed timer
#define LEDC_SPEED_MODE_USED LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_R LEDC_CHANNEL_0
#define LEDC_CHANNEL_G LEDC_CHANNEL_1
#define LEDC_CHANNEL_B LEDC_CHANNEL_2
#define LEDC_LIVE_DUTY 512 // Half the LEDC_TIMER_10_BIT duty_resolution resolution
#define LEDC_IDLE_DUTY 921 // 90% the LEDC_TIMER_10_BIT duty_resolution resolution
#define LEDC_LIVE_PWM_FREQ 4
#define LEDC_IDLE_PWM_FREQ 1

// ADC2
#define DAC_CH DAC_CHANNEL_1 // GPIO25

void gpioInit(void);
void configLedController(void);

// ADC EXT
void gpioDrdyIsrHandler(void);
void adcExtDrdyGpio(int io_num);
