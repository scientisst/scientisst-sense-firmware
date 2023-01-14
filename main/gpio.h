#ifndef _GPIO_H
#define _GPIO_H

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/dac.h"
#include "config.h"

#define MAX32664_MFIO_IO    GPIO_NUM_15
#define MAX32664_RSTN_IO    GPIO_NUM_4

#define STATE_LED_R_IO      GPIO_NUM_13
#define STATE_LED_B_IO      GPIO_NUM_22
#define STATE_LED_G_IO      GPIO_NUM_17

#define BAT_LED_STATUS_IO   GPIO_NUM_21
#define O0_IO               GPIO_NUM_0      
#define O1_IO               GPIO_NUM_2
#define I0_IO               GPIO_NUM_15
#define I1_IO               GPIO_NUM_14
#define CONFIG_BTN_IO       I1_IO

#define SPI3_MISO_IO    GPIO_NUM_19
#define SPI3_MOSI_IO    GPIO_NUM_23
#define SPI3_SCLK_IO    GPIO_NUM_18
#define SPI3_CS0_IO     GPIO_NUM_5
#define SPI3_CS1_IO     GPIO_NUM_4
#if _ADC_EXT_ == ADC_MCP
#define MCP_DRDY_IO     GPIO_NUM_16
#elif _ADC_EXT_ == ADC_ADS
#define ADS_DRDY_IO     GPIO_NUM_16
#endif

#define LEDC_LS_TIMER           LEDC_TIMER_1            //Low speed timer
#define LEDC_SPEED_MODE_USED    LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_R          LEDC_CHANNEL_0
#define LEDC_CHANNEL_G          LEDC_CHANNEL_1
#define LEDC_CHANNEL_B          LEDC_CHANNEL_2
#define LEDC_LIVE_DUTY          512                     //Half the LEDC_TIMER_10_BIT duty_resolution resolution
#define LEDC_IDLE_DUTY          50                     //10% the LEDC_TIMER_10_BIT duty_resolution resolution
#define LEDC_LIVE_PWM_FREQ      4
#define LEDC_IDLE_PWM_FREQ      1

#define DAC_CH   DAC_CHANNEL_1

#define ABAT_DIVIDER_FACTOR 2

void gpioConfig(gpio_mode_t mode, gpio_int_type_t intr_type, uint64_t pin_bit_mask, gpio_pulldown_t pull_down_en, gpio_pullup_t pull_up_en);
void gpioInit();
void configLedC();

#if _ADC_EXT_ != NO_ADC_EXT
bool IRAM_ATTR gpioDrdyIsrHandler();
void adcExtDrdyGpio(int io_num);
#endif

#endif