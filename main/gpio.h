#ifndef _GPIO_H
#define _GPIO_H

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"


#define MAX32664_MFIO_IO    GPIO_NUM_15
#define MAX32664_RSTN_IO    GPIO_NUM_4
#define STATE_LED_IO        GPIO_NUM_14     //GPIO_NUM_27
#define O0_IO               GPIO_NUM_0
#define O1_IO               GPIO_NUM_2
#define I0_IO               GPIO_NUM_14
#define I1_IO               GPIO_NUM_15


#define LEDC_LS_TIMER           LEDC_TIMER_1            //Low speed timer
#define LEDC_SPEED_MODE_USED    LEDC_LOW_SPEED_MODE
#define LEDC_DUTY               512                     //Half the LEDC_TIMER_10_BIT duty_resolution resolution
#define LEDC_LIVE_PWM_FREQ      4
#define LEDC_IDLE_PWM_FREQ      1

void gpioConfig(gpio_mode_t mode, gpio_int_type_t intr_type, uint64_t pin_bit_mask, gpio_pulldown_t pull_down_en, gpio_pullup_t pull_up_en);
void gpioInit();

#endif