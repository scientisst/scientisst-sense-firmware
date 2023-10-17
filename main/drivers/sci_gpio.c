/** \file gpio.c
    \brief GPIO interactions

    This file contains the functions to initialize the GPIOs and the LEDC
   module.
*/

#include "sci_gpio.h"

#include <stdlib.h>

#include "sci_adc_internal.h"
#include "sci_macros.h"

/**
 * \brief Configure LED Controller
 *
 * This function configures the LED Controller. It prepares and sets the
 * configuration of timers.
 */
void configLedC(void)
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // resolution of PWM duty
        .freq_hz = LEDC_IDLE_PWM_FREQ,        // frequency of PWM signal
        .speed_mode = LEDC_SPEED_MODE_USED,   // timer mode
        .timer_num = LEDC_LS_TIMER,           // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    // Set configuration of timer0 for low speed channels
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {.channel = LEDC_CHANNEL_R,
                                          .duty = 0,
                                          .gpio_num = STATE_LED_R_IO,
                                          .speed_mode = LEDC_SPEED_MODE_USED,
                                          .hpoint = 0,
                                          .timer_sel = LEDC_LS_TIMER};
    // Set LED Controller with previously prepared configuration
    ledc_channel_config(&ledc_channel);

#ifndef CONFIG_HARDWARE_VERSION_CARDIO
    ledc_channel.channel = LEDC_CHANNEL_G;
    ledc_channel.gpio_num = STATE_LED_G_IO;
    ledc_channel_config(&ledc_channel);

    ledc_channel.channel = LEDC_CHANNEL_B;
    ledc_channel.gpio_num = STATE_LED_B_IO;
    ledc_channel_config(&ledc_channel);
#endif

    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R, LEDC_IDLE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R);
#ifndef CONFIG_HARDWARE_VERSION_CARDIO
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G, LEDC_IDLE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G);
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B, LEDC_IDLE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B);
#endif
}

/**
 * \brief Configures GPIO
 *
 * This function configures the GPIO pins. It sets the mode, interrupt type,
 * pull up/down and pin bit mask.
 */
void gpioConfig(gpio_mode_t mode, gpio_int_type_t intr_type, uint64_t pin_bit_mask, gpio_pulldown_t pull_down_en,
                gpio_pullup_t pull_up_en)
{
    gpio_config_t io_conf;

    if (pull_down_en && pull_up_en)
    {
        DEBUG_PRINT_E("gpioConfig", "Pin cannot have both pull up and pull down enabled");
        return;
    }

    // config pin direction (output/input)
    io_conf.mode = mode;
    // config interrupt (disable/what type of interrupt)
    io_conf.intr_type = intr_type;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = pin_bit_mask;
    // config pull-down mode (enable/disable)
    io_conf.pull_down_en = pull_down_en;
    // config pull-up mode (enable/disable)
    io_conf.pull_up_en = pull_up_en;

    // configure GPIO with the given settings
    gpio_config(&io_conf);
}

/**
 * \brief Initializes GPIO
 *
 */
void gpioInit(void)
{
#ifdef CONFIG_HARDWARE_VERSION_CARDIO
    gpioConfig(
        GPIO_MODE_OUTPUT, GPIO_PIN_INTR_DISABLE,
        ((1ULL << STATE_LED_R_IO) | (1ULL << BAT_LED_STATUS_IO) | (1ULL << O0_IO) | (1ULL << O1_IO) | (1ULL << SPI3_CS0_IO)),
        0, 0);
#else
    gpioConfig(GPIO_MODE_OUTPUT, GPIO_PIN_INTR_DISABLE,
               ((1ULL << STATE_LED_R_IO) | (1ULL << STATE_LED_G_IO) | (1ULL << STATE_LED_B_IO) |
                (1ULL << BAT_LED_STATUS_IO) | (1ULL << O0_IO) | (1ULL << O1_IO) | (1ULL << SPI3_CS0_IO)),
               0, 0);
#endif

    gpioConfig(GPIO_MODE_INPUT, GPIO_PIN_INTR_DISABLE, ((1ULL << I0_IO) | (1ULL << I1_IO)), 1,
               0); // The 2 IO inputs
}

// Function has to be declared with void* as argument even though it is not
// used to avoid compiler warnings
void IRAM_ATTR gpioDrdyIsrHandler(void *not_used)
{
    return;
}

/**
 * \brief Configures DRDY GPIO
 *
 * This function configures the DRDY GPIO pin. //TODO: Add details and when it
 * is used.
 */
void adcExtDrdyGpio(int io_num)
{
    // Config DRDY gpio
    gpioConfig(GPIO_MODE_INPUT, GPIO_INTR_NEGEDGE, (1ULL << io_num), 0, 0);

    // install gpio isr service
    gpio_install_isr_service(0);

    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(io_num, (gpio_isr_t)gpioDrdyIsrHandler, NULL);
}
