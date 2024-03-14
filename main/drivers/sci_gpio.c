/**
 * \file gpio.c
 * \brief Management of GPIO interactions and LED controller configuration.
 *
 * This file contains functions responsible for initializing and managing GPIOs, including setting up the LED Controller
 * (LEDC) module. It handles the specifics of pin configuration, ISR handling, and LED control routines.
 */

#include "sci_gpio.h"

#include <stdlib.h>

#include "freertos/semphr.h"
#include "hal/gpio_types.h"

#include "sci_adc_internal.h"

SemaphoreHandle_t led_semaphore;

/**
 * \brief Updates the LED status code.
 *
 * \param[in] new_status The new status code to be set.
 *
 * \return None.
 */
void updateLEDStatusCode(sci_led_status_t new_status)
{
    DEBUG_PRINT_I("updateLEDStatusCode", "New status: %d", new_status);
    xSemaphoreTake(led_semaphore, portMAX_DELAY);

    switch (new_status)
    {
    case IDLE:
        ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_IDLE_PWM_FREQ);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R, LEDC_IDLE_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R);
#ifndef CONFIG_HARDWARE_VERSION_CARDIO
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G, LEDC_IDLE_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B, LEDC_IDLE_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B);
#endif
        break;
    case LIVE_AQUISITION:
        ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_LIVE_PWM_FREQ);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R, LEDC_LIVE_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R);
#ifndef CONFIG_HARDWARE_VERSION_CARDIO
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G, LEDC_LIVE_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B, LEDC_LIVE_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B);
#endif
        break;
#ifndef CONFIG_HARDWARE_VERSION_CARDIO
    case LIVE_AQUISITION_SDCARD:
        ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_LIVE_PWM_FREQ);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R, LEDC_OFF_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G, LEDC_LIVE_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B, LEDC_OFF_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B);
        break;
    case IDLE_SDCARD:
        ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_IDLE_PWM_FREQ);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R, LEDC_IDLE_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G, LEDC_IDLE_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B, LEDC_OFF_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B);
        break;
    case FORMATING_SDCARD:
        ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_IDLE_PWM_FREQ);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R, LEDC_OFF_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G, LEDC_FIXED_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B, LEDC_OFF_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B);
        break;
    case CALIBRATING_IMU:
        ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_IDLE_PWM_FREQ);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R, LEDC_FIXED_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G, LEDC_FIXED_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B, LEDC_OFF_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B);
        break;
    case WIFI_CONNECTING:
        ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_IDLE_PWM_FREQ);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R, LEDC_LIVE_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G, LEDC_FIXED_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B, LEDC_LIVE_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B);
        break;
    case WIFI_LOST_CONNECTION:
        ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_IDLE_PWM_FREQ);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R, LEDC_FIXED_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G, LEDC_LIVE_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G);
        ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B, LEDC_LIVE_DUTY);
        ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B);
        break;
#endif
    case CONFIG_MODE:
        gpio_set_level(STATE_LED_R_IO, 1);
#ifndef CONFIG_HARDWARE_VERSION_CARDIO
        gpio_set_level(STATE_LED_G_IO, 1);
        gpio_set_level(STATE_LED_B_IO, 1);
#endif
        break;
    default:
        break;
    }

    xSemaphoreGive(led_semaphore);
}

/**
 * \brief Configures the LED Controller (LEDC) timers and channels.
 *
 * Initializes the LEDC module to control the on-board LEDs. It sets up the timer and initializes the duty cycle  for the
 * LEDs. The function takes into consideration the hardware version to manage specific LEDs accordingly.
 *
 * \return None.
 */
void configLedController(void)
{
    led_semaphore = xSemaphoreCreateBinary();
    CHECK_NOT_NULL(led_semaphore);
    xSemaphoreGive(led_semaphore);
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

    updateLEDStatusCode(IDLE);
}

/**
 * \brief Configures individual GPIOs with specific settings.
 *
 * \param[in] mode Defines the direction (input/output) of the GPIO.
 * \param[in] intr_type Determines the type of interrupt that the GPIO responds to.
 * \param[in] pin_bit_mask Specifies the exact pins to be configured.
 * \param[in] pull_down_en Enables/Disables the pull-down mode.
 * \param[in] pull_up_en Enables/Disables the pull-up mode.
 *
 * The function aborts if both pull-up and pull-down are enabled simultaneously.
 *
 * \return None.
 */
static void gpioConfig(gpio_mode_t mode, gpio_int_type_t intr_type, uint64_t pin_bit_mask, gpio_pulldown_t pull_down_en,
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
 * \brief Main function to initialize various sets of GPIOs.
 *
 * Calls 'gpioConfig' with pre-defined parameters to set up different GPIOs for specific uses. This includes GPIOs used
 * as output (e.g., LEDs and digital output channels) and input (e.g., buttons and digital input channels) based on the
 * hardware version being used.
 */
void gpioInit(void)
{
#ifdef CONFIG_HARDWARE_VERSION_CARDIO
    gpioConfig(
        GPIO_MODE_OUTPUT, GPIO_PIN_INTR_DISABLE,
        ((1ULL << STATE_LED_R_IO) | (1ULL << BAT_LED_STATUS_IO) | (1ULL << O0_IO) | (1ULL << O1_IO) | (1ULL << MCP_CS)), 0,
        0);
#else
    gpioConfig(GPIO_MODE_OUTPUT, (gpio_int_type_t)GPIO_PIN_INTR_DISABLE,
               ((1ULL << STATE_LED_R_IO) | (1ULL << STATE_LED_G_IO) | (1ULL << STATE_LED_B_IO) |
                (1ULL << BAT_LED_STATUS_IO) | (1ULL << O0_IO) | (1ULL << O1_IO) | (1ULL << MCP_CS)),
               0, 0);
#endif

    gpioConfig(GPIO_MODE_INPUT, (gpio_int_type_t)GPIO_PIN_INTR_DISABLE, ((1ULL << I0_IO) | (1ULL << I1_IO)), 1,
               0); // The 2 IO inputs
}

/**
 * \brief ISR handler for the DRDY (Data Ready) GPIO signal.
 *
 * This function is executed when the DRDY signal indicates that data is ready. The function is empty because we disable
 * interrupts during communication.
 *
 * \return None.
 */
void IRAM_ATTR gpioDrdyIsrHandler(void)
{
    return;
}

/**
 * \brief Sets up a GPIO for DRDY (Data Ready) and installs the related ISR.
 *
 * \param[in] io_num The GPIO number to be used for the DRDY signal.
 *
 * Configures a specific GPIO to act as a signal for when data is ready. The function sets the GPIO mode, specifies the
 * interrupt trigger type, and then installs the GPIO ISR handler specifically for this pin.
 *
 * \return None.
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
