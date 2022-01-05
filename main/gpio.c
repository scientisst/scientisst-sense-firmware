#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "gpio.h"
#include "adc.h"
#include "macros.h"
#include "main.h"

//TODO: Meter os pins SPI3_MISO, U2RXD, U2TXD, SDA0, SCL0 a pull up

/* About this example
 * 1. Start with initializing LEDC module:
 *    a. Set the timer of LEDC first, this determines the frequency
 *       and resolution of PWM.
 *    b. Then set the LEDC channel you want to use,
 *       and bind with one of the timers.
 * 2. You need first to install a default fade function,
 *    then you can use fade APIs.
 * 3. You can also set a target duty directly without fading.
 */

void configLedC(void){
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,   // resolution of PWM duty
        .freq_hz = LEDC_IDLE_PWM_FREQ,          // frequency of PWM signal
        .speed_mode = LEDC_SPEED_MODE_USED,          // timer mode
        .timer_num = LEDC_LS_TIMER,             // timer index
        .clk_cfg = LEDC_AUTO_CLK,               // Auto select the source clock
    };
    // Set configuration of timer0 for low speed channels
    ledc_timer_config(&ledc_timer);

    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_USED,
        .duty       = 0,
        .gpio_num   = STATE_LED_IO,
        .speed_mode = LEDC_SPEED_MODE_USED,
        .hpoint     = 0,
        .timer_sel  = LEDC_LS_TIMER
    };

    // Set LED Controller with previously prepared configuration
    ledc_channel_config(&ledc_channel);
    
    // Initialize fade service.
    //ledc_fade_func_install(0);

    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, LEDC_IDLE_DUTY);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}

void gpioConfig(gpio_mode_t mode, gpio_int_type_t intr_type, uint64_t pin_bit_mask, gpio_pulldown_t pull_down_en, gpio_pullup_t pull_up_en){
    gpio_config_t io_conf;

    if(pull_down_en && pull_up_en){
        DEBUG_PRINT_E("gpioConfig", "Pin cannot have both pull up and pull down enabled");
        return;
    }

    //config pin direction (output/input)
    io_conf.mode = mode;
    //config interrupt (disable/what type of interrupt)
    io_conf.intr_type = intr_type;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = pin_bit_mask;
    //config pull-down mode (enable/disable)
    io_conf.pull_down_en = pull_down_en;
    //config pull-up mode (enable/disable)
    io_conf.pull_up_en = pull_up_en;

    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void gpioInit(){
    gpioConfig(GPIO_MODE_OUTPUT, GPIO_PIN_INTR_DISABLE, ((1ULL<< STATE_LED_IO) | (1ULL<< BAT_LED_STATUS_IO) | (1ULL<< O0_IO) | (1ULL<< O1_IO) | (1ULL<< SPI3_CS0_IO)), 0, 0);       
    gpioConfig(GPIO_MODE_INPUT, GPIO_PIN_INTR_DISABLE, ((1ULL<< I0_IO) | (1ULL<< I1_IO)), 1, 0);                                //The 2 IO inputs
    //gpioConfig(GPIO_MODE_INPUT, GPIO_PIN_INTR_DISABLE, ((1ULL<< A2_IO) | (1ULL<< A3_IO) | (1ULL<< A4_IO)), 1, 0);                            
    configLedC();
}

bool IRAM_ATTR gpioDrdyIsrHandler(){
    //Wake acqAdc1. This will only start when this handler is terminated.
    vTaskNotifyGiveFromISR(acquiring_1_task, NULL);

    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
    //portYIELD_FROM_ISR();

    return 1;       //Replaces portYIELD_FROM_ISR()
}

void adcExtDrdyGpio(int io_num){
    //Config DRDY gpio
	gpioConfig(GPIO_MODE_INPUT, GPIO_INTR_NEGEDGE, ((1ULL<< io_num)), 0, 0);

	//install gpio isr service
    gpio_install_isr_service(0);

	//hook isr handler for specific gpio pin
    gpio_isr_handler_add(io_num, gpioDrdyIsrHandler, NULL);
}



