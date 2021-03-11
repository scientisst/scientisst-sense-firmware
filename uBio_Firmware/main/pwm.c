#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "tasks.h"


#define LED_STAT_GPIO 		GPIO_NUM_16
#define LED_STAT_CHANNEL 	LEDC_CHANNEL_0
#define LED_STAT_TIMER 		LEDC_TIMER_0

#define PWM_GPIO			GPIO_NUM_13
#define PWM_CHANNEL 		LEDC_CHANNEL_1
#define PWM_TIMER 			LEDC_TIMER_1

uint8_t led_step = 10;

// forward declarations
static void pulseLEDTask(void *parameter);


void ledStatConfig() {
	ledc_channel_config_t led_stat_config = {
		.gpio_num = LED_STAT_GPIO,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.channel = LED_STAT_CHANNEL,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = LED_STAT_TIMER,
		.duty = 0,
	};
    ledc_channel_config(&led_stat_config);

    ledc_timer_config_t led_stat_timer_config = {
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.duty_resolution = LEDC_TIMER_10_BIT,
		.timer_num = LED_STAT_TIMER,
		.freq_hz = 5000,
	};
    ledc_timer_config(&led_stat_timer_config);

    createPulseLEDTask(&pulseLEDTask);

    printf(">> OK: stat LED configured\n");
}


void pwmConfig() {
	ledc_channel_config_t pwm_config = {
		.gpio_num = PWM_GPIO,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.channel = PWM_CHANNEL,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = PWM_TIMER,
		.duty = 0,
	};
    ledc_channel_config(&pwm_config);

    ledc_timer_config_t pwm_timer_config = {
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.duty_resolution = 8,
		.timer_num = PWM_TIMER,
		.freq_hz = 735,
	};
    ledc_timer_config(&pwm_timer_config);

    printf(">> OK: PWM configured\n");
}


void setDCpwm(uint8_t dutyCycle) {
	ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, dutyCycle);
	ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);

	printf(">> OK: PWM duty cycle updated\n");
}


void pulseLEDTask(void *parameter) {
	while (1) {
		extern uint8_t led_step;

		for (int dutyCycle = 0; dutyCycle < 1000; dutyCycle+=led_step){
			ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, dutyCycle);
			ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
			vTaskDelay(10/portTICK_PERIOD_MS);
		}
		 
		for (int dutyCycle = 1000; dutyCycle > 0; dutyCycle-=led_step){
			ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, dutyCycle);
			ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
     		vTaskDelay(10/portTICK_PERIOD_MS);
	  	}
    }
}