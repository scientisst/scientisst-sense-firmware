#include "include/sci_task_battery_monitor.h"

#include <stdint.h>

#include "esp_attr.h"
#include "sdkconfig.h"

#include "sci_adc_internal.h"
#include "sci_config.h"
#include "sci_gpio.h"
#include "sci_macros.h"
#include "sci_macros_conf.h"
#include "sci_scientisst.h"
#include "sci_timer.h"

#define ABAT_DIVIDER_FACTOR 2

/**
 * \brief Task that acquires data from adc2 (battery).
 *
 * This task is responsible for acquiring data from adc2 (battery). It is
 * notified by the timerGrp1Isr when to acquire data. It notifies the sendTask
 * when there is data to send. It is also the main task of CPU0 (PRO CPU)
 */
void IRAM_ATTR task_battery_monitor(void *not_used)
{
    uint16_t raw;
    uint16_t abat; // Battery voltage in mV
    uint8_t bat_led_status_gpio = 0;
    uint8_t turn_led_on; // Flag that indicates wether the bat_led_status should turn on or not

    // Init Timer 1_0 (timer 0 from group 1) and register it's interupt handler
    timerGrpInit(TIMER_GRP_ABAT, TIMER_IDX_ABAT, timerGrp1Isr);
    timerStart(TIMER_GRP_ABAT, TIMER_IDX_ABAT, (uint32_t)ABAT_CHECK_FREQUENCY);

    if (op_settings.is_battery_threshold_inflated == 1)
    {
        battery_threshold += 50;
        bat_led_status_gpio = 1;
    }

    while (1)
    {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
        {
            if (adc2_get_raw(ABAT_ADC_CH, ADC_RESOLUTION, (int *)&raw) != ESP_OK)
            {
                DEBUG_PRINT_E("adc2_get_raw", "Error!");
                continue;
            }
            abat = esp_adc_cal_raw_to_voltage((uint32_t)raw, &adc2_chars) * ABAT_DIVIDER_FACTOR;

            turn_led_on = abat <= battery_threshold;

            if (!bat_led_status_gpio && turn_led_on)
            {
                // Inflate threshold so that it doesn't blink due to abat oscilations in the edge of the
                // threshold
                battery_threshold += 50;
                op_settings.is_battery_threshold_inflated = 0;
                saveOpSettingsInfo(&op_settings);
            }
            else if (bat_led_status_gpio && !turn_led_on)
            {
                // It already charged passed the real threshold, so update the battery_threshold to its real
                // value
                battery_threshold -= 50;
                op_settings.is_battery_threshold_inflated = 0;
                saveOpSettingsInfo(&op_settings);
            }

            bat_led_status_gpio = turn_led_on;

            gpio_set_level(BAT_LED_STATUS_IO, bat_led_status_gpio);
        }
        else
        {
            DEBUG_PRINT_W("task_battery_monitor", "ulTaskNotifyTake timed out!");
        }
    }
}
