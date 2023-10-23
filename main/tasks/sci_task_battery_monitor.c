/**
 * \file sci_task_battery_monitor.h
 * \brief Battery Monitoring Task
 *
 * This file contains the implementation of the battery monitoring task.
 * The monitoring task is synchronized with a timer to check the battery status periodically. It interacts
 * with the ADC2 of the ESP32 to read battery levels and controls a status LED to indicate low battery conditions.
 * Additionally, it handles system behavior under low battery conditions, potentially stopping data acquisition
 * and initiating system restarts to preserve the integrity of operations and data.
 */

#include "sci_task_battery_monitor.h"

#include "esp_attr.h"

#include "sci_adc_internal.h"
#include "sci_com.h"
#include "sci_gpio.h"
#include "sci_timer.h"

/**
 * \brief Task monitoring the battery voltage.
 *
 * This function represents a continuously running task that monitors the battery's voltage through the ADC. It is triggered
 * by a timer to check the battery status at regular intervals. If the battery voltage drops below a certain threshold, the
 * function can inflate the threshold to prevent oscillatory behavior, control a status LED, and potentially halt data
 * acquisition and restart the system until the battery is recharged.
 *
 * \return Never returns.
 */
_Noreturn void IRAM_ATTR taskBatteryMonitor(void)
{
    uint16_t battery; // Battery voltage in mV
    uint8_t bat_led_status_gpio = 0;
    uint8_t turn_led_on; // Flag that indicates whether the bat_led_status should turn on or not

    // Init Timer 1_0 (timer 0 from group 1) and register it's interrupt handler
    timerGrpInit(TIMER_GRP_BATTERY, TIMER_IDX_BATTERY, &timerGrp1Isr);
    timerStart(TIMER_GRP_BATTERY, TIMER_IDX_BATTERY, (uint32_t)BATTERY_CHECK_FREQUENCY);

    if (scientisst_device_settings.op_settings.is_battery_threshold_inflated == 1)
    {
        scientisst_device_settings.battery_threshold += 50;
        bat_led_status_gpio = 1;
    }

    while (1)
    {
        if (!ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
            continue;
        battery = getAdcInternalValue(ADC_INTERNAL_2, BATTERY_ADC_CH, 1);

        turn_led_on = battery <= scientisst_device_settings.battery_threshold;

        if (!bat_led_status_gpio && turn_led_on)
        {
            // Inflate threshold so that it doesn't blink due to battery oscillations in the edge of the
            // threshold
            scientisst_device_settings.battery_threshold += 50;
            scientisst_device_settings.op_settings.is_battery_threshold_inflated = 0;
            saveOpSettingsInfo(&(scientisst_device_settings.op_settings));
        }
        else if (bat_led_status_gpio && !turn_led_on)
        {
            // It already charged passed the real threshold, so update the battery_threshold to its real
            // value
            scientisst_device_settings.battery_threshold -= 50;
            scientisst_device_settings.op_settings.is_battery_threshold_inflated = 0;
            saveOpSettingsInfo(&(scientisst_device_settings.op_settings));
        }

        bat_led_status_gpio = turn_led_on;

#ifdef CONFIG_PREVENT_ACQUISITION_ON_LOW_BATTERY
        if (turn_led_on && battery < scientisst_device_settings.battery_threshold - 50)
        {
            DEBUG_PRINT_E("task_battery_monitor", "Battery level is too low and sensor values will have errors");
            stopAcquisition();
            esp_restart();
        }
#endif

        gpio_set_level(BAT_LED_STATUS_IO, bat_led_status_gpio);
    }
}
