/** \file timer.c
    \brief Timer functions for the Scientisst firmware.

    This file contains the functions for the timer used in the Scientisst
   firmware.
*/

#include "driver/timer.h"

#include "driver/periph_ctrl.h"
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "gpio.h"
#include "macros.h"
#include "macros_conf.h"
#include "scientisst.h"
#include "timer.h"

/**
 * \brief Initialize selected timer of the specified timer group
 *
 * \param timer_group The timer group to be initialized
 * \param timer_idx The timer number to initialize
 *
 */
void timerGrpInit(int timer_group, int timer_idx, bool (*timer_isr)())
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = 1,
    };
#ifdef TIMER_GROUP_SUPPORTS_XTAL_CLOCK
    config.clk_src = TIMER_SRC_CLK_APB;
#endif

    timer_init(timer_group, timer_idx, &config);

    /* Configure the interrupt on alarm. */
    timer_enable_intr(timer_group, timer_idx);
    timer_isr_callback_add(timer_group, timer_idx, timer_isr, NULL, 0);
}

/**
 * \brief Starts selected timer of the specified timer group
 *
 * Starts <timer_idx> from <timer_group> and the alarm has an frequency of
 * <frequency> Hz
 *
 * \param timer_group The timer group of the timer to be started
 * \param timer_idx The timer number to start
 * \param frequency The frequency of the alarm
 *
 */
void timerStart(int timer_group, int timer_idx, uint32_t frequency)
{
    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on
       alarm */
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);

    DEBUG_PRINT_I("timerStart", "Starting timer with alarm period %f s", ((double)1 / (double)frequency));

    // Configure the alarm value (seconds*TIMER_SCALE = ticks)
    timer_set_alarm_value(timer_group, timer_idx, (uint64_t)(((double)1 / (double)frequency) * TIMER_SCALE));
    timer_start(timer_group, timer_idx);
}

/**
 * \brief Stops selected timer of the specified timer group
 *
 * \param timer_group The timer group of the timer to be stopped
 * \param timer_idx The timer number to stop
 *
 */
void timerPause(int timer_group, int timer_idx)
{
    timer_pause(timer_group, timer_idx);
}

/*
 * \brief Timer group0 ISR handler
 * This interrupt is handled by CPU1
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR
 * (This flag forces this function's text to reside in SRAM instead of Flash to
 * improve it's access speed). If we're okay with the timer irq not being
 * serviced while SPI flash cache is disabled, we can allocate this interrupt
 * without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 *
 * \return 1 Always returns 1
 */
bool IRAM_ATTR timerGrp0Isr(void)
{
    // Wake acqAdc1 in order to start ADC readings form adc1. This will only start when this handler is
    // terminated.
    vTaskNotifyGiveFromISR(acq_adc1_task, NULL);
    return 1;
}

/*
 * \brief Timer group1 ISR handler
 *
 * \return 1 Always returns 1
 */
bool IRAM_ATTR timerGrp1Isr(void)
{
    vTaskNotifyGiveFromISR(abat_task, NULL);
    return 1;
}

/**
 * \brief ISR handler for the Task Watchdog Timer
 *
 * This function is called when the Task Watchdog Timer (TWDT) triggers. If
 * bluetooth throughput is too high, either acquring task (CPU1) or sending task
 * (CP0) may starve other tasks of CPU. Hence, sampling rate or number of
 * channels need to be decreased. When that starvation happens, the Task
 * Watchdog Timer (TWDT) will trigger and this handler will be called and
 * livemode will be stop on its own. The ESP32 will then restart.
 *
 */
void esp_task_wdt_isr_user_handler(void)
{
    stopAcquisition();
    esp_restart();
}