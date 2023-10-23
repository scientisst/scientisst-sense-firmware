/**
 * \file sci_timer.c
 * \brief Timer functions for the ScientISST firmware.
 *
 * This file contains the functions related to timer configurations and operations, including initializing, starting, and
 * pausing timers, as well as defining ISR (Interrupt Service Routines) for the ScientISST firmware.
 */

#include "sci_timer.h"

#include "esp_types.h"

/**
 * \brief Initializes a timer with specific configurations.
 *
 * This function initializes a timer from a specified timer group with basic parameters and configurations. It also sets up
 * the interrupt service routine for the timer alarms.
 *
 * \param[in] timer_group The timer group number (0 or 1) to which the timer belongs.
 * \param[in] timer_idx The timer number within the group to initialize (usually 0 or 1).
 * \param[in] timer_isr A pointer to the interrupt service routine function to handle timer interrupts.
 *
 * \return None.
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
 * \brief Starts a timer with a specific alarm frequency.
 *
 * This function starts a timer from a specified timer group and sets an alarm with a specified frequency. The timer counts
 * from an initial value and generates an interrupt upon reaching the alarm value.
 *
 * \param[in] timer_group The timer group number (0 or 1) to which the timer belongs.
 * \param[in] timer_idx The timer number within the group to start (usually 0 or 1).
 * \param[in] frequency The frequency (in Hz) at which the timer alarm occurs.
 *
 * \return None.
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
 * \brief Pauses a specified timer.
 *
 * This function pauses the counting of a specified timer, effectively stopping the timer until it is restarted.
 *
 * \param[in] timer_group The timer group number (0 or 1) to which the timer belongs.
 * \param[in] timer_idx The timer number within the group to pause (usually 0 or 1).
 *
 * \return None.
 */
void timerPause(int timer_group, int timer_idx)
{
    timer_pause(timer_group, timer_idx);
}

/**
 * \brief ISR for timer group 0.
 *
 * Notifies acquisition to read sensor data.
 *
 * \note We don't call the timer API here because they are not declared with IRAM_ATTR.
 *
 * \return 1 - Always returns 1
 */
bool IRAM_ATTR timerGrp0Isr(void)
{
    // Wake acqAdc1 in order to start ADC readings form adc1. This will only start when this handler is
    // terminated.
    vTaskNotifyGiveFromISR(acq_adc1_task, NULL);
    return 1;
}

/**
 * \brief ISR for timer group 1.
 *
 * Notifies battery_monitor_task to read battery level.
 *
 * \note We don't call the timer API here because they are not declared with IRAM_ATTR.
 *
 * \return 1 - Always returns 1
 */
bool IRAM_ATTR timerGrp1Isr(void)
{
    vTaskNotifyGiveFromISR(battery_task, NULL);
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
 * the ESP32 will then restart.
 *
 */
void esp_task_wdt_isr_user_handler(void)
{
    esp_restart();
}
