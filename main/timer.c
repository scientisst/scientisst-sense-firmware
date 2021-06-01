#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "timer.h"
#include "main.h"
#include "macros.h"
#include "gpio.h"

/*
 * Initialize selected timer of the specified timer group 
 *
 * timer_idx - the timer number to initialize
 * timer_interval_sec - the interval of alarm to set
 */
void timerGrpInit(int timer_group, int timer_idx, void(*timer_isr)()){
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 1;
    #ifdef TIMER_GROUP_SUPPORTS_XTAL_CLOCK
        config.clk_src = TIMER_SRC_CLK_APB;
    #endif

    timer_init(timer_group, timer_idx, &config);

    /* Configure the interrupt on alarm. */
    timer_enable_intr(timer_group, timer_idx);
    timer_isr_register(timer_group, timer_idx, timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
}

//Starts <timer_idx> from <timer_group> and the alarm has an frequency of <frequency> Hz
void timerStart(int timer_group, int timer_idx, uint32_t frequency){
    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);

    DEBUG_PRINT_I("timerStart", "Starting timer with alarm period %f s", ((double)1/(double)frequency));

    //Configure the alarm value (seconds*TIMER_SCALE = ticks)
    timer_set_alarm_value(timer_group, timer_idx, ((double)1/(double)frequency) * TIMER_SCALE);
    timer_start(timer_group, timer_idx);
}

//Stops <timer_idx> from <timer_group>
void timerPause(int timer_group, int timer_idx){
    timer_pause(timer_group, timer_idx);
}

/*
 * Timer group0 ISR handler
 * This interrupt is handled by CPU1
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR (This flag forces this 
 * function's text to reside in SRAM instead of Flash to improve it's access speed).
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 * 
 */
void IRAM_ATTR timerGrp0Isr(){
    timer_spinlock_take(TIMER_GROUP_USED);

    //Clear the interrupt
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_USED, TIMER_IDX_USED);

    //After the alarm has been triggered we need enable it again, so it is triggered the next time
    timer_group_enable_alarm_in_isr(TIMER_GROUP_USED, TIMER_IDX_USED);

    timer_spinlock_give(TIMER_GROUP_USED);

    //Wake acqAdc2 in order to start ADC readings from adc2. CPU0 will start immediatly acquiring
    //vTaskNotifyGiveFromISR(acquiring_2_task, (BaseType_t*)NULL);

    //Wake acqAdc1 in order to start ADC readings form adc1. This will only start when this handler is terminated.
    vTaskNotifyGiveFromISR(acquiring_1_task, NULL);

    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR();
}

/*
 * Timer group1 ISR handler
 * This interrupt is handled by CPU1
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR (This flag forces this 
 * function's text to reside in SRAM instead of Flash to improve it's access speed).
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 * 
 */
void IRAM_ATTR timerGrp1Isr(){            

    timer_spinlock_take(TIMER_GRP_ACQ_I2C);

    //Clear the interrupt
    timer_group_clr_intr_status_in_isr(TIMER_GRP_ACQ_I2C, TIMER_IDX_ACQ_I2C);

    
    //After the alarm has been triggered we need enable it again, so it is triggered the next time
    timer_group_enable_alarm_in_isr(TIMER_GRP_ACQ_I2C, TIMER_IDX_ACQ_I2C);

    timer_spinlock_give(TIMER_GRP_ACQ_I2C);

    vTaskNotifyGiveFromISR(acquiring_i2c_task, pdFALSE);


    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR();
}


/*If bluetooth throughput is too high, either acquring task (CPU1) or sending task (CP0) may starve other tasks of CPU. Hence, sampling rate or
number of channels need to be decreased. When that starvation happens, the Task Watchdog Timer (TWDT) will trigger and
this handler will be called and livemode will be stop on its own.
*/
void esp_task_wdt_isr_user_handler(){
    stopAcquisition();
    esp_restart();
}