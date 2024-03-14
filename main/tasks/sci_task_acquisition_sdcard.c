/**
 * \file sci_task_acquisition_sdcard.h
 * \brief Interface for data acquisition tasks specific to the SD card.
 *
 * This file contains the functions used for acquiring data from ADCs and storing the data directly onto an SD card. It
 * includes functions for initializing data acquisition, handling data from different sources, and managing file
 * synchronization tasks. When using only the internal ADCs, the data is first stored in a buffer and then written to the SD
 * Card by another task, akin to how other modes work. When using the external ADC, the data is written directly to the SD
 * card after acquiring it. This is done because the SD card and external ADC use the same SPI lines and, through testing, it
 * was found that this approach would yield the best performance. More testing and performance optimizations will be
 * conducted.
 */

#include "sci_task_acquisition_sdcard.h"

#include <string.h>
#include <unistd.h>

#include "esp_attr.h"

#include "sci_adc_external.h"
#include "sci_adc_internal.h"
#include "sci_gpio.h"
#include "sci_sd_card.h"
#include "sci_task_imu.h"
#include "sci_timer.h"

DRAM_ATTR static TaskHandle_t file_sync_task; ///< Handle for the file synchronization task.

static uint8_t *acquireChannelsSDCard(uint8_t *buffer_ptr);
static void startAcquisitionSDCard(void);

/**
 * \brief Main task for data acquisition from the ADC and storage on the SD card.
 *
 * This task manages the continuous acquisition of data from when in SD Card mode. It replaces sci_task_acquisition,
 * sci_task_com_rx, and sci_task_com_tx.
 *
 * \return Never returns.
 */
_Noreturn void IRAM_ATTR acquisitionSDCard(void)
{
    uint8_t *buffer_ptr = scientisst_buffers.frame_buffer[0];

    startAcquisitionSDCard();

    while (1)
    {
        if (!ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
            continue;
        buffer_ptr = acquireChannelsSDCard(buffer_ptr);
    }
}

/**
 * \brief Acquires data from active channels and peripherals.
 *
 * This function is responsible for retrieving data from both internal and external ADCs and/or the IMU, based on the
 * current configuration. It either stores the data in a buffer or writes it directly to the SD card, depending on if the
 * external ADC is used or not.
 *
 * \param[in/out] buffer_ptr Pointer to the current position in the buffer, where new data will be written.
 *
 * \return A pointer to the buffer position following the data write.
 */
static uint8_t *IRAM_ATTR acquireChannelsSDCard(uint8_t *buffer_ptr)
{
    static uint16_t crc_seq = 0; // Static variable that counts the number of packets sent

    // Write sequence number and GPIO state
    *(uint16_t *)buffer_ptr = (uint16_t)((crc_seq & 0x0FFF) << 4);
    *(uint16_t *)buffer_ptr |= (gpio_get_level(I0_IO) & 0b1) | ((gpio_get_level(I1_IO) & 0b1) << 1) |
                               ((scientisst_device_settings.gpio_out_state[0] & 0b1) << 2) |
                               ((scientisst_device_settings.gpio_out_state[1] & 0b1) << 3);
    buffer_ptr += 2;

    ++crc_seq; // Increment the sequence number

#ifdef CONFIG_IMU
    // Store values of IMU data into frame
    for (int i = 5; i >= 0; --i)
    {
        *(uint16_t *)buffer_ptr = getImuValue(i);
        buffer_ptr += 2;
    }
#else
    // Get raw values from A1 to A6 (A1 to A6)
    for (int i = 5; i >= 0; --i)
    {
        *(uint16_t *)buffer_ptr = getAdcInternalValue(ADC_INTERNAL_1, (uint8_t)i, 0) & 0x0FFF;
        buffer_ptr += 2;
    }
#endif

#ifdef CONFIG_ADC_EXT
    // Get raw values from AX1 & AX2 (A6 and A7), store them in the frame
    esp_err_t res = ESP_OK;
    uint8_t channel_mask = 0;
    uint32_t adc_ext_values[CONFIG_NUMBER_CHANNELS_EXT_ADC];

    if (scientisst_device_settings.active_ext_chs[0] == 6 || scientisst_device_settings.active_ext_chs[1] == 6)
        channel_mask |= 0b01;
    if (scientisst_device_settings.active_ext_chs[0] == 7 || scientisst_device_settings.active_ext_chs[1] == 7)
        channel_mask |= 0b10;

    res = getAdcExtValuesRaw(channel_mask, adc_ext_values);
    ESP_ERROR_CHECK_WITHOUT_ABORT(res);
    for (uint8_t i = scientisst_device_settings.num_extern_active_chs - 1; i >= 0; --i)
    {
        *(uint32_t *)buffer_ptr = adc_ext_values[i];
        buffer_ptr += 3;
    }

#endif

    *(uint64_t *)buffer_ptr = esp_timer_get_time();
    buffer_ptr += 8;

#ifdef CONFIG_ADC_EXT
    if ((crc_seq & 127) == 0)
    {
        write(fileno(scientisst_buffers.sd_card_save_file), scientisst_buffers.frame_buffer[0],
              (buffer_ptr - (scientisst_buffers.frame_buffer[0])));
        buffer_ptr = scientisst_buffers.frame_buffer[0];
    }

    if ((crc_seq % 3000) == 0) // Every ~30 seconds force a sync so data is really stored on the SD card
    {
        fsync(fileno(scientisst_buffers.sd_card_save_file));
    }
#else
    // If the buffer is full, save it to the SD card
    if (buffer_ptr - (scientisst_buffers.frame_buffer[scientisst_buffers.acq_curr_buff]) >=
        MAX_BUFFER_SIZE_SDCARD - 22) // 28 is max size, 22 when no adc ext
    {
        scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.acq_curr_buff] =
            (uint16_t)(buffer_ptr - scientisst_buffers.frame_buffer[scientisst_buffers.acq_curr_buff]);

        xTaskNotifyGive(file_sync_task);

        scientisst_buffers.acq_curr_buff = (scientisst_buffers.acq_curr_buff + 1) % NUM_BUFFERS_SDCARD;

        while (scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.acq_curr_buff] != 0)
        {
            DEBUG_PRINT_E("acqAdc1", "Buffer overflow!");
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        buffer_ptr = (scientisst_buffers.frame_buffer[scientisst_buffers.acq_curr_buff]);
    }
#endif

    return buffer_ptr;
}

/**
 * \brief Background task for file synchronization on the SD card.
 *
 * Akin to how other com modes work, this task is notified whenever a buffer is ready for writing to the SD card. It writes
 * the buffer data to the file system and handles file synchronization to ensure data integrity. It is only active when the
 * external ADC is not used.
 *
 * \return Never returns.
 */
_Noreturn static void IRAM_ATTR fileSyncTask(void)
{
    while (1)
    {
        if (!ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
            continue;
        while (scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.tx_curr_buff] != 0)
        {
            write(fileno(scientisst_buffers.sd_card_save_file),
                  (scientisst_buffers.frame_buffer[scientisst_buffers.tx_curr_buff]),
                  scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.tx_curr_buff]);
            fflush(scientisst_buffers.sd_card_save_file);
            fsync(fileno(scientisst_buffers.sd_card_save_file));

            scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.tx_curr_buff] = 0;
            scientisst_buffers.tx_curr_buff = (scientisst_buffers.tx_curr_buff + 1) % NUM_BUFFERS_SDCARD;
        }
    }
}

/**
 * \brief Initializes the data acquisition process and starts writing data to the SD card.
 *
 * This function configures the system for data acquisition, setting up active channels, sample rates, and
 * initializing peripherals. If no external ADC is used, starts the fileSyncTask. It creates the header for the data file on
 * the SD card, start the timer and sets the state LED on the correct mode.
 *
 * \return None.
 */
static void startAcquisitionSDCard(void)
{
    int channel_number = DEFAULT_ADC_CHANNELS + 2;
    int active_channels_sd; // All internal channels are always active

    DEBUG_PRINT_I("acqAdc1", "NUMBER ACTIVE CHANNELS: %d", scientisst_device_settings.num_extern_active_chs);

    switch (scientisst_device_settings.num_extern_active_chs)
    {
    case 1:
        active_channels_sd = 0b01111111;
        scientisst_device_settings.sample_rate_hz = 100;
        break;
    case 2:
        active_channels_sd = 0b11111111;
        scientisst_device_settings.sample_rate_hz = 100;
        break;
    default:
        active_channels_sd = 0b00111111;
        scientisst_device_settings.sample_rate_hz = 2000;
        // If external ADC is not used, start the secondary task that writes the data to the SD card
        xTaskCreatePinnedToCore((TaskFunction_t)&fileSyncTask, "file_sync_task", DEFAULT_TASK_STACK_SIZE_XLARGE, NULL, 20,
                                &file_sync_task, 0);
        break;
    }

    scientisst_device_settings.num_intern_active_chs = 0;
    scientisst_device_settings.num_extern_active_chs = 0;
    scientisst_buffers.acq_curr_buff = 0;
    scientisst_buffers.tx_curr_buff = 0;

    // Select the channels that are activated (with corresponding bit equal to 1)
    for (uint8_t i = 1 << (DEFAULT_ADC_CHANNELS + 2 - 1); i > 0; i >>= 1)
    {
        if (!(active_channels_sd & i))
        {
            channel_number--;
            continue;
        }

        // Store the activated channels
        if (i == 1 << (DEFAULT_ADC_CHANNELS + 2 - 1) || i == 1 << (DEFAULT_ADC_CHANNELS + 2 - 2))
        {
            scientisst_device_settings.active_ext_chs[scientisst_device_settings.num_extern_active_chs] =
                (uint8_t)channel_number - 1;
            scientisst_device_settings.num_extern_active_chs++;
        }
        else
        {
            scientisst_device_settings.active_internal_chs[scientisst_device_settings.num_intern_active_chs] =
                (uint8_t)channel_number - 1;
            scientisst_device_settings.num_intern_active_chs++;
        }

        DEBUG_PRINT_I("selectChsFromMaskBitalino", "Channel A%d added", channel_number);
        channel_number--;
    }

    if (scientisst_device_settings.num_extern_active_chs != 0)
    {
        uint8_t channel_mask = 0;
        for (int i = 0; i < scientisst_device_settings.num_extern_active_chs; i++)
        {
            channel_mask |= 0b1 << (scientisst_device_settings.active_ext_chs[i] - 6);
        }
        mcpSetupRoutine(channel_mask);
        adcExtStart();
    }

    // Write the header to the file
    writeFileHeader();

    // Init timer for adc task top start
    timerStart(TIMER_GROUP_MAIN, TIMER_IDX_MAIN, scientisst_device_settings.sample_rate_hz);
    // Set led state to blink at live mode frequency
    ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_LIVE_PWM_FREQ);

    // Set live mode duty cycle for state led
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R, LEDC_LIVE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R);
#ifndef CONFIG_HARDWARE_VERSION_CARDIO
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G, LEDC_LIVE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G);
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B, LEDC_LIVE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B);
#endif

    scientisst_device_settings.op_mode = OP_MODE_LIVE;

    DEBUG_PRINT_I("startAcquisition", "Acquisition started: %lld", (int64_t)(esp_timer_get_time() & 0xFFFFFFFFFFFF));
}
