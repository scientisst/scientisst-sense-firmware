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

DRAM_ATTR static TaskHandle_t file_sync_task;

static uint8_t *acquireChannelsSDCard(uint8_t *buffer_ptr);
static void startAcquisitionSDCard(void);

/**
 * \brief Acquisition task of the data from the ADC and save it to the SD
 * card.
 *
 * This function is identical to the acquire task but instead of saving the data to
 * the buffers it saves it directly to the SD card. This also means that the
 * send task is not active.
 *
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
 * \brief Acquire the channels and store them in the SD card.
 *
 * This function acquires the channels and stores them in the SD card. If no EXT ADC is used, the data is
 * stored into buffers and a second task will write the data to the SD card. If an EXT ADC is used, the data
 * is written directly to the SD card. This is done because the SDCard and EXT ADC use the same SPI lines.
 *
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

    // All intern channels are always active, acquire them
    for (int i = 5; i >= 0; --i)
    {
        *(uint16_t *)buffer_ptr = getAdcInternalValue(ADC_INTERNAL_1, (uint8_t)i, 0) & 0x0FFF;
        buffer_ptr += 2;
    }

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

    if (scientisst_device_settings.active_ext_chs[0] == 6 || scientisst_device_settings.active_ext_chs[1] == 6)
        channel_mask |= 0b01;
    if (scientisst_device_settings.active_ext_chs[0] == 7 || scientisst_device_settings.active_ext_chs[1] == 7)
        channel_mask |= 0b10;

    res = getAdcExtValuesRaw(channel_mask, (uint32_t *)buffer_ptr);
    ESP_ERROR_CHECK_WITHOUT_ABORT(res);
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
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        buffer_ptr = (scientisst_buffers.frame_buffer[scientisst_buffers.acq_curr_buff]);
    }
#endif

    return buffer_ptr;
}
_Noreturn static void IRAM_ATTR fileSyncTask(void)
{
    while (1)
    {
        if (!ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
            continue;
        while (1)
        {
            if (scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.tx_curr_buff] == 0)
                break;

            write(fileno(scientisst_buffers.sd_card_save_file),
                  (scientisst_buffers.frame_buffer[scientisst_buffers.tx_curr_buff]),
                  scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.tx_curr_buff]);
            fflush(scientisst_buffers.sd_card_save_file);
            fsync(fileno(scientisst_buffers.sd_card_save_file));
            memset(scientisst_buffers.frame_buffer[scientisst_buffers.tx_curr_buff], 0,
                   scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.tx_curr_buff]);
            scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.tx_curr_buff] = 0;
            scientisst_buffers.tx_curr_buff = (scientisst_buffers.tx_curr_buff + 1) % NUM_BUFFERS_SDCARD;
        }
    }
}

/**
 * \brief Start the acquisition of data from the ADC and save it to the SD
 * card.
 *
 * This function will start the acquisition of data from the ADC and save it
 * to the SD card. It acquires from all available channels at 1000Hz and
 * saves the data to a CSV file on the SD card.
 *
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
        scientisst_device_settings.sample_rate = 100;
        break;
    case 2:
        active_channels_sd = 0b11111111;
        scientisst_device_settings.sample_rate = 100;
        break;
    default:
        active_channels_sd = 0b00111111;
        scientisst_device_settings.sample_rate = 1000;
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
    timerStart(TIMER_GROUP_MAIN, TIMER_IDX_MAIN, scientisst_device_settings.sample_rate);
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
