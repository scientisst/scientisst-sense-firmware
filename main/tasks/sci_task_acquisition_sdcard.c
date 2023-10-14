#include "include/sci_task_acquisition_sdcard.h"

#include <sys/cdefs.h>

#include "esp_attr.h"

#include "sci_adc_external.h"
#include "sci_adc_internal.h"
#include "sci_gpio.h"
#include "sci_macros.h"
#include "sci_scientisst.h"
#include "sci_timer.h"

#define BUFFER_SIZE (1024 * 7)
#define NUM_ACQ_BUF (16)

TaskHandle_t file_sync_task;

DMA_ATTR uint8_t *buffer[NUM_ACQ_BUF]; ///< Array buffers to store the data to be written to the SD card
/// Array to store the number of bytes ready to be written to the SD card
uint16_t buf_ready[NUM_ACQ_BUF] = {0};
SemaphoreHandle_t buf_ready_mutex[NUM_ACQ_BUF]; ///< Mutex to protect the buffer ready array, one for each
                                                ///< buffer for faster access times

FILE *sd_card_save_file = NULL; ///< File pointer to the file where the data is being saved

uint8_t *buffer_ptr = NULL; ///< Pointer to the current buffer to write to
uint8_t buf_num_acq = 0;    ///< Number of the buffer where the data is being written to on acquisition
uint8_t buf_num_write = 0;  ///< Number of the buffer that is being written to the SD card

void acquireChannelsSDCard(void);
void startAcquisitionSDCard(uint8_t num_active_channels_external_adc);

/**
 * \brief Acquisition task of the data from the ADC and save it to the SD
 * card.
 *
 * This function is identic to aquire adc1 but instead of saving the data to
 * the buffers it saves it directly to the SD card. This also means that the
 * send task is not active.
 *
 */
_Noreturn void IRAM_ATTR acquisitionSDCard(void *num_active_channels_external_adc)
{

    // Create mutexes for the buffer ready array
    for (int i = 0; i < NUM_ACQ_BUF; i++)
    {
        buf_ready_mutex[i] = xSemaphoreCreateMutex();
        CHECK_NOT_NULL(buf_ready_mutex[i]);
    }

    // Allocate memory for the buffers
    for (int i = 0; i < NUM_ACQ_BUF; i++)
    {
        buffer[i] = (uint8_t *)malloc(BUFFER_SIZE * sizeof(uint8_t));
        CHECK_NOT_NULL(buffer[i]);
        memset(buffer[i], 0, BUFFER_SIZE);
    }

    buffer_ptr = buffer[0];

    startAcquisitionSDCard(*(uint8_t *)num_active_channels_external_adc);

    while (1)
    {
        if (!ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
            continue;
        acquireChannelsSDCard();
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
void IRAM_ATTR acquireChannelsSDCard(void)
{
    static uint32_t crc_seq_num = 0; // Static variable that counts the number of packets sent

    *(uint16_t *)buffer_ptr = (crc_seq_num & 0xFFF) << 4;
    *(uint16_t *)buffer_ptr |= (gpio_get_level(I0_IO) & 0b1) | ((gpio_get_level(I1_IO) & 0b1) << 1) |
                               ((gpio_out_state[0] & 0b1) << 2) | ((gpio_out_state[1] & 0b1) << 3);
    buffer_ptr += 2;

    ++crc_seq_num;

    // All intern channels are always active
    for (int i = 0; i < 6; ++i)
    {
        *(uint16_t *)buffer_ptr = adc1_get_raw(analog_channels[active_internal_chs[i]]) & 0x0FFF;
        buffer_ptr += 2;
    }

#if _ADC_EXT_ == EXT_ADC_ENABLED
    // Get raw values from AX1 & AX2 (A6 and A7), store them in the frame
    mcpReadADCValues(REG_ADCDATA, 4 * num_extern_active_chs);
    for (int i = 1; i >= 0; --i) // Always both ext channels active
    {
        uint32_t ch_value = active_ext_chs[i] - 6;
        for (int j = 0; j < 3; j++)
        {
            if ((ext_adc_raw_data[j] >> 28) == ch_value)
            {
                *(uint32_t *)buffer_ptr =
                    ((ext_adc_raw_data[j] >> 24) & 0x01) ? 0 : (ext_adc_raw_data[j] & 0x00FFFFFF);
                buffer_ptr += 3;
                break;
            }
        }
    }
#endif

    *(uint64_t *)buffer_ptr = esp_timer_get_time();
    buffer_ptr += 8;

#if _ADC_EXT_ == EXT_ADC_ENABLED
    if ((crc_seq & 127) == 0)
    {
        write(fileno(save_file), buffer[0], (buffer_ptr - (buffer[0])));
        buffer_ptr = buffer[0];
    }

    if ((crc_seq_num % 30000) == 0) // Every ~30 seconds force a sync so data is really stored on the SD card
    {
        fsync(fileno(save_file));
    }
#else
    // If the buffer is full, save it to the SD card
    if (buffer_ptr - (buffer[buf_num_acq % NUM_ACQ_BUF]) >=
        BUFFER_SIZE - 22) // 28 is max size, 22 when no adc ext
    {
        xSemaphoreTake(buf_ready_mutex[buf_num_acq % NUM_ACQ_BUF], portMAX_DELAY);
        buf_ready[buf_num_acq % NUM_ACQ_BUF] = buffer_ptr - (buffer[buf_num_acq % NUM_ACQ_BUF]);
        xSemaphoreGive(buf_ready_mutex[buf_num_acq % NUM_ACQ_BUF]);
        xTaskNotifyGive(file_sync_task);
        while (buf_ready[(buf_num_acq + 1) % NUM_ACQ_BUF])
        {
            DEBUG_PRINT_E("acqAdc1", "Buffer overflow!");
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        buffer_ptr = (buffer[(++buf_num_acq) % NUM_ACQ_BUF]);
    }
#endif
}
_Noreturn void IRAM_ATTR fileSyncTask(void *not_used)
{
    while (1)
    {
        if (!ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
            continue;
        while (1)
        {
            xSemaphoreTake(buf_ready_mutex[buf_num_write % NUM_ACQ_BUF], portMAX_DELAY);
            if (buf_ready[buf_num_write % NUM_ACQ_BUF])
            {
                write(fileno(sd_card_save_file), (buffer[buf_num_write % NUM_ACQ_BUF]),
                      buf_ready[buf_num_write % NUM_ACQ_BUF]);
                buf_ready[buf_num_write % NUM_ACQ_BUF] = 0;
                xSemaphoreGive(buf_ready_mutex[buf_num_write % NUM_ACQ_BUF]);
                fflush(sd_card_save_file);
                fsync(fileno(sd_card_save_file));
                buf_num_write++;
            }
            else
            {
                xSemaphoreGive(buf_ready_mutex[buf_num_write % NUM_ACQ_BUF]);
                break;
            }
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
void startAcquisitionSDCard(uint8_t num_active_channels_external_adc)
{
    int channel_number = DEFAULT_ADC_CHANNELS + 2;
    int active_channels_sd = 0b00111111; // All internal channels are always active

    if (num_active_channels_external_adc != 0)
    {
        active_channels_sd = 0b11111111;
        sample_rate = 100; // 100Hz is the max tested frequency when using external ADC
    }
    else
    {
        sample_rate = 3000;
    }

    changeAPI(API_MODE_SCIENTISST); // Change API to ScientISST, not really needed but kept for consistency

    num_intern_active_chs = 0;
    num_extern_active_chs = 0;

    // Select the channels that are activated (with corresponding bit equal to 1)
    for (int i = 1 << (DEFAULT_ADC_CHANNELS + 2 - 1); i > 0; i >>= 1)
    {
        if (!(active_channels_sd & i))
        {
            channel_number--;
            continue;
        }

        // Store the activated channels
        if (i == 1 << (DEFAULT_ADC_CHANNELS + 2 - 1) || i == 1 << (DEFAULT_ADC_CHANNELS + 2 - 2))
        {
            active_ext_chs[num_extern_active_chs] = channel_number - 1;
            num_extern_active_chs++;
        }
        else
        {
            active_internal_chs[num_intern_active_chs] = channel_number - 1;
            num_intern_active_chs++;
        }

        DEBUG_PRINT_W("selectChsFromMask", "Channel A%d added", channel_number);
        channel_number--;
    }

    if (num_extern_active_chs != 0)
    {
        uint8_t channel_mask = 0;
        for (int i = 0; i < num_extern_active_chs; i++)
        {
            channel_mask |= 0b1 << (active_ext_chs[i] - 6);
        }
        mcpSetupRoutine(channel_mask);
        adcExtStart();
    }

    if (num_active_channels_external_adc == 0)
    {
        // If no external ADC is used, start the secondary task that writes the data to the SD card
        xTaskCreatePinnedToCore(&fileSyncTask, "file_sync_task", 4096 * 4, NULL, 20, &file_sync_task, 0);
    }

    // Init timer for adc task top start
    timerStart(TIMER_GROUP_USED, TIMER_IDX_USED, sample_rate);
    // Set led state to blink at live mode frequency
    ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_LIVE_PWM_FREQ);

    // Set live mode duty cycle for state led
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R, LEDC_LIVE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R);
#if _HW_VERSION_ != HW_VERSION_CARDIO
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G, LEDC_LIVE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G);
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B, LEDC_LIVE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B);
#endif

    op_mode = OP_MODE_LIVE;

    DEBUG_PRINT_I("startAcquisition", "Acquisition started: %lld",
                  (int64_t)(esp_timer_get_time() & 0xFFFFFFFFFFFF));
}
