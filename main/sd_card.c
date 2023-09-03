/** \file sd_card.c
    \brief SD card driver.
    This file implements the SD card driver.
*/

#include "sd_card.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "adc.h"
#include "bt.h"
#include "com.h"
#include "config.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_vfs_fat.h"
#include "gpio.h"
#include "macros.h"
#include "macros_conf.h"
#include "scientisst.h"
#include "sdmmc_cmd.h"
#include "spi.h"
#include "timer.h"

#if _SD_CARD_ENABLED_ == SD_CARD_ENABLED

#define DEFAULT_VREF 1100
#define BUFFER_SIZE (512 * 7 * 2) // 512 * 28 * 2
#define NUM_ACK_BUF (8 * 2)
#define MOUNT_POINT "/sdcard"

TaskHandle_t file_sync_task;
char full_file_name[100];
FILE *save_file = NULL;
sdmmc_card_t *card;
DMA_ATTR char *buffer[NUM_ACK_BUF];
char *buffer_ptr = NULL;
uint16_t buf_ready[NUM_ACK_BUF] = {0};
SemaphoreHandle_t buf_ready_mutex[NUM_ACK_BUF];
uint8_t buf_num_ack = 0;
uint8_t buf_num_write = 0;
uint32_t crc_seq_num = 0;
#define CONVERSION_FACTOR 0.000393f // 391 would be exact
//((3.3f * 2 * 1000) / (pow(2, 24) - 1))  // External ADC conversion factor

void initializeDevice(void);

/**
 * \brief Acquire the channels and store them in the SD card.
 *
 * This function acquires the channels and stores them in the SD card.
 *
 */
void IRAM_ATTR acquireChannelsSDCard(void)
{
    uint16_t adc_internal_res[6];
    float int_ch_mv[6];

#if _ADC_EXT_ != NO_EXT_ADC
    uint32_t adc_external_res[2] = {1, 1};
    float ext_ch_mv[2] = {0, 0};
    uint32_t *adc_external_res_ptr = adc_external_res;
#endif

    adc_internal_res[0] = adc1_get_raw(analog_channels[active_internal_chs[0]]);
    adc_internal_res[1] = adc1_get_raw(analog_channels[active_internal_chs[1]]);
    adc_internal_res[2] = adc1_get_raw(analog_channels[active_internal_chs[2]]);
    adc_internal_res[3] = adc1_get_raw(analog_channels[active_internal_chs[3]]);
    adc_internal_res[4] = adc1_get_raw(analog_channels[active_internal_chs[4]]);
    adc_internal_res[5] = adc1_get_raw(analog_channels[active_internal_chs[5]]);

#if _ADC_EXT_ != NO_EXT_ADC
    // Get raw values from AX1 & AX2 (A6 and A7), store them in the frame
    mcpReadADCValues(REG_ADCDATA, 4 * num_extern_active_chs);
    for (int i = 0; i < num_extern_active_chs; i++, adc_external_res_ptr++)
    {
        uint32_t ch_value = active_ext_chs[i] - 6;
        for (int j = 0; j < 3; j++)
        {
            if ((ext_adc_raw_data[j] >> 28) == ch_value)
            {
                *adc_external_res_ptr =
                    ((ext_adc_raw_data[j] >> 24) & 0x01) ? 0 : (ext_adc_raw_data[j] & 0x00FFFFFF);
                break;
            }
        }
    }
#endif

#if CONVERSION_MODE == RAW_AND_MV
    // Convert raw data to millivolts
    for (int j = 0; j < 6; j++)
    {
        uint32_t temp = (((adc1_chars.coeff_a * adc_internal_res[j]) + 32767) >> 16) + adc1_chars.coeff_b;
        int_ch_mv[j] = temp * 3.399;
    }
#if _ADC_EXT_ != NO_EXT_ADC
    for (int j = 0; j < 2; j++)
    {
        ext_ch_mv[j] = adc_external_res[j] * CONVERSION_FACTOR;
    }
#endif
#endif
    // Write the internal channel values
    // Loop unrolled for speed. All intern channels are always active
    buffer_ptr +=
        sprintf(buffer_ptr,
                "%hu\t%hhu\t%hhu\t%hhu\t%hhu\t%hu\t%.0f\t%hu\t%.0f\t%hu\t%.0f\t%hu\t%."
                "0f\t%hu\t%.0f\t%hu\t%.0f",
                (crc_seq_num & 0xFFF), gpio_get_level(I0_IO), gpio_get_level(I1_IO),
                (gpio_out_state[0] & 0b1), (gpio_out_state[1] & 0b1), adc_internal_res[5], int_ch_mv[5],
                adc_internal_res[4], int_ch_mv[4], adc_internal_res[3], int_ch_mv[3], adc_internal_res[2],
                int_ch_mv[2], adc_internal_res[1], int_ch_mv[1], adc_internal_res[0], int_ch_mv[0]);

    crc_seq_num++;

#if _ADC_EXT_ != NO_EXT_ADC
    // Write the external channel values
    buffer_ptr += sprintf(buffer_ptr, "\t%u\t%.3f\t%u\t%.3f", adc_external_res[1], ext_ch_mv[1],
                          adc_external_res[0], ext_ch_mv[0]);
#endif

    buffer_ptr += sprintf(buffer_ptr, "\t%lld\n", esp_timer_get_time());

#if _ADC_EXT_ != NO_EXT_ADC

    write(fileno(save_file), buffer[0], (buffer_ptr - (buffer[0])));
    buffer_ptr = (buffer[0]);

    if ((crc_seq_num % 3000) == 0)
    {
        fsync(fileno(save_file));
    }

#else
    // If the buffer is full, save it to the SD card
    if (buffer_ptr - (buffer[buf_num_ack % NUM_ACK_BUF]) >= BUFFER_SIZE - 97)
    {
        xSemaphoreTake(buf_ready_mutex[buf_num_ack % NUM_ACK_BUF], portMAX_DELAY);
        buf_ready[buf_num_ack % NUM_ACK_BUF] = buffer_ptr - (buffer[buf_num_ack % NUM_ACK_BUF]);
        xSemaphoreGive(buf_ready_mutex[buf_num_ack % NUM_ACK_BUF]);
        xTaskNotifyGive(file_sync_task);
        while (buf_ready[(buf_num_ack + 1) % NUM_ACK_BUF])
        {
            DEBUG_PRINT_E("acqAdc1", "Buffer overflow!");
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        buffer_ptr = (buffer[(++buf_num_ack) % NUM_ACK_BUF]);
    }
#endif
}

void IRAM_ATTR fileSyncTask(void *not_used)
{
    while (1)
    {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
        {
            while (1)
            {
                xSemaphoreTake(buf_ready_mutex[buf_num_write % NUM_ACK_BUF], portMAX_DELAY);
                if (buf_ready[buf_num_write % NUM_ACK_BUF])
                {
                    write(fileno(save_file), (buffer[buf_num_write % NUM_ACK_BUF]),
                          buf_ready[buf_num_write % NUM_ACK_BUF]);
                    buf_ready[buf_num_write % NUM_ACK_BUF] = 0;
                    xSemaphoreGive(buf_ready_mutex[buf_num_write % NUM_ACK_BUF]);
                    fflush(save_file);
                    fsync(fileno(save_file));
                    buf_num_write++;
                }
                else
                {
                    xSemaphoreGive(buf_ready_mutex[buf_num_write % NUM_ACK_BUF]);
                    break;
                }
            }
        }
        else
        {
            DEBUG_PRINT_W("fileSyncTask", "ulTaskNotifyTake timed out!");
        }
    }
}

/**
 * \brief Acquisition task of the data from the ADC and save it to the SD
 * card.
 *
 * This function is identic to aquire adc1 but instead of saving the data to
 * the buffers it saves it directly to the SD card. This also means that the
 * send task is not active.
 *
 */
void IRAM_ATTR acquisitionSDCard(void *not_used)
{
    // Init Timer 0_1 (timer 1 from group 0) and register it's interupt handler
    timerGrpInit(TIMER_GROUP_USED, TIMER_IDX_USED, timerGrp0Isr);

    // Config all possible adc channels
    initAdc(ADC_RESOLUTION, 1, !isComModeWifi());

    while (1)
    {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
        {
            acquireChannelsSDCard();
        }
        else
        {
            DEBUG_PRINT_W("acqAdc1", "ulTaskNotifyTake timed out!");
        }
    }
}

/**
 * \brief Initialize SPI host and SPI bus and then mount the SD card.
 *
 * \return ESP_OK if successful, ESP_FAIL otherwise.
 */
esp_err_t initSDCard(void)
{
    const char mount_point[] = MOUNT_POINT;
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < NUM_ACK_BUF; i++)
    {
        if ((buf_ready_mutex[i] = xSemaphoreCreateMutex()) == NULL)
        {
            DEBUG_PRINT_E("xSemaphoreCreateMutex", "Mutex creation failed");
            abort();
        }
    }

    // SD card mount config
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#if FORMAT_SDCARD_IF_MOUNT_FAILED == FORMAT_SDCARD
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 1,
        .allocation_unit_size = 16 * 1024,
    };

    // SPI bus config
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000, // 512 * 8
        .flags = 0,
        .intr_flags = 0,
    };

    ret = spi_bus_initialize(sd_spi_host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        DEBUG_PRINT_E("initSDCard", "Failed to initialize bus.");
        return ESP_FAIL;
    }
    gpio_pullup_en(PIN_NUM_MOSI); // Enable pull-up on MISO

    DEBUG_PRINT_W("sdcard", "Initializing device...");
    initializeDevice();

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = sd_spi_host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &sd_spi_host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            DEBUG_PRINT_E("initSDCard", "Failed to mount filesystem. "
                                        "If you want the card to be formatted, set the "
                                        "CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            DEBUG_PRINT_E("initSDCard",
                          "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                          esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    return createFile();
}

/**
 * \brief Open a new file on the SD card for writing.
 *
 * This function will open a new file on the SD card for writing. First
 * checks if a file with the same name already exists and if so, will
 * increment the file name by 1 until a file name is found that does not
 * exist.
 *
 * \return ESP_OK if successful, ESP_FAIL otherwise.
 */
esp_err_t createFile(void)
{
    char int_str[15];
    struct stat st;
    const char *file_name = MOUNT_POINT "/acquisition_datapoints";

    strcpy(full_file_name, file_name);
    strcat(full_file_name, ".csv");

    if (stat(full_file_name, &st) == 0)
    { // If file already exists
        for (int i = 1; i > 0; ++i)
        { // Find a new file name
            strcpy(full_file_name, file_name);
            sprintf(int_str, "%d", i);
            strcat(full_file_name, int_str);
            strcat(full_file_name, ".csv");
            if (stat(full_file_name, &st) != 0)
            {
                save_file = fopen(full_file_name, "w"); // Create new file
                i = -5;
                break;
            }
        }
    }
    else
    {
        save_file = fopen(full_file_name, "w"); // Create new file
    }

    if (save_file == NULL)
    {
        DEBUG_PRINT_E("initSDCard", "Failed to open file for writing");
        return ESP_FAIL;
    }

#if _ADC_EXT_ != NO_EXT_ADC
    if (setvbuf(save_file, NULL, _IONBF, 0) != 0)
    {
        DEBUG_PRINT_E("initSDCard", "Failed to open file for writing");
    }
#endif

    return ESP_OK;
}

/**
 * \brief Unmount the SD card and free the SPI bus.
 *
 * \return ESP_OK if successful, ESP_FAIL otherwise.
 */
void unmountSDCard(void)
{
    const char mount_point[] = MOUNT_POINT;
    closeSDCard();
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    spi_bus_free(sd_spi_host.slot);
}

void initializeDevice(void)
{
    uint8_t i;
    int channel_number = DEFAULT_ADC_CHANNELS + 2;
#if _ADC_EXT_ != NO_EXT_ADC
    int active_channels_sd = 0b11111111;
#else
    int active_channels_sd = 0b00111111;
#endif
    changeAPI(API_MODE_SCIENTISST);

    // Reset previous active chs
    num_intern_active_chs = 0;
    num_extern_active_chs = 0;

#if _ADC_EXT_ != NO_EXT_ADC
    sample_rate = 100;
#else
    sample_rate = 1000;
#endif

    // Select the channels that are activated (with corresponding bit equal
    // to 1)
    for (i = 1 << (DEFAULT_ADC_CHANNELS + 2 - 1); i > 0; i >>= 1)
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

    // Clear send buffs, because of potential previous live mode
    bt_curr_buff = 0;
    acq_curr_buff = 0;
    // Clean send buff, because of send status and send firmware string
    send_busy = 0;

    crc_seq_num = 0;

    // Clean send buffers, to be sure
    for (uint8_t i = 0; i < NUM_BUFFERS; i++)
    {
        memset(snd_buff[i], 0, send_buff_len);
        snd_buff_idx[i] = 0;
        bt_buffs_to_send[i] = 0;
    }

    // Start external
#if _ADC_EXT_ != NO_EXT_ADC
    uint8_t channel_mask = 0;
    for (int i = 0; i < num_extern_active_chs; i++)
    {
        channel_mask |= 0b1 << (active_ext_chs[i] - 6);
    }
    mcpSetupRoutine(channel_mask);
#endif

    packet_size = getPacketSize();
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
void startAcquisitionSDCard(void)
{
#if _ADC_EXT_ != NO_EXT_ADC
    fprintf(save_file,
            "#{'API version': 'NULL', 'Channels': [1, 2, 3, 4, 5, 6, 7, 8], "
            "'Channels indexes mV': [], 'Channels "
            "indexes raw': [5, 7, 9, 11, 13, 15, 17, 19], 'Channels labels': "
            "['AI1_raw', 'AI1_mv', 'AI2_raw', "
            "'AI2_mv', 'AI3_raw', 'AI3_mv', "
            "'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', 'AI6_raw', 'AI6_mv',"
            "'AX7_raw', 'AX7_mv'], 'Device': '%s', "
            "'Firmware version': '%s', 'Header': ['NSeq', 'I1', 'I2', 'O1', "
            "'O2', 'AI1_raw', 'AI1_mv', 'AI2_raw', "
            "'AI2_mv', 'AI3_raw', 'AI3_mv', "
            "'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', 'AI6_raw', 'AI6_mv',"
            "'AX7_raw', 'AX7_mv', 'AX8_raw', 'AX8_mv', 'Timestamp'], 'ISO 8601': "
            "'NULL', 'Resolution (bits)': [4, 1, 1, 1, 1, 12, 12, 12, 12, 12, "
            "12, 24, 24], 'Sampling rate (Hz)': 1000, 'Timestamp': 0.0}\n",
            device_name, FIRMWARE_VERSION);
    fprintf(save_file, "#NSeq\tI1\tI2\tO1\tO2\tAI1_raw\tAI1_mv\tAI2_raw\tAI2_mv\tAI3_"
                       "raw\tAI3_mv\tAI4_raw\tAI4_mv\tAI5_raw\tAI5_mv\tAI6_raw\tAI6_"
                       "mv\tAX7_raw\tAX7_mv\tAX8_raw\tAX8_mv\tTimestamp\n");

#else
    fprintf(save_file,
            "#{'API version': 'NULL', 'Channels': [1, 2, 3, 4, 5, 6], "
            "'Channels "
            "indexes mV': [6, 8, 10, 12, 14, 16], 'Channels indexes raw': [5, "
            "7, "
            "9, 11, 13, 15,], 'Channels labels': ['AI1_raw', 'AI1_mv', "
            "'AI2_raw', "
            "'AI2_mv', 'AI3_raw', 'AI3_mv', "
            "'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', 'AI6_raw', 'AI6_mv'], "
            "'Device': '%s','Firmware version': "
            "'%s', 'Header': ['NSeq', 'I1', 'I2', 'O1', 'O2', AI1_raw', "
            "'AI1_mv', "
            "'AI2_raw', "
            "'AI3_raw', 'AI3_mv', 'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', "
            "'AI6_raw', 'AI6_mv', 'Timestamp'], 'ISO 8601':'NULL', "
            "'Resolution (bits)': [4, 1, 1, 1, 1, 12, 12, 12, 12, 12, 12,], "
            "'Sampling rate (Hz)': 1000, 'Timestamp': 0.0}\n",
            device_name, FIRMWARE_VERSION);
    fprintf(save_file, "#NSeq\tI1\tI2\tO1\tO2\tAI1_raw\tAI1_mv\tAI2_raw\tAI2_mv\tAI3_"
                       "raw\tAI3_mv\tAI4_raw\tAI4_mv\tAI5_raw\tAI5_mv\tAI6_raw\tAI6_"
                       "mv\tTimestamp\n");
#endif
#if _ADC_EXT_ != NO_EXT_ADC
    adcExtStart();
#endif
    fflush(save_file);
    fsync(fileno(save_file));

#if _ADC_EXT_ == NO_EXT_ADC
    xTaskCreatePinnedToCore(&fileSyncTask, "file_sync_task", 4096 * 4, NULL, 20, &file_sync_task, 0);
#endif

    for (int i = 0; i < NUM_ACK_BUF; i++)
    {
        buffer[i] = (char *)malloc(BUFFER_SIZE * sizeof(char));
        if (buffer[i] == NULL)
        {
            DEBUG_PRINT_E("malloc", "Error allocating memory for send buffers");
            exit(-1);
        }
        memset(buffer[i], 0, BUFFER_SIZE);
    }

    buffer_ptr = buffer[0];

    // Init timer for adc task top start
    timerStart(TIMER_GROUP_USED, TIMER_IDX_USED, sample_rate);

    // Set led state to blink at live mode frequency
    ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_LIVE_PWM_FREQ);

    // Set live mode duty cycle for state led
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R, LEDC_LIVE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R);
#if HW_VERSION != HW_VERSION_CARDIO
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G, LEDC_LIVE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G);
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B, LEDC_LIVE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B);
#endif

    DEBUG_PRINT_W("startAcquisition", "Acquisition started: %lld",
                  (int64_t)(esp_timer_get_time() & 0xFFFFFFFFFFFF));
    op_mode = OP_MODE_LIVE;
}

#endif // USE_SDCARD