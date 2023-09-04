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

#define BUFFER_SIZE (512 * 7 * 2)
#define NUM_ACQ_BUF (8 * 2)

#define CONVERSION_FACTOR 0.000393f // 391 would be exact
//((3.3f * 2 * 1000) / (pow(2, 24) - 1))  // External ADC conversion factor

TaskHandle_t file_sync_task;

sdmmc_card_t *card;       ///< SD card handle
char full_file_name[100]; ///< Full file name of the file on the SD card
FILE *save_file = NULL;   ///< File handle of the file on the SD card

DMA_ATTR char *buffer[NUM_ACQ_BUF]; ///< Array buffers to store the data to be written to the SD card
/// Array to store the number of bytes ready to be written to the SD card
uint16_t buf_ready[NUM_ACQ_BUF] = {0};
SemaphoreHandle_t buf_ready_mutex[NUM_ACQ_BUF]; ///< Mutex to protect the buffer ready array, one for each
                                                ///< buffer for faster access times

char *buffer_ptr = NULL;   ///< Pointer to the current buffer to write to
uint8_t buf_num_ack = 0;   ///< Number of the buffer where the data is being written to on acquisition
uint8_t buf_num_write = 0; ///< Number of the buffer that is being written to the SD card

void initializeDevice(void);
esp_err_t createFile(void);
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
    static uint32_t crc_seq_num = 0;
    uint16_t adc_internal_res[6];
    float int_ch_mv[6];
    uint32_t temp; // Used to separate integer and floating point instructions, reducing the latter that are
                   // very expensive
#if _ADC_EXT_ != NO_EXT_ADC
    uint32_t adc_external_res[2] = {1, 1};
    float ext_ch_mv[2] = {0, 0};
#endif

    // Loop unrolled for speed. All intern channels are always active
    adc_internal_res[0] = adc1_get_raw(analog_channels[active_internal_chs[0]]);
    adc_internal_res[1] = adc1_get_raw(analog_channels[active_internal_chs[1]]);
    adc_internal_res[2] = adc1_get_raw(analog_channels[active_internal_chs[2]]);
    adc_internal_res[3] = adc1_get_raw(analog_channels[active_internal_chs[3]]);
    adc_internal_res[4] = adc1_get_raw(analog_channels[active_internal_chs[4]]);
    adc_internal_res[5] = adc1_get_raw(analog_channels[active_internal_chs[5]]);

#if _ADC_EXT_ == ADC_MCP
    // Get raw values from AX1 & AX2 (A6 and A7), store them in the frame
    mcpReadADCValues(REG_ADCDATA, 4 * num_extern_active_chs);
    for (int i = 0; i < 2; ++i) // Always both ext channels active
    {
        uint32_t ch_value = active_ext_chs[i] - 6;
        for (int j = 0; j < 3; j++)
        {
            if ((ext_adc_raw_data[j] >> 28) == ch_value)
            {
                adc_external_res[i] =
                    ((ext_adc_raw_data[j] >> 24) & 0x01) ? 0 : (ext_adc_raw_data[j] & 0x00FFFFFF);
                ext_ch_mv[i] = adc_external_res[i] * CONVERSION_FACTOR;
                break;
            }
        }
    }

#endif

    // Convert raw data to millivolts
    // Loop unrolled for speed. All intern channels are always active
    temp = (((adc1_chars.coeff_a * adc_internal_res[0]) + 32767) >> 16) + adc1_chars.coeff_b;
    int_ch_mv[0] = temp * 3.399;
    temp = (((adc1_chars.coeff_a * adc_internal_res[1]) + 32767) >> 16) + adc1_chars.coeff_b;
    int_ch_mv[1] = temp * 3.399;
    temp = (((adc1_chars.coeff_a * adc_internal_res[2]) + 32767) >> 16) + adc1_chars.coeff_b;
    int_ch_mv[2] = temp * 3.399;
    temp = (((adc1_chars.coeff_a * adc_internal_res[3]) + 32767) >> 16) + adc1_chars.coeff_b;
    int_ch_mv[3] = temp * 3.399;
    temp = (((adc1_chars.coeff_a * adc_internal_res[4]) + 32767) >> 16) + adc1_chars.coeff_b;
    int_ch_mv[4] = temp * 3.399;
    temp = (((adc1_chars.coeff_a * adc_internal_res[5]) + 32767) >> 16) + adc1_chars.coeff_b;
    int_ch_mv[5] = temp * 3.399;

    // Write the internal channel values
    buffer_ptr +=
        sprintf(buffer_ptr,
                "%hu\t%hhu\t%hhu\t%hhu\t%hhu\t%hu\t%.0f\t%hu\t%.0f\t%hu\t%.0f\t%hu\t%."
                "0f\t%hu\t%.0f\t%hu\t%.0f",
                (crc_seq_num & 0xFFF), gpio_get_level(I0_IO), gpio_get_level(I1_IO),
                (gpio_out_state[0] & 0b1), (gpio_out_state[1] & 0b1), adc_internal_res[5], int_ch_mv[5],
                adc_internal_res[4], int_ch_mv[4], adc_internal_res[3], int_ch_mv[3], adc_internal_res[2],
                int_ch_mv[2], adc_internal_res[1], int_ch_mv[1], adc_internal_res[0], int_ch_mv[0]);

    ++crc_seq_num;

#if _ADC_EXT_ != NO_EXT_ADC
    // Write the external channel values
    buffer_ptr += sprintf(buffer_ptr, "\t%u\t%.3f\t%u\t%.3f", adc_external_res[1], ext_ch_mv[1],
                          adc_external_res[0], ext_ch_mv[0]);
#endif

    buffer_ptr += sprintf(buffer_ptr, "\t%lld\n", esp_timer_get_time());

#if _ADC_EXT_ != NO_EXT_ADC

    write(fileno(save_file), buffer[0], (buffer_ptr - (buffer[0])));
    buffer_ptr = buffer[0];

    if ((crc_seq_num % 3000) == 0) // Every 30 seconds force a sync (hardware write)
    {
        fsync(fileno(save_file));
    }

#else
    // If the buffer is full, save it to the SD card
    if (buffer_ptr - (buffer[buf_num_ack % NUM_ACQ_BUF]) >= BUFFER_SIZE - 97)
    {
        xSemaphoreTake(buf_ready_mutex[buf_num_ack % NUM_ACQ_BUF], portMAX_DELAY);
        buf_ready[buf_num_ack % NUM_ACQ_BUF] = buffer_ptr - (buffer[buf_num_ack % NUM_ACQ_BUF]);
        xSemaphoreGive(buf_ready_mutex[buf_num_ack % NUM_ACQ_BUF]);
        xTaskNotifyGive(file_sync_task);
        while (buf_ready[(buf_num_ack + 1) % NUM_ACQ_BUF])
        {
            DEBUG_PRINT_E("acqAdc1", "Buffer overflow!");
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        buffer_ptr = (buffer[(++buf_num_ack) % NUM_ACQ_BUF]);
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
                xSemaphoreTake(buf_ready_mutex[buf_num_write % NUM_ACQ_BUF], portMAX_DELAY);
                if (buf_ready[buf_num_write % NUM_ACQ_BUF])
                {
                    write(fileno(save_file), (buffer[buf_num_write % NUM_ACQ_BUF]),
                          buf_ready[buf_num_write % NUM_ACQ_BUF]);
                    buf_ready[buf_num_write % NUM_ACQ_BUF] = 0;
                    xSemaphoreGive(buf_ready_mutex[buf_num_write % NUM_ACQ_BUF]);
                    fflush(save_file);
                    fsync(fileno(save_file));
                    buf_num_write++;
                }
                else
                {
                    xSemaphoreGive(buf_ready_mutex[buf_num_write % NUM_ACQ_BUF]);
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
    }
}

/**
 * \brief Initialize SPI host and SPI bus and then mount the SD card.
 *
 * \return ESP_OK if successful, ESP_FAIL otherwise.
 */
esp_err_t initSDCard(void)
{
    const char mount_point[] = "/sdcard";
    esp_err_t ret = ESP_OK;

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

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();

    gpio_pullup_en(PIN_NUM_MOSI); // Enable pull-up on MOSI

    ret = spi_bus_initialize(sd_spi_host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        DEBUG_PRINT_E("initSDCard", "Failed to initialize bus.");
        return ESP_FAIL;
    }

    initializeDevice(); // EXT ADC has to be initialized before mounting the SD card, because they share
                        // the same SPI bus. Other initializations are also done in this function.

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

    ret = createFile();
    if (ret != ESP_OK)
    {
        DEBUG_PRINT_E("initSDCard", "Failed to create file.");
        return ESP_FAIL;
    }

    // Create mutexes for the buffer ready array
    for (int i = 0; i < NUM_ACQ_BUF; i++)
    {
        if ((buf_ready_mutex[i] = xSemaphoreCreateMutex()) == NULL)
        {
            DEBUG_PRINT_E("xSemaphoreCreateMutex", "Mutex creation failed");
            abort();
        }
    }

    // Allocate memory for the buffers
    for (int i = 0; i < NUM_ACQ_BUF; i++)
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

    return ESP_OK;
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
    const char *file_name = "/sdcard/acquisition_datapoints";

    strcpy(full_file_name, file_name);
    strcat(full_file_name, ".csv");
    for (uint32_t i = 0; /*Stop condition inside*/; ++i) // Find the next file name that does not exist
    {
        strcpy(full_file_name, file_name);
        sprintf(int_str, "%d", i);
        strcat(full_file_name, int_str);
        strcat(full_file_name, ".csv");
        if (stat(full_file_name, &st) != 0) // If the file does not exist
        {
            save_file = fopen(full_file_name, "w"); // Create new file
            break;
        }
    }

    if (save_file == NULL)
    {
        DEBUG_PRINT_E("initSDCard", "Failed to open file for writing");
        return ESP_FAIL;
    }

#if _ADC_EXT_ != NO_EXT_ADC
    // When using the external ADC, the data is written directly to the SD card, so we need to disable
    // buffering
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
 */
void unmountSDCard(void)
{
    const char mount_point[] = "/sdcard";
    fclose(save_file);
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    spi_bus_free(sd_spi_host.slot);
}

void initializeDevice(void)
{
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
            "'Channels indexes mV': [6, 8, 10, 12, 14, 16, 18, 20], 'Channels "
            "indexes raw': [5, 7, 9, 11, 13, 15, 17, 19], 'Channels labels': "
            "['AI1_raw', 'AI1_mv', 'AI2_raw', "
            "'AI2_mv', 'AI3_raw', 'AI3_mv', "
            "'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', 'AI6_raw', 'AI6_mv',"
            "'AX7_raw', 'AX7_mv', 'AX8_raw', 'AX8_mv'], 'Device': '%s', "
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
            "9, 11, 13, 15], 'Channels labels': ['AI1_raw', 'AI1_mv', "
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
            "'Sampling rate (Hz)': 100, 'Timestamp': 0.0}\n",
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
    // If no external ADC is used, start the secondary task that writes the data to the SD card
    xTaskCreatePinnedToCore(&fileSyncTask, "file_sync_task", 4096 * 4, NULL, 20, &file_sync_task, 0);
#endif

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