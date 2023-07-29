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
#include "timer.h"

#if _SD_CARD_ENABLED_ == SD_CARD_ENABLED

#define DEFAULT_VREF 1100

#define MOUNT_POINT "/sdcard"

FILE* save_file = NULL;
char full_file_name[100];

sdmmc_card_t* card;

/**
 * \brief Send function for SD card.
 *
 * \param fd Not used.
 * \param len Length of the buffer.
 * \param buff Buffer.
 *
 * \return ESP_OK if successful, ESP_FAIL otherwise.
 */
esp_err_t IRAM_ATTR saveToSDCardSend(uint32_t fd, int len, uint8_t* buff) {
    uint16_t num_seq = 0;
    uint32_t int_ch_raw[6] = {0, 0, 0, 0, 0, 0};
    int32_t ext_ch_raw[2] = {0, 0};
    uint8_t mid_frame_flag = 0;
    uint32_t index = 0;
    uint8_t io[4] = {0, 0, 0, 0};
    float int_ch_mv[6] = {0, 0, 0, 0, 0, 0};
    float ext_ch_mv[2] = {0, 0};

    // While there is data to be read, write it to the SD card
    // Code is identical to python API implementation
    for (int i = 0; i < len; i += packet_size) {
        if (op_mode != OP_MODE_LIVE) return ESP_OK;
        mid_frame_flag = 0;
        for (int j = num_extern_active_chs - 1; j >= 0; j--) {
            ext_ch_raw[j] = *(int32_t*)(buff + index) & 0x00FFFFFF;
            index += 3;
        }

        for (int j = num_intern_active_chs - 1; j >= 0; j--) {
            if (!mid_frame_flag) {
                int_ch_raw[j] = (*(uint16_t*)(buff + index)) & 0x0FFF;
                index++;
                mid_frame_flag = 1;
            } else {
                int_ch_raw[j] = (*(uint16_t*)(buff + index)) >> 4;
                index += 2;
                mid_frame_flag = 0;
            }
        }

        io[0] = (buff[index] & 0b10000000) >> 7;
        io[1] = (buff[index] & 0b01000000) >> 6;
        io[2] = (buff[index] & 0b00100000) >> 5;
        io[3] = (buff[index] & 0b00010000) >> 4;

        index += 1;

        num_seq = (*(uint16_t*)(buff + index)) >> 4;

        index += 2;

#if CONVERSION_MODE == RAW_AND_MV
        // Convert raw data to millivolts
        for (int j = 0; j < num_intern_active_chs; j++) {
            int_ch_mv[j] =
                ((((adc1_chars.coeff_a * int_ch_raw[j]) + 32767) / 65536) +
                 adc1_chars.coeff_b) *
                3.399;
        }

        for (int j = 0; j < num_extern_active_chs; j++) {
            ext_ch_mv[j] =
                ((ext_ch_raw[j] * (3.3f * 2)) / (pow(2, 24) - 1)) * 1000;
        }
#endif
#ifdef MULTIPLE_SPI_DEVICES
        xSemaphoreTake(spi_mutex, portMAX_DELAY);  // Lock SPI bus
#endif
        fprintf(save_file, "%hu\t%hhu\t%hhu\t%hhu\t%hhu", num_seq, io[0], io[1],
                io[2], io[3]);
        for (int j = 0; j < num_intern_active_chs; j++) {
            fprintf(save_file, "\t%u", int_ch_raw[j]);
#if CONVERSION_MODE == RAW_AND_MV
            fprintf(save_file, "\t%.0f", int_ch_mv[j]);
#endif
        }
        for (int j = 0; j < num_extern_active_chs; j++) {
            fprintf(save_file, "\t%u", ext_ch_raw[j]);
#if CONVERSION_MODE == RAW_AND_MV
            fprintf(save_file, "\t%.3f", ext_ch_mv[j]);
#endif
        }

        fprintf(save_file, "\n");
#ifdef MULTIPLE_SPI_DEVICES
        xSemaphoreGive(spi_mutex);  // Free SPI bus
#endif
    }

    finalizeSend();

    // Try to send next buff
    sendData();
    return ESP_OK;
}

/**
 * \brief Initialize SPI host and SPI bus and then mount the SD card.
 *
 * \return ESP_OK if successful, ESP_FAIL otherwise.
 */
esp_err_t initSDCard(void) {
    const char mount_point[] = MOUNT_POINT;
    esp_err_t ret = ESP_OK;

    // SD card mount config
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#if FORMAT_SDCARD_IF_MOUNT_FAILED == FORMAT_SDCARD
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 1,
        .allocation_unit_size = 16 * 1024
    };

    // SPI bus config
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(sd_spi_host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        DEBUG_PRINT_E("initSDCard", "Failed to initialize bus.");
        return ESP_FAIL;
    }

#ifdef MULTIPLE_SPI_DEVICES
    // Initialize external ADC to not create noise on SPI bus
    mcpSetupRoutine(0x03);
#endif

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = sd_spi_host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &sd_spi_host, &slot_config,
                                  &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            DEBUG_PRINT_E(
                "initSDCard",
                "Failed to mount filesystem. "
                "If you want the card to be formatted, set the "
                "CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            DEBUG_PRINT_E(
                "initSDCard",
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
 * This function will open a new file on the SD card for writing. First checks
 * if a file with the same name already exists and if so, will increment the
 * file name by 1 until a file name is found that does not exist.
 *
 * \return ESP_OK if successful, ESP_FAIL otherwise.
 */
esp_err_t createFile(void) {
    char int_str[15];
    struct stat st;
    const char* file_name = MOUNT_POINT "/acquisition_datapoints";

    strcpy(full_file_name, file_name);
    strcat(full_file_name, ".csv");

    if (stat(full_file_name, &st) == 0) {  // If file already exists
        for (int i = 1; i > 0; ++i) {      // Find a new file name
            strcpy(full_file_name, file_name);
            sprintf(int_str, "%d", i);
            strcat(full_file_name, int_str);
            strcat(full_file_name, ".csv");
            if (stat(full_file_name, &st) != 0) {
                save_file = fopen(full_file_name, "w");  // Create new file
                i = -5;
                break;
            }
        }
    } else {
        save_file = fopen(full_file_name, "w");  // Create new file
    }

    if (save_file == NULL) {
        DEBUG_PRINT_E("initSDCard", "Failed to open file for writing");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * \brief Open the current file on the SD card for appending.
 *
 * \return ESP_OK if successful, ESP_FAIL otherwise.
 */
esp_err_t openFile(void) {
    save_file = fopen(full_file_name, "a");  // Create new file

    if (save_file == NULL) {
        DEBUG_PRINT_E("openFile", "Failed to open file for writing");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * \brief Close the file on the SD card.
 *
 * \return ESP_OK if successful, ESP_FAIL otherwise.
 */
void closeSDCard(void) { fclose(save_file); }

/**
 * \brief Unmount the SD card and free the SPI bus.
 *
 * \return ESP_OK if successful, ESP_FAIL otherwise.
 */
void unmountSDCard(void) {
    const char mount_point[] = MOUNT_POINT;
    closeSDCard();
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    spi_bus_free(sd_spi_host.slot);
}

/**
 * \brief Start the acquisition of data from the ADC and save it to the SD card.
 *
 * This function will start the acquisition of data from the ADC and save it to
 * the SD card. It acquires from all available channels at 1000Hz and saves the
 * data to a CSV file on the SD card.
 *
 */
void startAcquisitionSDCard(void) {
    // Get channels from mask
    uint8_t i;
    int channel_number = DEFAULT_ADC_CHANNELS + 2;

    changeAPI(API_MODE_SCIENTISST);

    // Reset previous active chs
    num_intern_active_chs = 0;
    num_extern_active_chs = 0;

    sample_rate = 1000;

    // Select the channels that are activated (with corresponding bit equal to
    // 1)
    for (i = 1 << (DEFAULT_ADC_CHANNELS + 2 - 1); i > 0; i >>= 1) {
        // Store the activated channels
        if (i == 1 << (DEFAULT_ADC_CHANNELS + 2 - 1) ||
            i == 1 << (DEFAULT_ADC_CHANNELS + 2 - 2)) {
#if _ADC_EXT_ != NO_EXT_ADC
            active_ext_chs[num_extern_active_chs] = channel_number - 1;
            num_extern_active_chs++;
#else
            channel_number--;
            continue;
#endif
        } else {
            active_internal_chs[num_intern_active_chs] = channel_number - 1;
            num_intern_active_chs++;
        }

        DEBUG_PRINT_W("selectChsFromMask", "Channel A%d added",
                      channel_number - 1);
        channel_number--;
    }

    // Clear send buffs, because of potential previous live mode
    bt_curr_buff = 0;
    acq_curr_buff = 0;
    // Clean send buff, because of send status and send firmware string
    send_busy = 0;

    crc_seq = 0;

    // Clean send buffers, to be sure
    for (uint8_t i = 0; i < NUM_BUFFERS; i++) {
        memset(snd_buff[i], 0, send_buff_len);
        snd_buff_idx[i] = 0;
        bt_buffs_to_send[i] = 0;
    }

    // Start external
#if _ADC_EXT_ != NO_EXT_ADC
    if (num_extern_active_chs) {
        uint8_t channel_mask = 0;
        for (int i = 0; i < num_extern_active_chs; i++) {
            channel_mask |= 0b1 << (active_ext_chs[i] - 6);
        }
        mcpSetupRoutine(channel_mask);
        adcExtStart();
    }
#endif

    packet_size = getPacketSize();

    send_threshold = !(send_buff_len % packet_size)
                         ? send_buff_len - packet_size
                         : send_buff_len - (send_buff_len % packet_size);

#if _ADC_EXT_ != NO_EXT_ADC
#if CONVERSION_MODE == RAW_AND_MV
    fprintf(save_file,
            "#{'API version': 'NULL', 'Channels': [1, 2, 3, 4, 5, 6, 7, 8], "
            "'Channels indexes mV': [], 'Channels "
            "indexes raw': [5, 7, 9, 11, 13, 15, 17, 19], 'Channels labels': "
            "['AI1_raw', 'AI1_mv', 'AI2_raw', "
            "'AI2_mv', 'AI3_raw', 'AI3_mv', "
            "'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', 'AI6_raw', 'AI6_mv',"
            "'AX7_raw', 'AX7_mv', 'AX8_raw', 'AX8_mv'], 'Device': '%s', "
            "'Firmware version': '%s', 'Header': ['NSeq', 'I1', 'I2', 'O1', "
            "'O2', 'AI1_raw', 'AI1_mv', 'AI2_raw', "
            "'AI2_mv', 'AI3_raw', 'AI3_mv', "
            "'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', 'AI6_raw', 'AI6_mv',"
            "'AX7_raw', 'AX7_mv', 'AX8_raw', 'AX8_mv'], 'ISO 8601': "
            "'NULL', 'Resolution (bits)': [4, 1, 1, 1, 1, 12, 12, 12, 12, 12, "
            "12, 24, 24], 'Sampling rate (Hz)': 1000, 'Timestamp': 0.0}\n",
            device_name, FIRMWARE_VERSION);
    fprintf(save_file,
            "#NSeq\tI1\tI2\tO1\tO2\tAI1_raw\tAI1_mv\tAI2_raw\tAI2_mv\tAI3_"
            "raw\tAI3_mv\tAI4_raw\tAI4_mv\tAI5_raw\tAI5_mv\tAI6_raw\tAI6_"
            "mv\tAX7_raw\tAX7_mv\tAX8_raw\tAX8_mv\n");
#elif CONVERSION_MODE == RAW_ONLY
    fprintf(save_file,
            "#{'API version': 'NULL', 'Channels': [1, 2, 3, 4, 5, 6, 7, 8], "
            "'Channels indexes mV': [], 'Channels "
            "indexes raw': [5, 7, 9, 11, 13, 15, 17, 19], 'Channels labels': "
            "['AI1_raw', 'AI2_raw', "
            "'AI3_raw', "
            "'AI4_raw', 'AI5_raw', 'AI6_raw', "
            "'AX7_raw', 'AX8_raw'], 'Device': '%s', "
            "'Firmware version': '%s', 'Header': ['NSeq', 'I1', 'I2', 'O1', "
            "'O2', 'AI1_raw', 'AI2_raw', "
            "'AI3_raw', "
            "'AI4_raw', 'AI5_raw', 'AI6_raw', "
            "'AX7_raw', 'AX8_raw'], 'ISO 8601': "
            "'NULL', 'Resolution (bits)': [4, 1, 1, 1, 1, 12, 12, 12, 12, 12, "
            "12, 24, 24], 'Sampling rate (Hz)': 1000, 'Timestamp': 0.0}\n",
            device_name, FIRMWARE_VERSION);
    fprintf(save_file,
            "#NSeq\tI1\tI2\tO1\tO2\tAI1_raw\tAI2_raw\tAI3_"
            "raw\tAI4_raw\tAI5_raw\tAI6_raw\tAX7_raw\tAX8_raw\n");
#endif
#else
#if CONVERSION_MODE == RAW_AND_MV
    fprintf(
        save_file,
        "#{'API version': 'NULL', 'Channels': [1, 2, 3, 4, 5, 6], 'Channels "
        "indexes mV': [6, 8, 10, 12, 14, 16], 'Channels indexes raw': [5, 7, "
        "9, 11, 13, 15,], 'Channels labels': ['AI1_raw', 'AI1_mv', 'AI2_raw', "
        "'AI2_mv', 'AI3_raw', 'AI3_mv', "
        "'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', 'AI6_raw', 'AI6_mv'], "
        "'Device': '%s','Firmware version': "
        "'%s', 'Header': ['NSeq', 'I1', 'I2', 'O1', 'O2', AI1_raw', 'AI1_mv', "
        "'AI2_raw', "
        "'AI3_raw', 'AI3_mv', 'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', "
        "'AI6_raw', 'AI6_mv'], 'ISO 8601':'NULL', "
        "'Resolution (bits)': [4, 1, 1, 1, 1, 12, 12, 12, 12, 12, 12,], "
        "'Sampling rate (Hz)': 1000, 'Timestamp': 0.0}\n",
        device_name, FIRMWARE_VERSION);
    fprintf(save_file,
            "#NSeq\tI1\tI2\tO1\tO2\tAI1_raw\tAI1_mv\tAI2_raw\tAI2_mv\tAI3_"
            "raw\tAI3_mv\tAI4_raw\tAI4_mv\tAI5_raw\tAI5_mv\tAI6_raw\tAI6_mv\n");
#elif CONVERSION_MODE == RAW_ONLY
    fprintf(
        save_file,
        "#{'API version': 'NULL', 'Channels': [1, 2, 3, 4, 5, 6], 'Channels "
        "indexes mV': [6, 8, 10, 12, 14, 16], 'Channels indexes raw': [5, 7, "
        "9, 11, 13, 15,], 'Channels labels': ['AI1_raw', 'AI2_raw', "
        "'AI3_raw', "
        "'AI4_raw', 'AI5_raw', 'AI6_raw'], "
        "'Device': '%s','Firmware version': "
        "'%s', 'Header': ['NSeq', 'I1', 'I2', 'O1', 'O2', AI1_raw', "
        "'AI2_raw', "
        "'AI3_raw', 'AI4_raw', 'AI5_raw', "
        "'AI6_raw'], 'ISO 8601':'NULL', "
        "'Resolution (bits)': [4, 1, 1, 1, 1, 12, 12, 12, 12, 12, 12,], "
        "'Sampling rate (Hz)': 1000, 'Timestamp': 0.0}\n",
        device_name, FIRMWARE_VERSION);
    fprintf(save_file,
            "#NSeq\tI1\tI2\tO1\tO2\tAI1_raw\tAI2_raw\tAI3_"
            "raw\tAI4_raw\tAI5_raw\tAI6_raw\n");
#endif
#endif

    closeSDCard();

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

    DEBUG_PRINT_W("startAcquisition", "Acquisition started");
    op_mode = OP_MODE_LIVE;
}

#endif  // USE_SDCARD