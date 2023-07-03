/** \file sd_card.c
    \brief SD card driver.
    This file implements the SD card driver.
*/

#include "sd_card.h"

#include <inttypes.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "bt.h"
#include "config.h"
#include "esp_vfs_fat.h"
#include "macros.h"
#include "macros_conf.h"
#include "scientisst.h"
#include "sdmmc_cmd.h"

#if _SD_CARD_ENABLED_ == 1

#define MOUNT_POINT "/sdcard"

FILE* save_file = NULL;
sdmmc_host_t host = SDSPI_HOST_DEFAULT();
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
    uint32_t ext_ch_raw[2] = {0, 0};
    uint8_t mid_frame_flag = 0;
    uint32_t index = 0;
    uint8_t io[4] = {0, 0, 0, 0};

    // While there is data to be read, write it to the SD card
    // Code is identical to python API implementation
    for (int i = 0; i < len; i += packet_size) {
        if (op_mode != OP_MODE_LIVE) return ESP_OK;
        mid_frame_flag = 0;
        for (int j = 0; j < num_extern_active_chs; j++) {
            ext_ch_raw[j] =
                (((uint32_t)buff[index]) | (((uint32_t)buff[index + 1]) << 8) |
                 ((uint32_t)buff[index + 2] << 16)) &
                0xFFFFFF;
            index += 3;
        }

        for (int j = 0; j < num_intern_active_chs; j++) {
            if (mid_frame_flag) {
                int_ch_raw[j] = (((uint32_t)buff[index]) |
                                 (((uint32_t)buff[index + 1]) << 8)) &
                                0xFFF;
                index += 2;
                mid_frame_flag = 0;
            } else {
                int_ch_raw[j] = (((uint32_t)buff[index]) |
                                 (((uint32_t)buff[index + 1]) << 8)) >>
                                4;
                index += 1;
                mid_frame_flag = 1;
            }
        }

        io[0] = (buff[index] & 0b10000000) >> 7;
        io[1] = (buff[index] & 0b01000000) >> 6;
        io[2] = (buff[index] & 0b00100000) >> 5;
        io[3] = (buff[index] & 0b00010000) >> 4;

        index += 1;

        num_seq = (*(uint16_t*)(buff + index)) >> 4;

        index += 2;

        fprintf(save_file, "%hu,%hhu,%hhu,%hhu,%hhu", num_seq, io[0], io[1],
                io[2], io[3]);
        for (int j = 0; j < num_intern_active_chs; j++) {
            fprintf(save_file, ",%u", int_ch_raw[j]);
        }
        for (int j = 0; j < num_extern_active_chs; j++) {
            fprintf(save_file, ",%u", ext_ch_raw[j]);
        }

        fprintf(save_file, "\n");
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
#if FORMAT_SDCARD_IF_MOUNT_FAILED == 1
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
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

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        DEBUG_PRINT_E("initSDCard", "Failed to initialize bus.");
        return ESP_FAIL;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config,
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

    return openSDCard();
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
esp_err_t openSDCard(void) {
    char int_str[15];
    char full_file_name[100];
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
    spi_bus_free(host.slot);
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
            active_ext_chs[num_extern_active_chs] = channel_number - 1;
            num_extern_active_chs++;
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

    // WARNING: if changed, change same code in API
    if (sample_rate > 100) {
        send_threshold = !(send_buff_len % packet_size)
                             ? send_buff_len - packet_size
                             : send_buff_len - (send_buff_len % packet_size);
    } else {
        send_threshold = packet_size;
    }

    // Start external
#if _ADC_EXT_ != NO_ADC_EXT
    if (num_extern_active_chs) {
        uint8_t channel_mask = 0;
        for (int i = 0; i < num_extern_active_chs; i++) {
            channel_mask |= 0b1 << (active_ext_chs[i] - 6);
        }
        mcpSetupRoutine(channel_mask);
        adcExtStart();
    }
#else
    num_extern_active_chs = 0;
    active_ext_chs[0] = 0;
    active_ext_chs[1] = 0;
#endif

    packet_size = getPacketSize();

#if _ADC_EXT_ != NO_ADC_EXT
    fprintf(save_file,
            "#{'API version': 'NULL', 'Channels': [1, 2, 3, 4, 5, 6, 7, 8], "
            "'Channels indexes mV': [6, 8, 10, 12, 14, 16, 18, 20], 'Channels "
            "indexes raw': [5, 7, 9, 11, 13, 15, 17, 19], 'Channels labels': "
            "['AI1_raw', 'AI1_mv', 'AI2_raw', 'AI2_mv', 'AI3_raw', 'AI3_mv', "
            "'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', 'AI6_raw', 'AI6_mv', "
            "'AX7_raw', 'AX7_mv', 'AX8_raw', 'AX8_mv'], 'Device': '%s', "
            "'Firmware version': '%s', 'Header': ['NSeq', 'I1', 'I2', 'O1', "
            "'O2', 'AI1_raw', 'AI1_mv', 'AI2_raw', 'AI2_mv', 'AI3_raw', "
            "'AI3_mv', 'AI4_raw', 'AI4_mv', 'AI5_raw', 'AI5_mv', 'AI6_raw', "
            "'AI6_mv', 'AX7_raw', 'AX7_mv', 'AX8_raw', 'AX8_mv'], 'ISO 8601': "
            "'NULL', 'Resolution (bits)': [4, 1, 1, 1, 1, 12, 12, 12, 12, 12, "
            "12, 24, 24], 'Sampling rate (Hz)': 1000, 'Timestamp': 0.0}\n",
            device_name, FIRMWARE_VERSION);
#else
    fprintf(
        save_file,
        "#{'API version': 'NULL', 'Channels': [1, 2, 3, 4, 5, 6], 'Channels "
        "indexes mV': [6, 8, 10, 12, 14, 16], 'Channels indexes raw': [5, 7, "
        "9, 11, 13, 15,], 'Channels labels': ['AI1_raw', 'AI1_mv', 'AI2_raw', "
        "'AI2_mv', 'AI3_raw', 'AI3_mv', 'AI4_raw', 'AI4_mv', 'AI5_raw', "
        "'AI5_mv', 'AI6_raw', 'AI6_mv'], 'Device': '%s','Firmware version': "
        "'%s', 'Header': ['NSeq', 'I1', 'I2', 'O1', 'O2', 'AI1_raw', 'AI1_mv', "
        "'AI2_raw', 'AI2_mv', 'AI3_raw', 'AI3_mv', 'AI4_raw', 'AI4_mv', "
        "'AI5_raw', 'AI5_mv', 'AI6_raw', 'AI6_mv'], 'ISO 8601':'NULL', "
        "'Resolution (bits)': [4, 1, 1, 1, 1, 12, 12, 12, 12, 12, 12,], "
        "'Sampling rate (Hz)': 1000, 'Timestamp': 0.0}\n",
        device_name, FIRMWARE_VERSION);
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

    DEBUG_PRINT_W("startAcquisition", "Acquisition started");
    op_mode = OP_MODE_LIVE;
}

#endif  // USE_SDCARD