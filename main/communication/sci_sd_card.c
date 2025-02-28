/**
 * \file sci_sd_card.c
 * \brief SD card driver.
 *
 * This file contains the implementation of the SD card driver, including initialization, file operations, and the handling
 * of data acquisition from various sources.
 */

#include "sci_sd_card.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sys/stat.h"
#include "sys/unistd.h"

#include "sci_adc_external.h"
#include "sci_adc_internal.h"
#include "sci_gpio.h"

#define DEFAULT_SAVE_FILE_NAME "/sdcard/acquisition_datapoints"

sdmmc_host_t sd_spi_host = {
    .flags = SDMMC_HOST_FLAG_SPI | SDMMC_HOST_FLAG_DEINIT_ARG,
    .slot = SDSPI_DEFAULT_HOST,
#ifdef CONFIG_ADC_EXT
    .max_freq_khz = ADC_EXT_SLCK_HZ_2_EXT_CH,
#else
    .max_freq_khz = SDMMC_FREQ_HIGHSPEED / 4,
#endif
    .io_voltage = 3.3f,
    .init = &sdspi_host_init,
    .set_bus_width = NULL,
    .get_bus_width = NULL,
    .set_bus_ddr_mode = NULL,
    .set_card_clk = &sdspi_host_set_card_clk,
    .do_transaction = &sdspi_host_do_transaction,
    .deinit_p = &sdspi_host_remove_device,
    .io_int_enable = &sdspi_host_io_int_enable,
    .io_int_wait = &sdspi_host_io_int_wait,
    .command_timeout_ms = 0,
};

static esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_FORMAT_SDCARD_IF_MOUNT_FAILED
    .format_if_mount_failed = true,
#else
    .format_if_mount_failed = false,
#endif
    .max_files = 1,
    .allocation_unit_size = 16 * 1024,
}; ///< SD card mount config

spi_bus_config_t bus_cfg = {
    .mosi_io_num = SD_CARD_MOSI,
    .miso_io_num = SD_CARD_MISO,
    .sclk_io_num = SD_CARD_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4000,
    .flags = 0,
    .intr_flags = 0,
}; ///< SPI bus config

static sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
static sdmmc_card_t *card;                   ///< SD card handle
static const char mount_point[] = "/sdcard"; ///< Mount point of the SD card

static esp_err_t createFile(void);

/**
 * \brief Initializes the SD card SPI bus.
 *
 * This function initializes the SPI bus used for SD card communication. It needs to be separate from the the rest of the SD
 * Card initialization because, if other SPI devices are used, they must be added to the SPI bus before the SD card.
 *
 * \return sdmmc_host_t* A pointer to the SD card host structure.
 */
sdmmc_host_t *initSdCardSpiBus(void)
{
    esp_err_t ret = ESP_OK;

    ret = spi_bus_initialize(sd_spi_host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    return &sd_spi_host;
}

/**
 * \brief Initializes and mounts the SD card.
 *
 * This function initializes the necessary hardware interfaces and mounts the SD card to a specified mount point in the
 * virtual file system. It handles possible errors and reports back the status.
 *
 * \return ESP_OK - Success, ESP_FAIL - Failure.
 */
esp_err_t initSDCard(void)
{
    esp_err_t ret = ESP_OK;

    gpio_pullup_en(SD_CARD_MOSI);           // Enable pull-up on MOSI, as per SD card spec
    slot_config.gpio_cs = SD_CARD_CS;       // Set the CS pin
    slot_config.host_id = sd_spi_host.slot; // Set the SPI host

    updateLEDStatusCode(FORMATING_SDCARD);
    ret = esp_vfs_fat_sdspi_mount(mount_point, &sd_spi_host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            DEBUG_PRINT_E("initSDCard", "Failed to mount filesystem. "
                                        "If you want the card to be formatted, set the "
                                        "FORMAT_SDCARD_IF_MOUNT_FAILED  option accordingly.");
        }
        else
        {
            DEBUG_PRINT_E("initSDCard",
                          "Failed to initialize the card with error: (%s). "
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

    return ESP_OK;
}

/**
 * \brief Writes the header information to the file on the SD card.
 *
 * This function prepares a specific header with data acquisition parameters and writes it to the beginning of the file,
 * ensuring that data is properly parsed by SDCardFileConversion.py script.
 *
 * \return None.
 */
void writeFileHeader(void)
{
    uint8_t file_header[25] = {0};

    file_header[0] = 0x01; // File version
    file_header[1] = 0x02; // # of Bytes of SeqNum
    file_header[2] = 0x01; // Is seq num fused with DIO? (4 bits for DIO, 12 bits for seq num)
    file_header[3] = 0x02; // # of digital inputs
    file_header[4] = 0x00; // # of Bytes for each digital input
    file_header[5] = 0x02; // # of digital outputs
    file_header[6] = 0x00; // # of Bytes for each digital output
    file_header[7] = 0x06; // # of internal ADC channels
    file_header[8] = 0x02; // # of Bytes for each internal ADC channel
    file_header[9] = scientisst_device_settings.num_extern_active_chs; // # of external ADC channels
    file_header[10] = 0x03;                                            // # of Bytes for each external ADC channel
    file_header[11] = 0x01;                                            // Internal ADC data type ID
    file_header[12] = 0x01;                                            // External ADC data type ID
    file_header[13] = 0x01;                                            // Timestamp enable?
    file_header[14] = 0x08;                                            // # of Bytes for timestamp
    *(uint16_t *)&file_header[15] = (uint16_t)scientisst_device_settings.sample_rate_hz;     // Sample rate
    *(int *)&file_header[17] = scientisst_device_settings.adc_chars[ADC_INTERNAL_1].coeff_a; // ADC1 coeff_a
    *(int *)&file_header[21] = scientisst_device_settings.adc_chars[ADC_INTERNAL_1].coeff_b; // ADC1 coeff_b

    // Write the file header
    fwrite(file_header, sizeof(uint8_t), 25, scientisst_buffers.sd_card_save_file);
    fflush(scientisst_buffers.sd_card_save_file);
    fsync(fileno(scientisst_buffers.sd_card_save_file));
}

/**
 * \brief Open a new file on the SD card for writing.
 *
 * This function will open a new file on the SD card for writing. First checks if a file with the same name already exists
 * and if so, will increment the file name by 1 until a file name is found that does not exist.
 *
 * \return ESP_OK - successful, ESP_FAIL - otherwise.
 */
static esp_err_t createFile(void)
{
    char full_file_name[100];
    char int_str[15];
    struct stat st;
    const char *base_file_name = DEFAULT_SAVE_FILE_NAME;

    strcpy(full_file_name, base_file_name);
    strcat(full_file_name, ".bin");
    for (uint32_t i = 0; /*Stop condition inside*/; ++i) // Find the next file name that does not exist
    {
        strcpy(full_file_name, base_file_name);
        sprintf(int_str, "%d", i);
        strcat(full_file_name, int_str);
        strcat(full_file_name, ".bin");
        if (stat(full_file_name, &st) != 0) // If the file does not exist
        {
            scientisst_buffers.sd_card_save_file = fopen(full_file_name, "wb"); // Create new file
            break;
        }
    }

    if (scientisst_buffers.sd_card_save_file == NULL)
    {
        DEBUG_PRINT_E("initSDCard", "Failed to open file for writing");
        return ESP_FAIL;
    }

    // When using the external ADC and because it uses the same SPI lines as the SDCard, the data has to be
    // written directly to the SD card for concurrency reasons. So we need to disable buffering
    if (scientisst_device_settings.num_extern_active_chs)
    {
        DEBUG_PRINT_I("initSDCard", "Buffering disabled");
        if (setvbuf(scientisst_buffers.sd_card_save_file, NULL, _IONBF, 0) != 0)
        {
            DEBUG_PRINT_E("initSDCard", "Failed to open file for writing");
        }
    }

    return ESP_OK;
}

/**
 * \brief Unmounts the SD card and frees the SPI bus.
 *
 * This function safely unmounts the SD card from the file system and releases the SPI bus resources.
 *
 * \note This function is unused for now, but it is kept here for future use.
 *
 * \return None.
 */
void unmountSDCard(void)
{
    fclose(scientisst_buffers.sd_card_save_file);
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    spi_bus_free(sd_spi_host.slot);
}
