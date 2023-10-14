#include <sys/cdefs.h>
#include <sys/queue.h>
/** \file scientisst.c
    \brief Main file for the Scientisst firmware.

    This file is the main file for the Scientisst firmware. It handles the
   initialization of the firmware and the creation of the tasks.
*/

#include <math.h>

#include "esp_vfs_fat.h"
#include "sdkconfig.h"
#include "sdmmc_cmd.h"

#include "sci_adc_external.h"
#include "sci_adc_internal.h"
#include "sci_ble.h"
#include "sci_bt.h"
#include "sci_com.h"
#include "sci_config.h"
#include "sci_gpio.h"
#include "sci_macros.h"
#include "sci_macros_conf.h"
#include "sci_scientisst.h"
#include "sci_sd_card.h"
#include "sci_serial.h"
#include "sci_task_acquisition_sdcard.h"
#include "sci_task_imu.h"
#include "sci_tcp.h"
#include "sci_timer.h"
#include "sci_udp.h"
#include "sci_wifi.h"
#include "sci_wifi_rest_server.h"
#include "sci_ws.h"

#define BT_SEND_PRIORITY 10 // MAX priority in ESP32 is 25
#define ABAT_PRIORITY 1
#define WIFI_RCV_PRIORITY 1
#define ACQ_ADC1_PRIORITY 10

// Functions have to be declared with void* as argument even though it is not used to avoid compiler warnings
void sendTask(void *not_used);
_Noreturn void rcvTask(void *not_used);
void task_battery_monitor(void *not_used);
_Noreturn void task_acquisition(void *not_used);
void opModeConfig(void);

TaskHandle_t send_task;
TaskHandle_t abat_task;
TaskHandle_t rcv_task;
TaskHandle_t acq_adc1_task;
TaskHandle_t imu_task;

// Mutex for the buffers
SemaphoreHandle_t mutex_buffers_ready_to_send;

// BT
char device_name[17] = BT_DEFAULT_DEVICE_NAME;

uint8_t *frame_buffer[NUM_BUFFERS]; ///< Data structure to hold the data to be sentvthrough bluetooth
uint32_t frame_buffer_length = 0;   ///< Length of each send buff
uint16_t frame_buffer_write_idx[NUM_BUFFERS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
///< It contains, for each buffer, the index of the first free element in the respective buffer
uint8_t frame_buffer_ready_to_send[NUM_BUFFERS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
///< If element 0 is set to 1, bt task has to send snd_buff[0]
uint8_t bt_curr_buff = 0;  ///< Index of the buffer that bt task is currently sending
uint8_t acq_curr_buff = 0; ///< Index of the buffer that adc task is currently using

uint8_t packet_size = 0; ///< Current packet size (dependent on number of channels used)
uint8_t send_busy = 0;
uint16_t send_threshold = MAX_BUFFER_SIZE;
///< Amount of bytes a buffer needs to have filled with packets in order to trigger a send

uint16_t battery_threshold = DEFAULT_BATTERY_THRESHOLD;

Api_Config api_config = {.api_mode = API_MODE_BITALINO, .select_ch_mask_func = &selectChsFromMask};
cJSON *json = NULL;

// ADC
DRAM_ATTR const uint8_t analog_channels[DEFAULT_ADC_CHANNELS] = {
    ADC1_CHANNEL_0, ADC1_CHANNEL_3, ADC1_CHANNEL_4, ADC1_CHANNEL_5, ADC1_CHANNEL_6, ADC1_CHANNEL_7};
DRAM_ATTR uint8_t active_internal_chs[DEFAULT_ADC_CHANNELS] = {0, 0, 0, 0, 0, 0};
///< Active internal channels | If all channels are active: = {5, 4, 3, 2, 1, 0}
DRAM_ATTR uint8_t active_ext_chs[EXT_ADC_CHANNELS] = {0, 0};
///< Active external channels | If all channels are active: = {7, 6}
DRAM_ATTR uint8_t num_intern_active_chs = 0;
DRAM_ATTR uint8_t num_extern_active_chs = 0;
DRAM_ATTR uint8_t op_mode = OP_MODE_IDLE; ///< Flag that indicastes if op mode is on (idle, live or config)
uint32_t sample_rate = DEFAULT_SAMPLE_RATE;
esp_adc_cal_characteristics_t adc1_chars;
esp_adc_cal_characteristics_t adc2_chars;
DRAM_ATTR uint8_t gpio_out_state[2] = {0, 0}; ///< Output of 01 & O2 (O0 & O1)

DRAM_ATTR uint32_t ext_adc_raw_data[3]; ///< Raw data from external adc

// SPI
DRAM_ATTR spi_device_handle_t adc_ext_spi_handler;

// Op settings
#if _SD_CARD_ == SD_CARD_ENABLED
op_settings_info_t op_settings = {
    .com_mode = COM_MODE_SD_CARD,
    .is_battery_threshold_inflated = 0,
}; ///< Struct that holds the wifi acquisition configuration (e.g. SSID, password, sample rate...)
#else
op_settings_info_t op_settings = {
    .com_mode = COM_MODE_SERIAL,
}; ///< Struct that holds the wifi acquisition configuration (e.g. SSID, password, sample rate...)
#endif
/*{
    .com_mode = COM_MODE_TCP_STA,
    .host_ip = "192.168.1.100",
    .port_number = "8810",
    .ssid = "riot",
    .password = "",
};*/

uint8_t is_op_settings_valid = 0;
///< Flag that indicates if a valid op_settings has been read successfuly from flash

FILE *sd_card_save_file = NULL; ///< File where data is saved in SD card mode

/**
 * \brief scientisst-firmware main function
 *
 * Creates Mutex, initializes nvs, loads op_settings from flash, initializes
 * wifi, bluetooth, adc, i2c etc. If pin 1 is enabled, enters config mode. If
 * wifi mode, tries to connect to wifi network. If connection is unsuccessful,
 * enter config mode. Either start web server (config mode), TCP server (wifi
 * mode) or bluetooth. Starts send data task, adc task and adcExt task. If wifi
 * start rcv task; else, start battery task (adc2) task.
 */
void initScientisst(void)
{
    esp_err_t ret = ESP_OK;
    sdmmc_host_t *sd_card_spi_host = NULL;

    // Create a mutex type semaphore
    mutex_buffers_ready_to_send = xSemaphoreCreateMutex();
    CHECK_NOT_NULL(mutex_buffers_ready_to_send);

    // Init nvs (Non volatile storage)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (!getOpSettingsInfo(&op_settings))
    {
        is_op_settings_valid = 1;
    }

    // Determine and save device name in device_name
    getDeviceName();

    // Init GPIOs
    gpioInit();

    // Config LED control core
    configLedC();

    // Enable DAC
    dac_output_enable(DAC_CH);

    // Check if CONFIG pin is 1 on startup
    if (gpio_get_level(CONFIG_BTN_IO))
    {
        wifiInit(1);
        opModeConfig();
    }

    // Initialize acquisition timer
    timerGrpInit(TIMER_GROUP_USED, TIMER_IDX_USED, &timerGrp0Isr);

    if (op_settings.com_mode == COM_MODE_BLE)
    {
        frame_buffer_length = GATTS_NOTIFY_LEN;
    }
    else
    {
        frame_buffer_length = MAX_BUFFER_SIZE;
    }

    // Allocate memory for send buffers+1 for firmware version buffer
    for (int i = 0; i < NUM_BUFFERS; i++)
    {
        frame_buffer[i] = (uint8_t *)malloc(frame_buffer_length * sizeof(uint8_t));
        CHECK_NOT_NULL(frame_buffer[i]);
    }

    // Init internal ADC
    initAdc(ADC_RESOLUTION, 1, !isComModeWifi());
#if _SD_CARD_ == SD_CARD_ENABLED
    init_sd_card_spi_bus(sd_card_spi_host);
#endif
    // Init external ADC
    adcExtInit(sd_card_spi_host);

#if _SD_CARD_ == SD_CARD_ENABLED
    uint8_t num_active_channels_ext_adc = _ADC_EXT_ == EXT_ADC_DISABLED ? num_extern_active_chs : 0;
    // Init SD card
    ret = initSDCard(num_active_channels_ext_adc, sd_card_save_file);
    if (ret != ESP_OK)
    {
        DEBUG_PRINT_E("SD", "SD card initialization failed. Changing to BT mode.");
        op_settings.com_mode = COM_MODE_BT;
    }
#endif

#if _IMU_ == IMU_ENABLED
    // Init and start in new task the IMU
    xTaskCreatePinnedToCore(&bno055_task, "imu_task", 4096, NULL, ACQ_ADC1_PRIORITY, &imu_task, 0);
#endif

    // If it's a wifi com mode, let's first try to setup the wifi and (if it's station) try to connect to the
    // access point. If it doesn't work, enter immediatly to config mode in order for the user to update SSID
    // and password
    if (isComModeWifi())
    {
        // If the wifi configuration fails, start softap to enable the user to change op_settings
        if (wifiInit(0) == ESP_FAIL)
        {
            wifi_init_softap();
            opModeConfig();
        }
    }

    switch (op_settings.com_mode)
    {
    case COM_MODE_BT:
        initBt();
        xTaskCreatePinnedToCore(&task_battery_monitor, "task_battery_monitor", DEFAULT_TASK_STACK_SIZE, NULL,
                                ABAT_PRIORITY, &abat_task, 0);
        break;
    case COM_MODE_BLE:
        initBle();
        xTaskCreatePinnedToCore(&task_battery_monitor, "task_battery_monitor", DEFAULT_TASK_STACK_SIZE, NULL,
                                ABAT_PRIORITY, &abat_task, 0);
        break;
    case COM_MODE_UDP_STA:
        xTaskCreatePinnedToCore(&rcvTask, "rcvTask", 4096, NULL, WIFI_RCV_PRIORITY, &rcv_task, 0);
        break;
    case COM_MODE_TCP_STA:
        xTaskCreatePinnedToCore(&rcvTask, "rcvTask", 4096, NULL, WIFI_RCV_PRIORITY, &rcv_task, 0);
        break;
    case COM_MODE_TCP_AP:

        xTaskCreatePinnedToCore(&rcvTask, "rcvTask", 4096, NULL, WIFI_RCV_PRIORITY, &rcv_task, 0);
        break;
    case COM_MODE_WS_AP:
        start_webserver();
        break;
    case COM_MODE_SERIAL:
        xTaskCreatePinnedToCore(&rcvTask, "rcvTask", 4096, NULL, WIFI_RCV_PRIORITY, &rcv_task, 0);
        break;
#if _SD_CARD_ == SD_CARD_ENABLED
    case COM_MODE_SD_CARD:
        xTaskCreatePinnedToCore(&task_battery_monitor, "task_battery_monitor", DEFAULT_TASK_STACK_SIZE,
                                (void *)&num_extern_active_chs, ABAT_PRIORITY, &abat_task, 0);
        xTaskCreatePinnedToCore(&acquisitionSDCard, "acqSDCard", 4096 * 2, NULL, 24, &acq_adc1_task, 1);
        return;
#endif
    default:
        DEBUG_PRINT_E("COM", "Invalid communication mode.");
        abort();
        break;
    }

    xTaskCreatePinnedToCore(&sendTask, "sendTask", 4096 * 4, NULL, BT_SEND_PRIORITY, &send_task, 0);
    xTaskCreatePinnedToCore(&task_acquisition, "task_acquisition", 4096, NULL, ACQ_ADC1_PRIORITY,
                            &acq_adc1_task, 1);
}

/**
 * \brief Puts the device in config mode.
 *
 * This function puts the device in config mode. It initializes the rest server
 * and turns on the state led (white).
 */
void opModeConfig(void)
{
    op_mode = OP_MODE_CONFIG;
    initRestServer();
    gpio_set_level(STATE_LED_R_IO, 1);
    gpio_set_level(STATE_LED_G_IO, 1);
    gpio_set_level(STATE_LED_B_IO, 1);

    // Hang here until user successfully submites a new config in the web page
    while (1)
    {
        vTaskDelay(portMAX_DELAY);
    }
}
