/** \file scientisst.c
    \brief Main file for the Scientisst firmware.

    This file is the main file for the Scientisst firmware. It handles the
   initialization of the firmware and the creation of the tasks.
*/

#include "sci_scientisst.h"

#include <math.h>
#include <sys/cdefs.h>

#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "sdmmc_cmd.h"

#include "sci_adc_external.h"
#include "sci_adc_internal.h"
#include "sci_ble.h"
#include "sci_bt.h"
#include "sci_com.h"
#include "sci_gpio.h"
#include "sci_macros.h"
#include "sci_sd_card.h"
#include "sci_task_acquisition_sdcard.h"
#include "sci_task_aquisition.h"
#include "sci_task_battery_monitor.h"
#include "sci_task_com_rx.h"
#include "sci_task_com_tx.h"
#include "sci_task_imu.h"
#include "sci_timer.h"
#include "sci_wifi.h"
#include "sci_wifi_rest_server.h"
#include "sci_ws.h"

#define BT_SEND_PRIORITY 10 // MAX priority in ESP32 is 25
#define BATTERY_PRIORITY 1
#define WIFI_RCV_PRIORITY 1
#define ACQ_ADC1_PRIORITY 10

#define KEY_SETTINGS_INFO "opSettingsInfo" // key used in NVS for connection info
#define SCI_BOOTWIFI_NAMESPACE "bootwifi"  // namespace in NVS, used to store connection info and some other op settings

DRAM_ATTR TaskHandle_t send_task;
DRAM_ATTR TaskHandle_t battery_task;
DRAM_ATTR TaskHandle_t rcv_task;
DRAM_ATTR TaskHandle_t acq_adc1_task;
DRAM_ATTR TaskHandle_t imu_task;

DRAM_ATTR scientisst_device_t scientisst_device_settings = {
    .device_name = BT_DEFAULT_DEVICE_NAME,
    .battery_threshold = DEFAULT_BATTERY_THRESHOLD,
    .gpio_out_state = {0, 0},
    .op_mode = OP_MODE_IDLE,
    .sample_rate = DEFAULT_SAMPLE_RATE,
    .num_intern_active_chs = 0,
    .num_extern_active_chs = 0,
    .active_internal_chs = {0, 0, 0, 0, 0, 0},
    .active_ext_chs = {0, 0},
    .api_config = {.api_mode = API_MODE_BITALINO, .select_ch_mask_func = &selectChsFromMaskBitalino},
    .is_op_settings_valid = 0,
    .send_busy = 0,
    .op_settings =
        {
#ifdef CONFIG_SD_CARD
            .com_mode = COM_MODE_SD_CARD,
#else
#ifdef CONFIG_DEFAULT_COM_MODE_BT
            .com_mode = COM_MODE_BT,
#endif
#ifdef CONFIG_DEFAULT_COM_MODE_BLE
            .com_mode = COM_MODE_BLE,
#endif
#ifdef CONFIG_DEFAULT_COM_MODE_TCP_AP
            .com_mode = COM_MODE_TCP_AP,
#endif
#ifdef CONFIG_DEFAULT_COM_MODE_TCP_STA
            .com_mode = COM_MODE_TCP_STA,
#endif
#ifdef CONFIG_DEFAULT_COM_MODE_UDP_STA
            .com_mode = COM_MODE_UDP_STA,
#endif
#ifdef CONFIG_DEFAULT_COM_MODE_SERIAL
            .com_mode = COM_MODE_SERIAL,
#endif
#ifdef CONFIG_DEFAULT_COM_MODE_WS_AP
            .com_mode = COM_MODE_WS_AP,
#endif
#endif
            .is_battery_threshold_inflated = 0,
            .host_ip = "192.168.1.100",
            .port_number = "8800",
            .ssid = "riot",
            .password = "",
        },
};

// We use DMA_ATTR because some of the functions require that the buffers are word aligned
DMA_ATTR scientisst_buffers_t scientisst_buffers = {
    .frame_buffer = {0},
    .frame_buffer_length_bytes = 0,
    .frame_buffer_write_idx = 0,
    .frame_buffer_ready_to_send = {0},
    .tx_curr_buff = 0,
    .acq_curr_buff = 0,
    .packet_size = 0,
    .send_threshold = MAX_BUFFER_SIZE,
    .sd_card_save_file = NULL,
    .json = NULL,
};

static void allocateFrameBuffers(void)
{
    uint8_t num_buffers =
        scientisst_device_settings.op_settings.com_mode == COM_MODE_SD_CARD ? NUM_BUFFERS_SDCARD : NUM_BUFFERS;
    // Allocate memory for send buffers+1 for firmware version and status packet
    for (int i = 0; i < num_buffers; i++)
    {
        scientisst_buffers.frame_buffer[i] =
            (uint8_t *)malloc(scientisst_buffers.frame_buffer_length_bytes * sizeof(uint8_t));
        CHECK_NOT_NULL(scientisst_buffers.frame_buffer[i]);
        memset(scientisst_buffers.frame_buffer[i], 0, scientisst_buffers.frame_buffer_length_bytes);
        scientisst_buffers.frame_buffer_ready_to_send[i] = 0;
    }
}

_Noreturn static void opModeConfig(void);
static esp_err_t getOpSettingsInfo(op_settings_info_t *_op_settings);

/**
 * \brief Initializes the Scientisst device.
 *
 * This function initializes various components and settings of the Scientisst device,
 * including the mutex, NVS, device name, GPIOs, LED control, DAC, Wi-Fi, timers, ADCs,
 * SD card (if enabled), IMU (if enabled), communication modes, and memory allocation
 * for send buffers.
 *
 *
 * \return None.
 */
void initScientisst(void)
{
    esp_err_t ret = ESP_OK;

    // Init nvs (Non volatile storage)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (getOpSettingsInfo(&(scientisst_device_settings.op_settings)) == ESP_OK)
    {
        scientisst_device_settings.is_op_settings_valid = 1;
    }

    // Determine and save device name in device_name
    getDeviceName();

    // Init GPIOs
    gpioInit();

    // Config LED control core
    configLedController();

    // Enable DAC
    dac_output_enable(DAC_CH);

    // Wait for IO initializations to settle
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Working version 3.3.0-750b400

    // Check if CONFIG pin is pressed on startup
    if (gpio_get_level(CONFIG_BTN_IO))
    {
        wifiInit(1);
        opModeConfig();
    }

    // Initialize acquisition timer
    timerGrpInit(TIMER_GROUP_MAIN, TIMER_IDX_MAIN, &timerGrp0Isr);

    // Init internal ADC
    initAdc(1, !IS_COM_TYPE_WIFI(scientisst_device_settings.op_settings.com_mode));

#ifdef CONFIG_SD_CARD
#ifdef CONFIG_ADC_EXT
    // If SD card is enabled, ext adc has to be added to the spi bus before the sd card
    const sdmmc_host_t *sd_card_spi_host = NULL;
    sd_card_spi_host = initSdCardSpiBus();
    adcExtInit(sd_card_spi_host);
    scientisst_device_settings.num_extern_active_chs = CONFIG_NUMBER_CHANNELS_EXT_ADC;
#else
    initSdCardSpiBus();
    scientisst_device_settings.num_extern_active_chs = 0;
#endif

    // Init SD card
    ret = initSDCard();
    if (ret != ESP_OK)
    {
        DEBUG_PRINT_E("SD", "SD card initialization failed. Changing to BT mode.");
        scientisst_device_settings.op_settings.com_mode = COM_MODE_BT;
    }
#endif

#ifdef CONFIG_IMU
    // Init and start in new task the IMU
    ret = initIMU();
    ESP_ERROR_CHECK(ret);
    xTaskCreatePinnedToCore((TaskFunction_t)&taskBno055, "imu_task", DEFAULT_TASK_STACK_SIZE_MEDIUM, NULL, ACQ_ADC1_PRIORITY,
                            &imu_task, 0);
#endif

    // If it's a Wi-Fi com mode, let's first try to set up the Wi-Fi and (if it's station) try to connect to the
    // access point. If it doesn't work, enter immediately to config mode in order for the user to update SSID
    // and password
    if (IS_COM_TYPE_WIFI(scientisst_device_settings.op_settings.com_mode) && (wifiInit(0) == ESP_FAIL))
    {
        wifiInitSoftap();
        opModeConfig();
    }

    switch (scientisst_device_settings.op_settings.com_mode)
    {
    case COM_MODE_BT:
        scientisst_buffers.frame_buffer_length_bytes = MAX_BUFFER_SIZE;
        allocateFrameBuffers();
        initBt();
        xTaskCreatePinnedToCore((TaskFunction_t)&taskBatteryMonitor, "taskBatteryMonitor", DEFAULT_TASK_STACK_SIZE_SMALL,
                                NULL, BATTERY_PRIORITY, &battery_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&sendTask, "sendTask", DEFAULT_TASK_STACK_SIZE_XLARGE, NULL,
                                BT_SEND_PRIORITY, &send_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&taskAcquisition, "taskAcquisition", DEFAULT_TASK_STACK_SIZE_MEDIUM, NULL,
                                ACQ_ADC1_PRIORITY, &acq_adc1_task, 1);
        break;
    case COM_MODE_BLE:
        scientisst_buffers.frame_buffer_length_bytes = GATTS_NOTIFY_LEN;
        allocateFrameBuffers();
        initBle();
        xTaskCreatePinnedToCore((TaskFunction_t)&taskBatteryMonitor, "taskBatteryMonitor", DEFAULT_TASK_STACK_SIZE_SMALL,
                                NULL, BATTERY_PRIORITY, &battery_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&sendTask, "sendTask", DEFAULT_TASK_STACK_SIZE_XLARGE, NULL,
                                BT_SEND_PRIORITY, &send_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&taskAcquisition, "taskAcquisition", DEFAULT_TASK_STACK_SIZE_MEDIUM, NULL,
                                ACQ_ADC1_PRIORITY, &acq_adc1_task, 1);
        break;
    case COM_MODE_UDP_STA:
    case COM_MODE_TCP_STA:
    case COM_MODE_TCP_AP:
    case COM_MODE_SERIAL:
        scientisst_buffers.frame_buffer_length_bytes = MAX_BUFFER_SIZE;
        allocateFrameBuffers();
        xTaskCreatePinnedToCore((TaskFunction_t)&rxTask, "rxTask", DEFAULT_TASK_STACK_SIZE_MEDIUM, NULL, WIFI_RCV_PRIORITY,
                                &rcv_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&sendTask, "sendTask", DEFAULT_TASK_STACK_SIZE_XLARGE, NULL,
                                BT_SEND_PRIORITY, &send_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&taskAcquisition, "taskAcquisition", DEFAULT_TASK_STACK_SIZE_MEDIUM, NULL,
                                ACQ_ADC1_PRIORITY, &acq_adc1_task, 1);
        break;
    case COM_MODE_WS_AP:
        scientisst_buffers.frame_buffer_length_bytes = MAX_BUFFER_SIZE;
        allocateFrameBuffers();
        startWebserver();
        xTaskCreatePinnedToCore((TaskFunction_t)&rxTask, "rxTask", DEFAULT_TASK_STACK_SIZE_MEDIUM, NULL, WIFI_RCV_PRIORITY,
                                &rcv_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&sendTask, "sendTask", DEFAULT_TASK_STACK_SIZE_XLARGE, NULL,
                                BT_SEND_PRIORITY, &send_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&taskAcquisition, "taskAcquisition", DEFAULT_TASK_STACK_SIZE_MEDIUM, NULL,
                                ACQ_ADC1_PRIORITY, &acq_adc1_task, 1);
        break;
#ifdef CONFIG_SD_CARD
    case COM_MODE_SD_CARD:
        scientisst_buffers.frame_buffer_length_bytes = MAX_BUFFER_SIZE_SDCARD;
        allocateFrameBuffers();
        xTaskCreatePinnedToCore((TaskFunction_t)&taskBatteryMonitor, "task_battery_monitor", DEFAULT_TASK_STACK_SIZE_SMALL,
                                NULL, BATTERY_PRIORITY, &battery_task, 1);
        xTaskCreatePinnedToCore((TaskFunction_t)&acquisitionSDCard, "acqSDCard", DEFAULT_TASK_STACK_SIZE_LARGE, NULL, 24,
                                &acq_adc1_task, 1);
        break;
#endif
    default:
        DEBUG_PRINT_E("COM", "Invalid communication mode.");
        abort();
    }
}

/**
 * \brief Puts the device in config mode.
 *
 * This function puts the device in config mode. It initializes the rest server
 * and sets the state led to fixed white.
 */
_Noreturn static void opModeConfig(void)
{
    scientisst_device_settings.op_mode = OP_MODE_CONFIG;
    initRestServer();
    gpio_set_level(STATE_LED_R_IO, 1);
    gpio_set_level(STATE_LED_G_IO, 1);
    gpio_set_level(STATE_LED_B_IO, 1);

    // Hang here until user successfully submits a new config in the web page
    while (1)
    {
        vTaskDelay(portMAX_DELAY);
    }
}

/**
 * \brief Read the operational settings from NVS
 *
 * \param _op_settings pointer to the operational settings structure
 *
 * \return:
 *     - ESP_OK if successful
 *     - ESP_FAIL if unsuccessful
 */
static esp_err_t getOpSettingsInfo(op_settings_info_t *_op_settings)
{
    nvs_handle handle;
    size_t size;
    esp_err_t ret = ESP_OK;
    op_settings_info_t pOpSettingsInfo;

    ret = nvs_open(SCI_BOOTWIFI_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK)
    {
        DEBUG_PRINT_E("getOpSettingsInfo", "ERROR: opening NVS");
        return ESP_FAIL;
    }

    size = sizeof(op_settings_info_t);
    ret = nvs_get_blob(handle, KEY_SETTINGS_INFO, &pOpSettingsInfo, &size);
    if (ret != ESP_OK)
    {
        DEBUG_PRINT_W("getOpSettingsInfo", "No op settings record found!");
        nvs_close(handle);
        return ESP_FAIL;
    }

    // cleanup
    nvs_close(handle);

    *_op_settings = pOpSettingsInfo;
    return ESP_OK;
}

/**
 * \brief Save the operational settings to NVS
 *
 * \param pOpSettingsInfo pointer to the operational settings structure
 *
 */
void saveOpSettingsInfo(const op_settings_info_t *pOpSettingsInfo)
{
    nvs_handle handle;
    ESP_ERROR_CHECK(nvs_open(SCI_BOOTWIFI_NAMESPACE, NVS_READWRITE, &handle));
    ESP_ERROR_CHECK(nvs_set_blob(handle, KEY_SETTINGS_INFO, pOpSettingsInfo, sizeof(op_settings_info_t)));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);
}
