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
#include "sci_task_aquisition.h"
#include "sci_task_battery_monitor.h"
#include "sci_task_com_rx.h"
#include "sci_task_com_tx.h"
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

_Noreturn void opModeConfig(void);

TaskHandle_t send_task;
TaskHandle_t abat_task;
TaskHandle_t rcv_task;
TaskHandle_t acq_adc1_task;
TaskHandle_t imu_task;

DRAM_ATTR scientisst_device_t scientisst_device_settings = {
    .device_name = BT_DEFAULT_DEVICE_NAME,
    .battery_threshold = DEFAULT_BATTERY_THRESHOLD,
    .gpio_out_state = {0, 0},
    .op_mode = OP_MODE_IDLE,
    .sample_rate = DEFAULT_SAMPLE_RATE,
    .num_intern_active_chs = 0,
    .num_extern_active_chs = 0,
    .active_internal_chs = {0, 0, 0, 0, 0, 0},
    .analog_channels = {ADC1_CHANNEL_0, ADC1_CHANNEL_3, ADC1_CHANNEL_4, ADC1_CHANNEL_5, ADC1_CHANNEL_6, ADC1_CHANNEL_7},
    .active_ext_chs = {0, 0},
    .api_config = {.api_mode = API_MODE_BITALINO, .select_ch_mask_func = &selectChsFromMask},
    .is_op_settings_valid = 0,
    .send_busy = 0,
    .op_settings =
        {
#if _SD_CARD_ == SD_CARD_ENABLED
            .com_mode = COM_MODE_SD_CARD,
#else
            .com_mode = DEFAULT_COM_MODE,
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
    .frame_buffer_write_idx = {0},
    .frame_buffer_ready_to_send = {0},
    .tx_curr_buff = 0,
    .acq_curr_buff = 0,
    .packet_size = 0,
    .send_threshold = MAX_BUFFER_SIZE,
    .sd_card_save_file = NULL,
    .json = NULL,
};

static void allocate_frame_buffers(void)
{
    // Allocate memory for send buffers+1 for firmware version and status packet
    for (int i = 0; i < NUM_BUFFERS; i++)
    {
        scientisst_buffers.frame_buffer[i] =
            (uint8_t *)malloc(scientisst_buffers.frame_buffer_length_bytes * sizeof(uint8_t));
        CHECK_NOT_NULL(scientisst_buffers.frame_buffer[i]);
        memset(scientisst_buffers.frame_buffer[i], 0, scientisst_buffers.frame_buffer_length_bytes);
    }
}

/**
 * \brief Initializes the Scientisst device.
 *
 * This function initializes various components and settings of the Scientisst device,
 * including the mutex, NVS, device name, GPIOs, LED control, DAC, Wi-Fi, timers, ADCs,
 * SD card (if enabled), IMU (if enabled), communication modes, and memory allocation
 * for send buffers.
 *
 * \note This function is only called during device startup.
 *
 * \return None.
 *
 * \par [in] None.
 * \par [out] None.
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

    if (!getOpSettingsInfo(&(scientisst_device_settings.op_settings)))
    {
        scientisst_device_settings.is_op_settings_valid = 1;
    }

    // Determine and save device name in device_name
    getDeviceName();

    // Init GPIOs
    gpioInit();

    // Config LED control core
    configLedC();

    // Enable DAC
    dac_output_enable(DAC_CH);

    DEBUG_PRINT_E("CONFIG BUTTON VALUE", "%d", gpio_get_level(CONFIG_BTN_IO));
    // Check if CONFIG pin is 1 on startup
    if (gpio_get_level(CONFIG_BTN_IO))
    {
        wifiInit(1);
        opModeConfig();
    }

    // Initialize acquisition timer
    timerGrpInit(TIMER_GROUP_USED, TIMER_IDX_USED, &timerGrp0Isr);

    // Init internal ADC
    initAdc(ADC_RESOLUTION, 1, !isComModeWifi());

#if _SD_CARD_ == SD_CARD_ENABLED
    // If SD card is enabled, ext adc has to be added to the spi bus before the sd card
    sdmmc_host_t *sd_card_spi_host = NULL;
    sd_card_spi_host = init_sd_card_spi_bus();
    adcExtInit(sd_card_spi_host);
    scientisst_device_settings.num_extern_active_chs = _ADC_EXT_ == EXT_ADC_ENABLED ? NUMBER_EXT_ADC_CHANNELS : 0;
    // Init SD card
    ret = initSDCard(scientisst_device_settings.num_extern_active_chs, &(scientisst_buffers.sd_card_save_file));
    if (ret != ESP_OK)
    {
        DEBUG_PRINT_E("SD", "SD card initialization failed. Changing to BT mode.");
        scientisst_device_settings.op_settings.com_mode = COM_MODE_BT;
    }
#endif

#if _IMU_ == IMU_ENABLED
    // Init and start in new task the IMU
    xTaskCreatePinnedToCore((TaskFunction_t)&bno055_task, "imu_task", 4096, NULL, ACQ_ADC1_PRIORITY, &imu_task, 0);
#endif

    // If it's a Wi-Fi com mode, let's first try to set up the Wi-Fi and (if it's station) try to connect to the
    // access point. If it doesn't work, enter immediately to config mode in order for the user to update SSID
    // and password
    if (isComModeWifi() && (wifiInit(0) == ESP_FAIL))
    {
        wifi_init_softap();
        opModeConfig();
    }

    switch (scientisst_device_settings.op_settings.com_mode)
    {
    case COM_MODE_BT:
        scientisst_buffers.frame_buffer_length_bytes = MAX_BUFFER_SIZE;
        allocate_frame_buffers();
        initBt();
        xTaskCreatePinnedToCore((TaskFunction_t)&task_battery_monitor, "task_battery_monitor", DEFAULT_TASK_STACK_SIZE, NULL,
                                ABAT_PRIORITY, &abat_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&sendTask, "sendTask", 4096 * 4, NULL, BT_SEND_PRIORITY, &send_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&task_acquisition, "task_acquisition", 4096, NULL, ACQ_ADC1_PRIORITY,
                                &acq_adc1_task, 1);
        break;
    case COM_MODE_BLE:
        scientisst_buffers.frame_buffer_length_bytes = GATTS_NOTIFY_LEN;
        allocate_frame_buffers();
        initBle();
        xTaskCreatePinnedToCore((TaskFunction_t)&task_battery_monitor, "task_battery_monitor", DEFAULT_TASK_STACK_SIZE, NULL,
                                ABAT_PRIORITY, &abat_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&sendTask, "sendTask", 4096 * 4, NULL, BT_SEND_PRIORITY, &send_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&task_acquisition, "task_acquisition", 4096, NULL, ACQ_ADC1_PRIORITY,
                                &acq_adc1_task, 1);
        break;
    case COM_MODE_UDP_STA:
    case COM_MODE_TCP_STA:
    case COM_MODE_TCP_AP:
    case COM_MODE_SERIAL:
        scientisst_buffers.frame_buffer_length_bytes = MAX_BUFFER_SIZE;
        allocate_frame_buffers();
        xTaskCreatePinnedToCore((TaskFunction_t)&rcvTask, "rcvTask", 4096, NULL, WIFI_RCV_PRIORITY, &rcv_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&sendTask, "sendTask", 4096 * 4, NULL, BT_SEND_PRIORITY, &send_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&task_acquisition, "task_acquisition", 4096, NULL, ACQ_ADC1_PRIORITY,
                                &acq_adc1_task, 1);
        break;
    case COM_MODE_WS_AP:
        scientisst_buffers.frame_buffer_length_bytes = MAX_BUFFER_SIZE;
        allocate_frame_buffers();
        start_webserver();
        xTaskCreatePinnedToCore((TaskFunction_t)&rcvTask, "rcvTask", 4096, NULL, WIFI_RCV_PRIORITY, &rcv_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&sendTask, "sendTask", 4096 * 4, NULL, BT_SEND_PRIORITY, &send_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&task_acquisition, "task_acquisition", 4096, NULL, ACQ_ADC1_PRIORITY,
                                &acq_adc1_task, 1);
        break;
#if _SD_CARD_ != SD_CARD_ENABLED
    case COM_MODE_SD_CARD:
        scientisst_buffers.frame_buffer_length_bytes = MAX_BUFFER_SIZE_SDCARD;
        allocate_frame_buffers();
        xTaskCreatePinnedToCore((TaskFunction_t)&task_battery_monitor, "task_battery_monitor", DEFAULT_TASK_STACK_SIZE, NULL,
                                ABAT_PRIORITY, &abat_task, 0);
        xTaskCreatePinnedToCore((TaskFunction_t)&acquisitionSDCard, "acqSDCard", 4096 * 2, NULL, 24, &acq_adc1_task, 1);
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
_Noreturn void opModeConfig(void)
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
