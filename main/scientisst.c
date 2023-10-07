/** \file scientisst.c
    \brief Main file for the Scientisst firmware.

    This file is the main file for the Scientisst firmware. It handles the
   initialization of the firmware and the creation of the tasks.
*/

#include "scientisst.h"

#include <math.h>

#include "adc.h"
#include "ble.h"
#include "bt.h"
#include "com.h"
#include "config.h"
#include "esp_vfs_fat.h"
#include "gpio.h"
#include "imu.h"
#include "macros.h"
#include "macros_conf.h"
#include "sd_card.h"
#include "sdkconfig.h"
#include "sdmmc_cmd.h"
#include "spi.h"
#include "tcp.h"
#include "timer.h"
#include "uart.h"
#include "udp.h"
#include "wifi.h"
#include "wifi_rest_server.h"
#include "ws.h"

#define BT_SEND_PRIORITY 10 // MAX priority in ESP32 is 25
#define ABAT_PRIORITY 1
#define WIFI_RCV_PRIORITY 1
#define ACQ_ADC1_PRIORITY 10

// Functions have to be declared with void* as argument even though it is not used to avoid compiler warnings
void IRAM_ATTR sendTask(void *not_used);
void rcvTask(void *not_used);
void IRAM_ATTR AbatTask(void *not_used);
void IRAM_ATTR acqAdc1Task(void *not_used);
void IRAM_ATTR acqAdcExtTask(void *not_used);
void opModeConfig(void);

TaskHandle_t send_task;
TaskHandle_t abat_task;
TaskHandle_t rcv_task;
TaskHandle_t acq_adc1_task;
TaskHandle_t imu_task;

// Mutex for the buffers
SemaphoreHandle_t bt_buffs_to_send_mutex;

sdmmc_host_t sd_spi_host = {
    .flags = SDMMC_HOST_FLAG_SPI | SDMMC_HOST_FLAG_DEINIT_ARG,
    .slot = SDSPI_DEFAULT_HOST, // TODO: SPI3_HOST
#if _ADC_EXT_ != EXT_ADC_DISABLED
    .max_freq_khz = (80 * 1000) / 64,
#else
    .max_freq_khz = SDMMC_FREQ_DEFAULT * 2,
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

// BT
char device_name[17] = BT_DEFAULT_DEVICE_NAME;

// COM
int send_fd = 0;

// TCP server
int listen_fd = 0;

uint8_t *snd_buff[NUM_BUFFERS]; ///< Data structure to hold the data to be sentvthrough bluetooth
uint32_t send_buff_len = 0;     ///< Length of each send buff
uint16_t snd_buff_idx[NUM_BUFFERS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
///< It contains, for each buffer, the index of the first free element in the respective buffer
uint8_t bt_buffs_to_send[NUM_BUFFERS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
///< If element 0 is set to 1, bt task has to send snd_buff[0]
uint8_t bt_curr_buff = 0;  ///< Index of the buffer that bt task is currently sending
uint8_t acq_curr_buff = 0; ///< Index of the buffer that adc task is currently using

uint8_t packet_size = 0; ///< Current packet size (dependent on number of channels used)
uint8_t send_busy = 0;
uint16_t send_threshold = MAX_BUFFER_SIZE;
///< Amount of bytes a buffer needs to have filled with packets in order to trigger a send

DRAM_ATTR const uint8_t crc_table[16] = {0, 3, 6, 5, 12, 15, 10, 9, 11, 8, 13, 14, 7, 4, 1, 2};
uint16_t crc_seq = 0;

uint16_t battery_threshold = DEFAULT_BATTERY_THRESHOLD;

Api_Config api_config = {.api_mode = API_MODE_BITALINO,
                         .aquire_func = &acquireAdc1Channels,
                         .select_ch_mask_func = &selectChsFromMask};
cJSON *json = NULL;

esp_err_t (*send_func)(uint32_t, int, uint8_t *) = NULL; ///< Send function pointer

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
    .com_mode = COM_MODE_BT,
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

// Firmware version
uint8_t sd_card_present = 0; ///< Flag that indicates if SD card is present

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
    // Create a mutex type semaphore
    if ((bt_buffs_to_send_mutex = xSemaphoreCreateMutex()) == NULL)
    {
        DEBUG_PRINT_E("xSemaphoreCreateMutex", "Mutex creation failed");
        abort();
    }

    // Init nvs (Non volatile storage)
    esp_err_t ret = nvs_flash_init();
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
    else
    {
        is_op_settings_valid = 0;
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

    xTaskCreatePinnedToCore(&sendTask, "sendTask", 4096 * 4, NULL, BT_SEND_PRIORITY, &send_task, 0);

    // If op_mode is Wifi or Serial
    if (isComModeWifi() || !strcmp(op_settings.com_mode, COM_MODE_SERIAL))
    {
        if (!strcmp(op_settings.com_mode, COM_MODE_WS_AP))
        {
            DEBUG_PRINT_W("WS", "Starting web server");
            start_webserver();
            send_func = &wsSerialSend;
        }
        else
        {
            if (!strcmp(op_settings.com_mode, COM_MODE_TCP_AP))
            {
                if ((listen_fd = initTcpServer(op_settings.port_number)) < 0)
                {
                    DEBUG_PRINT_E("serverTCP",
                                  "Cannot init TCP Server on port %s specified "
                                  "in the configuration. Redo the "
                                  "configuration. Trying again...",
                                  op_settings.port_number);
                }
            }
            xTaskCreatePinnedToCore(&rcvTask, "rcvTask", 4096, NULL, WIFI_RCV_PRIORITY, &rcv_task, 0);
        }
    }
    else // Wifi is mutually exclusive with ADC2
    {
        xTaskCreatePinnedToCore(&AbatTask, "AbatTask", DEFAULT_TASK_STACK_SIZE, NULL, ABAT_PRIORITY,
                                &abat_task, 0);
    }

#if _IMU_ == IMU_ENABLED
    xTaskCreatePinnedToCore(&bno055_task, "imu_task", 4096, NULL, ACQ_ADC1_PRIORITY, &imu_task, 0);
#endif

#if _SD_CARD_ == SD_CARD_ENABLED
    xTaskCreatePinnedToCore(&acquisitionSDCard, "acqSDCard", 4096 * 2, NULL, 24, &acq_adc1_task, 1);
#else
    xTaskCreatePinnedToCore(&acqAdc1Task, "acqAdc1Task", 4096 * 2, NULL, ACQ_ADC1_PRIORITY, &acq_adc1_task,
                            1);
#endif
}

/**
 * \brief Task that sends data to the client.
 *
 * This task is responsible for sending data to the client. It is notified by
 * the acqAdc1Task when there is data to send. If the client is BT or BLE it
 * initializes the respective component.
 *
 * This task can be removed and acqAdc1Task can do the sendData() when
 * !send_busy. But, atm acqAdc1Task is the bottleneck
 */
void IRAM_ATTR sendTask(void *not_used)
{
    void (*send_data_func)(void) = &sendData;
    if (!strcmp(op_settings.com_mode, COM_MODE_BT))
    {
        initBt();
        send_func = &esp_spp_write;
        send_data_func = &sendDataBluetooth;
    }
    else if (!strcmp(op_settings.com_mode, COM_MODE_BLE))
    {
        initBle();
        send_func = &sendBle;
    }

    else if (!strcmp(op_settings.com_mode, COM_MODE_SD_CARD))
    {

#if _SD_CARD_ == SD_CARD_ENABLED
        if (initSDCard() != ESP_OK)
        {
            vTaskDelete(acq_adc1_task);
            DEBUG_PRINT_E("SD_CARD", "Cannot init SD Card, changing to BT mode");
            xTaskCreatePinnedToCore(&acqAdc1Task, "acqAdc1Task", 4096, NULL, ACQ_ADC1_PRIORITY,
                                    &acq_adc1_task, 1);
            initBt();
            send_func = &esp_spp_write;
            send_data_func = &sendDataBluetooth;
        }
        else
        {
            sd_card_present = 1;
            startAcquisitionSDCard();
        }
#else
        DEBUG_PRINT_E(
            "SD_CARD",
            "SD Card is not enabled in config.h but set as com mode in op settings. Changing to BT mode");
        initBt();
        send_func = &esp_spp_write;
        send_data_func = &sendDataBluetooth;
#endif
    }
#if _SD_CARD_ == SD_CARD_ENABLED
    if (sd_card_present)
        vTaskDelete(NULL);
#endif

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        send_data_func();
    }
}

/**
 * \brief Task that receives data from the client.
 *
 * This task is responsible for receiving data from the client.
 */
void rcvTask(void *not_used)
{
    while (1)
    {
        if (sd_card_present)
            vTaskDelete(NULL);

        if (!strcmp(op_settings.com_mode, COM_MODE_TCP_STA)) // TCP client
        {
            while ((send_fd = initTcpClient(op_settings.host_ip, op_settings.port_number)) < 0)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                DEBUG_PRINT_E("rcvTask",
                              "Cannot connect to TCP Server %s:%s specified in the "
                              "configuration. Redo the configuration. Trying again...",
                              op_settings.host_ip, op_settings.port_number);
            }
            send_func = &tcpSerialSend;
        }
        else if (!strcmp(op_settings.com_mode, COM_MODE_TCP_AP)) // TCP server
        {
            while ((send_fd = initTcpConnection(listen_fd)) < 0)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                DEBUG_PRINT_E("rcvTask",
                              "Cannot connect to TCP Server %s:%s specified in the "
                              "configuration. Redo the configuration. Trying again...",
                              op_settings.host_ip, op_settings.port_number);
            }
            send_func = &tcpSerialSend;
        }
        else if (!strcmp(op_settings.com_mode, COM_MODE_UDP_STA)) // UDP client
        {
            while ((send_fd = initUdpClient(op_settings.host_ip, op_settings.port_number)) < 0)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                DEBUG_PRINT_E("rcvTask",
                              "Cannot connect to UDP Server %s:%s specified in the "
                              "configuration. Redo the configuration. Trying again...",
                              op_settings.host_ip, op_settings.port_number);
            }
            send_func = &udpSend;
        }
        else if (!strcmp(op_settings.com_mode, COM_MODE_SERIAL)) // Serial
        {
            while ((send_fd = serialInit()) < 0)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                DEBUG_PRINT_E("rcvTask", "Cannot connect to host USB Serial port. Trying again...");
            }
            send_func = &tcpSerialSend;
        }
        wifiSerialRcv();
    }
}

/**
 * \brief Task that acquires data from adc1.
 *
 * This task is responsible for acquiring data from adc1. It is notified by the
 * timerGrp0Isr when to acquire data. It notifies the sendTask when there is
 * data to send. It is also the main task of CPU1 (APP CPU)
 */
void IRAM_ATTR acqAdc1Task(void *not_used)
{
    int acq_next_buff = 0;

    // Init Timer 0_1 (timer 1 from group 0) and register it's interupt handler
    timerGrpInit(TIMER_GROUP_USED, TIMER_IDX_USED, timerGrp0Isr);

    // Config all possible adc channels
    initAdc(ADC_RESOLUTION, 1, !isComModeWifi());

    if (!strcmp(op_settings.com_mode, COM_MODE_BLE))
    {
        send_buff_len = GATTS_NOTIFY_LEN;
    }
    else
    {
        send_buff_len = MAX_BUFFER_SIZE;
    }

    // Allocate memory for send buffers+1 for firmware version buffer
    for (int i = 0; i < NUM_BUFFERS; i++)
    {
        snd_buff[i] = (uint8_t *)malloc(send_buff_len * sizeof(uint8_t));
        if (snd_buff[i] == NULL)
        {
            DEBUG_PRINT_E("malloc", "Error allocating memory for send buffers");
            exit(-1);
        }
    }

    while (1)
    {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
        {
            api_config.aquire_func(snd_buff[acq_curr_buff] + snd_buff_idx[acq_curr_buff]);
            snd_buff_idx[acq_curr_buff] += packet_size;

            // Check if acq_curr_buff is above send_threshold and consequently send acq_curr_buff. If we send
            // acq_curr_buff, we need to update it
            if (snd_buff_idx[acq_curr_buff] >= send_threshold)
            {
                // Tell bt task that it has acq_curr_buff to send (but it will only send after the buffer is
                // filled above the threshold)
                xSemaphoreTake(bt_buffs_to_send_mutex, portMAX_DELAY);
                bt_buffs_to_send[acq_curr_buff] = 1;
                xSemaphoreGive(bt_buffs_to_send_mutex);

                // If send task is idle, wake it up
                if (send_busy == 0)
                {
                    xTaskNotifyGive(send_task);
                }

                acq_next_buff = (acq_curr_buff + 1) % (NUM_BUFFERS - 1);

                // Check if next buffer is full. If this happens, it means all buffers are full and bt task
                // can't handle this sending throughput
                if (bt_buffs_to_send[acq_next_buff] == 1)
                {
                    DEBUG_PRINT_W("acqAdc1Task", "Sending buffer is full, cannot acquire");
                    do // Wait until next buffer is free
                    {
                        if (send_busy == 0)
                        {
                            xTaskNotifyGive(send_task);
                        }
                        vTaskDelay(10 / portTICK_PERIOD_MS); // TODO: test with 1 MS
                    } while (bt_buffs_to_send[acq_next_buff] == 1);
                }
                acq_curr_buff = acq_next_buff;
            }
        }
        else
        {
            DEBUG_PRINT_W("acqAdc1", "ulTaskNotifyTake timed out!");
        }
    }
}

/**
 * \brief Task that acquires data from adc2 (battery).
 *
 * This task is responsible for acquiring data from adc2 (battery). It is
 * notified by the timerGrp1Isr when to acquire data. It notifies the sendTask
 * when there is data to send. It is also the main task of CPU0 (PRO CPU)
 */
void IRAM_ATTR AbatTask(void *not_used)
{
    uint16_t raw;
    uint16_t abat; // Battery voltage in mV
    uint8_t bat_led_status_gpio = 0;
    uint8_t turn_led_on; // Flag that indicates wether the bat_led_status should turn on or not

    // Init Timer 1_0 (timer 0 from group 1) and register it's interupt handler
    timerGrpInit(TIMER_GRP_ABAT, TIMER_IDX_ABAT, timerGrp1Isr);
    timerStart(TIMER_GRP_ABAT, TIMER_IDX_ABAT, (uint32_t)ABAT_CHECK_FREQUENCY);

    if (op_settings.is_battery_threshold_inflated == 1)
    {
        battery_threshold += 50;
        bat_led_status_gpio = 1;
    }

    while (1)
    {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
        {
            if (adc2_get_raw(ABAT_ADC_CH, ADC_RESOLUTION, (int *)&raw) != ESP_OK)
            {
                DEBUG_PRINT_E("adc2_get_raw", "Error!");
                continue;
            }
            abat = esp_adc_cal_raw_to_voltage((uint32_t)raw, &adc2_chars) * ABAT_DIVIDER_FACTOR;

            turn_led_on = abat <= battery_threshold;

            if (!bat_led_status_gpio && turn_led_on)
            {
                // Inflate threshold so that it doesn't blink due to abat oscilations in the edge of the
                // threshold
                battery_threshold += 50;
                op_settings.is_battery_threshold_inflated = 0;
                saveOpSettingsInfo(&op_settings);
            }
            else if (bat_led_status_gpio && !turn_led_on)
            {
                // It already charged passed the real threshold, so update the battery_threshold to its real
                // value
                battery_threshold -= 50;
                op_settings.is_battery_threshold_inflated = 0;
                saveOpSettingsInfo(&op_settings);
            }

            bat_led_status_gpio = turn_led_on;

            gpio_set_level(BAT_LED_STATUS_IO, bat_led_status_gpio);
        }
        else
        {
            DEBUG_PRINT_W("AbatTask", "ulTaskNotifyTake timed out!");
        }
    }
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
