/**
 * \file sci_bt.c
 * \brief Bluetooth communication and general functions.
 *
 * This file contains the functions necessary for Bluetooth communication.
 */

#include "sci_bt.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "sci_com.h"

#define SEND_AFTER_C0NG_WRITE_FAILED 3
///< Marks send status to wait for congestion to clear and then send the rest of the data of a failed write
#define SEND_AFTER_C0NG 2            ///< Marks send status to wait for congestion to clear and then send the data

DRAM_ATTR SemaphoreHandle_t bt_send_semaphore;
///< Semaphore to synchronize the send task with the event handler, wait for the write to complete and then send the next
///< buffer or wait for congestion to clear.
DRAM_ATTR volatile uint16_t failed_write_bytes_sent_count = 0; ///< Number of bytes sent in a failed write
DRAM_ATTR char sci_spp_server_name[42] = "SPP_SERVER_";

/**
 * \brief Send data via Bluetooth.
 *
 * This function is responsible for sending data over a Bluetooth connection. It's designed to work within the constraints of
 * event-driven communication, avoiding potential stack overflows from recursive calls found in other modes. It also uses a
 * binary semaphore to synchronize the send task with the event handler, wait for the write to complete and then send the
 * next buffer or wait for congestion to clear.
 *
 * \return None.
 */
void IRAM_ATTR sendDataBluetooth(esp_err_t (*tx_write_func)(uint32_t, int, const uint8_t *))
{
    uint16_t buf_ready;
    // Check if there's anything to send and if there is, check if it's enough
    // to send
    buf_ready = scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.tx_curr_buff];

    if (buf_ready == 0)
    {
        scientisst_device_settings.send_busy = 0;
        return;
    }

    if (scientisst_device_settings.op_mode != OP_MODE_LIVE && scientisst_buffers.tx_curr_buff != (NUM_BUFFERS - 1))
    {
        scientisst_device_settings.send_busy = 0;
        return;
    }

    scientisst_device_settings.send_busy = 1;
    esp_spp_write(send_fd, buf_ready, scientisst_buffers.frame_buffer[scientisst_buffers.tx_curr_buff]);
    xSemaphoreTake(bt_send_semaphore, portMAX_DELAY);
}

/**
 * \brief Finalize the sending process.
 *
 * Once data is sent, this function is responsible for clearing the buffer that held the data and updating the buffer
 * ready to send flag.
 *
 * \return None.
 */
void IRAM_ATTR finalizeSend(void)
{
    // Clear recently sent buffer
    memset(scientisst_buffers.frame_buffer[scientisst_buffers.tx_curr_buff], 0,
           scientisst_buffers.frame_buffer_length_bytes);

    scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.tx_curr_buff] = 0;

    // Change send buffer
    if (scientisst_buffers.tx_curr_buff != NUM_BUFFERS - 1)
        scientisst_buffers.tx_curr_buff = (scientisst_buffers.tx_curr_buff + 1) % (NUM_BUFFERS - 1);
    else
        scientisst_buffers.tx_curr_buff = 0;
}

/**
 * \brief Callback for Bluetooth events.
 *
 * This function is a callback for handling a variety of Bluetooth events, including initialization, data reception, and
 * others. Actions taken during this callback depend on the event triggered by the Bluetooth ESP Stack. It notifies the send
 * task when the write is completed successfully and when the connection is not congested.
 *
 * \return None.
 */
static void IRAM_ATTR espSppCb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event)
    {
    case ESP_SPP_INIT_EVT:
        DEBUG_PRINT_I("espSppCb", "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(scientisst_device_settings.device_name);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        strncat(sci_spp_server_name, scientisst_device_settings.device_name, strlen(sci_spp_server_name) - 1);
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, sci_spp_server_name);
        ESP_LOGE("Init Bluetooth", "Device online with the name: %s", scientisst_device_settings.device_name);
        break;
    case ESP_SPP_SRV_OPEN_EVT: // Server connection open (first client connection)
        DEBUG_PRINT_I("espSppCb", "ESP_SPP_SRV_OPEN_EVT");
        if (!send_fd)
        {
            send_fd = param->open.handle;
        }
        else
        {
            esp_spp_disconnect(param->open.handle);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        DEBUG_PRINT_I("espSppCb", "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT: // Client connection open
        DEBUG_PRINT_I("espSppCb", "ESP_SPP_OPEN_EVT");
        // We only want one client
        esp_spp_disconnect(param->open.handle);
        break;
    case ESP_SPP_CLOSE_EVT: // Client connection closed
        DEBUG_PRINT_I("espSppCb", "ESP_SPP_CLOSE_EVT");
        send_fd = 0;
        stopAcquisition(); // Make sure that sendBtTask doesn't stay stuck waiting for a successful write
        xSemaphoreGive(bt_send_semaphore);
        break;
    case ESP_SPP_START_EVT: // server started
        DEBUG_PRINT_I("espSppCb", "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT: // client initiated a connection
        DEBUG_PRINT_I("espSppCb", "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT: // connection received data
        DEBUG_PRINT_I("BT Event", "BT data received length:%d  data:", param->data_ind.len);
        processPacket(param->data_ind.data);
        break;
    case ESP_SPP_CONG_EVT: // connection congestion status changed
        if (param->cong.cong == 0)
        {
            if (scientisst_device_settings.send_busy == SEND_AFTER_C0NG_WRITE_FAILED)
            {
                scientisst_device_settings.send_busy = 1;
                esp_spp_write(send_fd,
                              scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.tx_curr_buff] -
                                  failed_write_bytes_sent_count,
                              scientisst_buffers.frame_buffer[scientisst_buffers.tx_curr_buff] +
                                  failed_write_bytes_sent_count);
            }
            else if (scientisst_device_settings.send_busy == SEND_AFTER_C0NG)
            {
                scientisst_device_settings.send_busy = 1;
                xSemaphoreGive(bt_send_semaphore);
            }
        }
        else if (param->cong.cong == 1)
        {
            scientisst_device_settings.send_busy = SEND_AFTER_C0NG;
            DEBUG_PRINT_W("espSppCb", "ESP_SPP_CONG_EVT");
        }
        break;
    case ESP_SPP_WRITE_EVT: // write operation status changed
        if (param->write.status == ESP_SPP_SUCCESS)
        {
            failed_write_bytes_sent_count = 0;
            finalizeSend();
            // Try to send next buff
            if (param->write.cong == 0) // bt write is free
            {
                scientisst_device_settings.send_busy = 1;
                xSemaphoreGive(bt_send_semaphore);
            }
            else
            {
                scientisst_device_settings.send_busy = SEND_AFTER_C0NG;
            }
        }
        else
        {
            failed_write_bytes_sent_count += (uint16_t)param->write.len;
            if (failed_write_bytes_sent_count <
                scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.tx_curr_buff])
            {
                if (param->write.cong == 0)
                {
                    scientisst_device_settings.send_busy = 1;
                    esp_spp_write(send_fd,
                                  scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.tx_curr_buff] -
                                      failed_write_bytes_sent_count,
                                  scientisst_buffers.frame_buffer[scientisst_buffers.tx_curr_buff] +
                                      failed_write_bytes_sent_count);
                    failed_write_bytes_sent_count = 0;
                }
                else
                {
                    scientisst_device_settings.send_busy = SEND_AFTER_C0NG;
                }
            }
            else
            {
                if (param->write.cong == 0)
                {
                    scientisst_device_settings.send_busy = 1;
                    xSemaphoreGive(bt_send_semaphore);
                }
                else
                {
                    scientisst_device_settings.send_busy = SEND_AFTER_C0NG;
                }
            }
        }
        break;
    default:
        break;
    }
}

/**
 * \brief Callback for Bluetooth GAP events.
 *
 * This function is a callback that handles events related to Bluetooth GAP (Generic Access Profile).
 *
 * \return None.
 */
static void espBtGapCb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            DEBUG_PRINT_I("espBtGapCb", "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex("SPP_ACCEPTOR", param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        }
        else
        {
            DEBUG_PRINT_E("espBtGapCb", "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT: {
        DEBUG_PRINT_I("espBtGapCb", "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit)
        {
            DEBUG_PRINT_I("espBtGapCb", "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        }
        else
        {
            DEBUG_PRINT_I("espBtGapCb", "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

    case ESP_BT_GAP_CFM_REQ_EVT:
        DEBUG_PRINT_I("espBtGapCb", "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        DEBUG_PRINT_I("espBtGapCb", "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        DEBUG_PRINT_I("espBtGapCb", "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;

    default: {
        DEBUG_PRINT_I("espBtGapCb", "event: %d", event);
        break;
    }
    }
    return;
}

/**
 * \brief Initialize Bluetooth communication.
 *
 * This function sets up the necessary components for Bluetooth communication. This includes initializing the Bluetooth
 * stack, setting up the necessary Bluetooth profiles, and ensuring the device is ready to pair and connect with other
 * Bluetooth devices.
 *
 * \return None.
 */
void initBt(void)
{
    esp_err_t ret;
    esp_bt_sp_param_t param_type_bt = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;

    bt_send_semaphore = xSemaphoreCreateBinary();
    CHECK_NOT_NULL(bt_send_semaphore);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK)
    {
        DEBUG_PRINT_E("init", "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM)) != ESP_OK)
    {
        DEBUG_PRINT_E("init", "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK)
    {
        DEBUG_PRINT_E("init", "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK)
    {
        DEBUG_PRINT_E("init", "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(espBtGapCb)) != ESP_OK)
    {
        DEBUG_PRINT_E("init", "%s gap register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(espSppCb)) != ESP_OK)
    {
        DEBUG_PRINT_E("init", "%s spp register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(ESP_SPP_MODE_CB)) != ESP_OK)
    {
        DEBUG_PRINT_E("init", "%s spp init failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    // Set device as having no I/O -> change SPP paring to just works
    if ((ret = esp_bt_gap_set_security_param(param_type_bt, &iocap, sizeof(uint8_t))) != ESP_OK)
    {
        DEBUG_PRINT_E("init", "%s change security params failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    DEBUG_PRINT_I("initBt", "Init Bluetooth completed");
}

/**
 * \brief Generate a unique device name.
 *
 * This function creates a unique device name based on the device's MAC address. This helps in identifying the device when
 * attempting to pair or connect via Bluetooth.
 *
 * \return None.
 */
void getDeviceName(void)
{
    uint8_t mac[6];
    uint8_t mac_interface[6];
    uint8_t mac_bt[6];
    char mac_interface_str[18];
    char mac_bt_str[18];

    char *mac_str_arr[2] = {mac_interface_str, mac_bt_str};
    int res;

    // Read base Mac address from EFUSE, which is used to calculate the MAC addr
    // for all the interfaces
    if (esp_efuse_mac_get_default(mac) != ESP_OK)
    {
        DEBUG_PRINT_E("BtTask", "Couldn't read MAC address from Efuse");
        return;
    }

    memcpy(mac_interface, mac, 6);
    memcpy(mac_bt, mac, 6);

    // Use esp_read_mac to infer the Wi-Fi station mac addr based in the base mac addr read from EFUSE
    if (scientisst_device_settings.op_settings.com_mode == COM_MODE_TCP_AP)
    {
        res = esp_read_mac(mac_interface, ESP_MAC_WIFI_SOFTAP);
        // Use esp_read_mac to infer the Wi-Fi softap mac addr based in the base mac addr read from EFUSE
    }
    else if (scientisst_device_settings.op_settings.com_mode == COM_MODE_TCP_STA ||
             scientisst_device_settings.op_settings.com_mode == COM_MODE_UDP_STA)
    {
        res = esp_read_mac(mac_interface, ESP_MAC_WIFI_STA);
        // Use esp_read_mac to infer the bluetooth mac addr based in the base mac addr read from EFUSE
    }
    else
    {
        res = esp_read_mac(mac_interface, ESP_MAC_BT);
    }

    if (res != ESP_OK || esp_read_mac(mac_bt, ESP_MAC_BT) != ESP_OK)
    {
        DEBUG_PRINT_E("BtTask", "Couldn't read MAC address from Efuse");
        return;
    }

    // Make mac address human-readable
    sprintf(mac_interface_str, "%x:%x:%x:%x:%x:%x", mac_interface[0], mac_interface[1], mac_interface[2], mac_interface[3],
            mac_interface[4], mac_interface[5]);
    sprintf(mac_bt_str, "%x:%x:%x:%x:%x:%x", mac_bt[0], mac_bt[1], mac_bt[2], mac_bt[3], mac_bt[4], mac_bt[5]);

    // Turn all characters of both mac address strings from lower case to upper case
    for (int pntr_idx = 0; pntr_idx < 2; pntr_idx++)
    {
        char *str = mac_str_arr[pntr_idx];
        for (int i = 0; str[i] != '\0'; i++)
        {
            char c = str[i];

            // If it's a lower-case letter
            if (c >= 97 && c <= 122)
            {
                c -= 32; // Make it upper-case
            }
            str[i] = c;
        }
    }
    sprintf(scientisst_device_settings.device_name, "%s-%s", BT_DEFAULT_DEVICE_NAME, (mac_bt_str + 12));

    DEBUG_PRINT_W("getDeviceName",
                  "Device name is: %s, COM Mode MAC address is: %s, Bluetooth "
                  "Classic MAC address is: %s",
                  scientisst_device_settings.device_name, mac_interface_str, mac_bt_str);
}
