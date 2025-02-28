/**
 * \file ble.c
 * \brief Bluetooth Low Energy (BLE) mode functions.
 *
 * This file contains the functions for BLE mode, including initialization, callbacks, and utility functions, as well as
 * relevant macros.
 */

#include "sci_ble.h"

#include <stdlib.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_spp_api.h"
#include "esp_system.h"

#include "sci_com.h"

#define GATTS_SERVICE_UUID_TEST_A 0x00FF
#define GATTS_CHAR_UUID_TEST_A 0xFF01
#define GATTS_NUM_HANDLE_TEST_A 4

#define GATTS_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t *param);

static uint8_t char1_str[] = {0x11, 0x22, 0x33};
static esp_gatt_char_prop_t a_property = 0;

static esp_attr_value_t gatts_char1_val = {
    .attr_max_len = GATTS_CHAR_VAL_LEN_MAX,
    .attr_len = sizeof(char1_str),
    .attr_value = char1_str,
};

static uint8_t adv_config_done = 0;

static uint8_t adv_service_uuid128[32] = {
    /* LSB <-------------------------------------------------------------------------------->MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xEE,
    0x00,
    0x00,
    0x00,
    // second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xFF,
    0x00,
    0x00,
    0x00,
};

// The length of adv data must be less than 31 bytes
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x000C, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x000C,
    .appearance = 0x00,
    .manufacturer_len = 0, // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the
 * gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] =
        {
            .gatts_cb = gatts_profile_a_event_handler, .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is
                                                                                        ESP_GATT_IF_NONE */
        },
};

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;

/**
 * \brief BLE GAP event handler.
 *
 * Handles GAP events such as advertisement data set completion, advertisement start, advertisement stop, and connection
 * parameter update.
 *
 * \param[in] event Type of GAP event.
 * \param[in] param Parameters for the GAP event.
 *
 * \return None.
 */
static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // advertising start complete event to indicate advertising start
        // successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            DEBUG_PRINT_E("GATTS", "Advertising start failed");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            DEBUG_PRINT_E("GATTS", "Advertising stop failed");
        }
        else
        {
            DEBUG_PRINT_I("GATTS", "Stop adv successfully");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        DEBUG_PRINT_I("GATTS",
                      "update connetion params status = %d, min_int = %d, max_int = "
                      "%d,conn_int = %d,latency = %d, timeout = %d",
                      param->update_conn_params.status, param->update_conn_params.min_int, param->update_conn_params.max_int,
                      param->update_conn_params.conn_int, param->update_conn_params.latency,
                      param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

/**
 * \brief BLE write event handler.
 *
 * Handles write events, including preparation for long writes.
 *
 * \param[in] gatts_if GATT server interface.
 * \param[in/out] prepare_write_env Environment for preparing long writes.
 * \param[in] param Write event parameters.
 *
 * \return None.
 */
void writeEvent(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp)
    {
        if (param->write.is_prep)
        {
            if (prepare_write_env->prepare_buf == NULL)
            {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL)
                {
                    DEBUG_PRINT_E("GATTS", "Gatt_server prep no mem");
                    status = ESP_GATT_NO_RESOURCES;
                }
            }
            else if (param->write.offset > PREPARE_BUF_MAX_SIZE || prepare_write_env->prepare_len > param->write.offset)
            {
                status = ESP_GATT_INVALID_OFFSET;
            }
            else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
            {
                status = ESP_GATT_INVALID_ATTR_LEN;
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err =
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);

            if (response_err != ESP_OK)
            {
                DEBUG_PRINT_E("GATTS", "Send response error");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK)
            {
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset, param->write.value, param->write.len);
            prepare_write_env->prepare_len += param->write.len;
        }
        else
        {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

/**
 * \brief Execute write event handler.
 *
 * Handles execute write events, completing prepared long writes or cancelling them.
 *
 * \param[in/out] prepare_write_env Environment for preparing long writes, to be cleaned up after execution.
 * \param[in] param Execute write event parameters.
 *
 * \return None.
 */
void execWritEvent(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC)
    {
        esp_log_buffer_hex("GATTS", prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        DEBUG_PRINT_I("GATTS", "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

/**
 * \brief BLE GATT server event handler for profile A.
 *
 * Handles GATT server events specifically for profile A. Events include registration, read/write requests, service start,
 * connection, disconnection, and more.
 *
 * \param[in] event GATT server event.
 * \param[in] gatts_if GATT server interface.
 * \param[in] param Event parameters.
 *
 * \return None.
 */
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                          esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        DEBUG_PRINT_I("GATTS", "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;
        gl_profile_tab[PROFILE_A_APP_ID].gatts_if = gatts_if;
        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(scientisst_device_settings.device_name);
        if (set_dev_name_ret)
        {
            DEBUG_PRINT_E("GATTS", "set device name failed, error code = %x", set_dev_name_ret);
        }
        // config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            DEBUG_PRINT_E("GATTS", "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        // config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret)
        {
            DEBUG_PRINT_E("GATTS", "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
    case ESP_GATTS_READ_EVT: {
        DEBUG_PRINT_I("GATTS", "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d", param->read.conn_id,
                      param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT:
        DEBUG_PRINT_I("BLE Event", "BT data received length:%d", param->write.len);
        processPacket(param->write.value);
        // This function is used to handle write requeset of client. Because
        // server should send write response when receive write request.
        writeEvent(gatts_if, &a_prepare_write_env, param);
        break;

    case ESP_GATTS_EXEC_WRITE_EVT:
        DEBUG_PRINT_I("GATTS", "ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        execWritEvent(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        DEBUG_PRINT_I("GATTS", "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        DEBUG_PRINT_I("GATTS", "CREATE_SERVICE_EVT, status %d,  service_handle %d", param->create.status,
                      param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(
            gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, a_property, &gatts_char1_val, NULL);
        if (add_char_ret)
        {
            DEBUG_PRINT_E("GATTS", "add char failed, error code =%x", add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;

        DEBUG_PRINT_I("GATTS", "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d", param->add_char.status,
                      param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
        if (get_attr_ret == ESP_FAIL)
        {
            DEBUG_PRINT_E("GATTS", "ILLEGAL HANDLE");
        }

        DEBUG_PRINT_I("GATTS", "the gatts demo char length = %x", length);
        for (int i = 0; i < length; i++)
        {
            DEBUG_PRINT_I("GATTS", "prf_char[%x] =%x", i, prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                                                               &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret)
        {
            DEBUG_PRINT_E("GATTS", "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        DEBUG_PRINT_I("GATTS", "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d", param->add_char_descr.status,
                      param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        DEBUG_PRINT_I("GATTS", "SERVICE_START_EVT, status %d, service_handle %d", param->start.status,
                      param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents
         * about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        DEBUG_PRINT_I("GATTS",
                      "ESP_GATTS_CONNECT_EVT, conn_id %d, remote "
                      "%02x:%02x:%02x:%02x:%02x:%02x:",
                      param->connect.conn_id, param->connect.remote_bda[0], param->connect.remote_bda[1],
                      param->connect.remote_bda[2], param->connect.remote_bda[3], param->connect.remote_bda[4],
                      param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        // start sent the update connection parameters to the peer device.
        // esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:;
        DEBUG_PRINT_I("GATTS", "ESP_GATTS_DISCONNECT_EVT");
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        DEBUG_PRINT_I("GATTS", "ESP_GATTS_CONF_EVT, status %d", param->conf.status);
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
        break;
    case ESP_GATTS_CONGEST_EVT:
        if (param->congest.congested)
        {
            DEBUG_PRINT_W("gatts_profile_a_event_handler", "ESP_SPP_CONG_EVT");
        }
        break;
    default:
        break;
    }
}

/**
 * \brief BLE general GATT server event handler.
 *
 * Distributes GATT server events to appropriate profile handlers.
 *
 * \param[in] event GATT server event.
 * \param[in] gatts_if GATT server interface.
 * \param[in] param Event parameters.
 *
 * \return None.
 */
static void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        }
        else
        {
            DEBUG_PRINT_I("GATTS", "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    for (int idx = 0; idx < PROFILE_NUM; idx++)
    {
        /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call
         * every profile cb function */
        if ((gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if) && gl_profile_tab[idx].gatts_cb)
        {
            gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
        }
    }
}

/**
 * \brief Send data over BLE.
 *
 * Sends data to the connected BLE client.
 *
 * \param[in] fd File descriptor, unused in this function.
 * \param[in] len Length of the data to send.
 * \param[in] buff Buffer containing the data.
 *
 * \return ESP_OK - Success,  ESP_ERR - failure.
 */
esp_err_t sendBle(uint32_t fd, int len, const uint8_t *buff)
{
    esp_err_t res;

    res = esp_ble_gatts_send_indicate((esp_gatt_if_t)gl_profile_tab[PROFILE_A_APP_ID].gatts_if,
                                      gl_profile_tab[PROFILE_A_APP_ID].conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                      (uint16_t)len, (uint8_t *)buff, false);

    if (res != ESP_OK)
    {
        DEBUG_PRINT_E("sendBle", "ERROR: SEND FAILED");
    }

    return ESP_OK;
}

/**
 * \brief Initialize the BLE functionality.
 *
 * Sets up the BLE hardware, initializes the required services, and registers the GATT event handlers.
 *
 * \return None.
 */
void initBle(void)
{
    esp_err_t ret;
    esp_ble_sm_param_t param_type_ble = ESP_BLE_SM_IOCAP_MODE;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        DEBUG_PRINT_E("GATTS", "%s initialize controller failed", __func__);
        exit(-1);
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret)
    {
        DEBUG_PRINT_E("GATTS", "%s enable controller failed", __func__);
        exit(-1);
    }
    ret = esp_bluedroid_init();
    if (ret)
    {
        DEBUG_PRINT_E("GATTS", "%s init bluetooth failed", __func__);
        exit(-1);
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        DEBUG_PRINT_E("GATTS", "%s enable bluetooth failed", __func__);
        exit(-1);
    }

    ret = esp_ble_gatts_register_callback(gattsEventHandler);
    if (ret)
    {
        DEBUG_PRINT_E("GATTS", "gatts register error, error code = %x", ret);
        exit(-1);
    }
    ret = esp_ble_gap_register_callback(gapEventHandler);
    if (ret)
    {
        DEBUG_PRINT_E("GATTS", "gap register error, error code = %x", ret);
        exit(-1);
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
        DEBUG_PRINT_E("GATTS", "gatts app register error, error code = %x", ret);
        exit(-1);
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(490);
    if (local_mtu_ret)
    {
        DEBUG_PRINT_E("GATTS", "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    if ((ret = esp_ble_gap_set_security_param(param_type_ble, &iocap, sizeof(uint8_t))) != ESP_OK)
    {
        DEBUG_PRINT_E("init", "%s change security params failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
}
