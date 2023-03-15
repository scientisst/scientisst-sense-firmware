/** \file ble.c
    \brief Bluetooth Low Energy (BLE) mode functions.
    
    This file contains the functions for the BLE mode. And relevant macros.
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_timer.h"
#include "sdkconfig.h"

#include "scientisst.h"
#include "bt.h"
#include "macros.h"
#include "com.h"

//#define CONFIG_EXAMPLE_SET_RAW_ADV_DATA 0

#define SECOND_TO_USECOND          1000000

#define GATTS_TAG "GATTS_DEMO"

#define GATTS_NOTIFY_LEN    490

#define GATTS_SERVICE_UUID_TEST_A   0x00FF
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     4

#define GATTS_SERVICE_UUID_TEST_B   0x00EE
#define GATTS_CHAR_UUID_TEST_B      0xEE01
#define GATTS_DESCR_UUID_TEST_B     0x2222
#define GATTS_NUM_HANDLE_TEST_B     4

#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0



//static SemaphoreHandle_t gatts_semaphore;
//static bool can_send_notify = false;
//static uint8_t indicate_data[GATTS_NOTIFY_LEN] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a};

///Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static bool is_connect = false;
static uint8_t char1_str[] = {0x11,0x22,0x33};
static esp_gatt_char_prop_t a_property = 0;

static esp_attr_value_t gatts_demo_char1_val = {
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

static uint8_t adv_config_done = 0;


#ifdef CONFIG_EXAMPLE_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        0x02, 0x01, 0x06,
        0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
};
static uint8_t raw_scan_rsp_data[] = BT_DEFAULT_DEVICE_NAME;
#else

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x000C, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
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
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 32,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_EXAMPLE_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
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

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);


/**
 * \brief ESP32 BLE GATT Server Event Handler.
 *
 * \param event ESP32 BLE GATT Server Event.
 * \param param ESP32 BLE GATT Server Event Parameters.
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
#ifdef CONFIG_EXAMPLE_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~adv_config_flag);
            if(adv_config_done==0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~scan_rsp_config_flag);
            if(adv_config_done==0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
#else
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~adv_config_flag);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~scan_rsp_config_flag);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
#endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            //advertising start complete event to indicate advertising start successfully or failed
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                DEBUG_PRINT_E(GATTS_TAG, "Advertising start failed\n");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                DEBUG_PRINT_E(GATTS_TAG, "Advertising stop failed\n");
            } else {
                DEBUG_PRINT_I(GATTS_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            DEBUG_PRINT_I(GATTS_TAG,
                          "update connetion params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                          param->update_conn_params.status,
                          param->update_conn_params.min_int,
                          param->update_conn_params.max_int,
                          param->update_conn_params.conn_int,
                          param->update_conn_params.latency,
                          param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}


/**
 * \brief example write event.
 *
 * \param gatts_if GATT Server Interface.
 * \param prepare_write_env Prepare Write Environment.
 * \param param ESP32 BLE GATT Server Event Parameters.
 */
void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp) {
        if (param->write.is_prep) {
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t * )
                malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    DEBUG_PRINT_E(GATTS_TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if (param->write.offset > PREPARE_BUF_MAX_SIZE ||
                    prepare_write_env->prepare_len > param->write.offset) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *) malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                                                 status, gatt_rsp);

            if (response_err != ESP_OK) {
                DEBUG_PRINT_E(GATTS_TAG, "Send response error\n");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK) {
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset, param->write.value, param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        } else {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}


/**
 * \brief example exec write event.
 *
 * \param prepare_write_env Prepare Write Environment.
 * \param param ESP32 BLE GATT Server Event Parameters.
 */
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC) {
        esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    } else {
        DEBUG_PRINT_I(GATTS_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}


/**
 * \brief GATT Server Event Handler for profile A.
 *
 * This function is called when a GATT Server Event occurs for profile A. For now it is the only profile.
 *
 * \param event ESP32 BLE GATT Server Event.
 * \param gatts_if GATT Server Interface.
 * \param param ESP32 BLE GATT Server Event Parameters.
 */
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            DEBUG_PRINT_I(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
            gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
            gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;
            gl_profile_tab[PROFILE_A_APP_ID].gatts_if = gatts_if;
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(device_name);
            if (set_dev_name_ret) {
                DEBUG_PRINT_E(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
#ifdef CONFIG_EXAMPLE_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if(raw_adv_ret){
                DEBUG_PRINT_E(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= adv_config_flag;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if(raw_scan_ret){
                DEBUG_PRINT_E(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= scan_rsp_config_flag;
#else
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret) {
                DEBUG_PRINT_E(GATTS_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= adv_config_flag;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret) {
                DEBUG_PRINT_E(GATTS_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= scan_rsp_config_flag;

#endif
            esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id,
                                         GATTS_NUM_HANDLE_TEST_A);
            break;
        case ESP_GATTS_READ_EVT: {
            DEBUG_PRINT_I(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id,
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
            /*
            DEBUG_PRINT_I(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
            if(!param->write.is_prep){
                DEBUG_PRINT_I(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
                esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
    
                if(gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
    
                    if(descr_value == 0x0001){
                        if(a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
    
                            DEBUG_PRINT_I(GATTS_TAG, "notify enable");
                            can_send_notify = true;
                            xSemaphoreGive(gatts_semaphore);
                        }
                    }else if(descr_value == 0x0002){
                        if(a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                            DEBUG_PRINT_I(GATTS_TAG, "indicate enable");
                            uint8_t indicate_data[600];
                            for(int i = 0; i < sizeof(indicate_data); ++i){
                                indicate_data[i] = i%0xff;
                            }
    
                            for(int j = 0; j < 1000; j++){
                                //the size of indicate_data[] need less than MTU size
                                esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle, sizeof(indicate_data), indicate_data, true);
                            }
                        }
                    }
                    else if(descr_value == 0x0000){
                        can_send_notify = false;
                        a_property = 0;
                        DEBUG_PRINT_E(GATTS_TAG, "notify/indicate disable ");
                    }else{
                        DEBUG_PRINT_E(GATTS_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                    }
    
                }
            }
            */
#if(_DEBUG_ == 1 || _DEBUG_ == 2)
            printf("BT data recieved\n length:%d  data:", param->write.len);
            for (int i = 0; i < param->write.len; i++) {
                printf("%d ", param->write.value[i]);
            }
            printf("\n");
#endif
            processRcv(param->write.value, param->write.len);
            //This function is used to handle write requeset of client. Because server should send write response when receive write request.
            example_write_event_env(gatts_if, &a_prepare_write_env, param);
            break;

        case ESP_GATTS_EXEC_WRITE_EVT:
            DEBUG_PRINT_I(GATTS_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            example_exec_write_event_env(&a_prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            DEBUG_PRINT_I(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        case ESP_GATTS_CREATE_EVT:
            DEBUG_PRINT_I(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status,
                          param->create.service_handle);
            gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
            gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

            esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
            a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
            esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                                                            &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, a_property,
                                                            &gatts_demo_char1_val, NULL);
            if (add_char_ret) {
                DEBUG_PRINT_E(GATTS_TAG, "add char failed, error code =%x", add_char_ret);
            }
            break;
        case ESP_GATTS_ADD_INCL_SRVC_EVT:
            break;
        case ESP_GATTS_ADD_CHAR_EVT: {
            uint16_t length = 0;
            const uint8_t *prf_char;

            DEBUG_PRINT_I(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                          param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
            gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
            esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
            if (get_attr_ret == ESP_FAIL) {
                DEBUG_PRINT_E(GATTS_TAG, "ILLEGAL HANDLE");
            }

            DEBUG_PRINT_I(GATTS_TAG, "the gatts demo char length = %x\n", length);
            for (int i = 0; i < length; i++) {
                DEBUG_PRINT_I(GATTS_TAG, "prf_char[%x] =%x\n", i, prf_char[i]);
            }
            esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                                                                   &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL,
                                                                   NULL);
            if (add_descr_ret) {
                DEBUG_PRINT_E(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
            }
            break;
        }
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
            DEBUG_PRINT_I(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                          param->add_char_descr.status, param->add_char_descr.attr_handle,
                          param->add_char_descr.service_handle);
            break;
        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
            DEBUG_PRINT_I(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n", param->start.status,
                          param->start.service_handle);
            break;
        case ESP_GATTS_STOP_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT: {
            is_connect = true;
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            DEBUG_PRINT_I(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                          param->connect.conn_id, param->connect.remote_bda[0], param->connect.remote_bda[1],
                          param->connect.remote_bda[2], param->connect.remote_bda[3], param->connect.remote_bda[4],
                          param->connect.remote_bda[5]);
            gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
            //start sent the update connection parameters to the peer device.
            //esp_ble_gap_update_conn_params(&conn_params);
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT:
            is_connect = false;
            DEBUG_PRINT_I(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT");
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CONF_EVT:
            DEBUG_PRINT_I(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d", param->conf.status);

            //Try to send next buff
            /*if(!(param->congest.congested)){         //bt write is free
                sendData();
            }else{
                send_busy = SEND_AFTER_C0NG;
            }*/
            break;
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
            break;
        case ESP_GATTS_CONGEST_EVT:
            if (param->congest.congested == 0 && send_busy == SEND_AFTER_C0NG) {
                //sendData();                             //bt write is free
            } else if (param->congest.congested) {
                DEBUG_PRINT_W("gatts_profile_a_event_handler", "ESP_SPP_CONG_EVT");
            }
            break;
        default:
            break;
    }
}


/**
 * \brief GATT Server Event Handler.
 *
 * This function is called when a GATT Server event occurs. This is a general function that forwards the event to the correct profile.
 *
 * \param event ESP32 BLE GATT Server Event.
 * \param gatts_if GATT Server Interface.
 * \param param ESP32 BLE GATT Server Event Parameters.
 */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            DEBUG_PRINT_I(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0); //TODO: check this
}


/**
 * \brief send data function for BLE.
 *
 * \param fd File descriptor (unused for this function, created for consistency with other send functions).
 * \param len Length of data.
 * \param buff Data to send.
 */
esp_err_t IRAM_ATTR sendBle(uint32_t fd, int len, uint8_t *buff) {
    esp_err_t res;

    res = esp_ble_gatts_send_indicate(gl_profile_tab[PROFILE_A_APP_ID].gatts_if,
                                      gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                      gl_profile_tab[PROFILE_A_APP_ID].char_handle, len, buff, false);

    if (res != ESP_OK) {
        DEBUG_PRINT_E("sendBle", "ERROR: SEND FAILED\n");
    }

    //printf("Frame sn=%d (len=%d): %d %d %d\n", buff[2] >> 4, len, buff[0], buff[1], buff[2]);

    finalizeSend();

    // Try to send next buff
    sendData();
    return ESP_OK;
}


/**
 * \brief BLE GATT Server Initialization.
 *
 * This function initializes the BLE GATT Server.
 */
void initBle(void) {
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        DEBUG_PRINT_E(GATTS_TAG, "%s initialize controller failed\n", __func__);
        exit(-1);
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        DEBUG_PRINT_E(GATTS_TAG, "%s enable controller failed\n", __func__);
        exit(-1);
    }
    ret = esp_bluedroid_init();
    if (ret) {
        DEBUG_PRINT_E(GATTS_TAG, "%s init bluetooth failed\n", __func__);
        exit(-1);
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        DEBUG_PRINT_E(GATTS_TAG, "%s enable bluetooth failed\n", __func__);
        exit(-1);
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        DEBUG_PRINT_E(GATTS_TAG, "gatts register error, error code = %x", ret);
        exit(-1);
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        DEBUG_PRINT_E(GATTS_TAG, "gap register error, error code = %x", ret);
        exit(-1);
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret) {
        DEBUG_PRINT_E(GATTS_TAG, "gatts app register error, error code = %x", ret);
        exit(-1);
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(GATTS_NOTIFY_LEN);
    if (local_mtu_ret) {
        DEBUG_PRINT_E(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    return;
}
