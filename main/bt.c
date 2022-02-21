#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp_system.h"
#include "time.h"
#include "sys/time.h"
#include "bt.h"
#include "macros.h"
#include "com.h"
#include "main.h"

#define SEND_AFTER_C0NG 2

void IRAM_ATTR sendData(){
    //Check if there's anything to send and if there is, check if it's enough to send
    xSemaphoreTake(bt_buffs_to_send_mutex, portMAX_DELAY);
    if(bt_buffs_to_send[bt_curr_buff]){
        xSemaphoreGive(bt_buffs_to_send_mutex);
        if(snd_buff_idx[bt_curr_buff] < send_threshold){
            send_busy = 0;
            return;
        }
    //There's nothing to send
    }else{
        xSemaphoreGive(bt_buffs_to_send_mutex);
        send_busy = 0;
        return;
    }

    DEBUG_PRINT_I("sendData", "Data sent: %d bytes", snd_buff_idx[bt_curr_buff]);
    
    send_busy = 1;
    send_func(send_fd, snd_buff_idx[bt_curr_buff], snd_buff[bt_curr_buff]);

    //The buffer changing and clearing tasks are done after in finalizeSend() the ESP_SPP_WRITE_EVT event ocurred with writing completed successfully in esp_spp_cb()
}

void IRAM_ATTR finalizeSend(){
    xSemaphoreTake(bt_buffs_to_send_mutex, portMAX_DELAY);
        bt_buffs_to_send[bt_curr_buff] = 0;
    xSemaphoreGive(bt_buffs_to_send_mutex);

    //Clear recently sent buffer
    memset(snd_buff[bt_curr_buff], 0, snd_buff_idx[bt_curr_buff]);
    snd_buff_idx[bt_curr_buff] = 0;

    //Change send buffer
    bt_curr_buff = (bt_curr_buff+1)%4;
}

static void IRAM_ATTR esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
    switch (event){
    case ESP_SPP_INIT_EVT:
        DEBUG_PRINT_I("esp_spp_cb", "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(bt_device_name);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
        ESP_LOGI("Init Bluetooth", "Device online with the name: %s", bt_device_name);
        break;
    case ESP_SPP_SRV_OPEN_EVT:                      //Server connection open (first client connection)  
        DEBUG_PRINT_I("esp_spp_cb", "ESP_SPP_SRV_OPEN_EVT");  
        if (!send_fd){
            send_fd = param->open.handle;
        }else{
            esp_spp_disconnect(param->open.handle);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        DEBUG_PRINT_I("esp_spp_cb", "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:                          //Client connection open   
        DEBUG_PRINT_I("esp_spp_cb", "ESP_SPP_OPEN_EVT");
        //We only want one client
        esp_spp_disconnect(param->open.handle);
        break;
    case ESP_SPP_CLOSE_EVT:                         //Client connection closed
        DEBUG_PRINT_E("esp_spp_cb", "ESP_SPP_CLOSE_EVT");
        
        send_fd = 0;
        stopAcquisition();
        //Make sure that sendBtTask doesn't stay's stuck waiting for a sucessful write
        break;
    case ESP_SPP_START_EVT:                         //server started
        DEBUG_PRINT_I("esp_spp_cb", "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:                       //client initiated a connection
        DEBUG_PRINT_I("esp_spp_cb", "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:                      //connection received data
        #if(_DEBUG_ == 1 || _DEBUG_ == 2 )
            printf("BT data recieved\n length:%d  data:", param->data_ind.len);
            for(int i = 0; i < param->data_ind.len; i++){
                printf("%d ", param->data_ind.data[i]);
            }
            printf("\n");
        #endif
        processRcv(param->data_ind.data, param->data_ind.len);

        break;
    case ESP_SPP_CONG_EVT:                          //connection congestion status changed
        if(param->cong.cong == 0 && send_busy == SEND_AFTER_C0NG){
            sendData();                             //bt write is free
        }else if(param->cong.cong == 1){
            DEBUG_PRINT_W("esp_spp_cb", "ESP_SPP_CONG_EVT");
        }
        break;
        
    case ESP_SPP_WRITE_EVT:                         //write operation status changed
        //Completed successfuly send---------------------------------------------------------  
        if(param->write.status == ESP_SPP_SUCCESS){
            finalizeSend();
            //Try to send next buff
            if(param->write.cong == 0){         //bt write is free   
                sendData();                             
            }else{
                send_busy = SEND_AFTER_C0NG;
            }
        //write failed because congestion, so wait event 'ESP_SPP_CONG_EVT' and 'param->cong.cong == 0' to resend the failed writing data.
        }else{
            //To resend the same buffer (note that because the write was unsuccessful, bt_curr_buff wasn't updated)
            send_busy = SEND_AFTER_C0NG;
        }
        break;
    default:
        break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            DEBUG_PRINT_I("esp_spp_cb", "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            DEBUG_PRINT_E("esp_spp_cb", "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        DEBUG_PRINT_I("esp_spp_cb", "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            DEBUG_PRINT_I("esp_spp_cb", "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            DEBUG_PRINT_I("esp_spp_cb", "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

    #if (CONFIG_BT_SSP_ENABLED == true)
        case ESP_BT_GAP_CFM_REQ_EVT:
            DEBUG_PRINT_I("esp_spp_cb", "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
        case ESP_BT_GAP_KEY_NOTIF_EVT:
            DEBUG_PRINT_I("esp_spp_cb", "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
            break;
        case ESP_BT_GAP_KEY_REQ_EVT:
            DEBUG_PRINT_I("esp_spp_cb", "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
            break;
    #endif

    default: {
        DEBUG_PRINT_I("esp_spp_cb", "event: %d", event);
        break;
    }
    }
    return;
}

void initBt(){
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    getDeviceName();

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        DEBUG_PRINT_E("init", "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        DEBUG_PRINT_E("init", "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        DEBUG_PRINT_E("init", "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        DEBUG_PRINT_E("init", "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        DEBUG_PRINT_E("init", "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        DEBUG_PRINT_E("init", "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(ESP_SPP_MODE_CB)) != ESP_OK) {
        DEBUG_PRINT_E("init", "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    #if (CONFIG_BT_SSP_ENABLED == true)
        /* Set default parameters for Secure Simple Pairing */
        esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
        esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
        esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
    #endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    DEBUG_PRINT_I("initBt", "Init Bluetooth completed\n");
}

//Use the last 2 bytes of MAC-ADDRESS found on EFUSE to make the device name
void getDeviceName(){
    uint8_t mac[6];
    char last_five_chars_bt_name[6];

    //Read base Mac address from EFUSE, which is used to calculate the MAC addr for all the interfaces
    //Use esp_read_mac to infer the bluetooth mac addr based in the base mac addr read from EFUSE
    if(esp_efuse_mac_get_default(mac) == ESP_OK && esp_read_mac(mac, ESP_MAC_BT) == ESP_OK){
        sprintf(last_five_chars_bt_name, "%x-%x", mac[4], mac[5]);

        //Turn the last xx-xx of mac address from lower case to upper case
        for (int i = 0; last_five_chars_bt_name[i] != '\0'; i++){
            char c = last_five_chars_bt_name[i];
            if (c >= 97 && c <= 122){
                c -= 32;
            }
            last_five_chars_bt_name[i] = c;
        }
        sprintf(bt_device_name, "%s-%s", BT_DEFAULT_DEVICE_NAME, last_five_chars_bt_name);
    }else{
        DEBUG_PRINT_E("BtTask", "Couldn't read MAC address from Efuse\n");
    }
}