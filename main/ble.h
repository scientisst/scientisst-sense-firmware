/** \file ble.h
    \brief BLE mode functions header file.
    
    This file contains the functions to initialize and send data in BLE mode. It also contains the maximum length of the buffer in BLE mode.
*/

#ifndef BLE_H
#define BLE_H

#define GATTS_NOTIFY_LEN 517 //< Maximum length of the buffer to send in BLE mode

void initBle(void);
esp_err_t sendBle(uint32_t fd, int len, uint8_t *buff);

#endif