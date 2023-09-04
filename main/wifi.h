/** \file wifi.h
    \brief Wifi functions definitions and relevant defines
*/

#ifndef _WIFI_H
#define _WIFI_H

#include <stdint.h>

#define COM_MODE_TCP_AP "tcp_ap"
#define COM_MODE_TCP_STA "tcp_sta"
#define COM_MODE_UDP_STA "udp_sta"
#define COM_MODE_BT "bt"
#define COM_MODE_SERIAL "serial"
#define COM_MODE_WS_AP "ws_ap"
#define COM_MODE_BLE "ble"
#define COM_MODE_SD_CARD "sd_card"

typedef struct
{
    char ssid[32];
    char password[64];
    char com_mode[8];
    char host_ip[16];
    char port_number[6];
    char bit_when[9];
    char sampling_rate[5];
    char no_channels[2];
    char channels[23];
    char bit_mode[10];
    char port_o1[5];
    char port_o2[5];
} op_settings_info_t;

int wifiInit(uint8_t force_ap);
void wifi_init_softap(void);
int wifi_init_sta(void);

// Checks if com_mode is one of the wifi modes
uint8_t isComModeWifi(void);

int getOpSettingsInfo(op_settings_info_t *pOpSettingsInfo);
void saveOpSettingsInfo(op_settings_info_t *pOpSettingsInfo);

#endif