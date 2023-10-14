/** \file wifi.h
    \brief Wifi functions definitions and relevant defines
*/

#ifndef _WIFI_H
#define _WIFI_H

#include <stdint.h>

#include "sci_config.h"

typedef struct
{
    char ssid[32];
    char password[64];
    com_mode_t com_mode;
    char host_ip[16];
    char port_number[6];
    char bit_when[9];
    char sampling_rate[5];
    char no_channels[2];
    char channels[23];
    char bit_mode[10];
    char port_o1[5];
    char port_o2[5];
    uint8_t is_battery_threshold_inflated;
} op_settings_info_t;

int wifiInit(uint8_t force_ap);
void wifi_init_softap(void);
int wifi_init_sta(void);

// Checks if com_mode is one of the wifi modes
uint8_t isComModeWifi(void);

int getOpSettingsInfo(op_settings_info_t *pOpSettingsInfo);
void saveOpSettingsInfo(op_settings_info_t *pOpSettingsInfo);

#endif
