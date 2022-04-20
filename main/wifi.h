#ifndef _WIFI_H
#define _WIFI_H

#define SIZE_2 (2)
#define SIZE_5 (5)
#define SIZE_6 (6)
#define SIZE_8 (8)
#define SIZE_9 (9)
#define SIZE_10 (10)
#define SIZE_16 (16)
#define SIZE_23 (23)
#define SIZE_32 (32)
#define SIZE_64 (64)
#define BIGGEST_SIZE SIZE_64

#define COM_MODE_TCP_AP "tcp_ap"
#define COM_MODE_TCP_STA "tcp_sta"
#define COM_MODE_UDP_STA "udp_sta"
#define COM_MODE_BT "bt"
#define COM_MODE_SERIAL "serial"

typedef struct {
	char ssid[SIZE_32];
	char password[SIZE_64];

	char com_mode[SIZE_8];
	char host_ip[SIZE_16];
	char port_number[SIZE_6];
	char bit_when[SIZE_9];
	char sampling_rate[SIZE_5];
	char no_channels[SIZE_2];
	char channels[SIZE_23];
	char bit_mode[SIZE_10];
	char port_o1[SIZE_5];
	char port_o2[SIZE_5];
}op_settings_info_t;

int wifiInit(uint8_t force_ap);
void wifi_init_softap();

//Checks if com_mode is one of the wifi modes
uint8_t isComModeWifi(void);

int getOpSettingsInfo(op_settings_info_t *pOpSettingsInfo);
void saveOpSettingsInfo(op_settings_info_t *pOpSettingsInfo);

#endif