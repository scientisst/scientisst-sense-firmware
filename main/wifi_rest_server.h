#ifndef _WIFI_REST_SERVER_H
#define _WIFI_REST_SERVER_H

#define SIZE_2 (2)
#define SIZE_5 (5)
#define SIZE_8 (8)
#define SIZE_9 (9)
#define SIZE_10 (10)
#define SIZE_16 (16)
#define SIZE_23 (23)
#define SIZE_32 (32)
#define SIZE_64 (64)
#define BIGGEST_SIZE SIZE_64

typedef struct {
	char ssid[SIZE_32];
	char password[SIZE_64];

	char op_mode[SIZE_8];
	char host_ip[SIZE_16];
	char port_number[SIZE_5];
	char bit_when[SIZE_9];
	char sampling_rate[SIZE_5];
	char no_channels[SIZE_2];
	char channels[SIZE_23];
	char bit_mode[SIZE_10];
	char port_o1[SIZE_5];
	char port_o2[SIZE_5];
} op_settings_info_t;

void initRestServer(void);

#endif