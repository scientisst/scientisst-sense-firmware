#include <esp_err.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_wifi.h>
#include <tcpip_adapter.h>
#include <lwip/sockets.h>
#include <string.h>
#include "tcp.h"


/**
 * an ESP32 WiFi event handler.
 * the types of events that can be received here are:
 *
 * SYSTEM_EVENT_AP_PROBEREQRECVED
 * SYSTEM_EVENT_AP_STACONNECTED
 * SYSTEM_EVENT_AP_STADISCONNECTED
 * SYSTEM_EVENT_AP_START
 * SYSTEM_EVENT_AP_STOP
 * SYSTEM_EVENT_SCAN_DONE
 * SYSTEM_EVENT_STA_AUTHMODE_CHANGE
 * SYSTEM_EVENT_STA_CONNECTED
 * SYSTEM_EVENT_STA_DISCONNECTED
 * SYSTEM_EVENT_STA_GOT_IP
 * SYSTEM_EVENT_STA_START
 * SYSTEM_EVENT_STA_STOP
 * SYSTEM_EVENT_WIFI_READY
 */
static esp_err_t esp32_wifi_eventHandler(void *ctx, system_event_t *event) {
	switch(event->event_id) {
		case SYSTEM_EVENT_AP_START: { // handle the AP start event
			// when we have started being an access point
			tcpip_adapter_ip_info_t ip_info;
			tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info);
			printf(">> OK: softAP mode, IP is " IPSTR, IP2STR(&ip_info.ip));
			printf("\n");
			break;
		} // SYSTEM_EVENT_AP_START

		case SYSTEM_EVENT_AP_STADISCONNECTED: {
			printf(">> OK: softAP mode - station disconnected\n");
			break;
		} // SYSTEM_EVENT_AP_STADISCONNECTED

		default: // ignore the other event types
			break;
	} // switch event

	return ESP_OK;
} // esp32_wifi_eventHandler


static void readMAC() {
	uint8_t mac_address[6];
	esp_efuse_mac_get_default(mac_address);
	printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac_address[0], mac_address[1], mac_address[2], 
		mac_address[3], mac_address[4], mac_address[5]);
}


/**
 * become an access point
 */
static void becomeAccessPointMAC() {
	uint8_t mac_address[6];
	char ssid[15];
	esp_efuse_mac_get_default(mac_address);
	sprintf(ssid, "BITalino-%02X-%02X", mac_address[4], mac_address[5]);

	printf(">> OK: start being an access point ...\n");
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	wifi_config_t apConfig = {
		.ap = {
			.ssid_len=0,\
			.password="",
			.channel=0,
			.authmode=WIFI_AUTH_OPEN,
			.ssid_hidden=0,
			.max_connection=4,
			.beacon_interval=100
		}
	};
	strcpy((char *)apConfig.ap.ssid, ssid);
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &apConfig));
	ESP_ERROR_CHECK(esp_wifi_start());
	printf(">> OK: connect to access point \"%s\"\n", (char *)apConfig.ap.ssid);
} // becomeAccessPointMAC

/**
 * main entry point for the bootWiFi module.
 */
void bootWiFi() {
	tcpip_adapter_init();

	ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP)); // stop DHCP server

	// assign a static IP to the network interface
	tcpip_adapter_ip_info_t ip_info;
	memset(&ip_info, 0, sizeof(ip_info));
    IP4_ADDR(&ip_info.ip, 192, 168, 1, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    IP4_ADDR(&ip_info.gw, 192, 168, 1, 1); // ESP32 acts as router, so GW address will be its own address   
	ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info));

	// ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP)); // start the DHCP server

	ESP_ERROR_CHECK(esp_event_loop_init(esp32_wifi_eventHandler, NULL));

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	becomeAccessPointMAC();
} // bootWiFi
