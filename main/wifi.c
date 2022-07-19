#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/apps/netbiosns.h"

#include "mdns.h"
#include "wifi.h"
#include "scientisst.h"
#include "macros.h"

// if the structure of a record saved for a subsequent reboot changes
// then consider using semver to change the version number or else
// we may try and boot with the wrong data.
//#define KEY_VERSION "version"
// uint32_t g_version = 0x0100;

#define KEY_SETTINGS_INFO "opSettingsInfo" // key used in NVS for connection info
#define BOOTWIFI_NAMESPACE "bootwifi"      // namespace in NVS for bootwifi

/* The examples use WiFi configuration that you can set via project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID "ScientISST"
#define EXAMPLE_ESP_WIFI_PASS "12345678"
#define EXAMPLE_ESP_WIFI_CHANNEL 1 // Range: 1 to 13, default: 1
#define EXAMPLE_MAX_STA_CONN 4     // Default: 4

#define MDNS_HOST_NAME "scientisst" // Specify the domain name used in the mDNS service. Note that webpage also take it as a part of URL where it will send GET/POST requests to.
#define MDNS_INSTANCE "esp home web server"

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI("wifi softAP", "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI("wifi softAP", "station " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            //.ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = 0,
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };
    memcpy(wifi_config.ap.ssid, device_name, strlen(device_name) + 1);

    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    DEBUG_PRINT_W("wifi softAP", "wifi_init_softap finished. SSID:%s password:%s channel:%d", wifi_config.ap.ssid, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

// wifi station--------------------------------------------------------------------------------------------------------------------

#define EXAMPLE_ESP_MAXIMUM_RETRY 1 // Default is 5

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            DEBUG_PRINT_I("wifi station", "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        DEBUG_PRINT_I("wifi station", "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("wifi station", "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

int wifi_init_sta(void)
{
    int ret = 0; // Return if connection to Wifi fails or not
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "test",
            .password = "",
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };
    // Store
    //  If ssid is empty "", attempt to connect will not fail. This is not desirable
    if (strcmp(op_settings.ssid, ""))
    {
        strcpy((char *)wifi_config.sta.ssid, op_settings.ssid);
    }
    strcpy((char *)wifi_config.sta.password, op_settings.password);

    // If there's no password, there is no authmode.
    if (!strcmp((char *)wifi_config.sta.password, ""))
    {
        wifi_config.sta.threshold.authmode = 0;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for EXAMPLE_ESP_MAXIMUM_RETRY
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        DEBUG_PRINT_I("wifi station", "connected to ap SSID:%s password:%s", (char *)wifi_config.sta.ssid, (char *)wifi_config.sta.password);
        ret = 0;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        DEBUG_PRINT_W("wifi station", "Failed to connect to SSID:%s, password:%s", (char *)wifi_config.sta.ssid, (char *)wifi_config.sta.password);
        ret = ESP_FAIL;
    }
    else
    {
        DEBUG_PRINT_E("wifi station", "UNEXPECTED EVENT");
        ret = ESP_FAIL;
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
    return ret;
}

// Retrieve the connection info. if rc == 0 means ok.
int getOpSettingsInfo(op_settings_info_t *_op_settings)
{
    nvs_handle handle;
    size_t size;
    esp_err_t err;
    op_settings_info_t pOpSettingsInfo;
    // uint32_t version;

    err = nvs_open(BOOTWIFI_NAMESPACE, NVS_READWRITE, &handle);
    if (err != 0)
    {
        DEBUG_PRINT_E("getOpSettingsInfo", "ERROR: opening NVS");
        return -1;
    }

    // Get the version that the data was saved against.
    /*err = nvs_get_u32(handle, KEY_VERSION, &version);
    if (err != ESP_OK) {
        printf(">> ERROR: no version record found on NVS\n");
        nvs_close(handle);
        return -1;
    }*/

    // Check the versions match
    /*if ((version & 0xff00) != (g_version & 0xff00)) {
        printf(">> ERROR: incompatible versions - current is %x, found is %x\n", version, g_version);
        nvs_close(handle);
        return -1;
    }*/

    size = sizeof(op_settings_info_t);
    err = nvs_get_blob(handle, KEY_SETTINGS_INFO, &pOpSettingsInfo, &size);
    if (err != ESP_OK)
    {
        DEBUG_PRINT_E("getOpSettingsInfo", "No op settings record found!");
        nvs_close(handle);
        return -1;
    }

    // cleanup
    nvs_close(handle);

    *_op_settings = pOpSettingsInfo;
    return 0;
}

// Save our configuration info for retrieval on a subsequent restart.
void saveOpSettingsInfo(op_settings_info_t *pOpSettingsInfo)
{
    nvs_handle handle;
    ESP_ERROR_CHECK(nvs_open(BOOTWIFI_NAMESPACE, NVS_READWRITE, &handle));
    ESP_ERROR_CHECK(nvs_set_blob(handle, KEY_SETTINGS_INFO, pOpSettingsInfo, sizeof(op_settings_info_t)));
    // ESP_ERROR_CHECK(nvs_set_u32(handle, KEY_VERSION, g_version));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);
}

int wifiInit(uint8_t force_ap){
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    mdns_init();
    mdns_hostname_set(MDNS_HOST_NAME);
    mdns_instance_name_set(MDNS_INSTANCE);
    mdns_txt_item_t serviceTxtData[] = {{"board", "esp32"}, {"path", "/"}};
    ESP_ERROR_CHECK(mdns_service_add("ESP32-WebServer", "_http", "_tcp", 80, serviceTxtData, sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
    ESP_ERROR_CHECK(mdns_service_add("ESP32-WebServer", "_https", "_tcp", 443, serviceTxtData, sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
    netbiosns_init();
    netbiosns_set_name(MDNS_HOST_NAME);

    //Check if saved op_mode is access point
    if (force_ap || !strcmp(op_settings.com_mode, COM_MODE_TCP_AP) || !strcmp(op_settings.com_mode, COM_MODE_WS_AP)){
        wifi_init_softap();
        return ESP_OK;
    }else{
        //Return result of attempt connection to saved SSID
        return wifi_init_sta();
    }
}

uint8_t isComModeWifi(void){
    return (!strcmp(op_settings.com_mode, COM_MODE_TCP_AP) || !strcmp(op_settings.com_mode, COM_MODE_TCP_STA) || !strcmp(op_settings.com_mode, COM_MODE_UDP_STA) || !strcmp(op_settings.com_mode, COM_MODE_WS_AP));
}
