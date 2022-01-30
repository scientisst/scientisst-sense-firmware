/* HTTP Restful API Server

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <fcntl.h>
#include "esp_http_server.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "cJSON.h"
#include "sdkconfig.h"
#include "esp_vfs_semihost.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"
#include "sdmmc_cmd.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "mdns.h"
#include "lwip/apps/netbiosns.h"
#include "macros.h"
#include "main.h"
#include "bt.h"
#include "wifi_rest_server.h"

#define MDNS_HOST_NAME "scientisst"   //Specify the domain name used in the mDNS service. Note that webpage also take it as a part of URL where it will send GET/POST requests to.
#define WEB_MOUNT_POINT "/www"      //Specify the mount point in VFS.
#define MDNS_INSTANCE "esp home web server"

// if the structure of a record saved for a subsequent reboot changes
// then consider using semver to change the version number or else
// we may try and boot with the wrong data.
//#define KEY_VERSION "version"
//uint32_t g_version = 0x0100;

#define KEY_SETTINGS_INFO "opSettingsInfo" // key used in NVS for connection info
#define BOOTWIFI_NAMESPACE "bootwifi" // namespace in NVS for bootwifi

#define REST_CHECK(a, str, goto_tag, ...)                                              \
    do                                                                                 \
    {                                                                                  \
        if (!(a))                                                                      \
        {                                                                              \
            DEBUG_PRINT_E("esp-rest", "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            goto goto_tag;                                                             \
        }                                                                              \
    } while (0)

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 128)
#define SCRATCH_BUFSIZE (10240)

typedef struct rest_server_context {
    char base_path[ESP_VFS_PATH_MAX + 1];
    char scratch[SCRATCH_BUFSIZE];
} rest_server_context_t;

#define CHECK_FILE_EXTENSION(filename, ext) (strcasecmp(&filename[strlen(filename) - strlen(ext)], ext) == 0)

static int getOpSettingsInfo(op_settings_info_t *pOpSettingsInfo);
static void saveOpSettingsInfo(op_settings_info_t *pOpSettingsInfo);
static void parseHttpForm(char *buff);

/* Set HTTP response content type according to file extension */
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filepath)
{
    const char *type = "text/plain";
    if (CHECK_FILE_EXTENSION(filepath, ".html")) {
        type = "text/html";
    } else if (CHECK_FILE_EXTENSION(filepath, ".js")) {
        type = "application/javascript";
    } else if (CHECK_FILE_EXTENSION(filepath, ".css")) {
        type = "text/css";
    } else if (CHECK_FILE_EXTENSION(filepath, ".png")) {
        type = "image/png";
    } else if (CHECK_FILE_EXTENSION(filepath, ".ico")) {
        type = "image/x-icon";
    } else if (CHECK_FILE_EXTENSION(filepath, ".svg")) {
        type = "text/xml";
    }
    return httpd_resp_set_type(req, type);
}

/* Send HTTP response with the contents of the requested file */
static esp_err_t rest_common_get_handler(httpd_req_t *req){
    char filepath[FILE_PATH_MAX];

    rest_server_context_t *rest_context = (rest_server_context_t *)req->user_ctx;
    strlcpy(filepath, rest_context->base_path, sizeof(filepath));
    if (req->uri[strlen(req->uri) - 1] == '/') {
        strlcat(filepath, "/index.html", sizeof(filepath));
    } else {
        strlcat(filepath, req->uri, sizeof(filepath));
    }
    int fd = open(filepath, O_RDONLY, 0);
    if (fd == -1) {
        DEBUG_PRINT_E("esp-rest", "Failed to open file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    set_content_type_from_file(req, filepath);

    char *chunk = rest_context->scratch;
    ssize_t read_bytes;
    do{
        /* Read file in chunks into the scratch buffer */
        read_bytes = read(fd, chunk, SCRATCH_BUFSIZE);
        if (read_bytes == -1) {
            DEBUG_PRINT_E("esp-rest", "Failed to read file : %s", filepath);
        } else if (read_bytes > 0) {
            int ret;
            /* Send the buffer contents as HTTP response chunk */
            if ((ret = httpd_resp_send_chunk(req, chunk, read_bytes)) != ESP_OK) {
                printf("ret = %d\n", ret);
                close(fd);
                DEBUG_PRINT_E("esp-rest", "File %s sending failed!", filepath);
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }
    } while (read_bytes > 0);
    /* Close file after sending complete */
    close(fd);
    DEBUG_PRINT_I("esp-rest", "File sending complete");
    /* Respond with an empty chunk to signal HTTP response completion */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* Simple handler for getting system handler */
static esp_err_t op_settings_get_handler(httpd_req_t *req){
    cJSON *root,*connectionInfo, *bitalinoInfo;
    op_settings_info_t opSettingsInfo;

    httpd_resp_set_type(req, "application/json");

    root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "connectionInfo", connectionInfo = cJSON_CreateObject());
    cJSON_AddItemToObject(root, "bitalinoInfo", bitalinoInfo = cJSON_CreateObject());

    //if no error
    if(!getOpSettingsInfo(&opSettingsInfo)){
        cJSON_AddStringToObject(connectionInfo, "ssid", opSettingsInfo.ssid);
        cJSON_AddStringToObject(connectionInfo, "password", opSettingsInfo.password);
        cJSON_AddStringToObject(connectionInfo, "host_ip", opSettingsInfo.host_ip);
        cJSON_AddStringToObject(connectionInfo, "port_number", opSettingsInfo.port_number);
        cJSON_AddStringToObject(bitalinoInfo, "bit_when", opSettingsInfo.bit_when);
        cJSON_AddStringToObject(bitalinoInfo, "sampling_rate", opSettingsInfo.sampling_rate);	
        cJSON_AddStringToObject(bitalinoInfo, "no_channels", opSettingsInfo.no_channels);
        cJSON_AddStringToObject(bitalinoInfo, "channels", opSettingsInfo.channels);		
        cJSON_AddStringToObject(bitalinoInfo, "bit_mode", opSettingsInfo.bit_mode);
        cJSON_AddStringToObject(bitalinoInfo, "port_o1", opSettingsInfo.port_o1);	
        cJSON_AddStringToObject(bitalinoInfo, "port_o2", opSettingsInfo.port_o2);
    }else{					
        cJSON_AddStringToObject(connectionInfo, "ssid", "SSID");
        cJSON_AddStringToObject(connectionInfo, "password", "Password");
        cJSON_AddStringToObject(connectionInfo, "host_ip", "Host IP Address");
        cJSON_AddStringToObject(connectionInfo, "port_number", "Port Number");
        cJSON_AddStringToObject(bitalinoInfo, "bit_when", "bit_now");
        cJSON_AddStringToObject(bitalinoInfo, "sampling_rate", "1");	
        cJSON_AddStringToObject(bitalinoInfo, "no_channels", "1");
        cJSON_AddStringToObject(bitalinoInfo, "channels", "A1");		
        cJSON_AddStringToObject(bitalinoInfo, "bit_mode", "live");
        cJSON_AddStringToObject(bitalinoInfo, "port_o1", "low");
        cJSON_AddStringToObject(bitalinoInfo, "port_o2", "low");
    }

    const char *renderedJSON = cJSON_Print(root);
    httpd_resp_sendstr(req, renderedJSON);
    free((void *)renderedJSON);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t op_settings_post_handler(httpd_req_t *req){
    int total_len = req->content_len;
    int cur_len = 0;
    char *buf = ((rest_server_context_t *)(req->user_ctx))->scratch;
    int received = 0;
    if (total_len >= SCRATCH_BUFSIZE) {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "content too long");
        return ESP_FAIL;
    }
    while (cur_len < total_len) {
        received = httpd_req_recv(req, buf + cur_len, total_len);
        if (received <= 0) {
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    printf("%s \n", buf);
    parseHttpForm(buf);

    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_status(req, "301 Moved Permanently");
    httpd_resp_set_hdr(req, "Location", "/settings_done.html");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static void parseHttpForm(char *buff){
    const char s[2] = "&";
    char *token;
    char *element_seperator = NULL;
    char *eostring = NULL;
    char key[BIGGEST_SIZE];
    char value[BIGGEST_SIZE];
    
    //get the first token 
    token = strtok(buff, s);
    
    //walk through other tokens
    while(token != NULL){
        element_seperator = strchr(token, '=');
        eostring = strchr(token, '\0');

        //Get key from token
        memcpy(key, token, element_seperator-token);
        key[element_seperator-token] = '\0';
        //Get value from token
        memcpy(value, element_seperator+1, eostring-element_seperator);
        value[eostring-element_seperator] = '\0';

        printf("token: %s, key: %s, value: %s\n", token, key, value);

        token = strtok(NULL, s);
    }
}

static void OpSettingsSaveMember(char *member, char *value){
     
}

esp_err_t start_rest_server(const char *base_path){
    REST_CHECK(base_path, "wrong base path", err);
    rest_server_context_t *rest_context = calloc(1, sizeof(rest_server_context_t));
    REST_CHECK(rest_context, "No memory for rest context", err);
    strlcpy(rest_context->base_path, base_path, sizeof(rest_context->base_path));

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;

    DEBUG_PRINT_I("esp-rest", "Starting HTTP Server");
    REST_CHECK(httpd_start(&server, &config) == ESP_OK, "Start server failed", err_start);

    /* URI handler for fetching system info */
    httpd_uri_t op_settings_get_uri = {
        .uri = "/nvs",
        .method = HTTP_GET,
        .handler = op_settings_get_handler,
        .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &op_settings_get_uri);

    /* URI handler for light brightness control */
    httpd_uri_t op_settings_post_uri = {
        .uri = "/settingsDone",
        .method = HTTP_POST,
        .handler = op_settings_post_handler,
        .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &op_settings_post_uri);

    /* URI handler for getting web server files */
    httpd_uri_t common_get_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = rest_common_get_handler,
        .user_ctx = rest_context
    };
    httpd_register_uri_handler(server, &common_get_uri);

    return ESP_OK;
err_start:
    free(rest_context);
err:
    return ESP_FAIL;
}

//restful_server_main.c-------------------------------------------------------------------------------------------------------------------

static void initialise_mdns(void){
    mdns_init();
    mdns_hostname_set(MDNS_HOST_NAME);
    mdns_instance_name_set(MDNS_INSTANCE);

    mdns_txt_item_t serviceTxtData[] = {
        {"board", "esp32"},
        {"path", "/"}
    };

    ESP_ERROR_CHECK(mdns_service_add("ESP32-WebServer", "_http", "_tcp", 80, serviceTxtData, sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
}

esp_err_t init_fs(void){
    esp_vfs_spiffs_conf_t conf = {
        .base_path = WEB_MOUNT_POINT,
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            DEBUG_PRINT_E("init_fs", "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            DEBUG_PRINT_E("init_fs", "Failed to find SPIFFS partition");
        } else {
            DEBUG_PRINT_E("init_fs", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        DEBUG_PRINT_E("init_fs", "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        DEBUG_PRINT_I("init_fs", "Partition size: total: %d, used: %d", total, used);
    }
    return ESP_OK;
}

void initRestServer(void){
    ESP_ERROR_CHECK(esp_netif_init());
    //ESP_ERROR_CHECK(esp_event_loop_create_default());     commented cuz it's already called before in wifi.c
    initialise_mdns();
    netbiosns_init();
    netbiosns_set_name(MDNS_HOST_NAME);

    //ESP_ERROR_CHECK(example_connect());
    ESP_ERROR_CHECK(init_fs());
    ESP_ERROR_CHECK(start_rest_server(WEB_MOUNT_POINT));
}

/**
 * retrieve the connection info. if rc == 0 means ok.
 */
static int getOpSettingsInfo(op_settings_info_t *pOpSettingsInfo){
	nvs_handle handle;
	size_t size;
	esp_err_t err;
	//uint32_t version;

	err = nvs_open(BOOTWIFI_NAMESPACE, NVS_READWRITE, &handle);
	if (err != 0) {
		printf(">> ERROR: opening NVS\n");
		return -1;
	}

	// get the version that the data was saved against.
	/*err = nvs_get_u32(handle, KEY_VERSION, &version);
	if (err != ESP_OK) {
		printf(">> ERROR: no version record found on NVS\n");
		nvs_close(handle);
		return -1;
	}*/

	// check the versions match
	/*if ((version & 0xff00) != (g_version & 0xff00)) {
		printf(">> ERROR: incompatible versions - current is %x, found is %x\n", version, g_version);
		nvs_close(handle);
		return -1;
	}*/

	size = sizeof(op_settings_info_t);
	err = nvs_get_blob(handle, KEY_SETTINGS_INFO, pOpSettingsInfo, &size);
	if (err != ESP_OK) {
		printf(">> ERROR: no connection record found\n");
		nvs_close(handle);
		return -1;
	}

	// cleanup
	nvs_close(handle);

	// do a sanity check on the SSID
	if (strlen(pOpSettingsInfo->ssid) == 0) {
		printf(">> ERROR: NULL SSID detected\n");
		return -1;
	}
	return 0;
}


/**
 * save our configuration info for retrieval on a subsequent restart.
 */
static void saveOpSettingsInfo(op_settings_info_t *pOpSettingsInfo){
	nvs_handle handle;
	ESP_ERROR_CHECK(nvs_open(BOOTWIFI_NAMESPACE, NVS_READWRITE, &handle));
	ESP_ERROR_CHECK(nvs_set_blob(handle, KEY_SETTINGS_INFO, pOpSettingsInfo, sizeof(op_settings_info_t)));
	//ESP_ERROR_CHECK(nvs_set_u32(handle, KEY_VERSION, g_version));
	ESP_ERROR_CHECK(nvs_commit(handle));
	nvs_close(handle);
}