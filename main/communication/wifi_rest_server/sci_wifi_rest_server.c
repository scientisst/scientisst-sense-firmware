/** \file wifi_rest_server.c
    \brief This file contains the code for the RESTful API server.

    This file contains the code for the HTTP Restful API Server

    This example code is in the Public Domain (or CC0 licensed, at your option.)

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/

#include "communication/wifi_rest_server/sci_wifi_rest_server.h"

#include <fcntl.h>
#include <string.h>

#include "cJSON.h"
#include "drivers/include/sci_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_vfs_semihost.h"
#include "lwip/apps/netbiosns.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "sdmmc_cmd.h"

#include "sci_bt.h"
#include "sci_macros.h"
#include "sci_macros_conf.h"
#include "sci_scientisst.h"

#define WEB_MOUNT_POINT "/www" // Specify the mount point in VFS.

#define REST_CHECK(a, str, goto_tag, ...)                                                                                   \
    do                                                                                                                      \
    {                                                                                                                       \
        if (!(a))                                                                                                           \
        {                                                                                                                   \
            DEBUG_PRINT_E("esp-rest", "%s(%d): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__);                               \
            goto goto_tag;                                                                                                  \
        }                                                                                                                   \
    } while (0)

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 128)
#define SCRATCH_BUFSIZE (10240)

typedef struct rest_server_context
{
    char base_path[ESP_VFS_PATH_MAX + 1];
    char scratch[SCRATCH_BUFSIZE];
} rest_server_context_t;

#define CHECK_FILE_EXTENSION(filename, ext) (strcasecmp(&filename[strlen(filename) - strlen(ext)], ext) == 0)

static void parseHttpForm(char *buff);
static void opSettingsSaveMember(char *member, char *value);

/**
 * \brief Set HTTP response content type according to file extension
 *
 * \param req HTTP request
 * \param filepath File path
 *
 * \return //TODO: add details about return value
 */
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filepath)
{
    const char *type = "text/plain";
    if (CHECK_FILE_EXTENSION(filepath, ".html"))
    {
        type = "text/html";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".js"))
    {
        type = "application/javascript";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".css"))
    {
        type = "text/css";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".png"))
    {
        type = "image/png";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".ico"))
    {
        type = "image/x-icon";
    }
    else if (CHECK_FILE_EXTENSION(filepath, ".svg"))
    {
        type = "image/svg+xml";
    }
    return httpd_resp_set_type(req, type);
}

/**
 * \brief Send HTTP response with the contents of the requested file
 *
 * \param req HTTP request
 *
 * \return:
 *     - ESP_OK Success
 *     - ESP_FAIL Failed
 */
static esp_err_t rest_common_get_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];

    rest_server_context_t *rest_context = (rest_server_context_t *)req->user_ctx;
    strlcpy(filepath, rest_context->base_path, sizeof(filepath));
    if (req->uri[strlen(req->uri) - 1] == '/')
    {
        strlcat(filepath, "/index.html", sizeof(filepath));
    }
    else
    {
        strlcat(filepath, req->uri, sizeof(filepath));
    }
    int fd = open(filepath, O_RDONLY, 0);
    if (fd == -1)
    {
        DEBUG_PRINT_E("esp-rest", "Failed to open file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    set_content_type_from_file(req, filepath);

    char *chunk = rest_context->scratch;
    ssize_t read_bytes;
    do
    {
        /* Read file in chunks into the scratch buffer */
        read_bytes = read(fd, chunk, SCRATCH_BUFSIZE);
        if (read_bytes == -1)
        {
            DEBUG_PRINT_E("esp-rest", "Failed to read file : %s", filepath);
        }
        else if (read_bytes > 0)
        {
            int ret;
            /* Send the buffer contents as HTTP response chunk */
            if ((ret = httpd_resp_send_chunk(req, chunk, read_bytes)) != ESP_OK)
            {
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

    // op_settings are now saved, esp can reboot for those changes to take place
    if (!strcmp("/www/settings_done.html", filepath))
    {
        esp_restart();
    }
    return ESP_OK;
}

/**
 * \brief Simple handler for getting system handler
 *
 * \param req HTTP request
 *
 * \return:
 *    - ESP_OK Success
 *    - ESP_FAIL Failed
 */
static esp_err_t op_settings_get_handler(httpd_req_t *req)
{
    cJSON *root;
    cJSON *connectionInfo;
    cJSON *bitalinoInfo;

    httpd_resp_set_type(req, "application/json");

    root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "connectionInfo", connectionInfo = cJSON_CreateObject());
    cJSON_AddItemToObject(root, "bitalinoInfo", bitalinoInfo = cJSON_CreateObject());

    // if no error
    if (scientisst_device_settings.is_op_settings_valid)
    {
        DEBUG_PRINT_I("op_settings_get_handler", "Success reading from flash op_settings");
        cJSON_AddStringToObject(connectionInfo, "ssid", scientisst_device_settings.op_settings.ssid);
        cJSON_AddStringToObject(connectionInfo, "password", scientisst_device_settings.op_settings.password);
        cJSON_AddStringToObject(connectionInfo, "host_ip", scientisst_device_settings.op_settings.host_ip);
        cJSON_AddStringToObject(connectionInfo, "port_number", scientisst_device_settings.op_settings.port_number);
        cJSON_AddStringToObject(bitalinoInfo, "bit_when", scientisst_device_settings.op_settings.bit_when);
        cJSON_AddStringToObject(bitalinoInfo, "sampling_rate", scientisst_device_settings.op_settings.sampling_rate);
        cJSON_AddStringToObject(bitalinoInfo, "no_channels", scientisst_device_settings.op_settings.no_channels);
        cJSON_AddStringToObject(bitalinoInfo, "channels", scientisst_device_settings.op_settings.channels);
        cJSON_AddStringToObject(bitalinoInfo, "bit_mode", scientisst_device_settings.op_settings.bit_mode);
        cJSON_AddStringToObject(bitalinoInfo, "port_o1", scientisst_device_settings.op_settings.port_o1);
        cJSON_AddStringToObject(bitalinoInfo, "port_o2", scientisst_device_settings.op_settings.port_o2);
    }
    else
    {
        DEBUG_PRINT_W("op_settings_get_handler", "Failure reading from flash op_settings");
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

static esp_err_t op_settings_post_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int cur_len = 0;
    char *buf = ((rest_server_context_t *)(req->user_ctx))->scratch;
    int received = 0;
    if (total_len >= SCRATCH_BUFSIZE)
    {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "content too long");
        return ESP_FAIL;
    }
    while (cur_len < total_len)
    {
        received = httpd_req_recv(req, buf + cur_len, total_len);
        if (received <= 0)
        {
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    // Save op_settings in flash
    printf("%s \n", buf);
    parseHttpForm(buf);
    saveOpSettingsInfo(&(scientisst_device_settings.op_settings));

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_status(req, "301 Moved Permanently");
    httpd_resp_set_hdr(req, "Location", "/settings_done.html");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/**
 * \brief Parse recieved HTTP form and save each member in op_settings
 *
 * \param buff HTTP form
 */
static void parseHttpForm(char *buff)
{
    const char s[2] = "&";
    char *token;
    char *element_seperator = NULL;
    char *eostring = NULL;
    char key[64];
    char value[64];

    // get the first token
    token = strtok(buff, s);

    // walk through other tokens
    while (token != NULL)
    {
        element_seperator = strchr(token, '=');
        eostring = strchr(token, '\0');

        // Get key from token
        memcpy(key, token, element_seperator - token);
        key[element_seperator - token] = '\0';
        // Get value from token
        memcpy(value, element_seperator + 1, eostring - element_seperator);
        value[eostring - element_seperator] = '\0';

        // Save each member in global variable op_settings
        opSettingsSaveMember(key, value);

        token = strtok(NULL, s);
    }
}

/**
 * \brief Save each member in op_settings
 *
 * \param member Member of op_settings
 * \param value Value of member
 */
static void opSettingsSaveMember(char *member, char *value)
{
    if (!strcmp(member, "ssid"))
    {
        strcpy(scientisst_device_settings.op_settings.ssid, value);
    }
    else if (!strcmp(member, "password"))
    {
        strcpy(scientisst_device_settings.op_settings.password, value);
    }
    else if (!strcmp(member, "com_mode"))
    {
        if (!strcmp(value, "tcp_ap"))
            scientisst_device_settings.op_settings.com_mode = COM_MODE_TCP_AP;
        else if (!strcmp(value, "tcp_sta"))
            scientisst_device_settings.op_settings.com_mode = COM_MODE_TCP_STA;
        else if (!strcmp(value, "udp_sta"))
            scientisst_device_settings.op_settings.com_mode = COM_MODE_UDP_STA;
        else if (!strcmp(value, "bt"))
            scientisst_device_settings.op_settings.com_mode = COM_MODE_BT;
        else if (!strcmp(value, "serial"))
            scientisst_device_settings.op_settings.com_mode = COM_MODE_SERIAL;
        else if (!strcmp(value, "ws_ap"))
            scientisst_device_settings.op_settings.com_mode = COM_MODE_WS_AP;
        else if (!strcmp(value, "ble"))
            scientisst_device_settings.op_settings.com_mode = COM_MODE_BLE;
        else if (!strcmp(value, "sd_card"))
            scientisst_device_settings.op_settings.com_mode = COM_MODE_SD_CARD;
        else
            scientisst_device_settings.op_settings.com_mode = COM_MODE_BT;
    }
    else if (!strcmp(member, "host_ip"))
    {
        strcpy(scientisst_device_settings.op_settings.host_ip, value);
    }
    else if (!strcmp(member, "port_number"))
    {
        strcpy(scientisst_device_settings.op_settings.port_number, value);
    }
    else if (!strcmp(member, "bit_when"))
    {
        strcpy(scientisst_device_settings.op_settings.bit_when, value);
    }
    else if (!strcmp(member, "sampling_rate"))
    {
        strcpy(scientisst_device_settings.op_settings.sampling_rate, value);
    }
    else if (!strcmp(member, "no_channels"))
    {
        strcpy(scientisst_device_settings.op_settings.no_channels, value);
    }
    else if (!strcmp(member, "channels"))
    {
        strcpy(scientisst_device_settings.op_settings.channels, value);
    }
    else if (!strcmp(member, "bit_mode"))
    {
        strcpy(scientisst_device_settings.op_settings.bit_mode, value);
    }
    else if (!strcmp(member, "port_o1"))
    {
        strcpy(scientisst_device_settings.op_settings.port_o1, value);
    }
    else if (!strcmp(member, "port_o2"))
    {
        strcpy(scientisst_device_settings.op_settings.port_o2, value);
    }
    else
    {
        DEBUG_PRINT_E("opSettingsSaveMember", "Invalid member: %s", member);
    }
}

/**
 * \brief Start REST server
 *
 * \param base_path Base path of REST server
 *
 * \return:
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t start_rest_server(const char *base_path)
{
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
        .uri = "/nvs", .method = HTTP_GET, .handler = op_settings_get_handler, .user_ctx = rest_context};
    httpd_register_uri_handler(server, &op_settings_get_uri);

    /* URI handler for light brightness control */
    httpd_uri_t op_settings_post_uri = {
        .uri = "/settingsDone", .method = HTTP_POST, .handler = op_settings_post_handler, .user_ctx = rest_context};
    httpd_register_uri_handler(server, &op_settings_post_uri);

    /* URI handler for getting web server files */
    httpd_uri_t common_get_uri = {
        .uri = "/*", .method = HTTP_GET, .handler = rest_common_get_handler, .user_ctx = rest_context};
    httpd_register_uri_handler(server, &common_get_uri);

    return ESP_OK;
err_start:
    free(rest_context);
err:
    return ESP_FAIL;
}

// restful_server_main.c-------------------------------------------------------------------------------------------------------------------

/**
 * \brief Initialize SPIFFS
 *
 * \return:
 *    - ESP_OK Success
 *    - ESP_FAIL Fail
 */
esp_err_t init_fs_www(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = WEB_MOUNT_POINT, .partition_label = NULL, .max_files = 5, .format_if_mount_failed = false};
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            DEBUG_PRINT_E("init_fs_www", "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            DEBUG_PRINT_E("init_fs_www", "Failed to find SPIFFS partition");
        }
        else
        {
            DEBUG_PRINT_E("init_fs_www", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK)
    {
        DEBUG_PRINT_E("init_fs_www", "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        DEBUG_PRINT_I("init_fs_www", "Partition size: total: %d, used: %d", total, used);
    }
    return ESP_OK;
}

/**
 * \brief Initialize REST server
 */
void initRestServer(void)
{
    ESP_ERROR_CHECK(init_fs_www());
    ESP_ERROR_CHECK(start_rest_server(WEB_MOUNT_POINT));
}
