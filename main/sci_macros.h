/** \file macros.h
    \brief This file contains the definitions of the macros used in the project.

    This file contains the definitions of the macros used in the project. It
   contains MIN, DEBUG_PRINT, and GEN_ERR and GEN_OK macros (among some others).
   The DEBUG_PRINT macro is used to print debug messages. It can be used in two
   ways: _DEBUG_ = 1: DEBUG_PRINT_I, DEBUG_PRINT_W, DEBUG_PRINT_E (all debug
   messages) _DEBUG_ = 2: DEBUG_PRINT_W, DEBUG_PRINT_E (only warnings and
   errors)
*/

#pragma once

#include "esp_log.h"
#include "esp_spp_api.h"
#include "sdkconfig.h"

#define DEFAULT_TASK_STACK_SIZE_SMALL 2048
#define DEFAULT_TASK_STACK_SIZE_MEDIUM 4096
#define DEFAULT_TASK_STACK_SIZE_LARGE 8192
#define DEFAULT_TASK_STACK_SIZE_XLARGE 16384

#define NUM_BUFFERS 50
#define NUM_BUFFERS_SDCARD 16
#define MAX_BUFFER_SIZE (ESP_SPP_MAX_MTU) // If changed, change in API
#define MAX_BUFFER_SIZE_SDCARD (1024 * 7)

#define GATTS_NOTIFY_LEN 517 //< Maximum length of the buffer to send in BLE mode

#define DEFAULT_SAMPLE_RATE 1     // In Hz
#define BATTERY_CHECK_FREQUENCY 1 // 1 Hz

#define DEFAULT_ADC_CHANNELS 6 // Default number of active adc channels
#define EXT_ADC_CHANNELS 2     // Num of external adc channels

#define DEFAULT_BATTERY_THRESHOLD 3500 ///< mV

#define BT_DEFAULT_DEVICE_NAME "ScientISST\0"

/**************************
 * GLOBAL MACRO FUNCTIONS *
 **************************/
// clang-format off
#ifdef CONFIG_SCI_DEBUG_INFO_WARNINGS_AND_ERRORS
// This macros is only define if _DEBUG_ is defined
#define DEBUG_PRINT_I(func, ...) do { ESP_LOGI((func), __VA_ARGS__); } while (0)
#define DEBUG_PRINT_W(func, ...) do { ESP_LOGW((func), __VA_ARGS__); } while (0)
#define DEBUG_PRINT_E(func, ...) do { ESP_LOGE((func), __VA_ARGS__); } while (0)
#endif
#ifdef CONFIG_SCI_DEBUG_WARNINGS_AND_ERRORS
#define DEBUG_PRINT_I(...) do {} while (0)
#define DEBUG_PRINT_W(func, ...) do { ESP_LOGW((func), __VA_ARGS__); } while (0)
#define DEBUG_PRINT_E(func, ...) do { ESP_LOGE((func), __VA_ARGS__); } while (0)
#endif
#ifdef CONFIG_SCI_DEBUG_ERRORS
#define DEBUG_PRINT_I(...) do {} while (0)
#define DEBUG_PRINT_W(...) do {} while (0)
#define DEBUG_PRINT_E(...) do { ESP_LOGE((func), __VA_ARGS__); } while (0)
#endif
#ifdef CONFIG_SCI_DEBUG_NO_DEBUGGING
#define DEBUG_PRINT_I(...) do {} while (0)
#define DEBUG_PRINT_W(...) do {} while (0)
#define DEBUG_PRINT_E(...) do {} while (0)
#endif
// clang-format on

#define CHECK_NOT_NULL(ptr)                                                                                                 \
    do                                                                                                                      \
    {                                                                                                                       \
        if ((ptr) == NULL)                                                                                                  \
        {                                                                                                                   \
            DEBUG_PRINT_E("Memory allocation failed.");                                                                     \
            abort();                                                                                                        \
        }                                                                                                                   \
    } while (0)

#define IS_COM_TYPE_WIFI(_com_mode)                                                                                         \
    ((_com_mode == COM_MODE_TCP_AP) || (_com_mode == COM_MODE_TCP_STA) || (_com_mode == COM_MODE_UDP_STA) ||                \
     (_com_mode == COM_MODE_WS_AP))

/************************
 * GLOBAL ENUMS/STRUCTS *
 ************************/

typedef enum
{
    COM_MODE_TCP_AP = 1,
    COM_MODE_TCP_STA,
    COM_MODE_UDP_STA,
    COM_MODE_BT,
    COM_MODE_SERIAL,
    COM_MODE_WS_AP,
    COM_MODE_BLE,
    COM_MODE_SD_CARD,
} com_mode_t;

typedef enum
{
    OP_MODE_IDLE = 1,
    OP_MODE_LIVE,
    OP_MODE_CONFIG,
} operation_mode_t;

typedef enum
{
    API_MODE_BITALINO = 1,
    API_MODE_SCIENTISST,
    API_MODE_JSON,
} api_mode_t;
