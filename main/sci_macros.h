/**
 * \file macros.h
 * \brief Macro Definitions and Global Parameters for the Project.
 *
 * This file establishes various macro definitions and parameters that maintain global project standards for task stack
 * sizes, buffer limits, operational modes, and debugging levels.
 */

#pragma once

#include "esp_log.h"
#include "esp_spp_api.h"
#include "sdkconfig.h"

// Task stack sizes for different levels of task complexity.
#define DEFAULT_TASK_STACK_SIZE_SMALL 2048
#define DEFAULT_TASK_STACK_SIZE_MEDIUM 4096
#define DEFAULT_TASK_STACK_SIZE_LARGE 8192
#define DEFAULT_TASK_STACK_SIZE_XLARGE 16384

// Buffer management constants, defining limits and capacities.
#define NUM_BUFFERS 50
#define NUM_BUFFERS_SDCARD 16
#define MAX_BUFFER_SIZE (ESP_SPP_MAX_MTU) ///< Synchronized with maximum MTU size for ESP.
#define MAX_BUFFER_SIZE_SDCARD (1024 * 7) ///< Specific buffer size for SD Card operations.

#define GATTS_NOTIFY_LEN 517 ///< Maximum data length for BLE notifications.

#define DEFAULT_SAMPLE_RATE 1     // Default data sampling rate in Hz.
#define BATTERY_CHECK_FREQUENCY 1 ///< Frequency of battery status checks (in Hz).

#define DEFAULT_ADC_CHANNELS 6
#define EXT_ADC_CHANNELS 2

#define DEFAULT_BATTERY_THRESHOLD 3500 ///< Battery threshold level in millivolts (mV).

#define BT_DEFAULT_DEVICE_NAME "ScientISST\0" ///< Default name of the device for Bluetooth communication.

/**
 * \brief Global macro functions for debug logging.
 *
 * These macros are wrappers of ESP_LOG functions and handle debug logging at various levels of verbosity, controlled by
 * the build configuration.
 */
// clang-format off
#ifdef CONFIG_SCI_DEBUG_INFO_WARNINGS_AND_ERRORS
// Debugging macros for detailed logging.
#define DEBUG_PRINT_I(func, ...) do { ESP_LOGI((func), __VA_ARGS__); } while (0)
#define DEBUG_PRINT_W(func, ...) do { ESP_LOGW((func), __VA_ARGS__); } while (0)
#define DEBUG_PRINT_E(func, ...) do { ESP_LOGE((func), __VA_ARGS__); } while (0)
#endif
#ifdef CONFIG_SCI_DEBUG_WARNINGS_AND_ERRORS
// Debugging macros excluding informational logs.
#define DEBUG_PRINT_I(...) do {} while (0)
#define DEBUG_PRINT_W(func, ...) do { ESP_LOGW((func), __VA_ARGS__); } while (0)
#define DEBUG_PRINT_E(func, ...) do { ESP_LOGE((func), __VA_ARGS__); } while (0)
#endif
#ifdef CONFIG_SCI_DEBUG_ERRORS
// Debugging macros for critical logs only.
#define DEBUG_PRINT_I(...) do {} while (0)
#define DEBUG_PRINT_W(...) do {} while (0)
#define DEBUG_PRINT_E(func, ...) do { ESP_LOGE((func), __VA_ARGS__); } while (0)
#endif
#ifdef CONFIG_SCI_DEBUG_NO_DEBUGGING
// Debugging disabled.
#define DEBUG_PRINT_I(...) do {} while (0)
#define DEBUG_PRINT_W(...) do {} while (0)
#define DEBUG_PRINT_E(...) do {} while (0)
#endif
// clang-format on

/**
 * \brief Macro to check for null pointers and abort operation.
 *
 * This macro is a safety check used throughout the code to quickly verify pointer
 * validity. It helps prevent dereferencing null pointers and undefined behavior.
 */
#define CHECK_NOT_NULL(ptr)                                                                                                 \
    do                                                                                                                      \
    {                                                                                                                       \
        if ((ptr) == NULL)                                                                                                  \
        {                                                                                                                   \
            DEBUG_PRINT_E("Memory allocation failed.");                                                                     \
            abort();                                                                                                        \
        }                                                                                                                   \
    } while (0)

/**
 * \brief Determines if the communication type is WiFi-based.
 *
 * This macro checks if the current communication mode is one of the various
 * WiFi modes, simplifying conditional checks and improving readability.
 */
#define IS_COM_TYPE_WIFI(_com_mode)                                                                                         \
    ((_com_mode == COM_MODE_TCP_AP) || (_com_mode == COM_MODE_TCP_STA) || (_com_mode == COM_MODE_UDP_STA) ||                \
     (_com_mode == COM_MODE_WS_AP))

/**
 * \brief Enumerations for various operational modes.
 *
 * These enums define the high-level communication and operational modes
 * the system can be in, affecting data handling, communication, and control flow.
 */
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
    API_MODE_SCIENTISST = 2,
    API_MODE_JSON = 3,
    API_MODE_SCIENTISST_V2 = 14,
} api_mode_t;
