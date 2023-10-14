/** \file macros.h
    \brief This file contains the definitions of the macros used in the project.

    This file contains the definitions of the macros used in the project. It
   contains MIN, DEBUG_PRINT, and GEN_ERR and GEN_OK macros (among some others).
   The DEBUG_PRINT macro is used to print debug messages. It can be used in two
   ways: _DEBUG_ = 1: DEBUG_PRINT_I, DEBUG_PRINT_W, DEBUG_PRINT_E (all debug
   messages) _DEBUG_ = 2: DEBUG_PRINT_W, DEBUG_PRINT_E (only warnings and
   errors)
*/

#ifndef _MACROS_H
#define _MACROS_H

#include <stdarg.h>
#include <stdio.h>

#include "esp_log.h"

#define DEFAULT_TASK_STACK_SIZE 2048
#define _DEBUG_ 1 ///< 0: No debug, 1: Warning and Error only, 2: Full Debug

#if (_DEBUG_ == 2)
// This macros is only define if _DEBUG_ is defined
#define DEBUG_PRINT_I(func, ...) ({ ESP_LOGI((func), __VA_ARGS__); })
#define DEBUG_PRINT_W(func, ...) ({ ESP_LOGW((func), __VA_ARGS__); })
#define DEBUG_PRINT_E(func, ...) ({ ESP_LOGE((func), __VA_ARGS__); })

#elif (_DEBUG_ == 1)
#define DEBUG_PRINT_I(...)
#define DEBUG_PRINT_W(func, ...) ({ ESP_LOGW((func), __VA_ARGS__); })
#define DEBUG_PRINT_E(func, ...) ({ ESP_LOGE((func), __VA_ARGS__); })
#else
#define DEBUG_PRINT_I(...)
#define DEBUG_PRINT_W(...)
#define DEBUG_PRINT_E(...)
#endif

#define CHECK_NOT_NULL(ptr)                                                                                  \
    do                                                                                                       \
    {                                                                                                        \
        if ((ptr) == NULL)                                                                                   \
        {                                                                                                    \
            DEBUG_PRINT_E("Memory allocation failed.");                                                      \
            abort();                                                                                         \
        }                                                                                                    \
    } while (0)

#endif
