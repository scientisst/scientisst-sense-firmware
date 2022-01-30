#ifndef _MACROS_H
#define _MACROS_H

#include <stdio.h>
#include <stdarg.h>
#include "esp_log.h"

#define GEN_ERR -1
#define GEN_OK 1
#define DEFAULT_TASK_STACK_SIZE 2048
#define _DEBUG_ 2
#define ONE_HOUR_MS 3600000


#if (_DEBUG_ == 1)
    //This macros is only define if _DEBUG_ is defined
    #define DEBUG_PRINT_I(func, ... )({\
        ESP_LOGI((func), __VA_ARGS__);       \
    })
    #define DEBUG_PRINT_W(func, ... )({\
        ESP_LOGW((func), __VA_ARGS__);       \
    })
    #define DEBUG_PRINT_E(func, ... )({\
        ESP_LOGE((func), __VA_ARGS__);       \
    })

#elif(_DEBUG_ == 2)
    #define DEBUG_PRINT_I(...)
    #define DEBUG_PRINT_W(func, ... )({\
        ESP_LOGW((func), __VA_ARGS__);       \
    })
    #define DEBUG_PRINT_E(func, ... )({\
        ESP_LOGE((func), __VA_ARGS__);       \
    })
#else
    #define DEBUG_PRINT_I(...)
    #define DEBUG_PRINT_W(...)
    #define DEBUG_PRINT_E(...)
#endif


/*
#if (_DEBUG_ == 1)
    //Usage: mode= E (ERROR), I (INFO), W (WARNING) | Rest of arguments: just like in printf, string followed by the respective args
    //This macros is only define if _DEBUG_ is defined
    #define DEBUG_PRINT(mode, func, ... )({\
        ESP_LOG##mode((func), __VA_ARGS__);       \
    })
#else
    #define DEBUG_PRINT(...)
#endif
*/

#ifndef MIN
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#endif

#endif