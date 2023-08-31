/**
 * \file config.h
 * \brief Configuration file
 *
 * This file contains the configuration of the project.
 * It is used to enable/disable features and to select the hardware version.
 *
 */

#ifndef _CONFIG_H
#define _CONFIG_H

#include "macros_conf.h"

/*************************
 * GENERAL CONFIGURATION *
 *************************/
// Possible values for HW_VERSION_:
//   - HW_VERSION_CORE (Core) [Default]
//   - HW_VERSION_NANO (Nano)
//   - HW_VERSION_CARDIO (Cardio)
#define HW_VERSION HW_VERSION_CORE

// Possible values for _ADC_EXT_:
//   - NO_EXT_ADC (No external adc) [Default]
//   - ADC_MCP (Enable external adc)
#define _ADC_EXT_ ADC_MCP

// Possible values for _TIMESTAMP_:
//   - TIMESTAMP_DISABLED (Disable timestamp) [Default]
//   - TIMESTAMP_ENABLED (Enable timestamp on AX1 and AX2 channels)
#define _TIMESTAMP_ TIMESTAMP_ENABLED

/************************
 * SDCARD CONFIGURATION *
 ************************/
// Possible values for _SD_CARD_ENABLED_:
//   - SD_CARD_DISABLED (Disable sd card) [Default]
//   - SD_CARD_ENABLED (Enable sd card)
#define _SD_CARD_ENABLED_ SD_CARD_ENABLED

// Possible values for FORMAT_SDCARD_IF_MOUNT_FAILED:
//   - DO_NOT_FORMAT_SDCARD (Do not format sd card if mount failed) [Default]
//   - FORMAT_SDCARD (Format sd card if mount failed) (BE CAREFUL, THIS WILL
//     ERASE ALL DATA ON THE SD CARD)
#define FORMAT_SDCARD_IF_MOUNT_FAILED FORMAT_SDCARD

// Possible values for CONVERSION_MODE:
//   - RAW_ONLY (Only raw data)
//   - RAW_AND_MV (Raw data and millivolt) [Default]
#define CONVERSION_MODE RAW_AND_MV

/************************
 * CONFIGURATION CHECKS *
 ************************/
// Make sure no invalid parameters are written
// Make sure that no invalid combination of parameters is selected

// Check if the hardware version is valid
#if HW_VERSION != HW_VERSION_CORE && HW_VERSION != HW_VERSION_NANO && \
    HW_VERSION != HW_VERSION_CARDIO
#error invalid hardware version, check config.h
#endif

// Check if the external adc is valid
#if _ADC_EXT_ != NO_EXT_ADC && _ADC_EXT_ != ADC_MCP
#error invalid external adc configuration, check config.h
#endif

// Check if the sd card is valid
#if _SD_CARD_ENABLED_ != SD_CARD_DISABLED && \
    _SD_CARD_ENABLED_ != SD_CARD_ENABLED
#error invalid sd card configuration, check config.h
#endif

// Check if the sd card format is valid
#if FORMAT_SDCARD_IF_MOUNT_FAILED != DO_NOT_FORMAT_SDCARD && \
    FORMAT_SDCARD_IF_MOUNT_FAILED != FORMAT_SDCARD
#error invalid sd card format configuration, check config.h
#endif

// Check if the conversion mode is valid
#if CONVERSION_MODE != RAW_ONLY && CONVERSION_MODE != RAW_AND_MV
#error invalid conversion mode configuration, check config.h
#endif

// Check if the configuration is valid
// Timestamp requires that no external adc is enabled
#if _TIMESTAMP_ == TIMESTAMP_ENABLED && \
    (_ADC_EXT_ != NO_EXT_ADC && _SD_CARD_ENABLED_ == SD_CARD_DISABLED)
#error timestamp requires that no external adc is enabled, check config.h
#endif

// SD card requires that no external adc is enabled
#if _SD_CARD_ENABLED_ == SD_CARD_ENABLED && _ADC_EXT_ != NO_EXT_ADC
#define MULTIPLE_SPI_DEVICES 1
// #error sd card requires that no external adc is enabled, check config.h
#endif

#endif
