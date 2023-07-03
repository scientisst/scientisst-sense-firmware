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

// Possible values for _ADC_EXT_:
//   - NO_EXT_ADC (No external adc) [Default]
//   - ADC_MCP (Enable external adc)
#define _ADC_EXT_ NO_EXT_ADC

// Possible values for HW_VERSION_:
//   - HW_VERSION_CORE (Core and Cardio) [Default]
//   - HW_VERSION_NANO (Nano)
//   - HW_VERSION_CARDIO (Cardio)
#define HW_VERSION HW_VERSION_CORE

// Possible values for _SD_CARD_ENABLED_:
//   - SD_CARD_DISABLED (Disable sd card) [Default]
//   - SD_CARD_ENABLED (Enable sd card)
#define _SD_CARD_ENABLED_ SD_CARD_ENABLED

// Possible values for FORMAT_SDCARD_IF_MOUNT_FAILED:
//   - DO_NOT_FORMAT_SDCARD (Do not format sd card if mount failed) [Default]
//   - FORMAT_SDCARD (Format sd card if mount failed) (BE CAREFUL, THIS WILL
//   ERASE ALL DATA ON THE SD CARD)
#define FORMAT_SDCARD_IF_MOUNT_FAILED DO_NOT_FORMAT_SDCARD

// Possible values for _TIMESTAMP_:
//   - TIMESTAMP_DISABLED (Disable timestamp) [Default]
//   - TIMESTAMP_ENABLED (Enable timestamp on AX1 and AX2 channels)
#define _TIMESTAMP_ TIMESTAMP_DISABLED

// Check if the configuration is valid
// Timestamp requires that no external adc is enabled
#if _TIMESTAMP_ == 1 && _ADC_EXT_ != NO_EXT_ADC
#error timestamp requires that no external adc is enabled
#endif

// SD card requires that no external adc is enabled
#if _SD_CARD_ENABLED_ == 1 && _ADC_EXT_ != NO_EXT_ADC
#error sd card requires that no external adc is enabled
#endif

#endif
