/**
 * \file macros_conf.h
 * \brief Hidden macros for configuration file
 *
 * Separate macros for possible options from configuration file.
 * This makes it easier to change the configuration file and not change these
 * values by accident.
 *
 */

#ifndef SCIENTISST_SENSE_FIRMWARE_MACROS_CONF_H
#define SCIENTISST_SENSE_FIRMWARE_MACROS_CONF_H

// External ADC possible options
#define NO_EXT_ADC 0  // External ADC disabled
#define ADC_ADS 1     // ADS
#define ADC_MCP 2

// Hardware version possible options
#define HW_VERSION_CORE 0
#define HW_VERSION_NANO 1
#define HW_VERSION_CARDIO 2

// SD card possible options
#define SD_CARD_DISABLED 0
#define SD_CARD_ENABLED 1

// SD card format possible options
#define DO_NOT_FORMAT_SDCARD 0
#define FORMAT_SDCARD 1

// Timestamp possible options
#define TIMESTAMP_DISABLED 0
#define TIMESTAMP_ENABLED 1

#endif  // SCIENTISST_SENSE_FIRMWARE_MACROS_CONF_H
