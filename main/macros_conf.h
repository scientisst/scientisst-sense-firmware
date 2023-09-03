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
#define NO_EXT_ADC 1 // External ADC disabled
#define ADC_ADS 2    // ADS
#define ADC_MCP 3

// Hardware version possible options
#define HW_VERSION_CORE 1
#define HW_VERSION_NANO 2
#define HW_VERSION_CARDIO 3

// SD card possible options
#define SD_CARD_DISABLED 1
#define SD_CARD_ENABLED 2

// SD card format possible options
#define DO_NOT_FORMAT_SDCARD 1
#define FORMAT_SDCARD 2

// Timestamp possible options
#define TIMESTAMP_DISABLED 1
#define TIMESTAMP_ENABLED 2

// Conversion mode possible options
#define RAW_ONLY 1
#define RAW_AND_MV 2

#endif // SCIENTISST_SENSE_FIRMWARE_MACROS_CONF_H
