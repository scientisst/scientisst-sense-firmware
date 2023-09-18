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
#define _TIMESTAMP_ TIMESTAMP_DISABLED

/************************
 * SDCARD CONFIGURATION *
 ************************/
// Possible values for _SD_CARD_ENABLED_:
//   - SD_CARD_DISABLED (Disable sd card) [Default]
//   - SD_CARD_ENABLED (Enable sd card)
#define _SD_CARD_ENABLED_ SD_CARD_DISABLED

// Possible values for FORMAT_SDCARD_IF_MOUNT_FAILED:
//   - DO_NOT_FORMAT_SDCARD (Do not format sd card if mount failed) [Default]
//   - FORMAT_SDCARD (Format sd card if mount failed) (BE CAREFUL, THIS WILL
//     ERASE ALL DATA ON THE SD CARD)
#define FORMAT_SDCARD_IF_MOUNT_FAILED DO_NOT_FORMAT_SDCARD

/**********************
 * IMU CONFIGURATION *
 **********************/
// Possible values for _IMU_ENABLED_:
//   - IMU_DISABLED (Disable imu) [Default]
//   - IMU_ENABLED (Enable imu)
#define _IMU_ENABLED_ IMU_ENABLED

// Possible values for _IMU_CALIBRATION_:
//   - LOCK_IMU_ACQUISITION_UNTIL_CALIBRATED (Lock imu acquisition until calibrated)
//   - ALLOW_IMU_ACQUISITION_WHILE_CALIBRATING (Allow imu acquisition while calibrating) [Default]
#define _IMU_CALIBRATION_ LOCK_IMU_ACQUISITION_UNTIL_CALIBRATED

/************************
 * CONFIGURATION CHECKS *
 ************************/
// Make sure no invalid parameters are written
// Make sure that no invalid combination of parameters is selected

// Check if the hardware version is valid
#if HW_VERSION != HW_VERSION_CORE && HW_VERSION != HW_VERSION_NANO && HW_VERSION != HW_VERSION_CARDIO
#error invalid hardware version, check config.h
#endif

// Check if the external adc is valid
#if _ADC_EXT_ != NO_EXT_ADC && _ADC_EXT_ != ADC_MCP
#error invalid external adc configuration, check config.h
#endif

// Check if the sd card is valid
#if _SD_CARD_ENABLED_ != SD_CARD_DISABLED && _SD_CARD_ENABLED_ != SD_CARD_ENABLED
#error invalid sd card configuration, check config.h
#endif

// Check if the sd card format is valid
#if FORMAT_SDCARD_IF_MOUNT_FAILED != DO_NOT_FORMAT_SDCARD && FORMAT_SDCARD_IF_MOUNT_FAILED != FORMAT_SDCARD
#error invalid sd card format configuration, check config.h
#endif

// Check if the imu is valid
#if _IMU_ENABLED_ != IMU_DISABLED && _IMU_ENABLED_ != IMU_ENABLED
#error invalid imu configuration, check config.h
#endif

// Check if the configuration is valid
// Timestamp requires that no external adc is enabled
#if _TIMESTAMP_ == TIMESTAMP_ENABLED && (_ADC_EXT_ != NO_EXT_ADC && _SD_CARD_ENABLED_ == SD_CARD_DISABLED)
#error timestamp requires that no external adc is enabled, check config.h
#endif

#endif
