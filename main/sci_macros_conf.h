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

#include <stdint.h>

// External ADC possible options
#define EXT_ADC_DISABLED 1 // External ADC disabled
#define EXT_ADC_ENABLED 2

// Hardware version possible options
#define HW_VERSION_CORE 1
#define HW_VERSION_NANO 2
#define HW_VERSION_CARDIO 3

// SD card possible options
#define SD_CARD_DISABLED 1
#define SD_CARD_ENABLED 2

// SD card format possible options
#define DO_NOT_FORMAT_SDCARD false
#define FORMAT_SDCARD true

// IMU possible options
#define IMU_DISABLED 1
#define IMU_ENABLED 2

// IMU data acquisition possible options
#define EULER_ANGLES_AND_LINEAR_ACCELERATION 1
#define ANGULAR_VELOCITY_AND_LINEAR_ACCELERATION 2

// IMU calibration possible options
#define LOCK_IMU_ACQUISITION_UNTIL_CALIBRATED 1
#define ALLOW_IMU_ACQUISITION_WHILE_CALIBRATING 2

#endif // SCIENTISST_SENSE_FIRMWARE_MACROS_CONF_H
