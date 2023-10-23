/**
 * \file sci_task_imu.h
 * \brief Header file for sci_task_imu.c
 */

#pragma once

#include <stdint.h>
#include <sys/cdefs.h>

#include "esp_err.h"

#include "sci_scientisst.h"

typedef enum
{
    EULER_ANGLES = 1,
    LINEAR_ACCELERATION,
    ANGULAR_VELOCITY,
} bno055_data_types_t;

esp_err_t initIMU(void);
void bno055SetDataToAcquire(const bno055_data_types_t *data_to_acquire);
uint16_t getImuValue(uint8_t channel);
_Noreturn void taskBno055(void);
