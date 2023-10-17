//
// Created by frederico on 12-09-2023.
//

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

esp_err_t init_IMU(void);
void bno055_set_data_to_acquire(bno055_data_types_t *data_to_acquire); // TODO: Explain why this is not used yet
uint16_t get_imu_value(uint8_t channel);
_Noreturn void task_bno055(void);
