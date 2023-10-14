#include <sys/cdefs.h>
//
// Created by frederico on 12-09-2023.
//

#ifndef SCIENTISST_SENSE_SCI_IMU_H
#define SCIENTISST_SENSE_SCI_IMU_H

#include <stdint.h>

#include "esp_err.h"

typedef enum
{
    EULER_ANGLES,
    LINEAR_ACCELERATION,
    ANGULAR_VELOCITY,
} bno055_data_types_t;

esp_err_t init_IMU(void);
void bno055_set_data_to_acquire(
    bno055_data_types_t *data_to_acquire); // TODO: Explain why this is not used yet
uint16_t get_imu_value(uint8_t channel);
_Noreturn void task_bno055(void *not_used);

#endif // SCIENTISST_SENSE_SCI_IMU_H
