/** \file i2c.h
    \brief This file contains the definitions of the functions used to
   communicate with the e-covid sensor.

    This file contains the definitions of the functions used to communicate with
   the e-covid sensor through the I2C bus. It is specific to the e-covid sensor
   and the I2C bus used in the project. It is not a generic I2C driver. Not used
   anymore.
*/

#ifndef _I2C_H
#define _I2C_H

#include <stdio.h>

#include "driver/i2c.h"

#define I2C_USED I2C_NUM_0
#define I2C_MASTER_SDA_IO 21     // 25
#define I2C_MASTER_SCL_IO 22     // 26
#define I2C0_FREQUENCY_HZ 50000  // 50kHz

#define MLX90614_DEFAULT_ADDRESS \
    0x5A  // default chip address(slave address) of MLX90614
#define MAX32664_DEFAULT_ADDRESS \
    0x55  // default chip address(slave address) of MAX32664

#define ACQ_I2C_SAMPLE_RATE 10  // In Hz

#define CONFIDENCE_THRESHOLD \
    90  // Threshold in which a sample is considered valid (confidence >=
        // CONFIDENCE_THRESHOLD -> valid sample)

typedef struct {
    float temp_obj;  // temperature of object
    float temp_amb;  // temperature of ambient
    uint16_t temp_obj_int;
    uint16_t temp_amb_int;
    uint16_t heart_rate;
    uint16_t oxygen;
    uint8_t confidence;
    uint8_t status;
} I2c_Sensor_State;

esp_err_t i2cMasterInit(void);

int MLX90614_SMBusRead(uint8_t slaveAddr, uint8_t reg_addr, uint16_t* data);
uint8_t MLX90614_CalcPEC(uint8_t initPEC, uint8_t newData);
int MLX90614_GetTempAmb(uint8_t slaveAddr, float* temp_amb,
                        uint16_t* temp_amb_int);
int MLX90614_GetTempObj(uint8_t slaveAddr, float* temp_obj,
                        uint16_t* temp_obj_int);
int MAX32664_SMBusRead(uint8_t slave_addr, uint8_t family_byte,
                       uint8_t index_byte, uint8_t extra_byte,
                       uint8_t extra_byte_flag, uint8_t* rcv_data,
                       uint8_t rcv_data_size, uint8_t delay_ms);
int MAX32664_SMBusWrite(uint8_t slave_addr, uint8_t family_byte,
                        uint8_t index_byte, uint8_t* write_data,
                        uint8_t write_data_size, uint8_t delay_ms);
void MAX32664_Init(void);
uint8_t MAX32664_Config(void);
uint8_t MAX32664_GetBPM(uint16_t* heart_rate, uint16_t* oxygen,
                        uint8_t* confidence, uint8_t* status);
#endif