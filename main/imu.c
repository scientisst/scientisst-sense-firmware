#include "imu.h"
#include "bno055.h"
#include "driver/i2c.h"
#include "gpio.h"
#include "macros.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>
#include <stdio.h>

#define SDA_PIN SDA_IO
#define SCL_PIN SCL_IO

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1
#define I2C_TIMEOUT_TICKS 40000

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BNO055_delay_msek(u32 msek);

DRAM_ATTR struct bno055_t BNO_handle = {.bus_write = BNO055_I2C_bus_write,
                                        .bus_read = BNO055_I2C_bus_read,
                                        .dev_addr = BNO055_I2C_ADDR1,
                                        .delay_msec = BNO055_delay_msek};
DRAM_ATTR uint16_t imuValues[6] = {1, 1, 1, 1, 1, 1};

s8 IRAM_ATTR BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u8 iError = BNO055_INIT_VALUE;
    esp_err_t espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, 1);

    i2c_master_write_byte(cmd, reg_addr, 1);
    i2c_master_write(cmd, reg_data, cnt, 1);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, I2C_TIMEOUT_TICKS);
    if (espRc == ESP_OK)
    {
        iError = BNO055_SUCCESS;
    }
    else
    {
        iError = BNO055_ERROR;
    }
    i2c_cmd_link_delete(cmd);

    return (s8)iError;
}

s8 IRAM_ATTR BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u8 iError = BNO055_INIT_VALUE;
    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg_addr, 1);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, 1);

    if (cnt > 1)
    {
        i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, I2C_TIMEOUT_TICKS);
    if (espRc == ESP_OK)
    {
        iError = BNO055_SUCCESS;
    }
    else
    {
        iError = BNO055_ERROR;
    }

    i2c_cmd_link_delete(cmd);

    return (s8)iError;
}

void IRAM_ATTR BNO055_delay_msek(u32 msek)
{
    vTaskDelay(msek / portTICK_PERIOD_MS);
}

void init_IMU(void)
{
    unsigned char accel_calib_status = 0;
    unsigned char gyro_calib_status = 0;
    unsigned char mag_calib_status = 0;
    unsigned char sys_calib_status = 0;
    i2c_config_t i2c_config = {.mode = I2C_MODE_MASTER,
                               .sda_io_num = SDA_PIN,
                               .scl_io_num = SCL_PIN,
                               .sda_pullup_en = GPIO_PULLUP_ENABLE,
                               .scl_pullup_en = GPIO_PULLUP_ENABLE,
                               .master.clk_speed = 400000};

    // Initialize I2C
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    i2c_set_timeout(I2C_NUM_0, I2C_TIMEOUT_TICKS);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Initialize IMU and get calibration status
    bno055_init(&BNO_handle);
    bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

    bno055_get_accel_calib_stat(&accel_calib_status);
    bno055_get_mag_calib_stat(&mag_calib_status);
    bno055_get_gyro_calib_stat(&gyro_calib_status);
    bno055_get_sys_calib_stat(&sys_calib_status);
    DEBUG_PRINT_W("IMU_TASK", "Calibration status: %d %d %d %d", accel_calib_status, gyro_calib_status,
                  mag_calib_status, sys_calib_status);

#if _IMU_CALIBRATION_ == LOCK_IMU_ACQUISITION_UNTIL_CALIBRATED
    while (accel_calib_status != 3 || gyro_calib_status != 3 || mag_calib_status != 3 ||
           sys_calib_status != 3)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        bno055_get_accel_calib_stat(&accel_calib_status);
        bno055_get_mag_calib_stat(&mag_calib_status);
        bno055_get_gyro_calib_stat(&gyro_calib_status);
        bno055_get_sys_calib_stat(&sys_calib_status);
        DEBUG_PRINT_W("IMU_TASK", "Calibration status: %d %d %d %d", accel_calib_status, gyro_calib_status,
                      mag_calib_status, sys_calib_status);
    }
#elif _IMU_CALIBRATION_ == ALLOW_IMU_ACQUISITION_WHILE_CALIBRATING
    if (accel_calib_status != 3 || gyro_calib_status != 3 || mag_calib_status != 3 || sys_calib_status != 3)
    {
        DEBUG_PRINT_W("IMU_TASK", "IMU not calibrated, using it anyway");
    }
#endif

    vTaskDelay(2000 / portTICK_PERIOD_MS);
}

void IRAM_ATTR bno055_task(void *not_used)
{
#if _IMU_DATA_ACQUISITION_ == EULER_ANGLES_AND_LINEAR_ACCELERATION
    struct bno055_euler_t euler_hrp;
#elif _IMU_DATA_ACQUISITION_ == ANGULAR_VELOCITY_AND_LINEAR_ACCELERATION
    struct bno055_gyro_t gyro_xyz;
#endif
    struct bno055_linear_accel_t linear_acce_xyz;

    init_IMU();

    while (1)
    {
        // Read IMU data
#if _IMU_DATA_ACQUISITION_ == EULER_ANGLES_AND_LINEAR_ACCELERATION
        if (bno055_read_euler_hrp(&euler_hrp) == BNO055_ERROR)
        {
            DEBUG_PRINT_E("IMU_TASK", "Error reading IMU data");
            continue;
        }
#elif _IMU_DATA_ACQUISITION_ == ANGULAR_VELOCITY_AND_LINEAR_ACCELERATION
        if (bno055_read_linear_accel_xyz(&linear_acce_xyz) == BNO055_ERROR)
        {
            DEBUG_PRINT_E("IMU_TASK", "Error reading IMU data");
            continue;
        }
#endif
        if (bno055_read_gyro_xyz(&gyro_xyz) == BNO055_ERROR)
        {
            DEBUG_PRINT_E("IMU_TASK", "Error reading IMU data");
            continue;
        }

        // Convert IMU data to 12 bits and store it in imuValues. Check bno055.h for more info on how these
        // convertions are done

#if _IMU_DATA_ACQUISITION_ == EULER_ANGLES_AND_LINEAR_ACCELERATION
        imuValues[0] = (euler_hrp.r >> 4) & 0x0FFF; // /  Degrees, 12 bits
        imuValues[1] = (euler_hrp.p >> 4) & 0x0FFF;
        imuValues[2] = (euler_hrp.h >> 4) & 0x0FFF;
#elif _IMU_DATA_ACQUISITION_ == ANGULAR_VELOCITY_AND_LINEAR_ACCELERATION
        imuValues[0] = (gyro_xyz.x >> 4) & 0x0FFF; //  Degrees/s, 12 bits
        imuValues[1] = (gyro_xyz.y >> 4) & 0x0FFF;
        imuValues[2] = (gyro_xyz.z >> 4) & 0x0FFF;
#endif

        //  m/s^2 12 bits but has to be divided by 6.25 on post-processing (6.25 * 16 = 100)
        imuValues[3] = (linear_acce_xyz.x >> 4) & 0x0FFF;
        imuValues[4] = (linear_acce_xyz.y >> 4) & 0x0FFF;
        imuValues[5] = (linear_acce_xyz.z >> 4) & 0x0FFF;

        vTaskDelay(10 / portTICK_PERIOD_MS); // 10 ms delay == 100Hz
    }
}