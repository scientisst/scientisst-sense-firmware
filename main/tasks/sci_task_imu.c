#include "sci_task_imu.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sensors/include/bno055.h"

#include "sci_gpio.h"
#include "sci_macros.h"

#define SDA_PIN SDA_IO
#define SCL_PIN SCL_IO

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1
#define I2C_TIMEOUT_TICKS 40000

s8 bno055I2CBusWrite(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 bno055I2CBusRead(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void bno055DelayMSec(u32 msek);

DRAM_ATTR static struct bno055_t BNO_handle = {.dev_addr = BNO055_I2C_ADDR1,
                                               .bus_write = &bno055I2CBusWrite,
                                               .bus_read = &bno055I2CBusRead,
                                               .delay_msec = &bno055DelayMSec};
DRAM_ATTR uint16_t imuValues[6] = {1, 1, 1, 1, 1, 1};
#ifdef CONFIG_EULER_ANGLES_AND_LINEAR_ACCELERATION
DRAM_ATTR bno055_data_types_t bno055_data_to_acquire[2] = {EULER_ANGLES, LINEAR_ACCELERATION};
#endif
#ifdef CONFIG_ANGULAR_VELOCITY_AND_LINEAR_ACCELERATION
DRAM_ATTR bno055_data_types_t bno055_data_to_acquire[2] = {ANGULAR_VELOCITY, LINEAR_ACCELERATION};
#endif

_Noreturn void IRAM_ATTR taskBno055(void)
{
    struct bno055_euler_t euler_hrp;
    struct bno055_gyro_t gyro_xyz;
    struct bno055_linear_accel_t linear_acce_xyz;
    esp_err_t ret = BNO055_SUCCESS;

    while (1)
    {
        // Read IMU data, for each data type requested
        // Convert IMU data to 12 bits and store it in imuValues. Check bno055.h for more info on the math
        // behind these conversions
        for (int i = 0; i < 2; ++i)
        {
            switch (bno055_data_to_acquire[i])
            {
            case EULER_ANGLES: // In degrees, 12 bit resolution
                ret += bno055_read_euler_hrp(&euler_hrp);
                imuValues[i * 3] = (euler_hrp.r >> 4) & 0x0FFF;
                imuValues[i * 3 + 1] = (euler_hrp.p >> 4) & 0x0FFF;
                imuValues[i * 3 + 2] = (euler_hrp.h >> 4) & 0x0FFF;
                break;
            case LINEAR_ACCELERATION: // In m/s^2 12 bits but has to be divided by 6.25 on post-processing
                                      // (6.25 * 16 = 100)
                ret += bno055_read_linear_accel_xyz(&linear_acce_xyz);
                imuValues[i * 3] = (linear_acce_xyz.x >> 4) & 0x0FFF;
                imuValues[i * 3 + 1] = (linear_acce_xyz.y >> 4) & 0x0FFF;
                imuValues[i * 3 + 2] = (linear_acce_xyz.z >> 4) & 0x0FFF;
                break;
            case ANGULAR_VELOCITY: // In degrees/s, 12 bit resolution
                ret += bno055_read_gyro_xyz(&gyro_xyz);
                imuValues[i * 3] = (gyro_xyz.x >> 4) & 0x0FFF;
                imuValues[i * 3 + 1] = (gyro_xyz.y >> 4) & 0x0FFF;
                imuValues[i * 3 + 2] = (gyro_xyz.z >> 4) & 0x0FFF;
                break;
            default:
                DEBUG_PRINT_E("IMU_TASK", "Invalid acquisition data type requested");
                break;
            }
        }

        if (ret != BNO055_SUCCESS)
        {
            DEBUG_PRINT_E("IMU_TASK", "Error reading IMU data");
            ret = BNO055_SUCCESS;
            continue;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // 10 ms delay = 100Hz, bn0055 max sample rate
    }
}

esp_err_t initIMU(void)
{
    esp_err_t ret = ESP_OK;
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
    ret += i2c_param_config(I2C_NUM_0, &i2c_config);
    ret += i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    ret += i2c_set_timeout(I2C_NUM_0, I2C_TIMEOUT_TICKS);

    vTaskDelay(100 / portTICK_PERIOD_MS); // Put a delay to ensure that the I2C bus is ready

    // Initialize IMU and get calibration status
    ret += bno055_init(&BNO_handle);
    ret += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    ret += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

    ret += bno055_get_accel_calib_stat(&accel_calib_status);
    ret += bno055_get_mag_calib_stat(&mag_calib_status);
    ret += bno055_get_gyro_calib_stat(&gyro_calib_status);
    ret += bno055_get_sys_calib_stat(&sys_calib_status);
    DEBUG_PRINT_W("IMU_TASK", "Calibration status: %d %d %d %d", accel_calib_status, gyro_calib_status, mag_calib_status,
                  sys_calib_status);

#ifdef CONFIG_LOCK_IMU_ACQUISITION_UNTIL_CALIBRATED
    while (accel_calib_status != 3 || gyro_calib_status != 3 || mag_calib_status != 3 || sys_calib_status != 3)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        bno055_get_accel_calib_stat(&accel_calib_status);
        bno055_get_mag_calib_stat(&mag_calib_status);
        bno055_get_gyro_calib_stat(&gyro_calib_status);
        bno055_get_sys_calib_stat(&sys_calib_status);
        DEBUG_PRINT_W("IMU_TASK", "IMU not ready. Calibration status: %d %d %d %d", accel_calib_status, gyro_calib_status,
                      mag_calib_status, sys_calib_status);
    }
#endif
#ifdef CONFIG_ALLOW_IMU_ACQUISITION_WHILE_CALIBRATING
    if (accel_calib_status != 3 || gyro_calib_status != 3 || mag_calib_status != 3 || sys_calib_status != 3)
    {
        DEBUG_PRINT_W("IMU_TASK", "IMU not fully calibrated, using it anyway according to config");
    }
#endif
    return ret;
}

void bno055SetDataToAcquire(const bno055_data_types_t *data_to_acquire)
{
    if (data_to_acquire[0] == data_to_acquire[1])
    {
        DEBUG_PRINT_E("IMU_TASK", "Same data type requested twice, ignoring request.");
        return;
    }
    bno055_data_to_acquire[0] = data_to_acquire[0];
    bno055_data_to_acquire[1] = data_to_acquire[1];
}

uint16_t IRAM_ATTR getImuValue(uint8_t channel)
{
    return imuValues[channel];
}

s8 IRAM_ATTR bno055I2CBusWrite(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u8 ret = BNO055_INIT_VALUE;
    esp_err_t espRc;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((dev_addr << 1) | I2C_MASTER_WRITE), 1);

    i2c_master_write_byte(cmd, reg_addr, 1);
    i2c_master_write(cmd, reg_data, cnt, 1);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, I2C_TIMEOUT_TICKS);
    if (espRc == ESP_OK)
    {
        ret = BNO055_SUCCESS;
    }
    else
    {
        ret = BNO055_ERROR;
    }
    i2c_cmd_link_delete(cmd);

    return (s8)ret;
}

s8 IRAM_ATTR bno055I2CBusRead(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u8 iError = BNO055_INIT_VALUE;
    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((dev_addr << 1) | I2C_MASTER_WRITE), 1);
    i2c_master_write_byte(cmd, reg_addr, 1);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((dev_addr << 1) | I2C_MASTER_READ), 1);

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

void IRAM_ATTR bno055DelayMSec(u32 msek)
{
    vTaskDelay(msek / portTICK_PERIOD_MS);
}
