/**
 * \file sci_task_imu.h
 * \brief IMU Task for BNO055 Sensor
 *
 * This file provides the interface to the IMU task, specifically handling
 * the interactions with the BNO055 sensor. It manages the initialization of the sensor,
 * reading values, and processing the acquired data for further usage. The task runs
 * continuously after initialization, acquiring IMU data based on the specified configuration.
 */
#include "sci_task_imu.h"

#include "bno055.h"
#include "driver/i2c.h"

#include "sci_gpio.h"

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1
#define I2C_TIMEOUT_TICKS 40000

s8 bno055I2CBusWrite(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 bno055I2CBusRead(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void bno055DelayMSec(u32 msek);

DRAM_ATTR static struct bno055_t BNO_handle = {.dev_addr = BNO055_I2C_ADDR1,
                                               .bus_write = &bno055I2CBusWrite,
                                               .bus_read = &bno055I2CBusRead,
                                               .delay_msec = &bno055DelayMSec}; ///< BNO055 sensor handle
DRAM_ATTR static uint16_t imuValues[6] = {1, 1, 1, 1, 1, 1};                    ///< Buffer to store IMU values
DRAM_ATTR static bno055_data_types_t bno055_data_to_acquire[2] = {
// IMU data AI1-AI3
#if defined(CONFIG_IMU_H1_EULER_ANGLES)
    EULER_ANGLES,
#elif defined(CONFIG_IMU_H1_ANGULAR_VELOCITY)
    ANGULAR_VELOCITY,
#elif defined(CONFIG_IMU_H1_LINEAR_ACCELERATION)
    LINEAR_ACCELERATION,
#elif defined(CONFIG_IMU_H1_GRAVITY_VECTOR)
    GRAVITY_VECTOR,
#elif defined(CONFIG_IMU_H1_MAGNETIC_FIELD)
    MAGNETIC_FIELD,
#elif defined(CONFIG_IMU_H1_ACCELERATION)
    ACCELERATION,
#elif defined(CONFIG_IMU_H1_TEMPERATURE)
    TEMPERATURE,
#else
    NO_IMU_DATA,
#endif

// IMU data AI4-AI6
#if defined(CONFIG_IMU_H2_EULER_ANGLES)
    EULER_ANGLES,
#elif defined(CONFIG_IMU_H2_ANGULAR_VELOCITY)
    ANGULAR_VELOCITY,
#elif defined(CONFIG_IMU_H2_LINEAR_ACCELERATION)
    LINEAR_ACCELERATION,
#elif defined(CONFIG_IMU_H2_GRAVITY_VECTOR)
    GRAVITY_VECTOR,
#elif defined(CONFIG_IMU_H2_MAGNETIC_FIELD)
    MAGNETIC_FIELD,
#elif defined(CONFIG_IMU_H2_ACCELERATION)
    ACCELERATION,
#elif defined(CONFIG_IMU_H2_TEMPERATURE)
    TEMPERATURE,
#else
    NO_IMU_DATA,
#endif
}; ///< Data types to acquire from the sensor

/**
 * \brief Main task for handling BNO055 sensor operations.
 *
 * This function is the main task that continuously reads from the BNO055 sensor. It retrieves the IMU data, depending on the
 * configuration. The data is processed and stored in a buffer that can be accessed by other tasks using the getImuValue()
 * function.
 *
 * \return Never returns
 */
_Noreturn void IRAM_ATTR taskBno055(void)
{
    esp_err_t ret = BNO055_SUCCESS;

    struct bno055_euler_t euler_hrp;
    struct bno055_gyro_t gyro_xyz;
    struct bno055_linear_accel_t linear_acce_xyz;
    struct bno055_gravity_t gravity_xyz;
    struct bno055_mag_t mag_xyz;
    struct bno055_accel_t accel_xyz;
    int8_t temperature = 0;

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
            case GRAVITY_VECTOR: // In m/s^2, 12 bit resolution
                ret += bno055_read_gravity_xyz(&gravity_xyz);
                imuValues[i * 3] = (gravity_xyz.x >> 4) & 0x0FFF;
                imuValues[i * 3 + 1] = (gravity_xyz.y >> 4) & 0x0FFF;
                imuValues[i * 3 + 2] = (gravity_xyz.z >> 4) & 0x0FFF;
                break;
            case MAGNETIC_FIELD: // 12 bit resolution
                ret += bno055_read_mag_xyz(&mag_xyz);
                imuValues[i * 3] = (mag_xyz.x >> 4) & 0x0FFF;
                imuValues[i * 3 + 1] = (mag_xyz.y >> 4) & 0x0FFF;
                imuValues[i * 3 + 2] = (mag_xyz.z >> 4) & 0x0FFF;
                break;
            case ACCELERATION: // In m/s^2, 12 bit resolution
                ret += bno055_read_accel_xyz(&accel_xyz);
                imuValues[i * 3] = (accel_xyz.x >> 4) & 0x0FFF;
                imuValues[i * 3 + 1] = (accel_xyz.y >> 4) & 0x0FFF;
                imuValues[i * 3 + 2] = (accel_xyz.z >> 4) & 0x0FFF;
                break;
            case TEMPERATURE: // In degrees Celsius, 12 bit resolution
                ret += bno055_read_temp_data(&temperature);
                imuValues[i * 3] = (temperature >> 4) & 0x0FFF;
                imuValues[i * 3 + 1] = 0;
                imuValues[i * 3 + 2] = 0;
                break;
            case NO_IMU_DATA:
            default:
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

/**
 * \brief Initializes the Inertial Measurement Unit (IMU) for operation.
 *
 * This function sets up the IMU for data acquisition. It performs several tasks such as initializing the I2C bus,
 * configuring the sensor, checking calibration status, and setting the sensor's power and operation modes.
 * The function ensures that the IMU is ready for data acquisition tasks and checks the system's calibration
 * status to guarantee accurate data retrieval (if that option is enabled).
 *
 * \return ESP_OK - Success, ESP_FAIL - Failure
 */
esp_err_t initIMU(void)
{
    esp_err_t ret = ESP_OK;
    unsigned char accel_calib_status = 0;
    unsigned char gyro_calib_status = 0;
    unsigned char mag_calib_status = 0;
    unsigned char sys_calib_status = 0;
    i2c_config_t i2c_config = {.mode = I2C_MODE_MASTER,
                               .sda_io_num = IMU_SDA_PIN,
                               .scl_io_num = IMU_SCL_PIN,
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
    updateLEDStatusCode(CALIBRATING_IMU);
    while (accel_calib_status != 3 || gyro_calib_status != 3 || mag_calib_status != 3 || sys_calib_status != 3)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        bno055_get_accel_calib_stat(&accel_calib_status);
        bno055_get_mag_calib_stat(&mag_calib_status);
        bno055_get_gyro_calib_stat(&gyro_calib_status);
        bno055_get_sys_calib_stat(&sys_calib_status);
        DEBUG_PRINT_W("IMU_TASK", "IMU not ready. Calibration status: %d %d %d %d", accel_calib_status, gyro_calib_status,
                      mag_calib_status, sys_calib_status);
    }
    if (scientisst_device_settings.op_mode == OP_MODE_IDLE)
    {
        updateLEDStatusCode(IDLE);
    }
    else if (scientisst_device_settings.op_mode == OP_MODE_LIVE)
    {
        updateLEDStatusCode(LIVE_AQUISITION);
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

/**
 * \brief Sets the types of data to acquire from the BNO055 sensor.
 *
 * This function configures the types of data that the BNO055 sensor task should read.
 *
 * \param[in] data_to_acquire An array of data types to acquire from the sensor.
 *
 * \note Only two data types can be set at the same time. The function will ignore request if both data types are the same.
 * \return None.
 */
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

/**
 * \brief Retrieves the latest IMU value from a specific channel.
 *
 * This function provides a mechanism to read the processed IMU data from the task.
 * It reads from a buffer where the latest IMU values are stored.
 *
 * \param[in] channel The specific IMU channel to read.
 * \return The latest IMU value from the specified channel.
 */
uint16_t IRAM_ATTR getImuValue(uint8_t channel)
{
    return imuValues[channel];
}

/**
 * \brief Writes data to the I2C bus, targeting a specific device and register.
 *
 * \param[in] dev_addr The I2C address of the device to communicate with.
 * \param[in] reg_addr The register address within the target device where data will be written.
 * \param[in] reg_data A pointer to the data buffer that contains the information to be written.
 * \param[in] cnt The number of bytes to write from the data buffer to the device.
 * \return BNO055_SUCCESS - Success, BNO055_ERROR - Failure
 */
s8 IRAM_ATTR bno055I2CBusWrite(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u8 ret = BNO055_INIT_VALUE;
    esp_err_t res = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    res += i2c_master_start(cmd);
    res += i2c_master_write_byte(cmd, (uint8_t)((dev_addr << 1) | I2C_MASTER_WRITE), 1);

    res += i2c_master_write_byte(cmd, reg_addr, 1);
    res += i2c_master_write(cmd, reg_data, cnt, 1);
    res += i2c_master_stop(cmd);

    res += i2c_master_cmd_begin(I2C_NUM_0, cmd, I2C_TIMEOUT_TICKS);
    if (res == ESP_OK)
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

/**
 * \brief Reads data from the I2C bus, targeting a specific device and register.
 *
 * \param[in] dev_addr The I2C address of the device to communicate with.
 * \param[in] reg_addr The register address within the target device from where data will be read.
 * \param[in/out] reg_data A pointer to a buffer where the read data will be stored.
 * \param[in] cnt The number of bytes to read from the device into the data buffer.
 * \return BNO055_SUCCESS - Success, BNO055_ERROR - Failure.
 */
s8 IRAM_ATTR bno055I2CBusRead(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u8 ret = BNO055_INIT_VALUE;
    esp_err_t res = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    res += i2c_master_start(cmd);
    res += i2c_master_write_byte(cmd, (uint8_t)((dev_addr << 1) | I2C_MASTER_WRITE), 1);
    res += i2c_master_write_byte(cmd, reg_addr, 1);

    res += i2c_master_start(cmd);
    res += i2c_master_write_byte(cmd, (uint8_t)((dev_addr << 1) | I2C_MASTER_READ), 1);

    if (cnt > 1)
    {
        res += i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
    }
    res += i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
    res += i2c_master_stop(cmd);

    res += i2c_master_cmd_begin(I2C_NUM_0, cmd, I2C_TIMEOUT_TICKS);
    if (res == ESP_OK)
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

/**
 * \brief Introduces a delay in the execution of the program.
 *
 * \param[in] msek The delay period in milliseconds.
 * \return None
 */
void IRAM_ATTR bno055DelayMSec(u32 msek)
{
    vTaskDelay(msek / portTICK_PERIOD_MS);
}
