#include "imu.h"
#include "bno055.h"
#include "driver/i2c.h"
#include "gpio.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

#define SDA_PIN SDA_IO
#define SCL_PIN SCL_IO

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1
#define I2C_TIMEOUT_TICKS 40000

void IRAM_ATTR i2c_master_init()
{
    i2c_config_t i2c_config = {.mode = I2C_MODE_MASTER,
                               .sda_io_num = SDA_PIN,
                               .scl_io_num = SCL_PIN,
                               .sda_pullup_en = GPIO_PULLUP_ENABLE,
                               .scl_pullup_en = GPIO_PULLUP_ENABLE,
                               .master.clk_speed = 400000};
    i2c_param_config(I2C_NUM_0, &i2c_config);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    i2c_set_timeout(I2C_NUM_0, I2C_TIMEOUT_TICKS);
}

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

void IRAM_ATTR bno055_task(void *not_used)
{
    i2c_master_init();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    struct bno055_t myBNO = {.bus_write = BNO055_I2C_bus_write,
                             .bus_read = BNO055_I2C_bus_read,
                             .dev_addr = BNO055_I2C_ADDR1,
                             .delay_msec = BNO055_delay_msek};
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    bno055_init(&myBNO);
    bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    unsigned char accel_calib_status = 0;
    unsigned char gyro_calib_status = 0;
    unsigned char mag_calib_status = 0;
    unsigned char sys_calib_status = 0;

    bno055_get_accel_calib_stat(&accel_calib_status);
    bno055_get_mag_calib_stat(&mag_calib_status);
    bno055_get_gyro_calib_stat(&gyro_calib_status);
    bno055_get_sys_calib_stat(&sys_calib_status);
    float rollDeg;
    float pitchDeg;
    float yawDeg;
    struct bno055_euler_t euler_hrp;
    int i = 0;
    uint64_t last_time = esp_timer_get_time();
    uint64_t current_time;
    while (1)
    {
        if (bno055_read_euler_hrp(&euler_hrp) == BNO055_SUCCESS)
        {
            // printf("\nhead: %.2f pitch: %.2f roll: %.2f", yawDeg, pitchDeg, rollDeg);
            ++i;
            if (i == 1000)
            {
                rollDeg = euler_hrp.r >> 4;
                pitchDeg = euler_hrp.p >> 4;
                yawDeg = euler_hrp.h >> 4;
                current_time = esp_timer_get_time();
                printf("%.2f\n", (float)(current_time - last_time) / 1000000);
                last_time = current_time;
                printf("head: %.2f pitch: %.2f roll: %.2f\n", yawDeg, pitchDeg, rollDeg);
                i = 0;
            }
        }
        else
        {
            printf("\nFalho");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}