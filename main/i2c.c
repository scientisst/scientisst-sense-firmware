/** \file i2c.c
    \brief This file contains the definitions of the functions used to
   communicate with the e-covid sensor.

    This file contains the definitions of the functions used to communicate with
   the e-covid sensor through the I2C bus. It is specific to the e-covid sensor
   and the I2C bus used in the project. It is not a generic I2C driver. Not used
   anymore.
*/

#include "i2c.h"

#include "gpio.h"
#include "macros.h"

#define MAX32664_MFIO_IO    GPIO_NUM_15
#define MAX32664_RSTN_IO    GPIO_NUM_4

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

#define TMP_AMB_REG_ADDR 0x06
#define TMP_OBJ_REG_ADDR 0x07

#define OUTPUT_MODE 0x10
#define ENABLE_ALGORITHM 0x52
#define ENABLE_SENSOR 0x44

// MAX32664-------------------------
#define SET_FORMAT 0
#define WRITE_SET_THRESHOLD 0x01
#define ENABLE_AGC_ALGO 0x00
#define ENABLE_MAX30101 0x03
#define ENABLE_WHRM_ALGO 0x02
#define READ_DATA_OUTPUT 0x12

#define ALGO_DATA 2
#define FIFO_THRESHOLD 1
#define ENABLE 1
#define DISABLE 0
#define NUM_SAMPLES 0x00
#define READ_FIFO_DATA 0x01

#define CMD_DELAY 6
#define ENABLE_CMD_DELAY 45

#define SIZE_SAMPLE 6  // Size of each sample in bytes

//----------------------------------

/**
 * \brief i2c master initialization
 *
 * \return:
 *     - ESP_OK   Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Driver install error
 */
esp_err_t i2cMasterInit(void) {
    int i2c_master_port = I2C_USED;
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en =
        GPIO_PULLUP_ENABLE; /*!< Internal GPIO pull mode for I2C sda signal*/
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en =
        GPIO_PULLUP_ENABLE; /*!< Internal GPIO pull mode for I2C scl signal*/
    conf.master.clk_speed = I2C0_FREQUENCY_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE,
                              0);  // Master doesn't need RX and TX buffers
}

/**
 * \brief MAX32664 initialization
 *
 * MAX32664 is a sensor that measures heart rate and SpO2. It is connected to
 * the ESP32 through the I2C bus.
 */
void MAX32664_Init(void) {
    gpioConfig(GPIO_MODE_OUTPUT, GPIO_PIN_INTR_DISABLE,
               ((1ULL << MAX32664_MFIO_IO) | (1ULL << MAX32664_RSTN_IO)), 0, 0);

    gpio_set_level(MAX32664_MFIO_IO, 1);
    gpio_set_level(MAX32664_RSTN_IO, 0);
    vTaskDelay(10 / portTICK_RATE_MS);

    // After the 10ms has elapsed, set the RSTN pin to high. (MFIO pin should be
    // set at least 1ms before RSTN pin is set to high
    DEBUG_PRINT_I("MAX32664_Init", "Setting the RSTN pin to high");
    gpio_set_level(MAX32664_RSTN_IO, 1);

    // After an additional 50ms has elapsed, the MAX32664 is in application mode
    // and the application performs its initialization of the application
    // software

    vTaskDelay(2000 / portTICK_RATE_MS);

    gpioConfig(GPIO_MODE_INPUT, GPIO_PIN_INTR_DISABLE, 1ULL << MAX32664_MFIO_IO,
               0, 1);

    DEBUG_PRINT_I("MAX32664_Init", "MAX32664 is ready to accept I2C commands");
}

/**
 * \brief Implements the necessary configurations in order to read HR+SpO2.
 *
 * Implements the necessary configurations in order to read HR+SpO2. Based on
 * page 13 of "Measuring Heart Rate and SpO2Using the MAX32664A –   A Quick
 * Start Guide"
 * https://pdfserv.maximintegrated.com/en/an/ug7087-max32664a-quick-start-guide-rev-1-p1.pdf
 *
 * \return:
 *    - 0 Success
 *    - 1 Error
 */
uint8_t MAX32664_Config(void) {
    uint8_t aux;
    // Family Byte: OUTPUT_MODE (0x10), Index Byte: SET_FORMAT (0x00),
    // Write Byte : outputType (Parameter values in OUTPUT_MODE_WRITE_BYTE)
    aux = ALGO_DATA;
    if (MAX32664_SMBusWrite(MAX32664_DEFAULT_ADDRESS, OUTPUT_MODE, SET_FORMAT,
                            &aux, 1, CMD_DELAY)) {
        DEBUG_PRINT_E("MAX32664_Config", "Output mode config failed");
        return 1;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    // Family Byte: OUTPUT_MODE(0x10), Index Byte: WRITE_SET_THRESHOLD (0x01),
    // Write byte: intThres (parameter - value betwen 0 and 0xFF). This function
    // changes the threshold for the FIFO interrupt bit/pin (example: if fifo
    // thresh = 2, interrupt will be generated in MFIO pin when fifo has
    // 2samples)
    aux = FIFO_THRESHOLD;
    if (MAX32664_SMBusWrite(MAX32664_DEFAULT_ADDRESS, OUTPUT_MODE,
                            WRITE_SET_THRESHOLD, &aux, 1, CMD_DELAY)) {
        DEBUG_PRINT_E("MAX32664_Config", "Set fifo threshold failed");
        return 1;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    // Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
    // ENABLE_AGC_ALGO (0x00)
    // This function enables (one) or disables (zero) the automatic gain control
    // algorithm.
    aux = ENABLE;
    if (MAX32664_SMBusWrite(MAX32664_DEFAULT_ADDRESS, ENABLE_ALGORITHM,
                            ENABLE_AGC_ALGO, &aux, 1, 100)) {
        DEBUG_PRINT_E("MAX32664_Config",
                      "Set automatic gain control algorithm failed");
        return 1;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    // Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX30101 (0x03),
    // Write Byte: senSwitch  (parameter - 0x00 or 0x01). This function enables
    // the MAX30101.
    aux = ENABLE;
    if (MAX32664_SMBusWrite(MAX32664_DEFAULT_ADDRESS, ENABLE_SENSOR,
                            ENABLE_MAX30101, &aux, 1, 100)) {
        DEBUG_PRINT_E("MAX32664_Config", "Enable MAX30101 sensor failed");
        return 1;
    }
    vTaskDelay(60 / portTICK_RATE_MS);
    // Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
    // ENABLE_WHRM_ALGO (0x02)
    // This function enables (one) or disables (zero) the wrist heart rate
    // monitor algorithm. (maximFastAlgorithm)
    aux = ENABLE;
    if (MAX32664_SMBusWrite(MAX32664_DEFAULT_ADDRESS, ENABLE_ALGORITHM,
                            ENABLE_WHRM_ALGO, &aux, 1, 100)) {
        DEBUG_PRINT_E("MAX32664_Config", "Enable WHRM algorithm sensor failed");
        return 1;
    }

    vTaskDelay(1000 / portTICK_RATE_MS);
    DEBUG_PRINT_I("MAX32664_Config", "Config done");
    return 0;
}

/**
 * \brief Reads the HR values from the MAX32664.
 *
 * Reads the HR+SpO2 values from the MAX32664. Based on page 13 of "Measuring
 * Heart Rate and SpO2Using the MAX32664A –   A Quick Start Guide"
 * https://pdfserv.maximintegrated.com/en/an/ug7087-max32664a-quick-start-guide-rev-1-p1.pdf
 *
 * \param heart_rate Pointer to the variable where the heart rate value will be
 * stored. \param oxygen Pointer to the variable where the SpO2 value will be
 * stored. \param confidence Pointer to the variable where the confidence value
 * will be stored. \param status Pointer to the variable where the status value
 * will be stored.
 *
 * \return:
 *    - 0 Success
 *    - 1 Error
 */
uint8_t MAX32664_GetBPM(uint16_t* heart_rate, uint16_t* oxygen,
                        uint8_t* confidence, uint8_t* status) {
    uint8_t data_buff[6];

    // Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: NUM_SAMPLES (0x00)
    // This function returns the number samples (I think, maybe it's the amount
    // of bytes) available in the FIFO.
    /*if(MAX32664_SMBusRead(MAX32664_DEFAULT_ADDRESS, READ_DATA_OUTPUT,
    NUM_SAMPLES, 0, 0, &samples_ready, 1, CMD_DELAY)){
        DEBUG_PRINT_E("MAX32664_GetBPM", "Get number of bytes avaiable in FIFO
    failed"); return 1;
    }
    DEBUG_PRINT_I("MAX32664_GetBPM", "Number of samples avaiable in FIFO:%d",
    samples_ready);*/

    // Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: READ_FIFO_DATA (0x01)
    // This function returns the data available in the FIFO.
    if (MAX32664_SMBusRead(MAX32664_DEFAULT_ADDRESS, READ_DATA_OUTPUT,
                           READ_FIFO_DATA, 0, 0, data_buff, SIZE_SAMPLE,
                           CMD_DELAY)) {
        DEBUG_PRINT_E("MAX32664_GetBPM", "Get data from FIFO failed");
        return 1;
    }

    // last_sample = data_buff + (samples_ready-1)*SIZE_SAMPLE;

    // Heart Rate formatting
    *heart_rate = ((uint16_t)data_buff[0]) << 8;
    *heart_rate |= (data_buff[1]);
    *heart_rate /= 10;

    // Confidence formatting
    *confidence = data_buff[2];

    // Blood oxygen level formatting
    *oxygen = ((uint16_t)data_buff[3]) << 8;
    *oxygen |= data_buff[4];
    *oxygen /= 10;

    //"Machine State" - has a finger been detected?
    *status = data_buff[5];

    return 0;
}

/**
 * \brief MAX32664 SMBus read function.
 *
 * \param slave_addr Slave address of the MAX32664.
 * \param family_byte Family byte of the command.
 * \param index_byte Index byte of the command.
 * \param extra_byte Extra byte of the command.
 * \param extra_byte_flag Flag to indicate if the extra byte is used or not.
 * \param rcv_data Pointer to the buffer where the data will be stored.
 * \param rcv_data_size Size of recieving data of interest. Normally the slave
 * sends 1 byte for "Read status byte" followed by the data of interest (Total
 * recieved bytes should be always more than 1 byte -> status byte + data of
 * interest) \param delay_ms Delay in ms between the write and read commands.
 *
 * \return:
 *   - 0 Success
 *   - 1 Error
 */
int MAX32664_SMBusRead(uint8_t slave_addr, uint8_t family_byte,
                       uint8_t index_byte, uint8_t extra_byte,
                       uint8_t extra_byte_flag, uint8_t* rcv_data,
                       uint8_t rcv_data_size, uint8_t delay_ms) {
    esp_err_t ret;
    uint8_t status_byte;
    i2c_cmd_handle_t cmd;

    slave_addr <<= 1;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, family_byte,
                          ACK_CHECK_EN);  // Send addr of register to be read
    i2c_master_write_byte(cmd, index_byte,
                          ACK_CHECK_EN);  // Send addr of register to be read
    if (extra_byte_flag) {
        i2c_master_write_byte(
            cmd, extra_byte, ACK_CHECK_EN);  // Send addr of register to be read
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_USED, cmd, 3000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_ERR_TIMEOUT) {
        DEBUG_PRINT_E("MAX32664_SMBusRead", "Bus is busy");
        return 1;
    } else if (ret != ESP_OK) {
        DEBUG_PRINT_E("MAX32664_SMBusRead", "Read Failed");
        return 1;
    }

    vTaskDelay(delay_ms / portTICK_RATE_MS);  // Delay for the device to have
                                              // its data/response ready.

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &status_byte, ACK_VAL);
    if (rcv_data_size > 1) {
        i2c_master_read(cmd, rcv_data, rcv_data_size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, rcv_data + rcv_data_size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_USED, cmd, 3000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_ERR_TIMEOUT) {
        DEBUG_PRINT_E("MAX32664_SMBusRead", "Bus is busy, status byte:%d",
                      status_byte);
        return 1;
    } else if (ret != ESP_OK) {
        DEBUG_PRINT_E("MAX32664_SMBusRead", "Read Failed, status byte:%d",
                      status_byte);
        return 1;
    }

    if (status_byte) {
        DEBUG_PRINT_E("MAX32664_SMBusRead",
                      "MAX32664 read failed: status byte slave return error:%d",
                      status_byte);
        return 1;
    }

    return 0;
}

/**
 * \brief MAX32664 SMBus write function.
 *
 * \param slave_addr Slave address of the MAX32664.
 * \param family_byte Family byte of the command.
 * \param index_byte Index byte of the command.
 * \param write_data Pointer to the buffer where the data to be written is
 * stored. \param write_data_size Size of the data to be written. \param
 * delay_ms Delay in ms between the write and read commands.
 *
 * \return:
 *   - 0 Success
 *   - 1 Error
 */
int MAX32664_SMBusWrite(uint8_t slave_addr, uint8_t family_byte,
                        uint8_t index_byte, uint8_t* write_data,
                        uint8_t write_data_size, uint8_t delay_ms) {
    esp_err_t ret;
    uint8_t status_byte;
    i2c_cmd_handle_t cmd;

    slave_addr <<= 1;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, family_byte,
                          ACK_CHECK_EN);  // Send addr of register to be read
    i2c_master_write_byte(cmd, index_byte,
                          ACK_CHECK_EN);  // Send addr of register to be read
    i2c_master_write(cmd, write_data, write_data_size,
                     ACK_CHECK_EN);  // Send write_data_size bytes
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_USED, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_ERR_TIMEOUT) {
        DEBUG_PRINT_E("MAX32664_SMBusWrite", "Bus is busy");
        return 1;
    } else if (ret != ESP_OK) {
        DEBUG_PRINT_E("MAX32664_SMBusWrite", "Read Failed");
        return 1;
    }

    vTaskDelay(delay_ms / portTICK_RATE_MS);  // Delay for the device to have
                                              // its data/response ready.

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &status_byte, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_USED, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_ERR_TIMEOUT) {
        DEBUG_PRINT_E("MAX32664_SMBusWrite", "Bus is busy");
        return 1;
    } else if (ret != ESP_OK) {
        DEBUG_PRINT_E("MAX32664_SMBusWrite", "Read Failed");
        return 1;
    }

    if (status_byte) {
        DEBUG_PRINT_E(
            "MAX32664_SMBusWrite",
            "MAX32664 write failed: status byte slave return error:%d",
            status_byte);
        return 1;
    }

    return 0;
}

/**
 * \brief Get ambient temperature from MLX90614.
 *
 * \param slave_addr Slave address of the MLX90614.
 * \param temp_amb Pointer to the variable where the ambient temperature will be
 * stored. \param temp_amb_int Pointer to the variable where the ambient
 * temperature in integer format will be stored.
 *
 * \return:
 *   - 0 Success
 *   - 1 Error
 */
int MLX90614_GetTempAmb(uint8_t slaveAddr, float* temp_amb,
                        uint16_t* temp_amb_int) {
    int error = 0;
    uint16_t data = 0;

    error = MLX90614_SMBusRead(slaveAddr, TMP_AMB_REG_ADDR, &data);

    if (data > 0x7FFF) {
        return 0;
    }

    if (error == 0) {
        *temp_amb = (float)data * 0.02f - 273.15;
        *temp_amb_int = data;
    }

    return error;
}

/**
 * \brief Get object temperature from MLX90614.
 *
 * \param slave_addr Slave address of the MLX90614.
 * \param temp_obj Pointer to the variable where the object temperature will be
 * stored. \param temp_obj_int Pointer to the variable where the object
 * temperature in integer format will be stored.
 *
 * \return:
 *   - 0 Success
 *   - 1 Error
 */
int MLX90614_GetTempObj(uint8_t slaveAddr, float* temp_obj,
                        uint16_t* temp_obj_int) {
    int error = 0;
    uint16_t data = 0;

    error = MLX90614_SMBusRead(slaveAddr, TMP_OBJ_REG_ADDR, &data);

    if (data > 0x7FFF) {
        return 0;
    }

    if (error == 0) {
        *temp_obj = (float)data * 0.02f - 273.15;
        *temp_obj_int = data;
    }

    return error;
}

/**
 * \brief Read data from MLX90614.
 *
 * \param slave_addr Slave address of the MLX90614.
 * \param reg_addr Register address to read.
 * \param data Pointer to the variable where the data will be stored.
 *
 * \return:
 *   - 0 Success
 *   - 1 Error
 */
int MLX90614_SMBusRead(uint8_t slaveAddr, uint8_t reg_addr, uint16_t* data) {
    uint8_t chip_addr;
    uint8_t data_addr;
    uint8_t pec;
    uint16_t* p;

    p = data;
    chip_addr = (slaveAddr << 1);
    data_addr = reg_addr;
    pec = chip_addr;
    uint8_t recived_data[3] = {0, 0, 0};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    chip_addr = chip_addr | 0x01;
    i2c_master_write_byte(cmd, chip_addr | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, recived_data, 2, ACK_VAL);
    i2c_master_read_byte(cmd, recived_data + 2, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret =
        i2c_master_cmd_begin(I2C_USED, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        pec = MLX90614_CalcPEC(0, pec);
        pec = MLX90614_CalcPEC(pec, data_addr);
        pec = MLX90614_CalcPEC(pec, chip_addr);
        pec = MLX90614_CalcPEC(pec, recived_data[0]);
        pec = MLX90614_CalcPEC(pec, recived_data[1]);
    } else if (ret == ESP_ERR_TIMEOUT) {
        DEBUG_PRINT_E("MLX90614_SMBusRead", "Bus is busy");
    } else {
        DEBUG_PRINT_E("MLX90614_SMBusRead", "Read Failed");
    }

    if (pec != recived_data[2]) {
        DEBUG_PRINT_E("MLX90614_SMBusRead", "PEC Check returned error");
        return 1;
    }

    *p = (uint16_t)recived_data[1] * 256 + (uint16_t)recived_data[0];
    return 0;
}

/**
 * \brief Calculate PEC for MLX90614.
 *
 * \param initPEC Initial PEC value.
 * \param newData New data to calculate PEC.
 *
 * \return PEC value.
 */
uint8_t MLX90614_CalcPEC(uint8_t initPEC, uint8_t newData) {
    uint8_t data;
    uint8_t bitCheck;

    data = initPEC ^ newData;

    for (int i = 0; i < 8; i++) {
        bitCheck = data & 0x80;
        data = data << 1;

        if (bitCheck != 0) {
            data = data ^ 0x07;
        }
    }
    return data;
}

/**
 * \brief Scan I2C bus for devices.
 *
 */
void i2cscanner(void) {
    int i;
    esp_err_t espRc;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");
    for (i = 3; i < 0x78; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1);
        i2c_master_stop(cmd);

        espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
        if (i % 16 == 0) {
            printf("\n%.2x:", i);
        }
        if (espRc == 0) {
            printf(" %.2x", i);
        } else {
            printf(" --");
        }
        // ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
        i2c_cmd_link_delete(cmd);
    }
    printf("\n");
}