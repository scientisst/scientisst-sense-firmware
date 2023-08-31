/** \file spi.c
    \brief This file contains the definitions of the functions used to
   communicate with the external ADCs.

*/

#include "spi.h"

#include <math.h>

#include "adc.h"
#include "com.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio.h"
#include "macros.h"
#include "macros_conf.h"
#include "scientisst.h"

#define MAX_TRANSFER_SIZE 4

#if _ADC_EXT_ != NO_EXT_ADC

spi_transaction_t read_transaction1;
spi_transaction_t read_transaction2;
spi_transaction_t read_transaction3;
spi_transaction_t cmd_transaction;
uint8_t gpio_already_init_flag = 0;

/**
 * \brief Initializes SPI bus and SPI interface with the ext adc.
 * Initializes SPI bus and SPI interface with the ext adc. For ext adc MCP, it
 * also creates and initializes global data buffers for adc data value
 * transmission.
 *
 */
void adcExtInit(void) {
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .miso_io_num = SPI3_MISO_IO,
        .mosi_io_num = SPI3_MOSI_IO,
        .sclk_io_num = SPI3_SCLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_TRANSFER_SIZE,
        //.intr_flags = ESP_INTR_FLAG_IRAM
    };

    int adc_ext_slck_hz = ADC_EXT_SLCK_HZ_2_EXT_CH;

    if (num_extern_active_chs == 1) {
        adc_ext_slck_hz = ADC_EXT_SLCK_HZ_1_EXT_CH;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = adc_ext_slck_hz,
        .mode =
            SPI_MODE,  // SPI mode 1: (CPOL) = 0 and the clock phase (CPHA) = 1.
        .spics_io_num = -1,  // CS pin not used here, since ESP driver
                             // makes CS pin does not behave in the
                             // correct way for MCP to understand
        .queue_size =
            1,  // We want to be able to queue 1 transactions at a time
        //.pre_cb = lcd_spi_pre_transfer_callback  //Specify pre-transfer
        // callback to handle D/C line .post_cb = adsEndTransCb
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .duty_cycle_pos = 128,  // 50%
        .cs_ena_pretrans =
            0,  // this could be used instead of the manual workaround
        .cs_ena_posttrans = 0,  //  "
        .input_delay_ns =
            0,  // this can be important for higher frequencies (over 8 MHz)
        .flags = 0,
    };

#if _SD_CARD_ENABLED_ != SD_CARD_ENABLED
    // Initialize the SPI bus
    ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_DISABLED);
    ESP_ERROR_CHECK(ret);
    // Attach the device to the SPI bus
    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &adc_ext_spi_handler);
    ESP_ERROR_CHECK(ret);
#else
    // Attach to SD card SPI bus
    ret = spi_bus_add_device(sd_spi_host.slot, &devcfg, &adc_ext_spi_handler);
    ESP_ERROR_CHECK(ret);
#endif

    // Config Drdy GPIO and interrupt
#if (_ADC_EXT_ == ADC_MCP)
    if (!gpio_already_init_flag) adcExtDrdyGpio(MCP_DRDY_IO);

    memset(&read_transaction1, 0,
           sizeof(read_transaction1));  // zero out the transaction
    read_transaction1.flags = SPI_TRANS_USE_TXDATA /*| SPI_TRANS_USE_RXDATA*/;
    read_transaction1.cmd = 0;
    read_transaction1.addr = 0;
    read_transaction1.length = 32;  // length is MAX(in_bits, out_bits)
    read_transaction1.rxlength = 32;
    read_transaction1.rx_buffer = ext_adc_raw_data;

    memset(&read_transaction2, 0,
           sizeof(read_transaction2));  // zero out the transaction
    read_transaction2.flags = SPI_TRANS_USE_TXDATA /*| SPI_TRANS_USE_RXDATA*/;
    read_transaction2.cmd = 0;
    read_transaction2.addr = 0;
    read_transaction2.length = 32;  // length is MAX(in_bits, out_bits)
    read_transaction2.rxlength = 32;
    read_transaction2.rx_buffer = (ext_adc_raw_data + 1);

    memset(&read_transaction3, 0,
           sizeof(read_transaction3));  // zero out the transaction
    read_transaction3.flags = SPI_TRANS_USE_TXDATA /*| SPI_TRANS_USE_RXDATA*/;
    read_transaction3.cmd = 0;
    read_transaction3.addr = 0;
    read_transaction3.length = 32;  // length is MAX(in_bits, out_bits)
    read_transaction3.rxlength = 32;
    read_transaction3.rx_buffer = (ext_adc_raw_data + 2);

    memset(&cmd_transaction, 0,
           sizeof(cmd_transaction));  // zero out the transaction
    cmd_transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    cmd_transaction.length = 8;
    cmd_transaction.rxlength = 8;

#elif (_ADC_EXT_ == ADC_ADS)
    adcExtDrdyGpio(ADS_DRDY_IO);
#endif
}

/**
 * \brief Starts the external ADC.
 *
 */
void adcExtStart(void) {
#if (_ADC_EXT_ == ADC_MCP)
    mcpStart();
#elif (_ADC_EXT_ == ADC_ADS)
    adsStart();
#endif
}

void adcExtStop(void) {
#if (_ADC_EXT_ == ADC_MCP)
    mcpStop();
#elif (_ADC_EXT_ == ADC_ADS)
    adsStop();
#endif
}

#endif

#if (_ADC_EXT_ == ADC_MCP)

static uint8_t IRAM_ATTR mcpGetCmdByte(uint8_t addr, uint8_t cmd) {
    uint8_t cmd_byte = MCP_ADDR << 6;

    // If it is a reg operation
    if (cmd & 0b00000011) {
        cmd_byte |= addr << 2;
        cmd_byte |= cmd;

        // If it is a command not envolving reg
    } else {
        cmd_byte |= cmd;
    }

    return cmd_byte;
}

/**
 * \brief Sends a command to the MCP.
 *
 * \param cmd Command to be sent.
 */
void IRAM_ATTR mcpSendCmd(uint8_t cmd) {
    spi_transaction_t transaction;
    uint8_t cmd_byte = mcpGetCmdByte(0, cmd);

    memset(&transaction, 0, sizeof(transaction));  // zero out the transaction
    transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    transaction.length = 8;
    transaction.rxlength = 8;
    transaction.tx_data[0] = cmd_byte;

    // xSemaphoreTake(spi_mutex, portMAX_DELAY);  // Lock SPI bus
    gpio_set_level(SPI3_CS0_IO,
                   0);  // manually set CS\ active low -> begin comm
    spi_device_polling_transmit(adc_ext_spi_handler,
                                &transaction);  // Transmit!
    gpio_set_level(SPI3_CS0_IO, 1);             // manually set CS\ idle
    // xSemaphoreGive(spi_mutex);                  // Unlock SPI bus
}

/**
 * \brief Writes to a register of the MCP.
 *
 * \param address Address of the register to be written.
 * \param tx_data Data to be written.
 * \param tx_data_bytes Amount of bytes to be written.
 */
void IRAM_ATTR mcpWriteRegister(uint8_t address, uint32_t tx_data,
                                uint8_t tx_data_bytes) {
    spi_transaction_t transaction;
    uint8_t cmd_byte = mcpGetCmdByte(address, WREG);

    if (tx_data_bytes > 1) {
        tx_data = SPI_SWAP_DATA_TX((tx_data), (tx_data_bytes * 8));
    }

    memset(&transaction, 0, sizeof(transaction));  // zero out the transaction
    transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    transaction.length =
        (tx_data_bytes + 1) * 8;  // length is MAX(in_bits, out_bits)
    transaction.rxlength = 8;     // Recieve status byte
    transaction.tx_data[0] = cmd_byte;
    *(uint32_t*)(transaction.tx_data) |= tx_data << 8;

    // xSemaphoreTake(spi_mutex, portMAX_DELAY);  // Lock SPI bus
    //  Transmit
    gpio_set_level(SPI3_CS0_IO,
                   0);  // manually set CS\ active low -> begin comm
    spi_device_polling_transmit(adc_ext_spi_handler,
                                &transaction);  // Transmit!
    gpio_set_level(SPI3_CS0_IO, 1);             // manually set CS\ idle
    // xSemaphoreGive(spi_mutex);                  // Unlock SPI bus
}

/**
 * \brief Reads the ADC values from the MCP.
 *
 * \param address Address of the register to be read. Always REG_ADCDATA (0x00).
 * \param rx_data_bytes Amount of bytes to be read.
 */
void IRAM_ATTR mcpReadADCValues(uint8_t address, uint8_t rx_data_bytes) {
    gpio_intr_disable(MCP_DRDY_IO);

    cmd_transaction.tx_data[0] = 0b01000001;

    // xSemaphoreTake(spi_mutex, portMAX_DELAY);  // Lock SPI bus
    gpio_set_level(SPI3_CS0_IO,
                   0);  // manually set CS\ active low -> begin comm

    spi_device_polling_transmit(adc_ext_spi_handler,
                                &cmd_transaction);  // Transmit command
    spi_device_polling_transmit(adc_ext_spi_handler,
                                &read_transaction1);  // Receive data

    if (rx_data_bytes > 4) {
        spi_device_polling_transmit(adc_ext_spi_handler,
                                    &read_transaction2);  // Receive data
        spi_device_polling_transmit(adc_ext_spi_handler,
                                    &read_transaction3);  // Receive data
        ext_adc_raw_data[1] = SPI_SWAP_DATA_RX(ext_adc_raw_data[1], (32));
        ext_adc_raw_data[2] = SPI_SWAP_DATA_RX(ext_adc_raw_data[2], (32));
    }

    gpio_set_level(SPI3_CS0_IO, 1);  // manually set CS\ idle
    // xSemaphoreGive(spi_mutex);       // Unlock SPI bus

    ext_adc_raw_data[0] = SPI_SWAP_DATA_RX(ext_adc_raw_data[0], (32));

    gpio_intr_enable(MCP_DRDY_IO);
}

/**
 * \brief Reads a register of the MCP.
 *
 * \param address Address of the register to be read.
 * \param rx_data_bytes Amount of bytes to be read.
 * \return uint32_t Register value.
 */
uint32_t mcpReadRegister(uint8_t address, uint8_t rx_data_bytes) {
    spi_transaction_t read_transaction;
    spi_transaction_t cmd_transaction;
    uint8_t cmd_byte = mcpGetCmdByte(address, RREG);

    gpio_intr_disable(MCP_DRDY_IO);

    memset(&read_transaction, 0,
           sizeof(read_transaction));  // zero out the transaction
    read_transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    read_transaction.cmd = 0;
    read_transaction.addr = 0;
    read_transaction.length =
        rx_data_bytes * 8;  // length is MAX(in_bits, out_bits)
    read_transaction.rxlength = rx_data_bytes * 8;
    read_transaction.tx_data[0] = 0;

    memset(&cmd_transaction, 0,
           sizeof(cmd_transaction));  // zero out the transaction
    cmd_transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    cmd_transaction.length = 8;
    cmd_transaction.rxlength = 8;
    cmd_transaction.tx_data[0] = cmd_byte;

    // xSemaphoreTake(spi_mutex, portMAX_DELAY);  // Lock SPI bus
    gpio_set_level(SPI3_CS0_IO,
                   0);  // manually set CS\ active low -> begin comm
    spi_device_polling_transmit(adc_ext_spi_handler,
                                &cmd_transaction);  // Transmit command
    spi_device_polling_transmit(adc_ext_spi_handler,
                                &read_transaction);  // Receive data!
    gpio_set_level(SPI3_CS0_IO, 1);                  // manually set CS\ idle
    // xSemaphoreGive(spi_mutex);                       // Unlock SPI bus

    if (rx_data_bytes > 1) {
        *(uint32_t*)(read_transaction.rx_data) = SPI_SWAP_DATA_RX(
            (*(uint32_t*)(read_transaction.rx_data)), (rx_data_bytes * 8));
    }

    gpio_intr_enable(MCP_DRDY_IO);
    return *(uint32_t*)(read_transaction.rx_data);
}

#define VALUE_CONFIG0 (0b01 << 6) | (0b10 << 4) | (0b00 << 2) | (0b00)
#define VALUE_CONFIG1 0
#define VALUE_CONFIG2 (0b10 << 6) | (0b001 << 3) | (0b0 << 2) | (0b11)
#define VALUE_CONFIG3 \
    (0b11 << 6) | (0b11 << 4) | (0b0 << 3) | (0b0 << 2) | (0b0 << 1) | (0b0)
#define VALUE_IRQ (0b0 << 7) | (0b111 << 4) | (0b01 << 2) | (0b1 << 1) | 0b1
#define VALUE_MUX (0xF0)
#define VALUE_SCAN ((0b000 << 21) | (0b00000 << 16))
#define VALUE_TIMER 0
#define VALUE_OFFSETCAL 0
#define VALUE_GAINCAL (0x800000)

/**
 * \brief Configures the MCP.
 *
 * \param channel_mask Mask of the channels to be used. Can be 0b0001, 0b0010,
 * or 0b0011.
 */
void mcpSetupRoutine(uint8_t channel_mask) {
    adcExtInit();
    gpio_already_init_flag = 1;
    mcpSendCmd(RESET);
    mcpWriteRegister(REG_CONFIG0, VALUE_CONFIG0, 1);
    mcpWriteRegister(REG_CONFIG1, VALUE_CONFIG1, 1);
    mcpWriteRegister(REG_CONFIG2, VALUE_CONFIG2, 1);
    mcpWriteRegister(REG_CONFIG3, VALUE_CONFIG3, 1);
    mcpWriteRegister(REG_IRQ, VALUE_IRQ, 1);
    mcpWriteRegister(REG_MUX, VALUE_MUX, 1);
    mcpWriteRegister(REG_SCAN, VALUE_SCAN | channel_mask,
                     3);  // Delay time between conversions of each channel &
                          // Channels selection
    mcpWriteRegister(REG_TIMER, VALUE_TIMER,
                     3);  //(Delay) number of DMCLK periods between consecutive
                          // SCAN cycles (aka sample rate)
    mcpWriteRegister(REG_OFFSETCAL, VALUE_OFFSETCAL, 3);
    mcpWriteRegister(REG_GAINCAL, VALUE_GAINCAL, 3);

    vTaskDelay(50 / portTICK_PERIOD_MS);

    // Print reg values for debug
    /*for (int reg = REG_CONFIG0; reg <= REG_GAINCAL; reg++) {
        uint32_t value;
        uint32_t num_bytes;

        if (reg >= REG_SCAN) {
            num_bytes = 3;
        } else {
            num_bytes = 1;
        }

        value = mcpReadRegister(reg, num_bytes + 1);
        printf("Reg:%.2x, value:%.6x\n", reg, value);
    }*/
}

/**
 * \brief Starts the MCP.
 *
 */
void mcpStart(void) { mcpSendCmd(START); }

/**
 * \brief Stops the MCP.
 *
 */
void mcpStop(void) {
    mcpSendCmd(FSHUTDOWN);
    spi_bus_remove_device(adc_ext_spi_handler);
    spi_bus_free(SPI3_HOST);
}

/**************************************************************************************************************************************************/
#elif (_ADC_EXT_ == ADC_ADS)
void adsSendCmd(uint8_t cmd) {
    spi_transaction_t transaction;

    memset(&transaction, 0, sizeof(transaction));  // zero out the transaction
    transaction.flags = SPI_TRANS_USE_TXDATA;
    transaction.length = 8;
    transaction.tx_data[0] = cmd;
    transaction.rx_buffer = NULL;  // skip read phase
    spi_device_polling_transmit(adc_ext_spi_handler, &transaction);

    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void adsWriteRegister(uint8_t address, uint8_t data) {
    uint8_t first_opcode = address | WREG;
    uint8_t second_opcode = 0x00;

    switch (address) {
        case 1:
            data = data & 0x87;
            break;
        case 2:
            data = data & 0xFB;
            data |= 0x80;
            break;
        case 3:
            data = data & 0xFD;
            data |= 0x10;
            break;
        case 7:
            data = data & 0x3F;
            break;
        case 8:
            data = data & 0x5F;
            break;
        case 9:
            data |= 0x02;
            break;
        case 10:
            data = data & 0x87;
            data |= 0x01;
            break;
        case 11:
            data = data & 0x0F;
            break;
        default:
            break;
    }

    spi_transaction_t transaction;

    memset(&transaction, 0, sizeof(transaction));  // zero out the transaction
    transaction.flags = SPI_TRANS_USE_TXDATA;
    transaction.length = 24;
    transaction.tx_data[0] = first_opcode;
    transaction.tx_data[1] = second_opcode;
    transaction.tx_data[2] = data;
    transaction.rx_buffer = NULL;  // skip read phase
    spi_device_polling_transmit(adc_ext_spi_handler,
                                &transaction);  // Transmit!

    // vTaskDelay(10/portTICK_PERIOD_MS);
    vTaskDelay(40 / portTICK_PERIOD_MS);
}

void adsSetupRoutine() {
    // Config Drdy GPIO and interrupt
    adcExtDrdyGpio(ADS_DRDY_IO);

    adsSendCmd(RESET);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    adsSendCmd(SDATAC);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // adsReadRegister(ADS1292_REG_ID);
    // adsReadRegister(ADS1292_REG_CONFIG1);
    // adsReadRegister(ADS1292_REG_CONFIG2);

    adsWriteRegister(
        ADS1292_REG_CONFIG2,
        0x80);  // disable reference buffer, Lead-off comparator, etc
    adsWriteRegister(ADS1292_REG_LOFF, 0x10);  // lead-off defaults

    adsWriteRegister(ADS1292_REG_CH1SET,
                     0x10);  // Unit gain, normal electrode input
    adsWriteRegister(ADS1292_REG_CH2SET,
                     0x10);  // Unit gain, normal electrode input

    adsWriteRegister(ADS1292_REG_RLDSENS, 0x00);   // RLD settings defaults
    adsWriteRegister(ADS1292_REG_LOFFSENS, 0x00);  // LOFF settings defaults
    adsWriteRegister(ADS1292_REG_RESP1, 0x02);     // respiration defaults
    adsWriteRegister(ADS1292_REG_RESP2, 0x03);     // respiration defaults
}

//_sampling_rate (Hz) needs to be either 16kHz | 8kHz | 4kHz | 2kHz | 1kHz |
// 500Hz | 250Hz
void adsSetSamplingRate(uint16_t _sampling_rate) {
    if (_sampling_rate != 16000 && _sampling_rate != 8000 &&
        _sampling_rate != 4000 && _sampling_rate != 2000 &&
        _sampling_rate != 1000 && _sampling_rate != 500 &&
        _sampling_rate != 250) {
        DEBUG_PRINT_E("adsSetSamplingRate",
                      "ADS sample rate not valid. Defaulting to 16kHz");
        _sampling_rate = 16000;
    }
    // value to right in register: 0 - 16kHz || 1 - 8kHz || 2 - 4kHz || 3 - 2kHz
    // || 4 - 1kHz || 5 - 500Hz || 6 - 250Hz
    adsWriteRegister(ADS1292_REG_CONFIG1,
                     (uint8_t)(7 - log2(_sampling_rate / 125)));
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

// Configure ads with the channels chosen. Configure SPI transaction in function
// of the CHs chosen
void adsConfigureChannels(uint8_t ads_channel_mask) {
    // 24 status bits + 24 bits x 2 channels = 9bytes required. But DMA buffer
    // needs to be 32bit aligned
    uint8_t* rx_data_ads =
        heap_caps_malloc(3 * sizeof(uint32_t), MALLOC_CAP_DMA);
    memset(rx_data_ads, 0, 3 * sizeof(uint32_t));

    memset(&adc_ext_trans, 0,
           sizeof(adc_ext_trans));  // zero out the transaction

    switch (ads_channel_mask) {
        case 0:  // no ADS channels
        case 1:  // CH1
            adsWriteRegister(
                ADS1292_REG_CH1SET,
                0x10);  // CH1 enabled, gain 1, connected to electrode in
            adsWriteRegister(ADS1292_REG_CH2SET, 0x80);  // CH2 disabled
            adc_ext_trans.length =
                48;  // length is MAX(sending length, recieving length)
            adc_ext_trans.rxlength =
                48;  // 24 status bits + 24 bits x 1 channels
            break;
        case 2:                                          // CH2
            adsWriteRegister(ADS1292_REG_CH1SET, 0x80);  // CH1 disabled
            adsWriteRegister(
                ADS1292_REG_CH2SET,
                0x10);  // CH2 enabled, gain 1, connected to electrode in
            adc_ext_trans.length =
                48;  // length is MAX(sending length, recieving length)
            adc_ext_trans.rxlength =
                48;  // 24 status bits + 24 bits x 1 channels
            break;
        case 3:  // CH1 + CH2
            adsWriteRegister(
                ADS1292_REG_CH1SET,
                0x10);  // CH1 enabled, gain 1, connected to electrode in
            adsWriteRegister(
                ADS1292_REG_CH2SET,
                0x10);  // CH2 enabled, gain 1, connected to electrode in
            adc_ext_trans.length =
                72;  // length is MAX(sending length, recieving length)
            adc_ext_trans.rxlength =
                72;  // 24 status bits + 24 bits x 2 channels
            break;
    }
    adc_ext_trans.tx_buffer = NULL;  // skip write phase
    adc_ext_trans.rx_buffer = rx_data_ads;
}

void adsStart() {
    adsSetSamplingRate(sample_rate);
    // Update ads continous transaction to indicate flag for post call back of
    // spi transfer
    adc_ext_trans.user = (void*)ADS_CONTINUOUS_TRANS;
    adsSendCmd(RDATAC);
    adsSendCmd(START);
    // spi_device_queue_trans(adc_ext_spi_handler, &adc_ext_trans,
    // portMAX_DELAY);
}

void adsStop() {
    // spi_transaction_t *ads_rtrans;

    // Update ads continous transaction to indicate flag for post call back of
    // spi transfer
    adc_ext_trans.user = NULL;
    // Get the trans result of the last queued trans (by adsEndTransCb)
    // spi_device_get_trans_result(adc_ext_spi_handler, &ads_rtrans,
    // portMAX_DELAY);
    adsSendCmd(STOP);
}

// Call back in the end of a transaction made in live mode (when ads is in
// continous mode)
void IRAM_ATTR adsEndTransCb(spi_transaction_t* trans) {
    if ((uint8_t)(trans->user) == ADS_CONTINUOUS_TRANS) {
        spi_device_queue_trans(adc_ext_spi_handler, &adc_ext_trans,
                               portMAX_DELAY);
    }
}

#endif
