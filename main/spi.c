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

// Refer to MCP3564 Datasheet for configuration options
#define VALUE_CONFIG0 (0b01 << 6) | (0b10 << 4) | (0b00 << 2) | (0b00)
#define VALUE_CONFIG1 0
#define VALUE_CONFIG2 (0b10 << 6) | (0b001 << 3) | (0b0 << 2) | (0b11)
#define VALUE_CONFIG3 (0b11 << 6) | (0b11 << 4) | (0b0 << 3) | (0b0 << 2) | (0b0 << 1) | (0b0)
#define VALUE_IRQ (0b0 << 7) | (0b111 << 4) | (0b01 << 2) | (0b1 << 1) | 0b1
#define VALUE_MUX (0xF0)
#define VALUE_SCAN ((0b000 << 21) | (0b00000 << 16))
#define VALUE_TIMER 0
#define VALUE_OFFSETCAL 0
#define VALUE_GAINCAL (0x800000)

// Preallocated and fill SPI transactions for reading ADC values. This allows higher aquisition frequencies
spi_transaction_t read_transaction1 = {
    .flags = SPI_TRANS_USE_TXDATA,
    .cmd = 0,
    .addr = 0,
    .length = 32, // length is MAX(in_bits, out_bits)
    .rxlength = 32,
};
spi_transaction_t read_transaction2 = {
    .flags = SPI_TRANS_USE_TXDATA,
    .cmd = 0,
    .addr = 0,
    .length = 32, // length is MAX(in_bits, out_bits)
    .rxlength = 32,
};
spi_transaction_t read_transaction3 = {
    .flags = SPI_TRANS_USE_TXDATA,
    .cmd = 0,
    .addr = 0,
    .length = 32, // length is MAX(in_bits, out_bits)
    .rxlength = 32,
};
spi_transaction_t cmd_transaction = {
    .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    .length = 8,
    .rxlength = 8,
};
uint8_t gpio_already_init_flag = 0;

/**
 * \brief Initializes SPI bus and SPI interface with the ext adc.
 * Initializes SPI bus and SPI interface with the ext adc. For ext adc MCP, it
 * also creates and initializes global data buffers for adc data value
 * transmission.
 *
 */
void adcExtInit(void)
{
    esp_err_t ret;

#if _SD_CARD_ENABLED_ != SD_CARD_ENABLED
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI3_MISO_IO,
        .mosi_io_num = SPI3_MOSI_IO,
        .sclk_io_num = SPI3_SCLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4,
    };
#endif

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = (num_extern_active_chs == 1) ? ADC_EXT_SLCK_HZ_1_EXT_CH : ADC_EXT_SLCK_HZ_2_EXT_CH,
        .mode = SPI_MODE,   // SPI mode 1: (CPOL) = 0 and the clock phase (CPHA) = 1.
        .spics_io_num = -1, // CS pin not used here, since ESP driver makes CS pin does not behave in the
                            // correct way for MCP to understand
        .queue_size = 1,    // We want to be able to queue 1 transactions at a time
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .duty_cycle_pos = 128, // 50%
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .input_delay_ns = 0, // this can be important for higher frequencies (over 8 MHz)
        .flags = 0,
    };

#if _SD_CARD_ENABLED_ == SD_CARD_DISABLED
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
    if (!gpio_already_init_flag)
        adcExtDrdyGpio(MCP_DRDY_IO);

    read_transaction1.rx_buffer = ext_adc_raw_data;
    read_transaction2.rx_buffer = (ext_adc_raw_data + 1);
    read_transaction3.rx_buffer = (ext_adc_raw_data + 2);
}

/**
 * \brief Starts the external ADC.
 *
 */
void adcExtStart(void)
{
    mcpStart();
}

void adcExtStop(void)
{
    mcpStop();
}

static uint8_t IRAM_ATTR mcpGetCmdByte(uint8_t addr, uint8_t cmd)
{
    uint8_t cmd_byte = MCP_ADDR << 6;

    if (cmd & 0b00000011) // If it is a reg operation
    {
        cmd_byte |= addr << 2;
        cmd_byte |= cmd;
    }
    else // If it is a command not envolving reg
    {
        cmd_byte |= cmd;
    }

    return cmd_byte;
}

/**
 * \brief Sends a command to the MCP.
 *
 * \param cmd Command to be sent.
 */
void IRAM_ATTR mcpSendCmd(uint8_t cmd)
{
    spi_transaction_t transaction;
    uint8_t cmd_byte = mcpGetCmdByte(0, cmd);

    memset(&transaction, 0, sizeof(transaction)); // zero out the transaction
    transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    transaction.length = 8;
    transaction.rxlength = 8;
    transaction.tx_data[0] = cmd_byte;

    gpio_set_level(SPI3_CS0_IO, 0); // manually set CS\ active low -> begin comm
    spi_device_polling_transmit(adc_ext_spi_handler, &transaction);
    gpio_set_level(SPI3_CS0_IO, 1); // manually set CS\ idle
}

/**
 * \brief Writes to a register of the MCP.
 *
 * \param address Address of the register to be written.
 * \param tx_data Data to be written.
 * \param tx_data_bytes Amount of bytes to be written.
 */
void IRAM_ATTR mcpWriteRegister(uint8_t address, uint32_t tx_data, uint8_t tx_data_bytes)
{
    spi_transaction_t transaction;
    uint8_t cmd_byte = mcpGetCmdByte(address, WREG);

    if (tx_data_bytes > 1)
    {
        tx_data = SPI_SWAP_DATA_TX((tx_data), (tx_data_bytes * 8));
    }

    memset(&transaction, 0, sizeof(transaction)); // zero out the transaction
    transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    transaction.length = (tx_data_bytes + 1) * 8; // length is MAX(in_bits, out_bits)
    transaction.rxlength = 8;                     // Recieve status byte
    transaction.tx_data[0] = cmd_byte;
    *(uint32_t *)(transaction.tx_data) |= tx_data << 8;

    gpio_set_level(SPI3_CS0_IO, 0);
    spi_device_polling_transmit(adc_ext_spi_handler, &transaction);
    gpio_set_level(SPI3_CS0_IO, 1);
}

/**
 * \brief Reads the ADC values from the MCP.
 *
 * \param address Address of the register to be read. Always REG_ADCDATA (0x00).
 * \param rx_data_bytes Amount of bytes to be read.
 */
void IRAM_ATTR mcpReadADCValues(uint8_t address, uint8_t rx_data_bytes)
{
    gpio_intr_disable(MCP_DRDY_IO);

    cmd_transaction.tx_data[0] = 0b01000001;

    gpio_set_level(SPI3_CS0_IO, 0);

    spi_device_polling_transmit(adc_ext_spi_handler, &cmd_transaction);
    spi_device_polling_transmit(adc_ext_spi_handler, &read_transaction1);

    if (rx_data_bytes > 4)
    {
        spi_device_polling_transmit(adc_ext_spi_handler, &read_transaction2);
        spi_device_polling_transmit(adc_ext_spi_handler, &read_transaction3);
        ext_adc_raw_data[1] = SPI_SWAP_DATA_RX(ext_adc_raw_data[1], (32));
        ext_adc_raw_data[2] = SPI_SWAP_DATA_RX(ext_adc_raw_data[2], (32));
    }

    gpio_set_level(SPI3_CS0_IO, 1);

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
uint32_t mcpReadRegister(uint8_t address, uint8_t rx_data_bytes)
{
    spi_transaction_t read_transaction;
    spi_transaction_t cmd_transaction;
    uint8_t cmd_byte = mcpGetCmdByte(address, RREG);

    gpio_intr_disable(MCP_DRDY_IO);

    memset(&read_transaction, 0, sizeof(read_transaction));
    read_transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    read_transaction.cmd = 0;
    read_transaction.addr = 0;
    read_transaction.length = rx_data_bytes * 8; // length is MAX(in_bits, out_bits)
    read_transaction.rxlength = rx_data_bytes * 8;
    read_transaction.tx_data[0] = 0;

    memset(&cmd_transaction, 0, sizeof(cmd_transaction));
    cmd_transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    cmd_transaction.length = 8;
    cmd_transaction.rxlength = 8;
    cmd_transaction.tx_data[0] = cmd_byte;

    gpio_set_level(SPI3_CS0_IO, 0);
    spi_device_polling_transmit(adc_ext_spi_handler, &cmd_transaction);
    spi_device_polling_transmit(adc_ext_spi_handler, &read_transaction);
    gpio_set_level(SPI3_CS0_IO, 1);

    if (rx_data_bytes > 1)
    {
        *(uint32_t *)(read_transaction.rx_data) =
            SPI_SWAP_DATA_RX((*(uint32_t *)(read_transaction.rx_data)), (rx_data_bytes * 8));
    }

    gpio_intr_enable(MCP_DRDY_IO);
    return *(uint32_t *)(read_transaction.rx_data);
}

/**
 * \brief Configures the MCP.
 *
 * \param channel_mask Mask of the channels to be used. Can be 0b0001, 0b0010,
 * or 0b0011.
 */
void mcpSetupRoutine(uint8_t channel_mask)
{
    adcExtInit();
    gpio_already_init_flag = 1;
    mcpSendCmd(RESET);
    mcpWriteRegister(REG_CONFIG0, VALUE_CONFIG0, 1);
    mcpWriteRegister(REG_CONFIG1, VALUE_CONFIG1, 1);
    mcpWriteRegister(REG_CONFIG2, VALUE_CONFIG2, 1);
    mcpWriteRegister(REG_CONFIG3, VALUE_CONFIG3, 1);
    mcpWriteRegister(REG_IRQ, VALUE_IRQ, 1);
    mcpWriteRegister(REG_MUX, VALUE_MUX, 1);
    mcpWriteRegister(REG_SCAN, VALUE_SCAN | channel_mask, 3);
    mcpWriteRegister(REG_TIMER, VALUE_TIMER, 3);
    mcpWriteRegister(REG_OFFSETCAL, VALUE_OFFSETCAL, 3);
    mcpWriteRegister(REG_GAINCAL, VALUE_GAINCAL, 3);

    vTaskDelay(50 / portTICK_PERIOD_MS);
}

/**
 * \brief Starts the MCP.
 *
 */
void mcpStart(void)
{
    mcpSendCmd(START);
}

/**
 * \brief Stops the MCP.
 *
 */
void mcpStop(void)
{
    mcpSendCmd(FSHUTDOWN);
    spi_bus_remove_device(adc_ext_spi_handler);
    spi_bus_free(SPI3_HOST);
}
