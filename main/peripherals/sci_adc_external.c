/**
 * \file sci_adc_external.c
 * \brief This file contains the definitions of the functions used to communicate with the external ADC.
 *
 * The external ADC is a MCP3564. It is connected to the ESP32 through SPI.
 */

#include "sci_adc_external.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"

#include "sci_gpio.h"

#define MCP_ADDR 0b01

// Commands
#define RREG 0b01
#define WREG 0b10
#define RESET 0b111000
#define START 0b101000
#define FSHUTDOWN 0b110100 // Full Shutdown; the internal config registers keep the values

// Register Addresses
#define REG_ADCDATA 0x00   // R
#define REG_CONFIG0 0x01   // R/W
#define REG_CONFIG1 0x02   // R/W
#define REG_CONFIG2 0x03   // R/W
#define REG_CONFIG3 0x04   // R/W
#define REG_IRQ 0x05       // R/W
#define REG_MUX 0x06       // R/W
#define REG_SCAN 0x07      // R/W
#define REG_TIMER 0x08     // R/W
#define REG_OFFSETCAL 0x09 // R/W
#define REG_GAINCAL 0x0A   // R/W
#define REG_LOCK 0x0D      // R/W
#define REG_CRCCFG 0X0F    // R

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

#define SPI_MODE 0 // 0 or 3, MCP Only supports these two modes

static spi_bus_config_t buscfg = {
    .miso_io_num = MCP_MISO,
    .mosi_io_num = MCP_MOSI,
    .sclk_io_num = MCP_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4,
    .flags = 0,
}; ///< SPI bus configuration when not using SD card

static spi_device_interface_config_t devcfg = {
    .clock_speed_hz = ADC_EXT_SLCK_HZ_2_EXT_CH,
    .mode = SPI_MODE,   // SPI mode 1: (CPOL) = 0 and the clock phase (CPHA) = 1.
    .spics_io_num = -1, // CS pin not used here, since ESP driver makes CS pin does not behave in the
                        // correct way for MCP to understand
    .queue_size = 1,    // We want to be able to queue 1 transaction at a time
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .duty_cycle_pos = 128, // 50%
    .cs_ena_pretrans = 0,
    .cs_ena_posttrans = 0,
    .input_delay_ns = 0, // this can be important for higher frequencies (over 8 MHz)
    .flags = 0,
}; ///< Device configuration

// SPI
DRAM_ATTR static spi_device_handle_t adc_ext_spi_handle;

// Preallocated and fill SPI transactions for reading ADC values. This allows higher aquisition frequencies
// We need 2 transactions to aquire 1 channel and 4 to acquire 2 channels because sometimes the same channel is sent twice.
// mcp_transactions[0] is always the command transaction, the others are used to read the data.
DMA_ATTR static spi_transaction_t mcp_transactions[4] = {
    {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 8,
        .rxlength = 8,
    },
    {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .cmd = 0,
        .addr = 0,
        .length = 32, // length is MAX(in_bits, out_bits)
        .rxlength = 32,
    },
    {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .cmd = 0,
        .addr = 0,
        .length = 32, // length is MAX(in_bits, out_bits)
        .rxlength = 32,
    },
    {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .cmd = 0,
        .addr = 0,
        .length = 32, // length is MAX(in_bits, out_bits)
        .rxlength = 32,
    },
}; ///< SPI transactions for reading ADC values, 2 transactions for 1 channel, 4 for 2 channels

static void mcpReadADCValues(uint8_t rx_data_bytes);

/**
 * \brief Initializes the SPI bus and configures the external ADC.
 *
 * This function prepares the SPI bus for communication and sets up the initial configuration for the external ADC device. If
 * an SPI host is provided (when in SD Card mode) it attached to that bus, otherwise it initializes the SPI bus.
 *
 * \param[in] spi_host Pointer to the host configuration struct for the SPI bus (NULL if the SD card is not using the SPI
 * bus).
 *
 * \return None.
 */
void adcExtInit(const sdmmc_host_t *spi_host)
{
    esp_err_t ret;
    static uint8_t flag_gpio_already_initialized = 0;

    devcfg.clock_speed_hz =
        (scientisst_device_settings.num_extern_active_chs == 1) ? ADC_EXT_SLCK_HZ_1_EXT_CH : ADC_EXT_SLCK_HZ_2_EXT_CH;

    if (spi_host != NULL)
    { // If SD card is using the SPI bus
        // Attach to SD card SPI bus
        ret = spi_bus_add_device(spi_host->slot, &devcfg, &adc_ext_spi_handle);
        ESP_ERROR_CHECK(ret);
    }
    else
    {
        // gpio_set_level(GPIO_NUM_4, 1);
        // Initialize the SPI bus
        ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_DISABLED);
        ESP_ERROR_CHECK(ret);
        // Attach the device to the SPI bus
        ret = spi_bus_add_device(SPI3_HOST, &devcfg, &adc_ext_spi_handle);
        ESP_ERROR_CHECK(ret);
    }

    if (!flag_gpio_already_initialized)
    {
        flag_gpio_already_initialized = 1;
        adcExtDrdyGpio(MCP_DRDY_IO);
    }
}

/**
 * \brief Retrieves raw ADC values.
 *
 * This function is an interface so other tasks can get the external ADC's readings. It ensures the validity of the channels
 * mask and reads the ADC values accordingly. The values are stored in the provided array.
 *
 * \param[in] channels_mask Mask representing the active channels.
 * \param[in/out] values Array where the read values will be stored.
 *
 * \return ESP_OK if the operation was successful, ESP_FAIL otherwise.
 */
esp_err_t IRAM_ATTR getAdcExtValuesRaw(uint8_t channels_mask, uint32_t values[2])
{
    uint8_t pop_count = 0;
    while (channels_mask)
    {
        pop_count += channels_mask & 1;
        channels_mask >>= 1;
    }
    if (pop_count > 2)
    {
        DEBUG_PRINT_E("Get_ADC_Ext_Values", "Channel mask is not valid: %hhu", channels_mask);
        return ESP_FAIL;
    }

    mcpReadADCValues(4 * pop_count);

    for (int i = 0; i < pop_count; ++i)
    {
        values[i] = 1; // If the raw value is not found, it stays 1. Useful for debugging
        for (int j = 1; j < 4; ++j)
        {
            if ((*(uint32_t *)&(mcp_transactions[j].rx_data) >> 28) ==
                (scientisst_device_settings.active_ext_chs[i] - 6)) // Check if the channel is the one we want
            {
                // If the value is negative, round it to 0
                values[i] = ((*(uint32_t *)&(mcp_transactions[j].rx_data) >> 24) & 0x01)
                                ? 0
                                : (*(uint32_t *)&(mcp_transactions[j].rx_data) & 0x00FFFFFF);
                break;
            }
        }
    }

    return ESP_OK;
}

/**
 * \brief Build a command byte for the MCP.
 *
 * This function constructs a command for the MCP based on the register address and the command.
 *
 * \param[in] addr The address of the register to be accessed.
 * \param[in] cmd The command to be sent.
 *
 * \return uint8_t The command byte.
 */
static inline uint8_t IRAM_ATTR mcpGetCmdByte(uint8_t addr, uint8_t cmd)
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
 * \brief Sends a specific command to the MCP.
 *
 * This function constructs and sends a command to the MCP device via SPI.
 *
 * \param[in] cmd The command to be sent.
 *
 * \return None.
 */
static void mcpSendCmd(uint8_t cmd)
{
    spi_transaction_t transaction;
    uint8_t cmd_byte = mcpGetCmdByte(0, cmd);

    memset(&transaction, 0, sizeof(transaction)); // zero out the transaction
    transaction.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    transaction.length = 8;
    transaction.rxlength = 8;
    transaction.tx_data[0] = cmd_byte;

    gpio_set_level(MCP_CS, 0); // manually set CS\ active low -> begin comm
    spi_device_polling_transmit(adc_ext_spi_handle, &transaction);
    gpio_set_level(MCP_CS, 1); // manually set CS\ idle
}

/**
 * \brief Writes data to a specific register of the MCP.
 *
 * This function writes the specified data to a register on the MCP device using the SPI protocol.
 *
 * \param[in] address The register address.
 * \param[in] tx_data The data to be written.
 * \param[in] tx_data_bytes The size of the data to be written.
 *
 * \return None.
 */
static void mcpWriteRegister(uint8_t address, uint32_t tx_data, uint8_t tx_data_bytes)
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

    gpio_set_level(MCP_CS, 0);
    spi_device_polling_transmit(adc_ext_spi_handle, &transaction);
    gpio_set_level(MCP_CS, 1);
}

/**
 * \brief Function for reading ADC values from the MCP.
 *
 * This function initiates SPI transactions to read ADC values from the MCP. The ADC values are retrieved based on the number
 * of channels requested.
 *
 * \param[in] rx_data_bytes The number of bytes expected to be received representing ADC values.
 *
 * \return None.
 */
static void IRAM_ATTR mcpReadADCValues(uint8_t rx_data_bytes)
{
    gpio_intr_disable(MCP_DRDY_IO);

    mcp_transactions[0].tx_data[0] = 0b01000001;
    *(uint32_t *)&(mcp_transactions[1].rx_data) = 0;
    *(uint32_t *)&(mcp_transactions[2].rx_data) = 0;
    *(uint32_t *)&(mcp_transactions[3].rx_data) = 0;

    gpio_set_level(MCP_CS, 0);

    spi_device_polling_transmit(adc_ext_spi_handle, &mcp_transactions[0]);
    spi_device_polling_transmit(adc_ext_spi_handle, &mcp_transactions[1]);

    if (rx_data_bytes > 4)
    {
        spi_device_polling_transmit(adc_ext_spi_handle, &mcp_transactions[2]);
        spi_device_polling_transmit(adc_ext_spi_handle, &mcp_transactions[3]);
        *(uint32_t *)&(mcp_transactions[2].rx_data) = SPI_SWAP_DATA_RX(*(uint32_t *)&(mcp_transactions[2].rx_data), (32));
        *(uint32_t *)&(mcp_transactions[3].rx_data) = SPI_SWAP_DATA_RX(*(uint32_t *)&(mcp_transactions[3].rx_data), (32));
    }

    gpio_set_level(MCP_CS, 1);

    *(uint32_t *)&(mcp_transactions[1].rx_data) = SPI_SWAP_DATA_RX(*(uint32_t *)&(mcp_transactions[1].rx_data), (32));

    gpio_intr_enable(MCP_DRDY_IO);
}

/**
 * \brief Reads a specific register from the MCP.
 *
 * This function reads the value of a specified register from the MCP device. It handles the SPI communication protocol
 * details and returns the register value.
 *
 * \param[in] address The register address.
 * \param[in] rx_data_bytes The number of bytes to read.
 *
 * \return uint32_t The value of the register.
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

    gpio_set_level(MCP_CS, 0);
    spi_device_polling_transmit(adc_ext_spi_handle, &cmd_transaction);
    spi_device_polling_transmit(adc_ext_spi_handle, &read_transaction);
    gpio_set_level(MCP_CS, 1);

    if (rx_data_bytes > 1)
    {
        *(uint32_t *)(read_transaction.rx_data) =
            SPI_SWAP_DATA_RX((*(uint32_t *)(read_transaction.rx_data)), (rx_data_bytes * 8));
    }

    gpio_intr_enable(MCP_DRDY_IO);
    return *(uint32_t *)(read_transaction.rx_data);
}

/**
 * \brief Configures the MCP device.
 *
 * This function performs a setup routine for the MCP device, which includes sending various commands and configurations. The
 * routine is customized based on the channel mask provided. If the device is not in SD Card mode, it also initializes the
 * SPI bus.
 *
 * \param[in] channel_mask Mask indicating which channels to configure. can be 0b01, 0b10 or 0b11.
 *
 * \return None.
 */
void mcpSetupRoutine(uint8_t channel_mask)
{
    if (scientisst_device_settings.op_settings.com_mode != COM_MODE_SD_CARD)
        adcExtInit(NULL);

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
 * \brief Put the MCP in acquisition mode.
 *
 * This function sends a start command to the MCP, initiating the data acquisition process.
 *
 * \return None.
 */
void adcExtStart(void)
{
    mcpSendCmd(START);
}

/**
 * \brief Stops the ADC and releases resources.
 *
 * This function sends a shutdown command to the MCP, stops the ADC data acquisition, and cleans up the SPI bus resources.
 *
 * \return None.
 */
void adcExtStop(void)
{
    mcpSendCmd(FSHUTDOWN);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    spi_bus_remove_device(adc_ext_spi_handle);
    spi_bus_free(SPI3_HOST);
}
