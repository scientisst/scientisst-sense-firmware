/**
 * \file sci_com.c
 * \brief General communication functions.
 *
 * This file contains the implementation of functions responsible for processing commands from a client, managing different
 * modes of operation, and controlling communication parameters.
 */

#include "sci_com.h"

#include <string.h>

#include "sci_adc_external.h"
#include "sci_adc_internal.h"
#include "sci_gpio.h"
#include "sci_macros.h"
#include "sci_task_aquisition.h"
#include "sci_timer.h"

#define NUM_UNUSED_BITS_FOR_CH_MASK 2
#define STATUS_PACKET_SIZE 16

DRAM_ATTR int send_fd = 0; ///< File descriptor to send data to client, only part of communication so it is the only global
                           ///< variable not declared in sci_scientisst.h

static void triggerGpio(const uint8_t *buff);
static void triggerDAC(const uint8_t *buff);
static void changeAPI(uint8_t mode);
static void sendStatusPacket();
static void sendFirmwareVersionPacket();
static void selectChsFromMaskScientisst(const uint8_t *buff);
static void setSampleRate(uint8_t *buff);
static void startAcquisition(void);
static uint8_t getPacketSize(void);

/**
 * \brief Process a received packet from the client.
 *
 * This function decodes and processes a command packet received from the client. It performs actions based on the command,
 * such as starting or stopping data acquisition, changing settings, or triggering outputs.
 *
 * \param[in] buff Pointer to the buffer containing the received packet.
 *
 * \return None.
 */
void processPacket(uint8_t *buff)
{
    uint8_t cmd = buff[0] & 0b00000011;

    // Live mode with 0 channels selected
    if ((scientisst_device_settings.api_config.api_mode == API_MODE_BITALINO && buff[0] == 1) ||
        (scientisst_device_settings.api_config.api_mode != API_MODE_BITALINO && buff[0] == 1 && buff[1] == 0))
    {
        return;
    }

    // First check trigger command - it's regardless of the current mode
    if ((buff[0] & 0b10110011) == 0b10110011) // trigger command - Set output GPIO levels
    {
        triggerGpio(buff);
    }
    else if (buff[0] == 0b10100011) // trigger command - Set output DAC level
    {
        triggerDAC(buff);
    }
    else if (scientisst_device_settings.op_mode == OP_MODE_LIVE)
    {
        if (!buff[0])
        {
            stopAcquisition();
        }
    }
    else // If in idle mode
    {
        if (cmd == 0b01 || cmd == 0b10) // Set live mode
        {
            if (cmd == 0b10)
            {
                DEBUG_PRINT_E("processPacket", "Simulation mode is no longer supported");
            }
            // Get channels from mask
            scientisst_device_settings.api_config.select_ch_mask_func(buff);
            startAcquisition();
        }
        else if (cmd == 0b11) // Configuration command
        {
            if ((buff[0] & 0b00111100) == 0) // Set sample rate
            {
                setSampleRate(buff);
            }
            else if (((buff[0] >> 2) & 0b000011) == 0b10) // Send device status
            {
                sendStatusPacket();
            }
            else if (((buff[0] >> 2) & 0b000011) == 0b01) // Send firmware version string
            {
                DEBUG_PRINT_I("processPacket", "sendFirmwareVersion");
                sendFirmwareVersionPacket();
            }
            else if (buff[0] & 0b00110000) // Change API mode
            {
                changeAPI((buff[0] & 0b11110000) >> 4);
            }
        }
        else if (!cmd) // Set battery threshold
        {
            scientisst_device_settings.battery_threshold = 3400 + (((uint16_t)(buff[0] & 0b11111100) >> 2) * 400) / 64;
        }
    }
}

/**
 * \brief Trigger GPIO outputs based on the client's command.
 *
 * This function controls the digital output channels by setting their levels according to the values specified in the
 * received buffer.
 *
 * \param[in] buff Pointer to the buffer containing the trigger command and levels.
 *
 * \return None.
 */
static void triggerGpio(const uint8_t *buff)
{
    uint8_t o2_lvl = (buff[0] & 0b00001000) >> 3;
    uint8_t o1_lvl = (buff[0] & 0b00000100) >> 2;

    gpio_set_level(O0_IO, o1_lvl);
    scientisst_device_settings.gpio_out_state[0] = o1_lvl;
    gpio_set_level(O1_IO, o2_lvl);
    scientisst_device_settings.gpio_out_state[1] = o2_lvl;

    DEBUG_PRINT_I("triggerGpio", "O1 = %d, O2 = %d", o1_lvl, o2_lvl);
}

/**
 * \brief Set the output level of the DAC.
 *
 * This function controls the DAC output, setting its level according to the value specified in the received buffer.
 *
 * \param[in] buff Pointer to the buffer containing the DAC level value.
 *
 * \return None.
 */
static void triggerDAC(const uint8_t *buff)
{
    dac_output_voltage(DAC_CHANNEL_1, buff[1]);
    DEBUG_PRINT_I("triggerDAC", "DAC output to %d", buff[1]);
}

/**
 * \brief Change the API mode of the device.
 *
 * This function switches between different API modes available for the device. It affects the data frames' format.
 * Currently, the API modes are: Bitalino (Legacy) and Scientisst.
 *
 * \param[in] mode The new API mode to switch to.
 *
 * \return None.
 */
static void changeAPI(uint8_t mode)
{
    switch (mode)
    {
    case API_MODE_BITALINO:
        scientisst_device_settings.api_config.api_mode = API_MODE_BITALINO;
        scientisst_device_settings.api_config.select_ch_mask_func = &selectChsFromMaskBitalino;
        DEBUG_PRINT_I("changeAPI", "API changed to BITalino");
        break;
    case API_MODE_SCIENTISST:
        scientisst_device_settings.api_config.api_mode = API_MODE_SCIENTISST;
        scientisst_device_settings.api_config.select_ch_mask_func = &selectChsFromMaskScientisst;
        DEBUG_PRINT_I("changeAPI", "API changed to ScientISST");
        break;
    case API_MODE_SCIENTISST_V2:
        scientisst_device_settings.api_config.api_mode = API_MODE_SCIENTISST_V2;
        scientisst_device_settings.api_config.select_ch_mask_func = &selectChsFromMaskScientisst;
        DEBUG_PRINT_I("changeAPI", "API changed to ScientISST V2");
        break;
    default:
        DEBUG_PRINT_E("changeAPI", "Invalid API mode");
        return;
    }
}

/**
 * \brief Returns the packet size
 *
 * This function returns the packet size in function of the current API mode and the number of active channels.
 *
 * \return The packet size
 */
static uint8_t getPacketSize(void)
{
    uint8_t _packet_size = 0;
    // Table that has the packet size in function of the number of channels
    const uint8_t packet_size_num_chs[DEFAULT_ADC_CHANNELS + 1] = {0, 3, 4, 6, 7, 7, 8};

    switch (scientisst_device_settings.api_config.api_mode)
    {
    case API_MODE_BITALINO:
        _packet_size = packet_size_num_chs[scientisst_device_settings.num_intern_active_chs];
        break;
    case API_MODE_SCIENTISST_V2:
        _packet_size += 3 * scientisst_device_settings.num_extern_active_chs;
        if (!(scientisst_device_settings.num_intern_active_chs % 2)) // If it's an even number
        {
            _packet_size += ((scientisst_device_settings.num_intern_active_chs * 12) / 8);
        }
        else
        {
            //-4 because 4 bits can go in the I/0 byte
            _packet_size += (((scientisst_device_settings.num_intern_active_chs * 12) - 4) / 8);
        }
        _packet_size += 6; // for the I/Os and timestamp+crc bytes
        break;
    case API_MODE_SCIENTISST:
    default:
        _packet_size += 3 * scientisst_device_settings.num_extern_active_chs;
        if (!(scientisst_device_settings.num_intern_active_chs % 2)) // If it's an even number
        {
            _packet_size += ((scientisst_device_settings.num_intern_active_chs * 12) / 8);
        }
        else
        {
            //-4 because 4 bits can go in the I/0 byte
            _packet_size += (((scientisst_device_settings.num_intern_active_chs * 12) - 4) / 8);
        }
        _packet_size += 3; // for the I/Os and seq+crc bytes
        break;
    }

    DEBUG_PRINT_I("getPacketSize", "Packet size is %d bytes", _packet_size);

    return _packet_size;
}

/**
 * \brief Selects the active channels
 *
 * This function selects the active channels according to the received command for the ScientISSTAPI mode.
 *
 * \param[in] buff The received command
 *
 * \return None.
 */
static void selectChsFromMaskScientisst(const uint8_t *buff)
{
    int channel_number = DEFAULT_ADC_CHANNELS + 2;

    // Reset previous active chs
    scientisst_device_settings.num_intern_active_chs = 0;
    scientisst_device_settings.num_extern_active_chs = 0;

    // Select the channels that are activated (with corresponding bit equal to 1)
    for (uint8_t i = 1 << (DEFAULT_ADC_CHANNELS + 2 - 1); i > 0; i >>= 1)
    {
        if (buff[1] & i)
        {
            // Store the activated channels
            if (i == 1 << (DEFAULT_ADC_CHANNELS + 2 - 1) || i == 1 << (DEFAULT_ADC_CHANNELS + 2 - 2))
            {
                scientisst_device_settings.active_ext_chs[scientisst_device_settings.num_extern_active_chs] =
                    (uint8_t)channel_number - 1;
                scientisst_device_settings.num_extern_active_chs++;
            }
            else
            {
                scientisst_device_settings.active_internal_chs[scientisst_device_settings.num_intern_active_chs] =
                    (uint8_t)channel_number - 1;
                scientisst_device_settings.num_intern_active_chs++;
            }

            DEBUG_PRINT_I("selectChsFromMaskBitalino", "Channel A%d added", channel_number);
        }
        channel_number--;
    }

    scientisst_buffers.packet_size = getPacketSize();
}

/**
 * \brief Selects the active channels
 *
 * This function selects the active channels according to the received command for the BITalino API mode.
 *
 * \param[in] buff The received command
 *
 * \return None.
 */
void selectChsFromMaskBitalino(const uint8_t *buff)
{
    int channel_number = DEFAULT_ADC_CHANNELS;

    // Reset previous active chs
    scientisst_device_settings.num_intern_active_chs = 0;
    scientisst_device_settings.num_extern_active_chs = 0;

    // Select the channels that are activated (with corresponding bit equal to 1)
    for (int i = 1 << (DEFAULT_ADC_CHANNELS + NUM_UNUSED_BITS_FOR_CH_MASK - 1); i > NUM_UNUSED_BITS_FOR_CH_MASK; i >>= 1)
    {
        if (buff[0] & i)
        {
            // Store the activated channels into the respective acq_config.channels array
            scientisst_device_settings.active_internal_chs[scientisst_device_settings.num_intern_active_chs] =
                (uint8_t)channel_number - 1;
            scientisst_device_settings.num_intern_active_chs++;
            DEBUG_PRINT_I("selectChsFromMaskBitalino", "Channel A%d added", channel_number - 1);
        }
        channel_number--;
    }
    scientisst_buffers.packet_size = getPacketSize();
}

/**
 * \brief Sets the sample rate
 *
 * This function sets the sample rate according to the received command.
 *
 * \param[in] buff The received command
 *
 * \return None.
 */
static void setSampleRate(uint8_t *buff)
{
    uint32_t aux = 1;

    // API mode bitalino only needs the 2 bits for the sample rate.
    if (scientisst_device_settings.api_config.api_mode == API_MODE_BITALINO)
    {
        for (int i = 0; i < (buff[0] >> 6); i++)
        {
            aux *= 10;
        }
    }
    else
    {
        aux = (*(uint16_t *)(buff + 1) & 0xFFFF);
    }

    scientisst_device_settings.sample_rate_hz = aux;

    DEBUG_PRINT_I("processPacket", "Sampling rate received: %dHz", scientisst_device_settings.sample_rate_hz);
}

/**
 * \brief Starts the acquisition
 *
 * This function starts the acquisition according to the received command. It starts the ADC and the timer. It also changes
 * the LED state.
 *
 * \return None.
 */
static void startAcquisition(void)
{
    // Make sure the data on last buffer (firmware/status packets) are sent
    while (scientisst_buffers.frame_buffer_ready_to_send[NUM_BUFFERS - 1] != 0 && scientisst_device_settings.send_busy != 0)
    {
        DEBUG_PRINT_E("startAcquisition", "Acquisition stuck");
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Clear send buffs, because of potential previous live mode
    scientisst_buffers.tx_curr_buff = 0;
    scientisst_buffers.acq_curr_buff = 0;
    // Clean send buff, because of send status and send firmware string
    scientisst_device_settings.send_busy = 0;

    crc_seq = 0;

    // Clean send buffers, to be sure
    for (int i = 0; i < NUM_BUFFERS; i++)
    {
        memset(scientisst_buffers.frame_buffer[i], 0, scientisst_buffers.frame_buffer_length_bytes);
        scientisst_buffers.frame_buffer_ready_to_send[i] = 0;
    }
    scientisst_buffers.frame_buffer_write_idx = 0;

    // WARNING: if changed, change same code in API
    if (scientisst_device_settings.sample_rate_hz > 100)
    {
        scientisst_buffers.send_threshold =
            !(scientisst_buffers.frame_buffer_length_bytes % scientisst_buffers.packet_size)
                ? scientisst_buffers.frame_buffer_length_bytes // If buf len is a multiple of packet size
                : scientisst_buffers.frame_buffer_length_bytes -
                      (scientisst_buffers.frame_buffer_length_bytes % scientisst_buffers.packet_size); // Else
    }
    else
    {
        scientisst_buffers.send_threshold = scientisst_buffers.packet_size;
    }

    // Start external
#ifdef CONFIG_ADC_EXT
    if (scientisst_device_settings.num_extern_active_chs)
    {
        uint8_t channel_mask = 0;
        for (int i = 0; i < scientisst_device_settings.num_extern_active_chs; i++)
        {
            channel_mask |= 0b1 << (scientisst_device_settings.active_ext_chs[i] - 6);
        }
        mcpSetupRoutine(channel_mask);
        adcExtStart();
    }
#endif

    // Init timer for adc task top start
    timerStart(TIMER_GROUP_MAIN, TIMER_IDX_MAIN, scientisst_device_settings.sample_rate_hz);

    // Set led state to blink at live mode frequency
    updateLEDStatusCode(LIVE_AQUISITION);

    DEBUG_PRINT_W("startAcquisition", "Acquisition started");
    scientisst_device_settings.op_mode = OP_MODE_LIVE;
}

/**
 * \brief Stops the acquisition
 *
 * This function stops the acquisition. It stops the external ADC (if active) and the timer. It also changes the LED state.
 * It cleans all buffers and active channels.
 *
 * \return None.
 */
void stopAcquisition(void)
{
    timerPause(TIMER_GROUP_MAIN, TIMER_IDX_MAIN);

    updateLEDStatusCode(IDLE);

    // Stop external
#ifdef CONFIG_ADC_EXT
    if (scientisst_device_settings.num_extern_active_chs)
    {
        adcExtStop();
    }
#endif

    scientisst_device_settings.op_mode = OP_MODE_IDLE;

    vTaskDelay(50 / portTICK_PERIOD_MS);

    // Reset simulation's signal iterator
    crc_seq = 0;
    scientisst_device_settings.send_busy = 0;

    // Clean send buffers
    for (uint8_t i = 0; i < NUM_BUFFERS; i++)
    {
        memset(scientisst_buffers.frame_buffer[i], 0, scientisst_buffers.frame_buffer_length_bytes);
        scientisst_buffers.frame_buffer_ready_to_send[i] = 0;
    }
    scientisst_buffers.frame_buffer_write_idx = 0;

    scientisst_buffers.tx_curr_buff = 0;
    scientisst_buffers.acq_curr_buff = 0;
    scientisst_device_settings.send_busy = 0;

    // Reset previous active chs
    scientisst_device_settings.num_intern_active_chs = 0;
    scientisst_device_settings.num_extern_active_chs = 0;

    DEBUG_PRINT_W("startAcquisition", "Acquisition stopped");
}

/**
 * \brief Send a status packet to the client.
 *
 * This function prepares a status packet containing various information about the device's current state and sends it to the
 * client.
 *
 * \return None.
 */
static void sendStatusPacket(void)
{
    uint8_t crc = 0;

    // Make sure the buffer is empty and send task is not busy
    while (scientisst_buffers.frame_buffer_ready_to_send[NUM_BUFFERS - 1] != 0 && scientisst_device_settings.send_busy != 0)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    memset(scientisst_buffers.frame_buffer[NUM_BUFFERS - 1], 0, scientisst_buffers.frame_buffer_length_bytes);

    // calculate CRC (except last byte (seq+CRC) )
    for (uint8_t i = 0; i < STATUS_PACKET_SIZE - 1; i++)
    {
        // calculate CRC nibble by nibble
        crc = crc_table[crc] ^ (scientisst_buffers.frame_buffer[NUM_BUFFERS - 1][i] >> 4);
        crc = crc_table[crc] ^ (scientisst_buffers.frame_buffer[NUM_BUFFERS - 1][i] & 0x0F);
    }

    // calculate CRC for last byte (I1|I2|O1|O2|CRC)
    crc = crc_table[crc] ^ (crc_seq & 0x0F);
    crc = 0 | crc_table[crc];

    // store CRC and Seq in the last byte of the packet
    scientisst_buffers.frame_buffer[NUM_BUFFERS - 1][STATUS_PACKET_SIZE - 1] = crc;

    scientisst_buffers.tx_curr_buff = NUM_BUFFERS - 1;
    scientisst_buffers.frame_buffer_ready_to_send[NUM_BUFFERS - 1] = STATUS_PACKET_SIZE;

    // send new data
    xTaskNotifyGive(send_task);
}

/**
 * \brief Send the firmware version packet to the client.
 *
 * This function prepares a packet containing the device's firmware version and possibly other related information, then
 * sends it to the client.
 *
 * \return None.
 */
static void sendFirmwareVersionPacket(void)
{
    uint16_t write_idx = 0;

    // Make sure the buffer is empty and send task is not busy
    while (scientisst_buffers.frame_buffer_ready_to_send[NUM_BUFFERS - 1] != 0 && scientisst_device_settings.send_busy != 0)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    memset(scientisst_buffers.frame_buffer[NUM_BUFFERS - 1], 0, scientisst_buffers.frame_buffer_length_bytes);

    if (scientisst_device_settings.api_config.api_mode != API_MODE_BITALINO)
    {
        memcpy(scientisst_buffers.frame_buffer[NUM_BUFFERS - 1], FIRMWARE_VERSION, strlen(FIRMWARE_VERSION) + 1);
        write_idx += strlen(FIRMWARE_VERSION) + 1;

        // Send ADC1 configurations for raw2voltage precision conversions
        memcpy(scientisst_buffers.frame_buffer[NUM_BUFFERS - 1] + write_idx,
               &(scientisst_device_settings.adc_chars[ADC_INTERNAL_1]), 6 * sizeof(uint32_t));
        // We don't want to send the 2 last pointers of adc_chars struct
        write_idx += 6 * sizeof(uint32_t);
    }
    else
    {
        memcpy(scientisst_buffers.frame_buffer[NUM_BUFFERS - 1], FIRMWARE_BITALINO_VERSION,
               strlen(FIRMWARE_BITALINO_VERSION));
        write_idx += strlen(FIRMWARE_BITALINO_VERSION);
    }

    scientisst_buffers.frame_buffer_ready_to_send[NUM_BUFFERS - 1] = write_idx;
    scientisst_buffers.tx_curr_buff = NUM_BUFFERS - 1;

    xTaskNotifyGive(send_task);

    DEBUG_PRINT_I("sendFirmwareVersionPacket", "Sent firmware version and adc chars");
}
