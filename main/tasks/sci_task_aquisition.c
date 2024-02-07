/**
 * \file sci_task_acquisition.h
 * \brief Sensor Data Acquisition Task
 *
 * This file contains the implementation of the data acquisition task. This task is responsible for continuously acquiring
 * all sensor data. The acquired data is formatted and stored in frames following the format described in /docs/frames.jpg.
 * The acquisition task is synchronized with a timer to sample data at consistent intervals. It interacts with internal and
 * external ADCs, GPIO for IO states, and potentially an IMU, depending on the configuration.
 *
 */

#include "sci_task_aquisition.h"

#include <string.h>

#include "esp_attr.h"

#include "sci_adc_external.h"
#include "sci_adc_internal.h"
#include "sci_gpio.h"
#include "sci_task_imu.h"
#include "sci_timer.h"

#define CALC_BYTE_CRC(_crc, _byte, _crc_table)                                                                              \
    ({                                                                                                                      \
        (_crc) = _crc_table[(_crc)] ^ ((_byte) >> 4);                                                                       \
        (_crc) = _crc_table[(_crc)] ^ ((_byte) & 0x0F);                                                                     \
    })

DRAM_ATTR const uint8_t crc_table[16] = {0, 3, 6, 5, 12, 15, 10, 9, 11, 8, 13, 14, 7, 4, 1, 2}; ///< CRC table
DRAM_ATTR uint16_t crc_seq = 0;                                                                 ///< Frame sequence number

static void getSensorData(uint8_t *io_state, uint16_t *adc_internal_res, uint32_t adc_external_res[2],
                          uint8_t *timestamp_lsb, uint32_t *timestamp_msb);
static void writeFrameScientisst(uint8_t *frame, const uint16_t *adc_internal_res, const uint32_t *adc_external_res,
                                 const uint8_t io_state);
static void writeFrameScientisst_v2(uint8_t *frame, const uint16_t *adc_internal_res, const uint32_t *adc_external_res,
                                    const uint8_t io_state, const uint8_t timestamp_lsb, const uint32_t timestamp_msb);
static void writeFrameBitalino(uint8_t *frame, const uint16_t *adc_internal_res, const uint8_t io_state);
static void writeFrameJSON(const uint8_t *frame, const uint16_t *adc_internal_res, const uint32_t *adc_external_res,
                           const uint8_t io_state);
static inline void writeFrame(uint8_t *frame, const uint16_t *adc_internal_res, const uint32_t *adc_external_res,
                              const uint8_t io_state, const uint8_t timestamp_lsb, const uint32_t timestamp_msb);

/**
 * \brief Main routine for sensor data acquisition.
 *
 * This continuous task is responsible for the regular acquisition of sensor data. It formats this acquired data into
 * specific frame structures (depending on the current API mode) for later transmission. The task manages buffer writing,
 * handles "all buffers full" scenarios, and maintains notifies the sending task when a buffer is ready for transmission.
 *
 * \return Never returns.
 */
_Noreturn void IRAM_ATTR taskAcquisition(void)
{
    uint16_t adc_internal_res[DEFAULT_ADC_CHANNELS] = {0, 0, 0, 0, 0, 0};
    uint32_t adc_external_res[EXT_ADC_CHANNELS] = {0, 0};
    uint8_t io_state = 0;
    uint8_t timestamp_lsb = 0;
    uint32_t timestamp_msb = 0;

    uint8_t acq_next_buff;

    while (1)
    {
        if (!ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
            continue;

        getSensorData(&io_state, adc_internal_res, adc_external_res, &timestamp_lsb, &timestamp_msb);

        // Store sensor data on current buffer with de adequate frame format
        writeFrame(scientisst_buffers.frame_buffer[scientisst_buffers.acq_curr_buff] +
                       scientisst_buffers.frame_buffer_write_idx,
                   adc_internal_res, adc_external_res, io_state, timestamp_lsb, timestamp_msb);

        scientisst_buffers.frame_buffer_write_idx += scientisst_buffers.packet_size; // Update write index
        ++crc_seq;                                                                   // Increment sequence number;

        // Check if acq_curr_buff is above send_threshold and consequently send acq_curr_buff. If we send
        // acq_curr_buff, we need to update it
        if (scientisst_buffers.frame_buffer_write_idx >= scientisst_buffers.send_threshold)
        {
            // Tell bt task that it has acq_curr_buff to send (but it will only send after the buffer is
            // filled above the threshold)
            scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.acq_curr_buff] =
                scientisst_buffers.frame_buffer_write_idx;

            // If send task is idle, wake it up
            xTaskNotifyGive(send_task);

            acq_next_buff = (scientisst_buffers.acq_curr_buff + 1) % (NUM_BUFFERS - 1);

            // Check if next buffer is full. If this happens, it means all buffers are full and bt task
            // can't handle this sending throughput
            while (scientisst_buffers.frame_buffer_ready_to_send[acq_next_buff] != 0)
            {
                DEBUG_PRINT_E("taskAcquisition", "Sending buffer is full, cannot acquire");
                xTaskNotifyGive(send_task);
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
            scientisst_buffers.acq_curr_buff = acq_next_buff;
            scientisst_buffers.frame_buffer_write_idx = 0; // Reset write index
        }
    }
}

/**
 * \brief Collects sensor data from various sources.
 *
 * This function acquires the current state of IOs, reads values from the internal ADC or IMU and/or external ADC. The
 * acquired data is stored in structures passed by reference.
 *
 * \param[in/out] io_state Pointer to a variable where the IO state will be stored.
 * \param[in/out] adc_internal_res Pointer to an array where the internal ADC readings will be stored.
 * \param[in/out] adc_external_res Pointer to an array where the external ADC readings will be stored, if applicable.
 * \param[in/out] timestamp_lsb Pointer to a variable where the 4 LSB of the timestamp will be stored.
 * \param[in/out] timestamp_msb Pointer to a variable where the 32 MSB of the timestamp will be stored.

 *
 * \return None.
 */
static void IRAM_ATTR getSensorData(uint8_t *io_state, uint16_t *adc_internal_res, uint32_t adc_external_res[2],
                                    uint8_t *timestamp_lsb, uint32_t *timestamp_msb)
{
    // Get the IO states
    *io_state = (uint8_t)(gpio_get_level(I0_IO) << 7);
    *io_state |= (uint8_t)(gpio_get_level(I1_IO) << 6);
    *io_state |= (scientisst_device_settings.gpio_out_state[0] & 0b1) << 5;
    *io_state |= (scientisst_device_settings.gpio_out_state[1] & 0b1) << 4;

#ifdef CONFIG_IMU
    // Store values of IMU data into frame
    for (int i = 0; i < scientisst_device_settings.num_intern_active_chs; ++i)
    {
        adc_internal_res[i] = getImuValue(i);
    }
#else
    // Get raw values from A1 to A6 (A1 to A6)
    for (uint8_t i = 0; i < scientisst_device_settings.num_intern_active_chs; ++i)
    {
        adc_internal_res[i] = getAdcInternalValue(ADC_INTERNAL_1, i, 0);
    }
#endif

#ifdef CONFIG_ADC_EXT
    if (scientisst_device_settings.num_extern_active_chs > 0)
    {
        // Get raw values from AX1 & AX2
        esp_err_t res = ESP_OK;
        uint8_t channel_mask = 0;

        if (scientisst_device_settings.active_ext_chs[0] == 6 || scientisst_device_settings.active_ext_chs[1] == 6)
            channel_mask |= 0b01;
        if (scientisst_device_settings.active_ext_chs[0] == 7 || scientisst_device_settings.active_ext_chs[1] == 7)
            channel_mask |= 0b10;

        res = getAdcExtValuesRaw(channel_mask, adc_external_res);
        ESP_ERROR_CHECK_WITHOUT_ABORT(res);
    }
#else
    if (scientisst_device_settings.api_config.api_mode == API_MODE_SCIENTISST_V2)
    {
        uint64_t temp_timestamp = (uint64_t)esp_timer_get_time();
        *timestamp_msb = (uint32_t)((temp_timestamp & 0x0000000FFFFFFFF0) >> 4);
        *timestamp_lsb = (uint8_t)(temp_timestamp & 0x000000000000000F);
    }
    else if (scientisst_device_settings.num_extern_active_chs == 2)
    {
        uint64_t timestamp = (uint64_t)esp_timer_get_time() & 0x0000FFFFFFFFFFFF;
        adc_external_res[0] = (uint32_t)(timestamp & 0x0000000000FFFFFF);
        adc_external_res[1] = (uint32_t)((timestamp >> 24) & 0x0000000000FFFFFF);
    }
#endif
}

/**
 * \brief Based on API mode, selects the appropriate frame format and writes the frame in buffer.
 *
 * This function takes the raw sensor data and formats it into a frame structure
 * that's suitable for transmission, based on the current API mode (ScientISST, BITalino, or JSON).
 *
 * \param[in/out] frame Pointer to the frame buffer where the formatted data will be written.
 * \param[in] adc_internal_res Pointer to an array containing the internal ADC readings.
 * \param[in] adc_external_res Pointer to an array containing the external ADC readings, if applicable.
 * \param[in] io_state The current state of the IOs.
 * \param[in] timestamp_lsb The 4 LSB of the timestamp.
 * \param[in] timestamp_msb The 32 MSB of the timestamp.
 *
 * \return None.
 */
static inline void IRAM_ATTR writeFrame(uint8_t *frame, const uint16_t *adc_internal_res, const uint32_t *adc_external_res,
                                        const uint8_t io_state, const uint8_t timestamp_lsb, const uint32_t timestamp_msb)
{
    switch (scientisst_device_settings.api_config.api_mode)
    {
    case API_MODE_SCIENTISST:
        writeFrameScientisst(frame, adc_internal_res, adc_external_res, io_state);
        break;
    case API_MODE_SCIENTISST_V2:
        writeFrameScientisst_v2(frame, adc_internal_res, adc_external_res, io_state, timestamp_lsb, timestamp_msb);
        break;
    case API_MODE_JSON:
        writeFrameJSON(frame, adc_internal_res, adc_external_res, io_state);
        break;
    case API_MODE_BITALINO:
    default:
        writeFrameBitalino(frame, adc_internal_res, io_state);
        break;
    }
}

/**
 * \brief Writes sensor data into a frame in the ScientISST_V2 format.
 *
 * This function formats and stores sensor data, including ADC readings and IO states, into a frame with the ScientISST_V2
 * API format. ScientISST_V2 always includes a 36bit timestamp in microseconds (since boot). It also calculates and appends
 * the necessary CRC values for data integrity verification.
 *
 * \param[in/out] frame Pointer to the frame buffer where the formatted data will be written.
 * \param[in] adc_internal_res Pointer to an array containing the internal ADC readings.
 * \param[in] adc_external_res Pointer to an array containing the external ADC readings, if applicable.
 * \param[in] io_state The current state of the IOs.
 * \param[in] timestamp_lsb The 4 LSB of the timestamp.
 * \param[in] timestamp_msb The 32 MSB of the timestamp.
 *
 * \return None.
 */
static void IRAM_ATTR writeFrameScientisst_v2(uint8_t *frame, const uint16_t *adc_internal_res,
                                              const uint32_t *adc_external_res, const uint8_t io_state,
                                              const uint8_t timestamp_lsb, const uint32_t timestamp_msb)
{
    uint8_t crc = 0;
    uint8_t frame_next_wr = 0;
    uint8_t wr_mid_byte_flag = 0;

    for (uint8_t i = 0; i < scientisst_device_settings.num_extern_active_chs; ++i)
    {
        *(uint32_t *)(frame + frame_next_wr) |= adc_external_res[i];
        frame_next_wr += 3;
    }

    // Store values of internal channels into frame
    for (int i = 0; i < scientisst_device_settings.num_intern_active_chs; ++i)
    {
        if (!wr_mid_byte_flag)
        {
            *(uint16_t *)(frame + frame_next_wr) |= adc_internal_res[i];
            ++frame_next_wr;
            wr_mid_byte_flag = 1;
        }
        else
        {
            *(uint16_t *)(frame + frame_next_wr) |= adc_internal_res[i] << 4;
            frame_next_wr += 2;
            wr_mid_byte_flag = 0;
        }
    }

    // Store IO states into frame
    frame[scientisst_buffers.packet_size - 3] = io_state;

    // Calculate CRC & store timestamp ---------------------------------------------------------------------------
    *(uint8_t *)(frame + scientisst_buffers.packet_size - 5) = (uint8_t)((timestamp_lsb & 0x0F) << 4);
    *(uint32_t *)(frame + scientisst_buffers.packet_size - 4) = timestamp_msb;

    // calculate CRC (except last byte (seq+CRC) )
    for (int i = 0; i < scientisst_buffers.packet_size - 5; ++i)
    {
        // calculate CRC nibble by nibble
        CALC_BYTE_CRC(crc, frame[i], crc_table);
    }

    // calculate CRC for timestamp
    crc = crc_table[crc] ^ (frame[scientisst_buffers.packet_size - 5] >> 4);
    CALC_BYTE_CRC(crc, frame[scientisst_buffers.packet_size - 4], crc_table);
    CALC_BYTE_CRC(crc, frame[scientisst_buffers.packet_size - 3], crc_table);
    CALC_BYTE_CRC(crc, frame[scientisst_buffers.packet_size - 2], crc_table);
    CALC_BYTE_CRC(crc, frame[scientisst_buffers.packet_size - 1], crc_table);

    crc = crc_table[crc];

    frame[scientisst_buffers.packet_size - 5] |= crc;
}

/**
 * \brief Writes sensor data into a frame in the ScientISST format.
 *
 * This function formats and stores sensor data, including ADC readings and IO states, into a frame with the ScientISST API
 * format. It also calculates and appends the necessary CRC values for data integrity verification.
 *
 * \param[in/out] frame Pointer to the frame buffer where the formatted data will be written.
 * \param[in] adc_internal_res Pointer to an array containing the internal ADC readings.
 * \param[in] adc_external_res Pointer to an array containing the external ADC readings, if applicable.
 * \param[in] io_state The current state of the IOs.
 *
 * \return None.
 */
static void IRAM_ATTR writeFrameScientisst(uint8_t *frame, const uint16_t *adc_internal_res,
                                           const uint32_t *adc_external_res, const uint8_t io_state)
{
    uint8_t crc = 0;
    uint8_t frame_next_wr = 0;
    uint8_t wr_mid_byte_flag = 0;

    for (uint8_t i = 0; i < scientisst_device_settings.num_extern_active_chs; ++i)
    {
        *(uint32_t *)(frame + frame_next_wr) |= adc_external_res[i];
        frame_next_wr += 3;
    }

    // Store values of internal channels into frame
    for (int i = 0; i < scientisst_device_settings.num_intern_active_chs; ++i)
    {
        if (!wr_mid_byte_flag)
        {
            *(uint16_t *)(frame + frame_next_wr) |= adc_internal_res[i];
            ++frame_next_wr;
            wr_mid_byte_flag = 1;
        }
        else
        {
            *(uint16_t *)(frame + frame_next_wr) |= adc_internal_res[i] << 4;
            frame_next_wr += 2;
            wr_mid_byte_flag = 0;
        }
    }

    // Store IO states into frame
    frame[scientisst_buffers.packet_size - 3] = io_state;

    // Calculate CRC & SEQ Number---------------------------------------------------------
    // Store seq number
    *(uint16_t *)(frame + scientisst_buffers.packet_size - 2) = (uint16_t)((crc_seq & 0x0FFF) << 4);

    // calculate CRC (except last byte (seq+CRC) )
    for (int i = 0; i < scientisst_buffers.packet_size - 2; ++i)
    {
        // calculate CRC nibble by nibble
        CALC_BYTE_CRC(crc, frame[i], crc_table);
    }

    // calculate CRC for seq
    crc = crc_table[crc] ^ (frame[scientisst_buffers.packet_size - 2] >> 4);  // Calculate CRC for first 4 bits of seq
    CALC_BYTE_CRC(crc, frame[scientisst_buffers.packet_size - 1], crc_table); // Calcultate CRC for last byte of seq

    crc = crc_table[crc];

    frame[scientisst_buffers.packet_size - 2] |= crc;
}

/**
 * \brief Writes sensor data into a frame in the BITalino format.
 *
 * This function formats and stores sensor data, including ADC readings and IO states, into a frame with the BITalino API
 * format. It also calculates and appends the necessary CRC values for data integrity verification.
 *
 * \param[in/out] frame Pointer to the frame buffer where the formatted data will be written.
 * \param[in] adc_internal_res Pointer to an array containing the internal ADC readings.
 * \param[in] io_state The current state of the IOs.
 *
 * \return None.
 */
static void writeFrameBitalino(uint8_t *frame, const uint16_t *adc_internal_res, const uint8_t io_state)
{
    uint16_t adc_res[6] = {0, 0, 0, 0, 0, 0};
    uint8_t crc = 0;

    // Bitalino API only supports 10 bit resolution for the internal adc, convert it with lsr
    for (int i = 0; i < scientisst_device_settings.num_intern_active_chs; ++i)
    {
        adc_res[i] = adc_internal_res[i] >> 2;
        DEBUG_PRINT_I("writeFrameBitalino", "(adc_res)A%d=%d", scientisst_device_settings.active_internal_chs[i],
                      adc_res[i]);
    }

    frame[scientisst_buffers.packet_size - 2] = io_state;

    // Store values of channels into frame
    // This is legacy code kept for compatibility with an old API. It is not optimized, but it works.
    switch (scientisst_device_settings.num_intern_active_chs)
    {
    case 6:
        frame[0] |= adc_res[scientisst_device_settings.num_intern_active_chs - 6] >> 4;
        __attribute__((fallthrough));
    case 5:
        *(uint16_t *)frame |= (adc_res[scientisst_device_settings.num_intern_active_chs - 5] & 0x3F0) << 2;
        __attribute__((fallthrough));
    case 4:
        *(uint16_t *)(frame + scientisst_buffers.packet_size - 7) |=
            adc_res[scientisst_device_settings.num_intern_active_chs - 4] << 4;
        __attribute__((fallthrough));
    case 3:
        *(uint16_t *)(frame + scientisst_buffers.packet_size - 6) |=
            adc_res[scientisst_device_settings.num_intern_active_chs - 3] << 6;
        __attribute__((fallthrough));
    case 2:
        *(uint16_t *)(frame + scientisst_buffers.packet_size - 4) |=
            adc_res[scientisst_device_settings.num_intern_active_chs - 2];
        __attribute__((fallthrough));
    case 1:
        *(uint16_t *)(frame + scientisst_buffers.packet_size - 3) |=
            adc_res[scientisst_device_settings.num_intern_active_chs - 1] << 2;
        __attribute__((fallthrough));
    default:
        break;
    }

    // calculate CRC (except last byte (seq+CRC))
    for (int i = 0; i < scientisst_buffers.packet_size - 1; ++i)
    {
        CALC_BYTE_CRC(crc, frame[i], crc_table); // calculate CRC nibble by nibble
    }

    // calculate CRC for last byte (seq+CRC)
    crc = crc_table[crc] ^ (crc_seq & 0x0F);
    crc = (uint8_t)((crc_seq << 4) | crc_table[crc]);

    // store CRC and Seq in the last byte of the packet
    frame[scientisst_buffers.packet_size - 1] = crc;
}

/**
 * \brief Writes sensor data into a frame in the JSON format.
 *
 * This function formats and stores sensor data, including ADC readings and IO states, into a JSON structure.
 *
 * \param[in/out] frame Pointer to the frame buffer where the formatted data will be written.
 * \param[in] adc_internal_res Pointer to an array containing the internal ADC readings.
 * \param[in] adc_external_res Pointer to an array containing the external ADC readings, if applicable.
 * \param[in] io_state The current state of the IOs.
 *
 * \return None.
 *
 * \warning This API mode is not fully working.
 */
static void writeFrameJSON(const uint8_t *frame, const uint16_t *adc_internal_res, const uint32_t *adc_external_res,
                           const uint8_t io_state)
{
    char ch_str[10];
    char value_str[10];
    cJSON *item;

    // Get and store the IO states into json
    sprintf(value_str, "%01d", (io_state >> 7) & 0b1);
    item = cJSON_GetObjectItemCaseSensitive(scientisst_buffers.json, "I1");
    strcpy(item->valuestring, value_str);

    sprintf(value_str, "%01d", (io_state >> 6) & 0b1);
    item = cJSON_GetObjectItemCaseSensitive(scientisst_buffers.json, "I2");
    strcpy(item->valuestring, value_str);

    sprintf(value_str, "%01d", (io_state >> 5) & 0b1);
    item = cJSON_GetObjectItemCaseSensitive(scientisst_buffers.json, "O1");
    strcpy(item->valuestring, value_str);

    sprintf(value_str, "%01d", (io_state >> 4) & 0b1);
    item = cJSON_GetObjectItemCaseSensitive(scientisst_buffers.json, "O2");
    strcpy(item->valuestring, value_str);

    // Store values of channels into JSON object
    for (int i = scientisst_device_settings.num_intern_active_chs - 1; i >= 0; --i)
    {
        sprintf(value_str, "%04d", adc_internal_res[i]);
        sprintf(ch_str, "AI%d", scientisst_device_settings.active_internal_chs[i] + 1);
        item = cJSON_GetObjectItemCaseSensitive(scientisst_buffers.json, ch_str);
        strcpy(item->valuestring, value_str);
    }

    if (scientisst_device_settings.num_extern_active_chs > 0)
    {
        for (int i = scientisst_device_settings.num_extern_active_chs - 1; i >= 0; --i)
        {
            sprintf(value_str, "%08d", adc_external_res[i]);
            sprintf(ch_str, "AX%d", scientisst_device_settings.active_ext_chs[i] + 1 - 6);
            item = cJSON_GetObjectItemCaseSensitive(scientisst_buffers.json, ch_str);
            strcpy(item->valuestring, value_str);
        }
    }
}
