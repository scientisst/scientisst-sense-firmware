#include "include/sci_task_aquisition.h"

#include "esp_attr.h"
#include "sdkconfig.h"

#include "sci_adc_external.h"
#include "sci_adc_internal.h"
#include "sci_gpio.h"
#include "sci_macros.h"
#include "sci_scientisst.h"
#include "sci_task_imu.h"
#include "sci_timer.h"

#define CALC_BYTE_CRC(_crc, _byte, _crc_table)                                                                              \
    ({                                                                                                                      \
        (_crc) = _crc_table[(_crc)] ^ ((_byte) >> 4);                                                                       \
        (_crc) = _crc_table[(_crc)] ^ ((_byte)&0x0F);                                                                       \
    })

#define CHECK_NUM_CHANNELS_IS_VALID(_num_int_chan, _num_ext_chan)                                                           \
    ({                                                                                                                      \
        if (_num_int_chan > 6 || _num_ext_chan > 2)                                                                         \
        {                                                                                                                   \
            DEBUG_PRINT_E("task_acquisition", "Invalid number of channels, access out of bounds");                          \
            abort();                                                                                                        \
        }                                                                                                                   \
    })

const uint8_t crc_table[16] = {0, 3, 6, 5, 12, 15, 10, 9, 11, 8, 13, 14, 7, 4, 1, 2};
uint16_t crc_seq = 0; ///< Frame sequence number

static void get_sensor_data(uint8_t *io_state, uint16_t *adc_internal_res, uint32_t *adc_external_res);
static void write_frame_Scientisst(uint8_t *frame, const uint16_t *adc_internal_res, const uint32_t *adc_external_res,
                                   const uint8_t io_state);
static void write_frame_Bitalino(uint8_t *frame, const uint16_t *adc_internal_res, const uint8_t io_state);
static void write_frame_JSON(uint8_t *frame, const uint16_t *adc_internal_res, const uint32_t *adc_external_res,
                             const uint8_t io_state);
static inline void IRAM_ATTR write_frame(uint8_t *frame, const uint16_t *adc_internal_res, const uint32_t *adc_external_res,
                                         const uint8_t io_state);

/**
 * \brief Task that acquires data from adc1.
 *
 * This task is responsible for acquiring data from adc1. It is notified by the
 * timerGrp0Isr when to acquire data. It notifies the sendTask when there is
 * data to send. It is also the main task of CPU1 (APP CPU)
 */
_Noreturn void IRAM_ATTR task_acquisition(void *not_used)
{
    uint16_t adc_internal_res[DEFAULT_ADC_CHANNELS] = {0, 0, 0, 0, 0, 0};
    uint32_t adc_external_res[EXT_ADC_CHANNELS] = {0, 0};
    uint8_t io_state = 0;

    uint8_t acq_next_buff;

    while (1)
    {
        if (!ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
            continue;

        CHECK_NUM_CHANNELS_IS_VALID(scientisst_device_settings.num_intern_active_chs,
                                    scientisst_device_settings.num_extern_active_chs);
        get_sensor_data(&io_state, adc_internal_res, adc_external_res);

        // Store sensor data on current buffer with de adequate frame format
        write_frame(scientisst_buffers.frame_buffer[scientisst_buffers.acq_curr_buff] +
                        scientisst_buffers.frame_buffer_write_idx[scientisst_buffers.acq_curr_buff],
                    adc_internal_res, adc_external_res, io_state);
        ++crc_seq; // Increment sequence number
        scientisst_buffers.frame_buffer_write_idx[scientisst_buffers.acq_curr_buff] +=
            scientisst_buffers.packet_size; // Update write index

        // Check if acq_curr_buff is above send_threshold and consequently send acq_curr_buff. If we send
        // acq_curr_buff, we need to update it
        if (scientisst_buffers.frame_buffer_write_idx[scientisst_buffers.acq_curr_buff] >= scientisst_buffers.send_threshold)
        {
            // Tell bt task that it has acq_curr_buff to send (but it will only send after the buffer is
            // filled above the threshold)
            scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.acq_curr_buff] = 1;

            // If send task is idle, wake it up
            xTaskNotifyGive(send_task);

            acq_next_buff = (scientisst_buffers.acq_curr_buff + 1) % (NUM_BUFFERS - 1);

            // Check if next buffer is full. If this happens, it means all buffers are full and bt task
            // can't handle this sending throughput
            while (scientisst_buffers.frame_buffer_ready_to_send[acq_next_buff] == 1)
            {
                DEBUG_PRINT_W("task_acquisition", "Sending buffer is full, cannot acquire");
                xTaskNotifyGive(send_task);
                vTaskDelay(10 / portTICK_PERIOD_MS); // TODO: test with 1 MS
            }
            scientisst_buffers.acq_curr_buff = acq_next_buff;
        }
    }
}

static void IRAM_ATTR get_sensor_data(uint8_t *io_state, uint16_t *adc_internal_res,
                                      uint32_t adc_external_res[EXT_ADC_CHANNELS])
{
    // Get the IO states
    *io_state = (uint8_t)(gpio_get_level(I0_IO) << 7);
    *io_state |= (uint8_t)(gpio_get_level(I1_IO) << 6);
    *io_state |= (scientisst_device_settings.gpio_out_state[0] & 0b1) << 5;
    *io_state |= (scientisst_device_settings.gpio_out_state[1] & 0b1) << 4;

#if _IMU_ == IMU_ENABLED
    // Store values of IMU data into frame
    for (int i = 0; i < num_intern_active_chs; ++i)
    {
        adc_internal_res[i] = get_imu_value(i);
    }
#else
    // Get raw values from A1 to A6 (A1 to A6)
    for (int i = 0; i < scientisst_device_settings.num_intern_active_chs; ++i)
    {
        adc_internal_res[i] = (uint16_t)adc1_get_raw(
            scientisst_device_settings.analog_channels[scientisst_device_settings.active_internal_chs[i]]);
    }
#endif

#if _ADC_EXT_ == EXT_ADC_ENABLED
    if (scientisst_device_settings.num_extern_active_chs > 0)
    {
        // Get raw values from AX1 & AX2
        esp_err_t res = ESP_OK;
        uint8_t channel_mask = 0;

        if (scientisst_device_settings.active_ext_chs[0] == 6 || scientisst_device_settings.active_ext_chs[1] == 6)
            channel_mask |= 0b01;
        if (scientisst_device_settings.active_ext_chs[0] == 7 || scientisst_device_settings.active_ext_chs[1] == 7)
            channel_mask |= 0b10;

        res = get_adc_ext_values_raw(channel_mask, adc_external_res);
        ESP_ERROR_CHECK_WITHOUT_ABORT(res);
    }
#else
    if (num_extern_active_chs == 2)
    {
        // Timestamp uses the frame space of external adc channels. It has to be trimmed down from 64 to
        // 48 bits. TODO: Verify the precision when lsr to get the 48 MSB of the timestamp
        *(uint64_t *)(adc_external_res) = (uint64_t)(esp_timer_get_time() & 0xFFFFFFFFFFFF);
    }
#endif
}

static inline void IRAM_ATTR write_frame(uint8_t *frame, const uint16_t *adc_internal_res, const uint32_t *adc_external_res,
                                         const uint8_t io_state)
{
    if (scientisst_device_settings.api_config.api_mode == API_MODE_SCIENTISST)
    {
        write_frame_Scientisst(frame, adc_internal_res, adc_external_res, io_state);
    }
    else if (scientisst_device_settings.api_config.api_mode == API_MODE_JSON)
    {
        write_frame_JSON(frame, adc_internal_res, adc_external_res, io_state);
    }
    else
    {
        write_frame_Bitalino(frame, adc_internal_res, io_state);
    }
}

/**
 * \brief acquire adc1 channels
 *
 * Acquires and stores into frame the channel acquisitions of adc1. This is used
 * for ScientISST mode and API (Current).
 *
 * \param frame pointer of frame position in memory to store the adc1 channels.
 */
static void IRAM_ATTR write_frame_Scientisst(uint8_t *frame, const uint16_t *adc_internal_res,
                                             const uint32_t *adc_external_res, const uint8_t io_state)
{
    uint8_t crc = 0;
    uint8_t frame_next_wr = 0;
    uint8_t wr_mid_byte_flag = 0;

    for (int i = 0; i < scientisst_device_settings.num_extern_active_chs; ++i)
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
    *(uint16_t *)(frame + scientisst_buffers.packet_size - 2) = (uint16_t)(crc_seq << 4);

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
 * \brief acquire adc1 channels
 *
 * Acquires and stores into frame the channel acquisitions of adc1. This is used
 * for Bitalino mode and API (Legacy).
 *
 * \param frame pointer of frame position in memory to store the adc1 channels
 */
static void write_frame_Bitalino(uint8_t *frame, const uint16_t *adc_internal_res, const uint8_t io_state)
{
    uint16_t adc_res[6] = {0, 0, 0, 0, 0, 0};
    uint8_t crc = 0;

    // Bitalino API only supports 10 bit resolution for the internal adc, convert it with lsr
    for (int i = 0; i < scientisst_device_settings.num_intern_active_chs; ++i)
    {
        adc_res[i] = adc_internal_res[i] >> 2;
        DEBUG_PRINT_I("write_frame_Bitalino", "(adc_res)A%d=%d", active_internal_chs[i], adc_res[i]);
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
 * \brief acquire adc1 channels
 *
 * Acquires and stores into frame the channel acquisitions of adc1. This is used
 * for JSON mode.
 *
 * \param frame pointer of frame position in memory to store the adc1 channels
 */
static void write_frame_JSON(uint8_t *frame, const uint16_t *adc_internal_res, const uint32_t *adc_external_res,
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
