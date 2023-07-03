/** \file com.c
    \brief This file contains the functions that handle the communication with
   the client

    This file provides the functions that handle the communication with the
   client.
*/

#include "com.h"

#include "adc.h"
#include "bt.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio.h"
#include "macros.h"
#include "scientisst.h"
#include "sd_card.h"
#include "spi.h"
#include "timer.h"

/**
 * \brief Processes the command received from the client
 *
 * This function processes the command received from the client. It works in any
 * mode and with all API's.
 *
 * \param buff The buffer received
 * \param len The length of the buffer
 */
void processRcv(uint8_t* buff, int len) {
    uint8_t cmd = buff[0] & 0b00000011;

    // Live mode with 0 channels selected
    if ((api_config.api_mode == API_MODE_BITALINO && buff[0] == 1) ||
        (api_config.api_mode != API_MODE_BITALINO && buff[0] == 1 &&
         buff[1] == 0)) {
        return;
    }

    // Check trigger command - it's regardeless of the current mode
    if ((buff[0] & 0b10110011) ==
        0b10110011) {  // trigger command - Set output GPIO levels
        triggerGpio(buff);
    } else if (buff[0] ==
               0b10100011) {  // trigger command - Set output DAC level
        triggerDAC(buff);
    }
    if (op_mode == OP_MODE_LIVE) {
        if (!buff[0]) {
            stopAcquisition();
        }
        // If in idle mode
    } else {
        // Set live mode
        if (cmd == 0b01 || cmd == 0b10) {
            startAcquisition(buff, cmd);

            // Configuration command
        } else if (cmd == 0b11) {
            // Set sample rate
            if ((buff[0] & 0b00111100) == 0) {
                setSampleRate(buff);

                // Send device status
            } else if (((buff[0] >> 2) & 0b000011) == 0b10) {
                sendStatusPacket();

                // Send firmware version string
            } else if (((buff[0] >> 2) & 0b000011) == 0b01) {
                DEBUG_PRINT_W("processRcv", "sendFirmwareVersion");
                sendFirmwareVersionPacket();

                // Change API mode
            } else if (buff[0] & 0b00110000) {
                changeAPI((buff[0] & 0b00110000) >> 4);
            }

            // Set battery threshold
        } else if (!cmd) {
            // TODO: Why is this empty??
        }
    }
}

/**
 * \brief Set output GPIO levels
 *
 * This function sets the output GPIO levels according to the received command.
 * //TODO: Confirm this
 */
void triggerGpio(uint8_t* buff) {
    uint8_t o1_lvl = (buff[0] & 0b00001000) >> 3;
    uint8_t o2_lvl = (buff[0] & 0b00000100) >> 2;

    gpio_set_level(O0_IO, o1_lvl);
    gpio_out_state[0] = o1_lvl;
    gpio_set_level(O1_IO, o2_lvl);
    gpio_out_state[1] = o2_lvl;

    DEBUG_PRINT_W("triggerGpio", "O1 = %d, O2 = %d", o1_lvl, o2_lvl);
}

/**
 * \brief Set output DAC level
 *
 * This function sets the output DAC level according to the received command.
 * //TODO: Confirm this
 */
void triggerDAC(uint8_t* buff) {
    dac_output_voltage(DAC_CHANNEL_1, buff[1]);

    DEBUG_PRINT_I("triggerDAC", "DAC output to %d", buff[1]);
}

/**
 * \brief Changes the API mode
 *
 * This function changes the API mode and updates the functions that are used to
 * acquire data and select channels. Currently, the API modes are: Bitalino
 * (Legacy), Scientisst and JSON.
 */
void changeAPI(uint8_t mode) {
    if (mode == API_MODE_BITALINO) {
        api_config.api_mode = API_MODE_BITALINO;
        api_config.aquire_func = &acquireAdc1Channels;
        api_config.select_ch_mask_func = &selectChsFromMask;
    } else if (mode == API_MODE_SCIENTISST) {
        api_config.api_mode = API_MODE_SCIENTISST;
        api_config.aquire_func = &acquireChannelsScientisst;
        api_config.select_ch_mask_func = &selectChsFromMaskScientisstJson;
    } else if (mode == API_MODE_JSON) {
        api_config.api_mode = API_MODE_JSON;
        api_config.aquire_func = &acquireChannelsJson;
        api_config.select_ch_mask_func = &selectChsFromMaskScientisstJson;
    }

    DEBUG_PRINT_W("changeAPI", "API changed to %d", mode);
}

/**
 * \brief Returns the packet size
 *
 * This function returns the packet size in function of the current API mode and
 * the number of active channels.
 *
 * \return The packet size
 */
uint8_t getPacketSize(void) {
    uint8_t _packet_size = 0;

    if (api_config.api_mode == API_MODE_BITALINO) {
        const uint8_t packet_size_num_chs[DEFAULT_ADC_CHANNELS + 1] = {
            0,
            3,
            4,
            6,
            7,
            7,
            MAX_LIVE_MODE_PACKET_SIZE};  // Table that has the packet size in
                                         // function of the number of channels
        _packet_size = packet_size_num_chs[num_intern_active_chs];
    } else if (api_config.api_mode == API_MODE_SCIENTISST) {
        // Add 24bit channel's contributuion to packet size
        _packet_size += 3 * num_extern_active_chs;

        // Add 12bit channel's contributuion to packet size
        if (!(num_intern_active_chs % 2)) {  // If it's an even number
            _packet_size += ((num_intern_active_chs * 12) / 8);
        } else {
            _packet_size += (((num_intern_active_chs * 12) - 4) /
                             8);  //-4 because 4 bits can go in the I/0 byte
        }
        _packet_size += 3;  // for the I/Os and seq+crc bytes
    } else if (api_config.api_mode == API_MODE_JSON) {
        const char* json_str = cJSON_Print(json);
        _packet_size = strlen(json_str) + 1;
        free((void*)json_str);
    }
    DEBUG_PRINT_I("getPacketSize", "Packet size is %d bytes", _packet_size);

    return _packet_size;
}

/**
 * \brief Selects the active channels
 *
 * This function selects the active channels according to the received command
 * for the ScientISST and JSON API modes.
 *
 * \param buff The received command
 */
void selectChsFromMaskScientisstJson(uint8_t* buff) {
    uint8_t i;
    char aux_str[10];
    char value_str[10];
    int channel_number = DEFAULT_ADC_CHANNELS + 2;

    // Reset previous active chs
    num_intern_active_chs = 0;
    num_extern_active_chs = 0;

    // Select the channels that are activated (with corresponding bit equal to
    // 1)
    for (i = 1 << (DEFAULT_ADC_CHANNELS + 2 - 1); i > 0; i >>= 1) {
        if (buff[1] & i) {
            // Store the activated channels
            if (i == 1 << (DEFAULT_ADC_CHANNELS + 2 - 1) ||
                i == 1 << (DEFAULT_ADC_CHANNELS + 2 - 2)) {
                active_ext_chs[num_extern_active_chs] = channel_number - 1;
                num_extern_active_chs++;
            } else {
                active_internal_chs[num_intern_active_chs] = channel_number - 1;
                num_intern_active_chs++;
            }

            DEBUG_PRINT_W("selectChsFromMask", "Channel A%d added",
                          channel_number - 1);
        }
        channel_number--;
    }

    if (api_config.api_mode == API_MODE_JSON) {
        if (json != NULL) {
            cJSON_Delete(json);
            json = NULL;
        }
        json = cJSON_CreateObject();

        sprintf(value_str, "%04d", 4095);
        for (int j = num_intern_active_chs - 1; j >= 0; j--) {
            sprintf(aux_str, "AI%d", active_internal_chs[j] + 1);
            cJSON_AddStringToObject(json, aux_str, value_str);
        }

        sprintf(value_str, "%08d", 16777215);
        for (int j = num_extern_active_chs - 1; j >= 0; j--) {
            sprintf(aux_str, "AX%d", active_ext_chs[j] + 1 - 6);
            cJSON_AddStringToObject(json, aux_str, value_str);
        }

        // Add IO state json objects
        cJSON_AddStringToObject(json, "I1", "0");
        cJSON_AddStringToObject(json, "I2", "0");
        cJSON_AddStringToObject(json, "O1", "0");
        cJSON_AddStringToObject(json, "O2", "0");
    }
    packet_size = getPacketSize();
}

/**
 * \brief Sets the sample rate
 *
 * This function sets the sample rate according to the received command.
 */
void setSampleRate(uint8_t* buff) {
    uint32_t aux = 1;
    uint8_t i;

    // API mode bitalino only needs the 2 bits for the sample rate.
    if (api_config.api_mode == API_MODE_BITALINO) {
        for (i = 0; i < (buff[0] >> 6); i++) {
            aux *= 10;
        }

        // The other APIs need more bits for the sample rate
    } else {
        aux = (*(uint16_t*)(buff + 1) & 0xFFFF);
    }

    sample_rate = aux;
    // sample_rate = (*(uint16_t*)(buff+1) & 0xFFFF);

    DEBUG_PRINT_W("processRcv", "Sampling rate recieved: %dHz", sample_rate);
}

/**
 * \brief Selects the active channels
 *
 * This function selects the active channels according to the received command.
 *
 * \param buff The received command
 */
void selectChsFromMask(uint8_t* buff) {
    int i;
    int channel_number = DEFAULT_ADC_CHANNELS;

    // Reset previous active chs
    num_intern_active_chs = 0;
    num_extern_active_chs = 0;

    // Select the channels that are activated (with corresponding bit equal to
    // 1)
    for (i = 1 << (DEFAULT_ADC_CHANNELS + NUM_UNUSED_BITS_FOR_CH_MASK - 1);
         i > NUM_UNUSED_BITS_FOR_CH_MASK; i >>= 1) {
        if (buff[0] & i) {
            // Store the activated channels into the respective
            // acq_config.channels array
            active_internal_chs[num_intern_active_chs] = channel_number - 1;
            num_intern_active_chs++;
            DEBUG_PRINT_I("selectChsFromMask", "Channel A%d added",
                          channel_number - 1);
        }
        channel_number--;
    }
    packet_size = getPacketSize();
}

/**
 * \brief Starts the acquisition
 *
 * This function starts the acquisition according to the received command. It
 * starts the ADC and the timer. It also sets the acquisition mode (live or
 * simulated). It also changes the LED state.
 *
 * \param buff The received command
 * \param cmd The command (identifies if it is a live or a simulated
 * acquisition) \warning If changed, change same code in API (warning on
 * identified on code)
 */
void startAcquisition(uint8_t* buff, uint8_t cmd) {
    // Get channels from mask
    api_config.select_ch_mask_func(buff);

    // Clear send buffs, because of potential previous live mode
    bt_curr_buff = 0;
    acq_curr_buff = 0;
    // Clean send buff, because of send status and send firmware string
    send_busy = 0;

    crc_seq = 0;

    // Clean send buffers, to be sure
    for (uint8_t i = 0; i < NUM_BUFFERS; i++) {
        memset(snd_buff[i], 0, send_buff_len);
        snd_buff_idx[i] = 0;
        bt_buffs_to_send[i] = 0;
    }

    // WARNING: if changed, change same code in API
    if (sample_rate > 100) {
        send_threshold = !(send_buff_len % packet_size)
                             ? send_buff_len - packet_size
                             : send_buff_len - (send_buff_len % packet_size);
    } else {
        send_threshold = packet_size;
    }

    if (cmd == 0b10) {
        sim_flag = 1;
        DEBUG_PRINT_W("processRcv", "Simulation mode");
    } else {
        sim_flag = 0;
    }

    // Start external
#if _ADC_EXT_ != NO_ADC_EXT
    if (num_extern_active_chs) {
        uint8_t channel_mask = 0;
        for (int i = 0; i < num_extern_active_chs; i++) {
            channel_mask |= 0b1 << (active_ext_chs[i] - 6);
        }
        mcpSetupRoutine(channel_mask);
        adcExtStart();
    }
#endif

    // Init timer for adc task top start
    timerStart(TIMER_GROUP_USED, TIMER_IDX_USED, sample_rate);

    // Set led state to blink at live mode frequency
    ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_LIVE_PWM_FREQ);

    // Set live mode duty cycle for state led
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R, LEDC_LIVE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R);
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G, LEDC_LIVE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G);
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B, LEDC_LIVE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B);

    DEBUG_PRINT_W("startAcquisition", "Acquisition started");
    op_mode = OP_MODE_LIVE;
}

/**
 * \brief Stops the acquisition
 *
 * This function stops the acquisition. It stops the ADC and the timer.
 * It also changes the LED state. It cleans all buffers and active channels.
 */
void stopAcquisition(void) {
    timerPause(TIMER_GROUP_USED, TIMER_IDX_USED);

    ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_IDLE_PWM_FREQ);

    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R, LEDC_IDLE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_R);
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G, LEDC_IDLE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_G);
    ledc_set_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B, LEDC_IDLE_DUTY);
    ledc_update_duty(LEDC_SPEED_MODE_USED, LEDC_CHANNEL_B);

    // Stop external
#if _ADC_EXT_ != NO_ADC_EXT
    if (num_extern_active_chs) {
        adcExtStop();
    }
#endif

    op_mode = OP_MODE_IDLE;

    vTaskDelay(50 / portTICK_PERIOD_MS);

    // Reset simulation's signal iterator
    sin_i = 0;

    crc_seq = 0;

    // Clean send buffers
    for (uint8_t i = 0; i < NUM_BUFFERS; i++) {
        memset(snd_buff[i], 0, send_buff_len);
        snd_buff_idx[i] = 0;
        bt_buffs_to_send[i] = 0;
    }

    bt_curr_buff = 0;
    acq_curr_buff = 0;
    send_busy = 0;

    // Reset previous active chs
    num_intern_active_chs = 0;
    num_extern_active_chs = 0;

    DEBUG_PRINT_W("startAcquisition", "Acquisition stopped");
}

/**
 * \brief Sends the status packet
 *
 * This function sends the status packet. It is called when the client
 * requests the status packet.
 */
void sendStatusPacket(void) {
    uint8_t crc = 0;
    uint8_t i;
    uint16_t true_send_threshold;

    memset(snd_buff[NUM_BUFFERS - 1], 0, snd_buff_idx[NUM_BUFFERS - 1]);
    snd_buff_idx[NUM_BUFFERS - 1] = 0;
    bt_buffs_to_send[NUM_BUFFERS - 1] = 1;
    true_send_threshold = send_threshold;
    send_threshold = 0;

    //-------------------Bytes 0 -
    // 11-------------------------------------------------
    /**(uint16_t*)(snd_buff[bt_curr_buff]) = i2c_sensor_values.oxygen;
     *(uint16_t*)(snd_buff[bt_curr_buff]+2) = i2c_sensor_values.heart_rate;
     *(uint16_t*)(snd_buff[bt_curr_buff]+4) =
     *(uint16_t)i2c_sensor_values.confidence;
     *(uint16_t*)(snd_buff[bt_curr_buff]+6) =
     *(uint16_t)i2c_sensor_values.status; (uint16_t*)(snd_buff[bt_curr_buff]+8)
     *= i2c_sensor_values.temp_obj_int; (uint16_t*)(snd_buff[bt_curr_buff]+10) =
     *i2c_sensor_values.temp_amb_int;*/

    //-----------------------Calc CRC
    //-------------------------------------------------

    // calculate CRC (except last byte (seq+CRC) )
    for (i = 0; i < STATUS_PACKET_SIZE - 1; i++) {
        // calculate CRC nibble by nibble
        crc = crc_table[crc] ^ (snd_buff[NUM_BUFFERS - 1][i] >> 4);
        crc = crc_table[crc] ^ (snd_buff[NUM_BUFFERS - 1][i] & 0x0F);
    }

    // calculate CRC for last byte (I1|I2|O1|O2|CRC)
    crc = crc_table[crc] ^ (crc_seq & 0x0F);
    crc = (0) |
          crc_table[crc];  // TODO: Onde está 0, meter os valores de I1|I2|O1|O2

    // store CRC and Seq in the last byte of the packet
    snd_buff[NUM_BUFFERS - 1][STATUS_PACKET_SIZE - 1] = crc;

    //----------------------------Store packet
    // size-------------------------------------
    snd_buff_idx[NUM_BUFFERS - 1] += STATUS_PACKET_SIZE;

    bt_curr_buff = NUM_BUFFERS - 1;
    // send new data
    sendData();
    bt_curr_buff = 0;
    send_threshold = true_send_threshold;
}

/**
 * \brief Sends the firmware version packet
 *
 * This function sends the firmware version packet. It is called when the client
 * requests the firmware version packet. It fills one position of the send
 * buffer with the information and marks it as ready to be sent. It also changes
 * the lowers the send threshold to 0, so that the data is sent as soon as
 * possible and then restores the send threshold to its previous value.
 */
void sendFirmwareVersionPacket(void) {
    uint16_t true_send_threshold;

    memset(snd_buff[NUM_BUFFERS - 1], 0, snd_buff_idx[NUM_BUFFERS - 1]);
    snd_buff_idx[NUM_BUFFERS - 1] = 0;
    bt_buffs_to_send[NUM_BUFFERS - 1] = 1;
    true_send_threshold = send_threshold;
    send_threshold = 0;

    if (api_config.api_mode != API_MODE_BITALINO) {
        memcpy(snd_buff[NUM_BUFFERS - 1], FIRMWARE_VERSION,
               strlen(FIRMWARE_VERSION) + 1);
        snd_buff_idx[NUM_BUFFERS - 1] += strlen(FIRMWARE_VERSION) + 1;

        // Send ADC1 configurations for raw2voltage precision conversions
        memcpy(snd_buff[NUM_BUFFERS - 1] + snd_buff_idx[NUM_BUFFERS - 1],
               &adc1_chars,
               6 * sizeof(uint32_t));  // We don't want to send the 2 last
                                       // pointers of adc1_chars struct
        snd_buff_idx[NUM_BUFFERS - 1] += 6 * sizeof(uint32_t);
    } else {
        memcpy(snd_buff[NUM_BUFFERS - 1], FIRMWARE_BITALINO_VERSION,
               strlen(FIRMWARE_BITALINO_VERSION));
        snd_buff_idx[NUM_BUFFERS - 1] += strlen(FIRMWARE_BITALINO_VERSION);
    }

    bt_curr_buff = NUM_BUFFERS - 1;

    sendData();

    bt_curr_buff = 0;
    send_threshold = true_send_threshold;

    DEBUG_PRINT_I("sendFirmwareVersionPacket",
                  "Sent firmware version and adc chars");
}
