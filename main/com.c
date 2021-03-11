#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/timer.h"
#include "com.h"
#include "adc.h"
#include "macros.h"
#include "timer.h"
#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "bt.h"

//Processes the buffer recieved in the bluetooth event
void processRcv(uint8_t* buff, int buff_size){
    uint8_t cmd = buff[0] & 0b00000011;

    //Live mode with 0 channels selected
    if(buff[0] == 1){
        return;
    }

    if(live_mode){
        if(!buff[0]){
            stopAcquisition();
        }
    }else{                                                      //If in idle mode
        if(cmd == 0b01){                                        //Set live mode
            //Get channels from mask
            api_config.select_ch_mask_func(buff);
            //Clear send buffs, because of potential previous live mode
            bt_curr_buff = 0;
            acq_curr_buff = 0;
            //Clean send buff, because of send status and send firmware stringC
            bt_write_busy = 0;
            memset(snd_buff[bt_curr_buff], 0, snd_buff_idx[bt_curr_buff]);
            snd_buff_idx[bt_curr_buff] = 0;
            bt_buffs_to_send[bt_curr_buff] = 0;
            //Init timer for adc task top start
            timerStart(TIMER_GROUP_USED, TIMER_IDX_USED, sample_rate);
            //Set led state to blink at live mode frequency
            ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_LIVE_PWM_FREQ);
            //Start external
            adsStart();
            
            live_mode = 1;
        }else if(cmd == 0b11){                                  //Configuration command
            if((buff[0] & 0b00111111) == 0){                      //Set sample rate
                setSampleRate(buff);
            }else if(((buff[0] >> 2) & 0b00000011) == 0b10){    //Send device status
                sendStatusPacket();
            }else if(((buff[0] >> 2) & 0b00000011) == 0b01){    //Send firmware version string
                sendFirmwareVersionPacket();
            }else if((buff[0] & 0b00110000) >> 4){              //Change API mode
                changeAPI((buff[0] & 0b00110000) >> 4);
            }

        }else if(!cmd){                                         //Set battery threshold

        }
    }
}

void changeAPI(uint8_t mode){

    if(mode == API_MODE_BITALINO){
        api_config.api_mode = API_MODE_BITALINO;
        api_config.aquire_func = &acquireAdc1Channels;
        api_config.select_ch_mask_func = &selectChsFromMask;
    }else if(mode == API_MODE_EXTENDED){
        api_config.api_mode = API_MODE_EXTENDED;
        api_config.aquire_func = &acquireChannelsExtended;
        api_config.select_ch_mask_func = &selectChsFromMaskExtendedJson;
    }else if(mode == API_MODE_JSON){
        api_config.api_mode = API_MODE_JSON;
        api_config.aquire_func = &acquireChannelsJson;
        api_config.select_ch_mask_func = &selectChsFromMaskExtendedJson;
    }
       
}

uint8_t getPacketSize(){
    uint8_t _packet_size = 0;

    if(api_config.api_mode == API_MODE_BITALINO){
        _packet_size = packet_size_num_chs[num_intern_active_chs];
        _packet_size += 2;  //for the I/Os and seq+crc bytes

    }else if(api_config.api_mode == API_MODE_EXTENDED){
        //Add 24bit channel's contributuion to pakcet size
        _packet_size += 3*num_extern_active_chs;

        //Add 12bit channel's contributuion to packet size 
        if(!(num_intern_active_chs % 2)){                    //If it's an even number
            _packet_size += ((num_intern_active_chs*12)/8);
        }else{
            _packet_size += (((num_intern_active_chs*12)-4)/8); //-4 because 4 bits can go in the I/0 byte 
        }
        _packet_size += 2;  //for the I/Os and seq+crc bytes

    }else if(api_config.api_mode == API_MODE_JSON){
        const char *json_str = cJSON_Print(json);
        _packet_size = strlen(json_str) + 1;
        free((void *)json_str);
    }

    
    return _packet_size;
}

void selectChsFromMaskExtendedJson(uint8_t* buff){
    uint8_t i;
    char aux_str[10];
    char value_str[10];
    int channel_number = DEFAULT_ADC_CHANNELS+2;

    //Reset previous active chs
    num_intern_active_chs = 0;
    num_extern_active_chs = 0;

    //Select the channels that are activated (with corresponding bit equal to 1)
    for(i = 1 << (DEFAULT_ADC_CHANNELS+2-1); i > 0; i >>= 1){
        if(buff[1] & i){
            //Store the activated channels
            if(i == 1 << (DEFAULT_ADC_CHANNELS+2-1) || i == 1 << (DEFAULT_ADC_CHANNELS+2-2)){
                active_ext_chs[num_extern_active_chs] = channel_number-1;
                num_extern_active_chs++;
            }else{
                active_internal_chs[num_intern_active_chs] = channel_number-1;
                num_intern_active_chs++;
            }
            
            DEBUG_PRINT_I("selectChsFromMask", "Channel A%d added", channel_number-1);
        }
        channel_number--;
    }

    if(api_config.api_mode == API_MODE_JSON){
        if(json != NULL){
            cJSON_Delete(json);
            json = NULL;
        }
        json = cJSON_CreateObject();

        sprintf(value_str, "%04d", 4095);         
        for(int j = num_intern_active_chs-1; j >= 0; j--){
            sprintf(aux_str, "AI%d", active_internal_chs[j]+1);
            cJSON_AddStringToObject(json, aux_str, value_str);
        }

        sprintf(value_str, "%08d", 16777215);
        for(int j = num_extern_active_chs-1; j >= 0; j--){
            sprintf(aux_str, "AX%d", active_ext_chs[j]+1-6);
            cJSON_AddStringToObject(json, aux_str, value_str);
        }
    }
    packet_size = getPacketSize();
}

void setSampleRate(uint8_t* buff){
    uint32_t aux = 1;
    uint16_t coded_3bit_sr;
    uint8_t i;

    //API mode bitalino only needs the 2 bits for the sample rate.
    if(api_config.api_mode == API_MODE_BITALINO){
        for(i = 0; i < (buff[0] >> 6); i++){
            aux *= 10;
        }
    
    //The other APIs need 3 bits for the sample rate (hence the extra byte in the set sample rate config)
    }else{
        coded_3bit_sr = *(uint16_t*)(buff) >> 6;
        aux = 2000 + (coded_3bit_sr & 0b011)*2000;
    }

    sample_rate = aux;
    
    DEBUG_PRINT_W("processRcv", "Sampling rate recieved: %dHz", sample_rate);
    if(sample_rate >= 100){
        send_threshold = MAX_BUFFER_SIZE-10;
    }else{
        send_threshold = DEFAULT_SEND_THRESHOLD;
    }
}

void selectChsFromMask(uint8_t* buff){
    int i;
    int channel_number = DEFAULT_ADC_CHANNELS;

    //Reset previous active chs
    num_intern_active_chs = 0;

    //Select the channels that are activated (with corresponding bit equal to 1)
    for(i = 1 << (DEFAULT_ADC_CHANNELS+NUM_UNUSED_BITS_FOR_CH_MASK-1); i > NUM_UNUSED_BITS_FOR_CH_MASK; i >>= 1){
        if(buff[0] & i){
            //Store the activated channels into the respective acq_config.channels array
            active_internal_chs[num_intern_active_chs] = channel_number-1;
            num_intern_active_chs++;
            DEBUG_PRINT_I("selectChsFromMask", "Channel A%d added", channel_number-1);
        }
        channel_number--;
    }
    packet_size = getPacketSize();
}

void stopAcquisition(void){
    timerPause(TIMER_GROUP_USED, TIMER_IDX_USED);
    ledc_set_freq(LEDC_SPEED_MODE_USED, LEDC_LS_TIMER, LEDC_IDLE_PWM_FREQ);
    live_mode = 0;
    crc_seq = 0;
    adsStop();
}

void sendStatusPacket(){
    uint8_t crc = 0;
    uint8_t i;
    uint16_t true_send_threshold;

    bt_curr_buff = 0;
    memset(snd_buff[bt_curr_buff], 0, snd_buff_idx[bt_curr_buff]);
    snd_buff_idx[bt_curr_buff] = 0;
    bt_buffs_to_send[bt_curr_buff] = 1;
    true_send_threshold = send_threshold;
    send_threshold = 0;
        
    // -------------------Bytes 0 - 11-------------------------------------------------
    *(uint16_t*)(snd_buff[bt_curr_buff]) = i2c_sensor_values.oxygen;
    *(uint16_t*)(snd_buff[bt_curr_buff]+2) = i2c_sensor_values.heart_rate;
    *(uint16_t*)(snd_buff[bt_curr_buff]+4) = (uint16_t)i2c_sensor_values.confidence;
    *(uint16_t*)(snd_buff[bt_curr_buff]+6) = (uint16_t)i2c_sensor_values.status;
    *(uint16_t*)(snd_buff[bt_curr_buff]+8) = i2c_sensor_values.temp_obj_int;
    *(uint16_t*)(snd_buff[bt_curr_buff]+10) = i2c_sensor_values.temp_amb_int;

    //-----------------------Calc CRC -------------------------------------------------

    //calculate CRC (except last byte (seq+CRC) )
    for(i = 0; i < STATUS_PACKET_SIZE-1; i++){
        // calculate CRC nibble by nibble
        crc = crc_table[crc] ^ (snd_buff[bt_curr_buff][i] >> 4);
        crc = crc_table[crc] ^ (snd_buff[bt_curr_buff][i] & 0x0F);
    }

    //calculate CRC for last byte (I1|I2|O1|O2|CRC)
    crc = crc_table[crc] ^ (crc_seq & 0x0F);
    crc = (0) | crc_table[crc];                 //TODO: Onde está 0, meter os valores de I1|I2|O1|O2

    //store CRC and Seq in the last byte of the packet
    snd_buff[bt_curr_buff][STATUS_PACKET_SIZE-1] = crc;

    //----------------------------Store packet size-------------------------------------
    snd_buff_idx[bt_curr_buff] += STATUS_PACKET_SIZE;

    //send new data
    sendData();
    send_threshold = true_send_threshold;   
}

void sendFirmwareVersionPacket(){
    uint16_t true_send_threshold;

    bt_curr_buff = 0;
    memset(snd_buff[bt_curr_buff], 0, snd_buff_idx[bt_curr_buff]);
    snd_buff_idx[bt_curr_buff] = 0;
    bt_buffs_to_send[bt_curr_buff] = 1;
    true_send_threshold = send_threshold;
    send_threshold = 0;

    memcpy(snd_buff[bt_curr_buff], FIRMWARE_VERSION_STR, strlen(FIRMWARE_VERSION_STR)+1);

    //----------------------------Store packet size-------------------------------------
    snd_buff_idx[bt_curr_buff] += strlen(FIRMWARE_VERSION_STR)+1;

    sendData();   
    send_threshold = true_send_threshold;
}