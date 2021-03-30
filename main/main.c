#include "main.h"
#include "bt.h"
#include "macros.h"
#include "timer.h"
#include "gpio.h"
#include "spi.h"

#define BT_SEND_PRIORITY_TASK 10        //MAX priority in ESP32 is 25
#define ACQ_PRIORITY_TASK 10
#define I2C_ACQ_PRIORITY_TASK 1

void IRAM_ATTR btTask();
void IRAM_ATTR acqAdc1Task();
void acqI2cTask();

TaskHandle_t bt_task;
TaskHandle_t acquiring_1_task;
TaskHandle_t acquiring_i2c_task;

//SemaphoreHandle_t snd_buff_mutex[NUM_BUFFERS];
SemaphoreHandle_t bt_buffs_to_send_mutex;

//BT
uint32_t bt_client = 0;
char bt_device_name[17] = BT_DEFAULT_DEVICE_NAME;

//COM
uint8_t snd_buff[NUM_BUFFERS][MAX_BUFFER_SIZE];         //Data structure to hold the data to be sent through bluetooth        
uint16_t snd_buff_idx[NUM_BUFFERS] = {0, 0, 0, 0};      //It contains, for each buffer, the index of the first free element in the respective buffer 
uint8_t bt_buffs_to_send[NUM_BUFFERS] = {0, 0, 0, 0};               //If element 0 is set to 1, bt task has to send snd_buff[0]
uint8_t bt_curr_buff = 0;
uint8_t acq_curr_buff = 0;                              //Index of the buffer that adc task is currently using

uint8_t packet_size = 0;                //Current packet size (dependent on number of channels used)
uint8_t bt_write_busy = 0;
uint16_t send_threshold = DEFAULT_SEND_THRESHOLD;

DRAM_ATTR const uint8_t crc_table[16] = {0, 3, 6, 5, 12, 15, 10, 9, 11, 8, 13, 14, 7, 4, 1, 2};
uint8_t crc_seq = 0;
const uint8_t packet_size_num_chs[DEFAULT_ADC_CHANNELS+1] = {0, 3, 4, 6, 7, 7, MAX_LIVE_MODE_PACKET_SIZE};       //Table that has the packet size in function of the number of channels 

Api_Config api_config = {.api_mode = API_MODE_BITALINO, .aquire_func = &acquireAdc1Channels, .select_ch_mask_func = &selectChsFromMask};
cJSON *json = NULL;

//ADC
DRAM_ATTR const uint8_t analog_channels[DEFAULT_ADC_CHANNELS] = {A0_ADC_CH, A1_ADC_CH, A2_ADC_CH, A3_ADC_CH, A4_ADC_CH, A5_ADC_CH};
uint8_t active_internal_chs[DEFAULT_ADC_CHANNELS] = {0, 0, 0, 0, 0, 0};      //Active internal channels | If all channels are active: = {5, 4, 3, 2, 1, 0}
uint8_t active_ext_chs[EXT_ADC_CHANNELS] = {0, 0};                  //Active external channels | If all channels are active: = {7, 6}
uint8_t num_intern_active_chs = 0;
uint8_t num_extern_active_chs = 0;
uint8_t live_mode = 0;                                  //Flag that indicastes if live mode (acquiring) is on    
uint32_t sample_rate = DEFAULT_SAMPLE_RATE;    
esp_adc_cal_characteristics_t adc1_chars;    
uint8_t gpio_out_state[2] = {0, 0};                 //Output of 01 & O2 (O0 & O1)

DRAM_ATTR const uint8_t sin10Hz[100] =  {31, 33, 35, 37, 39, 41, 42, 44, 46, 48, 49, 51, 52, 54, 55, 56, 57, 58, 59, 60, 60, 61, 61, 62, 62, 62, 62, 62, 61, 61, 60, 60, 59, 58, 57, 56, 55, 54, 52, 51, 49, 48, 46, 44, 42, 41, 39, 37, 35, 33, 31, 29, 27, 25, 23, 21, 20, 18, 16, 14, 13, 11, 10, 8, 7, 6, 5, 4, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 5, 6, 7, 8, 10, 11, 13, 14, 16, 18, 20, 21, 23, 25, 27, 29};
uint8_t sim_flag = 0;
uint8_t sin_i = 0;

//I2C
I2c_Sensor_State i2c_sensor_values;

//SPI
spi_device_handle_t ads_spi_handler;

//TODO: DEBUG ONLY VARIABLES!
struct timeval begin, end;

void app_main(void){ 

    // Create a mutex type semaphore
    if((bt_buffs_to_send_mutex = xSemaphoreCreateMutex()) == NULL){
        DEBUG_PRINT_E("xSemaphoreCreateMutex", "Mutex creation failed");
        abort();
    }
    

    //Inicialize send buffers
    for(uint8_t i = 0; i < NUM_BUFFERS; i++){
        memset(snd_buff[i], 0, MAX_BUFFER_SIZE);
    }
    
    xTaskCreatePinnedToCore(&btTask, "btTask", DEFAULT_TASK_STACK_SIZE, NULL, BT_SEND_PRIORITY_TASK, &bt_task, 0);

    //Create the 1st task that will acquire data from adc. This task will be responsible for acquiring the data from adc1
    xTaskCreatePinnedToCore(&acqAdc1Task, "acqAdc1Task", DEFAULT_TASK_STACK_SIZE, NULL, ACQ_PRIORITY_TASK, &acquiring_1_task, 1);

    //Create the 1st task that will acquire data from i2c. This task will be responsible for acquiring the data from i2c
    //xTaskCreatePinnedToCore(&acqI2cTask, "acqI2cTask", DEFAULT_TASK_STACK_SIZE, NULL, I2C_ACQ_PRIORITY_TASK, &acquiring_i2c_task, 1);

    vTaskDelete(NULL);
}

//THis task can be removed and acqAdc1Task can do the sendData() when !bt_write_busy. But, atm acqAdc1Task is the bottleneck
void IRAM_ATTR btTask(){
    initBt();

    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            sendData();
    }

}
 
//Task that adc reads using adc1, it's also the main task of CPU1 (APP CPU)
void IRAM_ATTR acqAdc1Task(){
    uint8_t* rx_data_ads = NULL;
    spi_transaction_t ads_trans;
    

    //Init Timer 0_1 (timer 1 from group 0) and register it's interupt handler
    timerGrpInit(TIMER_GROUP_USED, TIMER_IDX_USED, timerGrp0Isr);

    //Config all possible adc channels
    initAdc(ADC_RESOLUTION);
    gpioInit();

    //24 status bits + 24 bits x 2 channels = 9bytes required. But DMA buffer needs to be 32bit aligned
    rx_data_ads = heap_caps_malloc(3*sizeof(uint32_t), MALLOC_CAP_DMA);  
    memset(rx_data_ads, 0, 9);
    adsInit();
    adsSetupRoutine();
    adsSetSamplingRate(250);
    adsConfigureChannels(1);
    memset(&ads_trans, 0, sizeof(ads_trans));   //zero out the transaction
    ads_trans.length = 72;                   //send nothing
    ads_trans.rxlength = 72;                //24 status bits + 24 bits x 2 channels
    ads_trans.tx_buffer = NULL;             //skip write phase
    ads_trans.rx_buffer = rx_data_ads;
    
    while(1){
        if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY)){

            /*if(api_config.api_mode != API_MODE_BITALINO){
                spi_device_queue_trans(ads_spi_handler, &ads_trans, portMAX_DELAY);
            }*/
            api_config.aquire_func(snd_buff[acq_curr_buff]+snd_buff_idx[acq_curr_buff]);

            //spi_device_get_trans_result(ads_spi_handler, &ads_rtrans, portMAX_DELAY);
            //uint8_t* recv_ads = ads_rtrans->rx_buffer;
            //int swag = (recv_ads[3] << 16) | ((uint8_t)recv_ads[4] << 8) | (recv_ads[5]);
            //printf("Raw:%d, Voltage:%f\n", swag, (double)swag*(3.3/16777216));

            snd_buff_idx[acq_curr_buff] += packet_size;

            //Check if acq_curr_buff is above send_threshold and consequently send acq_curr_buff. If we send acq_curr_buff, we need to update it
            if(snd_buff_idx[acq_curr_buff] >= send_threshold){
                //Tell bt task that it has acq_curr_buff to send (but it will only send after the buffer is filled above the threshold)
                xSemaphoreTake(bt_buffs_to_send_mutex, portMAX_DELAY);
                    bt_buffs_to_send[acq_curr_buff] = 1;
                xSemaphoreGive(bt_buffs_to_send_mutex);

                //If bt task is idle, wake it up
                if(bt_write_busy == 0){
                    xTaskNotifyGive(bt_task);
                }
                //Check if next buffer is full. If this happens, it means all 4 buffers are full and bt task can't handle this sending throughput
                if(snd_buff_idx[acq_curr_buff] + packet_size >= MAX_BUFFER_SIZE && bt_buffs_to_send[(acq_curr_buff+1)%4] == 1){
                    DEBUG_PRINT_W("acqAdc1Task", "Sending buffer is full, cannot acquire");
                    continue;
                }

                //Next buffer is free, change buffer
                acq_curr_buff = (acq_curr_buff+1)%4;
                //Clear this new buffer
                //memset(snd_buff[acq_curr_buff], 0, snd_buff_idx[acq_curr_buff]);
                snd_buff_idx[acq_curr_buff] = 0;
            }
        }else{
            DEBUG_PRINT_W("acqAdc1", "ulTaskNotifyTake timed out!");
        }
    }
}

void acqI2cTask(){
    /*uint16_t heart_rate;
    uint16_t oxygen;
    uint8_t confidence;*/
    uint32_t test;

    i2cMasterInit();
    MAX32664_Init();    
    if(MAX32664_Config()){
        DEBUG_PRINT_E("MAX32664_Config", "Config failed");
    }

    //Init Timer 1_1 (timer 1 from group 1) and register it's interupt handler
    timerGrpInit(TIMER_GRP_ACQ_I2C, TIMER_IDX_ACQ_I2C, timerGrp1Isr);
    timerStart(TIMER_GRP_ACQ_I2C, TIMER_IDX_ACQ_I2C, ACQ_I2C_SAMPLE_RATE);

    while(1){
        if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY)){
            
            gettimeofday(&begin, NULL); 
            while((end.tv_sec - begin.tv_sec) + ((end.tv_usec - begin.tv_usec)/1000000.0) < 0.08){
                gettimeofday(&end, NULL);
                test++;
                vTaskDelay(10/portTICK_RATE_MS);
            }
            //printf("Time elapsed %f s\n", ((end.tv_sec - begin.tv_sec) + ((end.tv_usec - begin.tv_usec)/1000000.0)));

            /*MLX90614_GetTempObj(MLX90614_DEFAULT_ADDRESS, &(i2c_sensor_values.temp_obj), &(i2c_sensor_values.temp_obj_int));
            MLX90614_GetTempAmb(MLX90614_DEFAULT_ADDRESS, &(i2c_sensor_values.temp_amb), &(i2c_sensor_values.temp_amb_int));
            MAX32664_GetBPM(&heart_rate, &oxygen, &confidence, &(i2c_sensor_values.status));
            if(confidence >= CONFIDENCE_THRESHOLD){
                i2c_sensor_values.heart_rate = heart_rate;
                i2c_sensor_values.oxygen = oxygen;
                i2c_sensor_values.confidence = confidence;
            }*/

            //printf("BPM:%d, SpO2:%d, Confidence:%d, Status:%d, Temp Object:%.1f, Temp Ambient:%.1f\n", i2c_sensor_values.heart_rate, i2c_sensor_values.oxygen, i2c_sensor_values.confidence, i2c_sensor_values.status, i2c_sensor_values.temp_obj, i2c_sensor_values.temp_amb);
        }else{
            DEBUG_PRINT_W("acqI2cTask", "ulTaskNotifyTake timed out!");
        }
    }
}