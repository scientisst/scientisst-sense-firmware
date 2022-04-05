#include "main.h"
#include "macros.h"
#include "uart.h"
#include "bt.h"
#include "driver/uart.h"
#include "include/soc/uart_channel.h"

const uart_port_t serial_com_uart_num = UART_NUM_0;

void uart_init(void){
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        //.source_clk = UART_SCLK_APB,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(serial_com_uart_num, &uart_config));

    //Set UART pins
    ESP_ERROR_CHECK(uart_set_pin(serial_com_uart_num,  UART_PIN_NO_CHANGE,  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(serial_com_uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
}

esp_err_t IRAM_ATTR serialSend(uint32_t fd, int len, uint8_t *buff){
    uart_write_bytes(serial_com_uart_num, (const char*)buff, len);

    return ESP_OK;
}