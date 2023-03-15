/** \file uart.c
    \brief UART functions
    
    This file contains the functions for the UART communication. Only has the init function for now.
    //Todo: add details and what undesrtand what this is for
*/
#include "driver/uart.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include <sys/fcntl.h>
#include <sys/errno.h>
#include <sys/unistd.h>
#include <sys/select.h>

#include "scientisst.h"
#include "macros.h"
#include "uart.h"
#include "bt.h"

const uart_port_t serial_com_uart_num = UART_NUM_0;

//Based on https://github.com/espressif/esp-idf/blob/master/examples/peripherals/uart/uart_select/main/uart_select_example_main.c
int serialInit(void) {
    int fd;

    uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 512, 2048, 0, NULL, 0));

    //Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(serial_com_uart_num, &uart_config));

    if ((fd = open("/dev/uart/0", O_RDWR)) == -1) {
        DEBUG_PRINT_E("serialInit", "ERROR: OPENING UART PORT FAILED");
        return -1;
    }

    // We have a driver now installed so set up the read/write functions to use driver also.
    esp_vfs_dev_uart_use_driver(0);

    return fd;
}