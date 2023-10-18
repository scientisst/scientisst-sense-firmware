/** \file uart.c
    \brief UART functions

    This file contains the functions for the UART communication. Only has the
   init function for now.
*/
#include "sci_serial.h"

#include <sys/errno.h>
#include <sys/fcntl.h>

#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "sys/select.h"

static const uart_port_t serial_com_uart_num = UART_NUM_0;

int serialInit(void)
{
    int fd;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 512, 2048, 0, NULL, 0));

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(serial_com_uart_num, &uart_config));

    if ((fd = open("/dev/uart/0", O_RDWR)) == -1)
    {
        DEBUG_PRINT_E("serialInit", "ERROR: OPENING UART PORT FAILED");
        return -1;
    }

    // We have a driver now installed so set up the read/write functions to use driver also.
    esp_vfs_dev_uart_use_driver(0);

    return fd;
}
