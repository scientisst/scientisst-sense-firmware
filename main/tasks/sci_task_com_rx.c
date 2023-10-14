#include "include/sci_task_com_rx.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

#include "sci_com.h"
#include "sci_macros.h"
#include "sci_scientisst.h"
#include "sci_serial.h"
#include "sci_tcp.h"
#include "sci_udp.h"
#include "sci_ws.h"

static void wifiSerialRcv(void);

/**
 * \brief Task that receives data from the client.
 *
 * This task is responsible for receiving data from the client.
 */
_Noreturn void rcvTask(const void *not_used)
{
    while (1)
    {
        if (op_settings.com_mode == COM_MODE_TCP_STA) // TCP client
        {
            while ((send_fd = initTcpClient(op_settings.host_ip, op_settings.port_number)) < 0)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                DEBUG_PRINT_E("rcvTask",
                              "Cannot connect to TCP Server %s:%s specified in the "
                              "configuration. Redo the configuration. Trying again...",
                              op_settings.host_ip, op_settings.port_number);
            }
        }
        else if (op_settings.com_mode == COM_MODE_TCP_AP) // TCP server
        {
            if (initTcpServer(op_settings.port_number) != ESP_OK)
            {
                DEBUG_PRINT_E("serverTCP",
                              "Cannot init TCP Server on port %s specified "
                              "in the configuration. Redo the "
                              "configuration. Trying again...",
                              op_settings.port_number);
            }
            while ((send_fd = initTcpConnection()) < 0)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                DEBUG_PRINT_E("rcvTask",
                              "Cannot connect to TCP Server %s:%s specified in the "
                              "configuration. Redo the configuration. Trying again...",
                              op_settings.host_ip, op_settings.port_number);
            }
        }
        else if (op_settings.com_mode == COM_MODE_UDP_STA) // UDP client
        {
            while ((send_fd = initUdpClient(op_settings.host_ip, op_settings.port_number)) < 0)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                DEBUG_PRINT_E("rcvTask",
                              "Cannot connect to UDP Server %s:%s specified in the "
                              "configuration. Redo the configuration. Trying again...",
                              op_settings.host_ip, op_settings.port_number);
            }
        }
        else if (op_settings.com_mode == COM_MODE_SERIAL) // Serial
        {
            while ((send_fd = serialInit()) < 0)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                DEBUG_PRINT_E("rcvTask", "Cannot connect to host USB Serial port. Trying again...");
            }
        }
        wifiSerialRcv();
    }
}

/**
 * \brief Receives data through a TCP connection and then call processRcv on the
 * data.
 *
 * This function receives data through a TCP connection and then call processRcv
 * on the data. If connection is closed unexpectedly, it will try to reconnect
 * in a loop. If connection is closed gracefully, it will stop acquiring data.
 *
 */
static void wifiSerialRcv(void)
{
    uint8_t buff[CMD_MAX_BYTES];
    int len;
    int read_bytes;

    while (send_fd <= 0) // Wait until a connection is made
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    while (1)
    {
        if ((read_bytes = read(send_fd, buff, CMD_MAX_BYTES)) == 0)
        {
            DEBUG_PRINT_W("wifiSerialRcv", "Connection closed gracefully");
            buff[0] = 0;
            len = 1;
        }
        else if (read_bytes < 0)
        {
            DEBUG_PRINT_W("wifiSerialRcv", "Connection closed with error: %d", errno);
            buff[0] = 0;
            len = 1;
        }
        else
        {
            if (read_bytes != CMD_MAX_BYTES)
            {
                DEBUG_PRINT_E("wifiSerialRcv", "Received %d bytes and the acceptable amount is %d",
                              read_bytes, CMD_MAX_BYTES);
            }
            len = CMD_MAX_BYTES;
        }

        // If connection was broken, close socket and return
        if (read_bytes <= 0)
        {
            shutdown(send_fd, 0);
            close(send_fd);
            send_fd = 0;

            stopAcquisition();

            DEBUG_PRINT_E("wifiSerialRcv", "Disconnected from Wifi or socket error");

            return;
        }

        processRcv(buff, len);
    }
}
