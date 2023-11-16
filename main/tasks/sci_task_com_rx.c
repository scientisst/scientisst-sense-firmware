/**
 * \file sci_task_com_rx.h
 * \brief Communication Reception Task
 *
 * This file establishes the functionality required for the continuous reception of data
 * from different communication mediums. It supports various modes, including TCP (both server
 * and client roles), UDP, and Serial communication. The task dynamically adapts to the communication
 * settings specified in the system's operational parameters.
 *
 * The primary function of this file is to initialize the appropriate communication protocol,
 * actively listen for incoming data, and handle that data upon receipt. It includes mechanisms
 * to reconnect in case of connection loss and to halt data acquisition gracefully upon a controlled
 * connection close.
 */

#include "sci_task_com_rx.h"

#include "lwip/err.h"
#include "lwip/sockets.h"

#include "sci_com.h"
#include "sci_serial.h"
#include "sci_tcp_and_serial.h"
#include "sci_udp.h"

#define CMD_MAX_BYTES 4

static void wifiSerialRxHandler(void);

/**
 * \brief Task handling the reception of data from various communication interfaces.
 *
 * The rxTask function is an infinite loop task responsible for managing incoming data based on the
 * configured communication method. It initializes the necessary communication parameters, handles reconnection
 * attempts in case of connection failure, and delegates the data handling to the appropriate function once
 * data is received. The task adapts to TCP, UDP, and Serial communication modes.
 *
 * \return Never returns.
 */
_Noreturn void rxTask(void)
{
    while (1)
    {
        if (scientisst_device_settings.op_settings.com_mode == COM_MODE_TCP_STA) // TCP client
        {
            while ((send_fd = initTcpClient(scientisst_device_settings.op_settings.host_ip,
                                            scientisst_device_settings.op_settings.port_number)) < 0)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                DEBUG_PRINT_E("rxTask",
                              "Cannot connect to TCP Server %s:%s specified in the "
                              "configuration. Redo the configuration. Trying again...",
                              scientisst_device_settings.op_settings.host_ip,
                              scientisst_device_settings.op_settings.port_number);
            }
        }
        else if (scientisst_device_settings.op_settings.com_mode == COM_MODE_TCP_AP) // TCP server
        {
            if (initTcpServer(scientisst_device_settings.op_settings.port_number) != ESP_OK)
            {
                DEBUG_PRINT_E("serverTCP",
                              "Cannot init TCP Server on port %s specified "
                              "in the configuration. Redo the "
                              "configuration. Trying again...",
                              scientisst_device_settings.op_settings.port_number);
            }
            while ((send_fd = initTcpConnection()) < 0)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                DEBUG_PRINT_E("rxTask",
                              "Cannot connect to TCP Server %s:%s specified in the "
                              "configuration. Redo the configuration. Trying again...",
                              scientisst_device_settings.op_settings.host_ip,
                              scientisst_device_settings.op_settings.port_number);
            }
        }
        else if (scientisst_device_settings.op_settings.com_mode == COM_MODE_UDP_STA) // UDP client
        {
            while ((send_fd = initUdpClient(scientisst_device_settings.op_settings.host_ip,
                                            scientisst_device_settings.op_settings.port_number)) < 0)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                DEBUG_PRINT_E("rxTask",
                              "Cannot connect to UDP Server %s:%s specified in the "
                              "configuration. Redo the configuration. Trying again...",
                              scientisst_device_settings.op_settings.host_ip,
                              scientisst_device_settings.op_settings.port_number);
            }
        }
        else if (scientisst_device_settings.op_settings.com_mode == COM_MODE_SERIAL) // Serial
        {
            while ((send_fd = serialInit()) < 0)
            {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                DEBUG_PRINT_E("rxTask", "Cannot connect to host USB Serial port. Trying again...");
            }
        }
        wifiSerialRxHandler();
    }
}

/**
 * \brief Handler for receiving and processing data through a Wi-Fi connection.
 *
 * The wifiSerialRxHandler function continuously listens for data on the established Wi-Fi connection.
 * Upon receiving data, it checks for the integrity and amount of the data, handles any connection errors,
 * and passes valid packets to the  processPacket() function for further processing. If the connection is
 * closed (either gracefully or due to an error), it shuts down the socket and handles task cleanup.
 *
 * \return None.
 */
static void wifiSerialRxHandler(void)
{
    uint8_t buff[CMD_MAX_BYTES];
    int read_bytes;

    while (1)
    {
        if ((read_bytes = read(send_fd, buff, CMD_MAX_BYTES)) == 0)
        {
            DEBUG_PRINT_I("wifiSerialRxHandler", "Connection closed gracefully");
            buff[0] = 0;
        }
        else if (read_bytes < 0)
        {
            DEBUG_PRINT_E("wifiSerialRxHandler", "Connection closed with error: %d", errno);
            buff[0] = 0;
        }
        else
        {
            if (read_bytes != CMD_MAX_BYTES)
            {
                DEBUG_PRINT_E("wifiSerialRxHandler", "Received %d bytes and the acceptable amount is %d", read_bytes,
                              CMD_MAX_BYTES);
            }
        }

        // If connection was broken, close socket and return
        if (read_bytes <= 0)
        {
            shutdown(send_fd, 0);
            close(send_fd);
            send_fd = 0;

            stopAcquisition();

            DEBUG_PRINT_E("wifiSerialRxHandler", "Disconnected from Wifi or socket error");

            return;
        }

        processPacket(buff);
    }
}
