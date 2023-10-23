/**
 * \file sci_task_com_tx.h
 * \brief Communication Transmission Task
 *
 * This file contains the implementation of the task responsible for handling data transmission
 * across various communication protocols. It is designed to work with multiple communication
 * methods, including BLE, Bluetooth Classic (BT), TCP, UDP, WebSocket, and Serial communication.
 *
 * The core functionality involves waiting for notifications from the data acquisition task,
 * indicating that data is ready for transmission. Based on the system's current communication
 * settings, it selects the appropriate transmission method, initiates the send process, and
 * manages any necessary post-transmission operations.
 */

#include "sci_task_com_tx.h"

#include "esp_attr.h"

#include "sci_ble.h"
#include "sci_bt.h"
#include "sci_com.h"
#include "sci_tcp_and_serial.h"
#include "sci_udp.h"
#include "sci_ws.h"

static void sendData(esp_err_t (*tx_write_func)(uint32_t, int, const uint8_t *));

/**
 * \brief Task responsible for managing and routing data transmission based on communication settings.
 *
 * This task is continuously running after system initialization. It listens for notifications from the
 * data acquisition task, signaling that new data is ready for transmission. Upon trigger, it selects the
 * appropriate data transmission method based on current communication settings and initiates the sending
 * process.
 *
 * \return Never returns.
 */
_Noreturn void IRAM_ATTR sendTask(void)
{
    esp_err_t (*tx_write_func)(uint32_t, int, const uint8_t *) = NULL; ///< Send function pointer
    void (*send_data_func)(esp_err_t (*send_func)(uint32_t, int, const uint8_t *)) = &sendData;

    switch (scientisst_device_settings.op_settings.com_mode)
    {
    case COM_MODE_TCP_STA:
        tx_write_func = &tcpSerialSend;
        break;
    case COM_MODE_TCP_AP:
        tx_write_func = &tcpSerialSend;
        break;
    case COM_MODE_UDP_STA:
        tx_write_func = &udpSend;
        break;
    case COM_MODE_WS_AP:
        tx_write_func = &wsSerialSend;
        break;
    case COM_MODE_SERIAL:
        tx_write_func = &tcpSerialSend;
        break;
    case COM_MODE_BLE:
        tx_write_func = &sendBle;
        break;
    case COM_MODE_BT:
    case COM_MODE_SD_CARD:
        send_data_func = &sendDataBluetooth;
        break;
    default:
        DEBUG_PRINT_E("task_com_tx", "Invalid COM mode, cannot send data");
        abort();
    }

    while (1)
    {
        if (!ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
            continue;
        send_data_func(tx_write_func);
    }
}

/**
 * \brief Function responsible for checking buffers ready for transmission, sending data and clearing sent buffers.
 *
 * This function is used for all communication modes except Bluetooth and SDCard. This function checks the readiness of
 * the data buffers, sends the data through the appropriate channel, and manages buffer state post-transmission.
 *
 * \param[in] tx_write_func Pointer to the function responsible for data transmission over the selected communication
 * protocol.
 * \return None.
 */
static void IRAM_ATTR sendData(esp_err_t (*tx_write_func)(uint32_t, int, const uint8_t *))
{
    CHECK_NOT_NULL(tx_write_func);

    while (1) // Loop to send all the available data
    {
        esp_err_t ret;
        uint16_t buf_ready;

        // Check if there's anything to send and if there is, check if it's enough
        // to send
        buf_ready = scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.tx_curr_buff];

        if (buf_ready == 0)
        {
            scientisst_device_settings.send_busy = 0;
            return;
        }

        if (scientisst_device_settings.op_mode != OP_MODE_LIVE && scientisst_buffers.tx_curr_buff != (NUM_BUFFERS - 1))
        {
            scientisst_device_settings.send_busy = 0;
            return;
        }

        scientisst_device_settings.send_busy = 1;
        ret = tx_write_func(send_fd, buf_ready, scientisst_buffers.frame_buffer[scientisst_buffers.tx_curr_buff]);

        if (ret == ESP_FAIL) // If the send function failed, stop sending data. Connection was possibly lost
            break;

        // Clear recently sent buffer
        memset(scientisst_buffers.frame_buffer[scientisst_buffers.tx_curr_buff], 0,
               scientisst_buffers.frame_buffer_length_bytes);

        scientisst_buffers.frame_buffer_ready_to_send[scientisst_buffers.tx_curr_buff] = 0;

        // Change send buffer
        if (scientisst_buffers.tx_curr_buff != NUM_BUFFERS - 1)
            scientisst_buffers.tx_curr_buff = (scientisst_buffers.tx_curr_buff + 1) % (NUM_BUFFERS - 1);
        else
            scientisst_buffers.tx_curr_buff = 0;
    }
}
