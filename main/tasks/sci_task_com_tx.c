#include "include/sci_task_com_tx.h"

#include "drivers/include/sci_wifi.h"
#include "esp_attr.h"

#include "sci_ble.h"
#include "sci_bt.h"
#include "sci_config.h"
#include "sci_macros.h"
#include "sci_macros_conf.h"
#include "sci_scientisst.h"
#include "sci_tcp.h"
#include "sci_udp.h"
#include "sci_ws.h"

static void sendData(esp_err_t (*tx_write_func)(uint32_t, int, uint8_t *));

/**
 * \brief Task that sends data to the client.
 *
 * This task is responsible for sending data to the client. It is notified by
 * the task_acquisition when there is data to send. If the client is BT or BLE it
 * initializes the respective component.
 *
 * This task can be removed and task_acquisition can do the sendData() when
 * not send_busy. But, atm task_acquisition is the bottleneck
 */
_Noreturn void IRAM_ATTR sendTask(const void *not_used)
{
    esp_err_t (*tx_write_func)(uint32_t, int, uint8_t *); ///< Send function pointer
    void (*send_data_func)(esp_err_t (*send_func)(uint32_t, int, uint8_t *)) = &sendData;

    switch (op_settings.com_mode)
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
        tx_write_func = &esp_spp_write;
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
 * \brief Function to send data using the appropriate send function.
 *
 */
static void IRAM_ATTR sendData(esp_err_t (*tx_write_func)(uint32_t, int, uint8_t *))
{
    while (1) // Loop to send all the available data
    {
        int buf_ready;
        esp_err_t ret;
        // Check if there's anything to send and if there is, check if it's enough
        // to send
        // TODO: Remove semaphores. Race condition exists but each task can only mark values that would stop
        // their own execution and do not interfere with the other task.
        xSemaphoreTake(mutex_buffers_ready_to_send, portMAX_DELAY);
        buf_ready = frame_buffer_ready_to_send[bt_curr_buff];
        xSemaphoreGive(mutex_buffers_ready_to_send);

        if (!buf_ready)
        {
            send_busy = 0;
            return;
        }

        if (op_mode != OP_MODE_LIVE && bt_curr_buff != (NUM_BUFFERS - 1))
        {
            send_busy = 0;
            return;
        }

        send_busy = 1;
        ret = tx_write_func(send_fd, frame_buffer_write_idx[bt_curr_buff], frame_buffer[bt_curr_buff]);

        if (ret == ESP_FAIL) // If the send function failed, stop sending data. Connection was possibly lost
            break;

        xSemaphoreTake(mutex_buffers_ready_to_send, portMAX_DELAY);
        frame_buffer_ready_to_send[bt_curr_buff] = 0;
        xSemaphoreGive(mutex_buffers_ready_to_send);

        // Clear recently sent buffer
        memset(frame_buffer[bt_curr_buff], 0, frame_buffer_write_idx[bt_curr_buff]);
        frame_buffer_write_idx[bt_curr_buff] = 0;

        // Change send buffer
        bt_curr_buff = (bt_curr_buff + 1) % (NUM_BUFFERS - 1);
    }
}
