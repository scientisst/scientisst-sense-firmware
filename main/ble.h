#ifndef BLE_H
#define BLE_H

#define GATTS_NOTIFY_LEN 517

void initBle(void);
esp_err_t sendBle(uint32_t fd, int len, uint8_t *buff);

#endif