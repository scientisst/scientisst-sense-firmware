#ifndef _UART_H
#define _UART_H

void uart_init(void);
esp_err_t IRAM_ATTR serialSend(uint32_t fd, int len, uint8_t *buff);

#endif