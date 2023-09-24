#include "Usart.h"

void usartIfInit(UsartIF_t *usart_if, uint8_t rxbuf_len, uint8_t txbuf_len) {
    usart_if->rx_flag = 0;
    usart_if->rx_buf = (uint8_t *)pvPortMalloc(rxbuf_len);
    usart_if->tx_flag = 0;
    usart_if->tx_buf = (uint8_t *)pvPortMalloc(txbuf_len);
    return;
}

void usartIfRelease(UsartIF_t *usart_if) {
    vPortFree(usart_if->rx_buf);
    vPortFree(usart_if->tx_buf);
    return;
}