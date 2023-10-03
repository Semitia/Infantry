/**********************************************************************************************************
 * @文件     Usart.c
 * @说明     串口调用接口封装
 * @版本  	 V1.0
 * @作者     吴磊
 * @日期     2023-9-24
 *********************************************************************************************************/

#include "Usart.h"

/**
 * @brief usart interface init
 * @param usart_if usart interface
 * @param rxbuf_len rx buffer length
 * @param txbuf_len tx buffer length
 * @note dynamic memory allocation for rx_buf and tx_buf
*/
void usartIfInit(UsartIF_t *usart_if, uint8_t rxbuf_len, uint8_t txbuf_len, void (*usartInit)(void)) {
    usart_if->rx_flag = 0;
    usart_if->rx_buf = (uint8_t *)pvPortMalloc(rxbuf_len);
    usart_if->tx_flag = 0;
    usart_if->tx_buf = (uint8_t *)pvPortMalloc(txbuf_len);
    usartInit();
    return;
}

/**
 * @brief usart interface release
 * @param usart_if usart interface
 * @note free rx_buf and tx_buf
*/
void usartIfRelease(UsartIF_t *usart_if) {
    vPortFree(usart_if->rx_buf);
    vPortFree(usart_if->tx_buf);
    return;
}

