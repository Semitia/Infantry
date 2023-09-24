#ifndef __USART_H
#define __USART_H

#include "main.h"

/**
 * @brief 串口统一接口 usart interface
 * @note 其实也像是对标准库的一点补充？添加了一些常用的标志或变量
*/
typedef struct __UsartIF_t{
    uint8_t rx_flag;        //接收完成标志
    uint8_t *rx_buf;        //接收缓冲区
    uint8_t tx_flag;        //发送完成标志
    uint8_t *tx_buf;        //发送缓冲区
}UsartIF_t;

void usartIfInit(UsartIF_t *usart_if, uint8_t rxbuf_len, uint8_t txbuf_len);
void usartIfRelease(UsartIF_t *usart_if);

#endif

