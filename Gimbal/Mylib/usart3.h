#ifndef __USART3_H__
#define __USART3_H__

#include "sys.h"
#include "usart.h"

#define RX_USART3_BUFFER 30     //DMA接收缓存大小

extern UsartIF_t usart3_if;

void USART3_Configuration(void);
void RC_Rst(void);

#endif

