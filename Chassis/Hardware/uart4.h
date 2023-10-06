#ifndef __UART4_H
#define __UART4_H

#include "usart.h"

#define TX_MAX_SIZE 128
#define RX_MAX_SIZE 45

extern UsartIF_t usart4_if;

void UART4_Configuration(void);
#endif
