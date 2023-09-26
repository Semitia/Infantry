#ifndef __UART4_H
#define __UART4_H

#include "usart.h"
#define JudgeBufBiggestSize 45

void UART4_Configuration(void);
extern UsartIF_t usart4_if;
#endif
