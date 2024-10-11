#ifndef __USART_H
#define __USART_H

#include "main.h"

extern UART_HandleTypeDef huart1;

void uart1_init(void);
void uprintf(const char *fmt, ...);

void USART1_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);

#endif /* __USART_H */
