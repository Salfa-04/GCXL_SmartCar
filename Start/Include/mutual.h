#ifndef __MUTUAL_H
#define __MUTUAL_H

#include "stm32f4xx_hal.h"

const int16_t* getp_mutual(void);
void mutual_handle(uint8_t* data);

/// @brief Mutual UART handle
/// !!!串口1
extern UART_HandleTypeDef U_Mutual;

#endif /* __MUTUAL_H */
