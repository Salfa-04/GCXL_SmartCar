#ifndef __HWT101_H
#define __HWT101_H

#include "stm32f4xx_hal.h"

/// HWT101 initialization
void hwt101_init(void);

/// 中断数据接收回调函数, 请把该函数放到中断服务函数中, 如:
///
/// ```
/// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
///     if (huart->Instance == UART4 ) hwt101_event_callback();
/// }
/// ```
void hwt101_event_callback(void);

/// 偏航角数据接收回调函数: d(°)
void hwt101_angle_callback(float angle);

#endif /* __HWT101_H */
