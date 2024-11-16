#ifndef __HWT101_H
#define __HWT101_H

#include "stm32f4xx_hal.h"

/// HWT101 initialization
void hwt101_init(void);

/// 清除偏航角数据
void hwt101_angle_clear(void);

/// 中断数据接收回调函数, 请把该函数放到中断服务函数中, 如:
///
/// ```
/// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
///     if (huart->Instance == huart4.Instance) hwt101_event_callback(Size);
/// }
/// ```
void hwt101_event_callback(uint16_t size);

/// 偏航角数据接收回调函数: d(°)
void hwt101_angle_callback(float angle);

#endif /* __HWT101_H */
