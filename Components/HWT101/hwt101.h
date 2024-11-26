#ifndef __HWT101_H
#define __HWT101_H

#include <type_def.h>

/// HWT101 initialization
void hwt101_init(void);

/// 中断数据接收回调函数, 请把该函数放到中断服务函数中, 如:
///
/// ```
/// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
///     if (huart->Instance == huart4.Instance) hwt101_event_callback(Size);
/// }
/// ```
void hwt101_event_callback(uint16_t size);

/// 获取偏航角
fp32 hwt101_get_angle(void);

#endif /* __HWT101_H */
