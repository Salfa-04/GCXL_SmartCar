#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"

#define MotPort UART_HandleTypeDef

#define MotIdA 1  // Motor A    CW  左前
#define MotIdB 1  // Motor B    CW  左后
#define MotIdC 1  // Motor C   CCW  右后
#define MotIdD 1  // Motor D   CCW  右前

// 在使用 `HAL_UART_TxCpltCallback()` `HAL_UART_TxCpltCallback()` 回调函数时，
// 请务必检查进入中断的端口是否是需要的端口, 否则会引起异常

void motor_init(void);
void motor_speed_ctrl(int16_t a, int16_t b, int16_t c, int16_t d,
                      uint8_t accel);
void motor_addup_get(int64_t *a, int64_t *b, int64_t *c, int64_t *d);
void motor_addup_clear(void);

#endif /* __MOTOR_H */
