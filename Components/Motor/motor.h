#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"
#include "type_def.h"

#define MotPort UART_HandleTypeDef

#define MotIdA 1  // Motor A    CW  左前
#define MotIdB 1  // Motor B    CW  左后
#define MotIdC 1  // Motor C   CCW  右后
#define MotIdD 1  // Motor D   CCW  右前

/// CHASSIS_RW = CHASSIS_RA + CHASSIS_RB
#define CHASSIS_RW 141.f   // 单位 mm
#define WHEEL_RADIUS 38.f  // 单位 mm
#define __PI 3.14159265358979323846f

/// 计算后的比例系数
#define ADUPX_PROP 0.00093f
#define ADUPY_PROP 0.00090f
#define OPUT_PROP 0.25f

// 在使用 `HAL_UART_TxCpltCallback()` `HAL_UART_TxCpltCallback()` 回调函数时，
// 请务必检查进入中断的端口是否是需要的端口, 否则会引起异常

void motor_init(void);
void motor_speed_ctrl(fp32 *oput);
void motor_addup_get(fp32 *dest);

#endif /* __MOTOR_H */
