#ifndef __CHASSIS_H
#define __CHASSIS_H

#include <stdint.h>

/// CHASSIS_RW = CHASSIS_RA + CHASSIS_RB
#define CHASSIS_RW 141.f   // 单位 mm
#define WHEEL_RADIUS 38.f  // 单位 mm
#define __PI 3.14159265358979323846f

void chassis_init(void);

/// 控制相对位置, 单位为 mm
/// !!! 不要在中断里使用
void chassis_control_dest(int16_t x, int16_t y);

/// 控制相对角度
/// !!! 不要在中断里使用
void chassis_control_angu(int8_t w);

/// 电机事件回调函数, 控制速度更新频率
/// ```
/// static unsigned char count = 0;
/// /* 进入底盘控制函数的周期函数, 期待调用频率与陀螺仪回报频率一致 */
/// if (flag && count++ > 10) motor_event_callback(), count = 0;
/// ```
void motor_event_callback(void);

#endif /* __CHASSIS_H */
