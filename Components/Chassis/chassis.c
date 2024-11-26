#include "chassis.h"

#include "hwt101.h"
#include "motor.h"
#include "pid.h"
#include "type_def.h"

// #define ANGLE_90DEG (90.f)
#define ANGLE_90DEG (90.f - 0.045f)

/// 位置环 PIDS 参数
#define MT_KP 3.5f
#define MT_KS 8U
#define MT_MAXI 1U
#define MT_MAXOUT 600U

/// 角度环 PIDS 参数
#define MA_KP 6.5f
#define MA_KS 2.5f
#define MA_MAXI 1U
#define MA_MAXOUT 350U

extern fp32 fabsf(fp32);

static pid_t PID_MotX = {0};
static pid_t PID_MotY = {0};
static pid_t PID_MotA = {0};

static bool_t mutex = 0;
static uint8_t mutex_cnt = 0;

void chassis_init(void) {
  hwt101_init();
  motor_init();

  /// 初始化 位置环 角度环 PID
  pid_init(&PID_MotX, MT_KP, 0, 0, MT_KS, MT_MAXI, MT_MAXOUT);
  pid_init(&PID_MotY, MT_KP, 0, 0, MT_KS, MT_MAXI, MT_MAXOUT);
  pid_init(&PID_MotA, MA_KP, 0, 0, MA_KS, MA_MAXI, MA_MAXOUT);
}

/// 控制相对位置, 单位为 mm
/// !!! 不要在中断里使用
void chassis_control_dest(int16_t x, int16_t y) {
  PID_MotX.target += (fp32)x, PID_MotY.target += (fp32)y;

  uint32_t tick = HAL_GetTick();

  /// 等待位置环稳定
  mutex = 1, mutex_cnt = 7;
  while (mutex && HAL_GetTick() - tick < 50000U);
}

/// 控制相对坐标系下的绝对位置, 单位为 mm
/// !!! 不要在中断里使用
void chassis_control_point(int16_t x, int16_t y) {
  PID_MotX.target = (fp32)x, PID_MotY.target = (fp32)y;

  uint32_t tick = HAL_GetTick();

  /// 等待位置环稳定
  mutex = 1, mutex_cnt = 7;
  while (mutex && HAL_GetTick() - tick < 50000U);
}

/// 正向旋转 90°
/// !!! 不要在中断里使用
void chassis_angle_tanp(void) {
  PID_MotA.target += ANGLE_90DEG;

  uint32_t tick = HAL_GetTick();

  /// 等待角度环稳定
  mutex = 1, mutex_cnt = 7;
  while (mutex && HAL_GetTick() - tick < 5000U);
}

/// 逆向旋转 90°
/// !!! 不要在中断里使用
void chassis_angle_tanm(void) {
  PID_MotA.target -= ANGLE_90DEG;

  uint32_t tick = HAL_GetTick();

  /// 等待角度环稳定
  mutex = 1, mutex_cnt = 7;
  while (mutex && HAL_GetTick() - tick < 5000U);
}

/// 电机事件回调函数, 控制速度更新频率
/// 原则上应该与陀螺仪更新速度一致
void motor_event_callback(void) {
  static fp32 OutputA = 0, OutputB = 0, OutputC = 0, OutputD = 0;
  static fp32 destx = 0, desty = 0;

  motor_addup_get(&destx, &desty);
  const fp32 angle = hwt101_get_angle();

  pid_update(&PID_MotX, (int)destx);
  pid_update(&PID_MotY, (int)desty);
  fp32 SpeedX = PID_MotX.output, SpeedY = PID_MotY.output;

  /// 运动解算: dps -> rpm W: V(dps); T: 60s/min
  ///
  /// Angular Velocity (角速度): dps -> rpm
  /// V = W * 2π * (Ra + Rb) / 360.f            mm/s
  /// V = V * T / (2π * WHEEL_RADIUS)           rpm
  /// SO:
  ///   V = W * (Ra + Rb) / (WHEEL_RADIUS * 6.f);
  ///
  /// 单位与 `WHEEL_RADIUS` `CHASSIS_RW` 的单位相同
  pid_update(&PID_MotA, angle);

  fp32 vm = PID_MotA.output * CHASSIS_RW / (WHEEL_RADIUS * 6.f);
  OutputA = -vm, OutputB = -vm, OutputC = vm, OutputD = vm;

  /// 运动解算: mm/s -> rpm X: V(mm/s); Y: V(mm/s); T: 60s/min
  ///
  /// Line Velocity (线速度): mm/s -> rpm
  /// V = X - Y                                 mm/s
  /// V = V * T / (2π * WHEEL_RADIUS)           rpm
  /// SO:
  ///   V = (X - Y) * 30.f / (WHEEL_RADIUS * __PI);
  ///
  /// 单位与 `WHEEL_RADIUS` `CHASSIS_RW` 的单位相同
  OutputA += (SpeedX - SpeedY) * OPUT_PROP;
  OutputB += (SpeedX + SpeedY) * OPUT_PROP;
  OutputC += (SpeedX - SpeedY) * OPUT_PROP;
  OutputD += (SpeedX + SpeedY) * OPUT_PROP;

  /// 并集 PID 输出
  motor_speed_ctrl((int16_t)OutputA, (int16_t)OutputB, (int16_t)OutputC,
                   (int16_t)OutputD);

  /// 位置环稳定后解锁
  if (mutex_cnt < 3) {
    if (fabsf(PID_MotA.output) < 15.f && fabsf(PID_MotX.output) < 15.f &&
        fabsf(PID_MotY.output) < 5.f)
      mutex = 0;
  } else {
    mutex_cnt--;
  }
}
