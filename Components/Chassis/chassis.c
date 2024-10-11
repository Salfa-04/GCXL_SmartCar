#include "chassis.h"

#include <stdint.h>

#include "hwt101.h"
#include "motor.h"
#include "pid.h"

#define THRESHOLD 1.f

#define ACCEL_PROP 0.3f  /// 加速度平滑比例
#define ACCEL (uint8_t)(ACCEL_PROP * 256.f)

/// 位置环 PID 参数
#define MOT_KP 1.f
#define MOT_KI 0.00009765625f
#define MOT_KD 0U
#define MOT_MAXI 60U
#define MOT_MAXOUT 600U

/// 角度环 PID 参数
#define MA_KP 0U
#define MA_KI 0U
#define MA_KD 0U
#define MA_MAXI 60U
#define MA_MAXOUT 60U

void chassis_delay(void);

static pid_t PID_MotX = {0};
static pid_t PID_MotY = {0};
static pid_t PID_MotA = {0};

static int64_t AddupA = 0;
static int64_t AddupB = 0;
static int64_t AddupC = 0;
static int64_t AddupD = 0;

static float ANGLE = 0;

void chassis_pid_set(float kp, float ki, float kd) {
  PID_MotA.output = PID_MotA.error = PID_MotA.integral = PID_MotA.lastError = 0;
  PID_MotA.kp = kp, PID_MotA.ki = ki, PID_MotA.kd = kd;
}

void chassis_init(void) {
  hwt101_init();
  motor_init();

  /// 初始化 位置环 角度环 PID
  pid_init(&PID_MotX, MOT_KP, MOT_KI, MOT_KD, MOT_MAXI, MOT_MAXOUT);
  pid_init(&PID_MotY, MOT_KP, MOT_KI, MOT_KD, MOT_MAXI, MOT_MAXOUT);
  pid_init(&PID_MotA, MA_KP, MA_KI, MA_KD, MA_MAXI, MA_MAXOUT);
}

/// 控制相对位置, 单位为 mm
/// !!! 不要在中断里使用
void chassis_control_dest(float x, float y) {
  PID_MotX.target = x, PID_MotY.target = y;
  PID_MotX.output = PID_MotX.error = PID_MotX.lastError = PID_MotX.integral = 0;
  PID_MotY.output = PID_MotY.error = PID_MotY.lastError = PID_MotY.integral = 0;

  motor_addup_clear();
  // chassis_delay();

  // /// 等待位置环稳定
  // while (PID_MotX.output != 0 || PID_MotY.output != 0);
}

/// 控制相对角度
/// !!! 不要在中断里使用
void chassis_control_angu(float w) {
  float dest = PID_MotA.target + w;
  PID_MotA.output = PID_MotA.error = PID_MotA.lastError = PID_MotA.integral = 0;

  while (dest > 180) dest -= 360;
  while (dest < -180) dest += 360;

  PID_MotA.target = dest;
  // chassis_delay();

  // /// 等待角度环稳定
  // while (PID_MotA.output != 0);
}

#include "usart.h"

/// 电机事件回调函数, 控制速度更新频率
/// 原则上应该与陀螺仪更新速度一致
void motor_event_callback(void) {
  float OutputA = 0, OutputB = 0, OutputC = 0, OutputD = 0;
  motor_addup_get(&AddupA, &AddupB, &AddupC, &AddupD);

  /// 运动解算: cnt -> mm
  /// X = (+ A + B + C + D);                    cnt
  /// Y = (- A + B - C + D);                    cnt
  /// W = (- A - B + C + D);                    cnt
  ///
  /// Line Velocity (线速度): cnt -> mm
  /// N = N * 2π / 65536.f                      rad
  /// N = N * WHEEL_RADIUS;                     mm
  /// SO:
  ///   N = N * π * WHEEL_RADIUS / 131072.f;
  ///
  /// 单位与 `WHEEL_RADIUS` `CHASSIS_RW` 的单位相同
  float DestX =
      (AddupA + AddupB + AddupC + AddupD) * __PI * WHEEL_RADIUS / 131072.f;
  float DestY =
      (-AddupA + AddupB - AddupC + AddupD) * __PI * WHEEL_RADIUS / 131072.f;

  pid_update(&PID_MotX, DestX);
  pid_update(&PID_MotY, DestY);

  float SpeedX = PID_MotX.output, SpeedY = PID_MotY.output;

  /// 运动解算: dps -> rpm W: V(dps); T: 60s/min
  ///
  /// Angular Velocity (角速度): dps -> rpm
  /// V = W * 2π * (Ra + Rb) / 360.f            mm/s
  /// V = V * T / (2π * WHEEL_RADIUS)           rpm
  /// SO:
  ///   V = W * (Ra + Rb) / (WHEEL_RADIUS * 6.f);
  ///
  /// 单位与 `WHEEL_RADIUS` `CHASSIS_RW` 的单位相同
  float vm = PID_MotA.output * CHASSIS_RW / (WHEEL_RADIUS * 6.f);
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
  OutputA += (SpeedX - SpeedY) * 30.f / (WHEEL_RADIUS * __PI);
  OutputB += (SpeedX + SpeedY) * 30.f / (WHEEL_RADIUS * __PI);
  OutputC += (SpeedX - SpeedY) * 30.f / (WHEEL_RADIUS * __PI);
  OutputD += (SpeedX + SpeedY) * 30.f / (WHEEL_RADIUS * __PI);

  float prop_x = 0, prop_y = 0;

  if (PID_MotX.target) prop_x = DestX / PID_MotX.target;
  if (PID_MotY.target) prop_y = DestY / PID_MotY.target;

  uint8_t accel =
      (prop_x > ACCEL_PROP)   ? ((prop_x < 1.f) ? (uint8_t)(prop_x * 256) : 0)
      : (prop_y > ACCEL_PROP) ? ((prop_y < 1.f) ? (uint8_t)(prop_y * 256) : 0)
      : (prop_x || prop_y)    ? ACCEL
                              : 0;

  /// 并集 PID 输出
  motor_speed_ctrl((int16_t)OutputA, (int16_t)OutputB, (int16_t)OutputC,
                   (int16_t)OutputD, accel);

  uprintf("D: %d ;;; C: %f ;;; PID: %f, %f, %f ;;; A: %f\r\n",
          (int)PID_MotA.target, PID_MotA.output, PID_MotA.kp, PID_MotA.ki,
          PID_MotA.kd, ANGLE);
}

void hwt101_angle_callback(float angle) {
  // 角度环 PID 更新 d(°)
  pid_update(&PID_MotA, angle);

  ANGLE = angle;
}

void chassis_delay(void) {
  /* delay 1ms */
  HAL_Delay(1);
}
