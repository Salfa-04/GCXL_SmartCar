#include "chassis.h"

#include <stdint.h>

#include "hwt101.h"
#include "motor.h"
#include "pid.h"

#define THRESHOLD 1.f

#define ACCEL_PROP 0.2f  /// 加速度平滑比例
#define ACCEL (uint8_t)(ACCEL_PROP * 256.f)

/// 位置环 PID 参数
#define MOT_KP 1U
#define MOT_KI 0U
#define MOT_KD 0U
#define MOT_MAXI 1U
#define MOT_MAXOUT 450U

/// 角度环 PID 参数
#define MA_KP 6U
#define MA_KI 0U
#define MA_KD 0U
#define MA_MAXI 1U
#define MA_MAXOUT 120U

void chassis_delay(void);

static pid_t PID_MotX = {0};
static pid_t PID_MotY = {0};
static pid_t PID_MotA = {0};

static int64_t AddupA = 0;
static int64_t AddupB = 0;
static int64_t AddupC = 0;
static int64_t AddupD = 0;

static float ANGLE = 0;

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
void chassis_control_dest(uint16_t x, uint16_t y) {
  PID_MotX.target = (float)x, PID_MotY.target = (float)y;
  PID_MotX.output = PID_MotX.error = PID_MotX.lastError = PID_MotX.integral = 0;
  PID_MotY.output = PID_MotY.error = PID_MotY.lastError = PID_MotY.integral = 0;
  motor_addup_clear();
  // chassis_delay();

  // /// 等待位置环稳定
  // while (PID_MotX.output != 0 || PID_MotY.output != 0);
}

/// 控制相对角度
/// !!! 不要在中断里使用
void chassis_control_angu(int8_t w) {
  PID_MotA.target = (float)w;
  PID_MotA.output = PID_MotA.error = PID_MotA.lastError = PID_MotA.integral = 0;
  hwt101_angle_clear();

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

  pid_update(&PID_MotX, (int)DestX);
  pid_update(&PID_MotY, (int)DestY);

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
      : (PID_MotX.target || PID_MotY.target) ? ACCEL
                                             : 0;

  /// 并集 PID 输出
  motor_speed_ctrl((int16_t)OutputA, (int16_t)OutputB, (int16_t)OutputC,
                   (int16_t)OutputD, accel);

  // uprintf(
  //     "OA: %d ;;; CA: %f ;;; OD: %d %d ;;; CD: %f, %f ;; ioe: %f %f %f\r\n ",
  //     (int)PID_MotA.target, ANGLE, (int)PID_MotX.target,
  //     (int)PID_MotY.target, DestX, DestY, PID_MotA.integral, PID_MotA.output,
  //     PID_MotA.error);

  // uprintf("oca: %d %f ;;; pid: %f %f %f ;;; ioe: %f %f %f\r\n",
  //         (int)PID_MotA.target, ANGLE, PID_MotA.kp, PID_MotA.ki, PID_MotA.kd,
  //         PID_MotA.integral, PID_MotA.output, PID_MotA.error);

  uprintf("od: %d %f, %d %f ; pid: %f %f %f; ioe: %f %f %f, %f %f %f\r\n",
          (int)PID_MotX.target, DestX, (int)PID_MotY.target, DestY, PID_MotX.kp,
          PID_MotX.ki, PID_MotX.kd, PID_MotX.integral, PID_MotX.output,
          PID_MotX.error, PID_MotY.integral, PID_MotY.output, PID_MotY.error);
}

void angle_pid_set(float p, float i, float d) {
  PID_MotA.kp = p, PID_MotA.ki = i, PID_MotA.kd = d;
  PID_MotA.output = PID_MotA.error = PID_MotA.integral = PID_MotA.lastError = 0;
}

void dest_pid_set(float p, float i, float d) {
  PID_MotX.kp = p, PID_MotX.ki = i, PID_MotX.kd = d;
  PID_MotY.kp = p, PID_MotY.ki = i, PID_MotY.kd = d;
  PID_MotX.output = PID_MotX.error = PID_MotX.integral = PID_MotX.lastError = 0;
  PID_MotY.output = PID_MotY.error = PID_MotY.integral = PID_MotY.lastError = 0;
}

void hwt101_angle_callback(float angle) {
  // 角度环 PID 更新 d(°)
  pid_update(&PID_MotA, (int)angle);

  ANGLE = angle;
}

void chassis_delay(void) {
  /* delay 1ms */
  HAL_Delay(1);
}
