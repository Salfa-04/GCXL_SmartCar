#include "chassis.h"

#include "hwt101.h"
#include "motor.h"
#include "pid.h"
#include "type_def.h"

// #define ANGLE_90DEG (90.f)
#define ANGLE_90DEG (90.f - 0.045f)

/// 位置环 PIDS 参数
#define MT_KP 3U
#define MT_KS 8U
#define MT_MAXI 1U
#define MT_MAXOUT 600U

/// 角度环 PIDS 参数
#define MA_KP 6U
#define MA_KS 2.5f
#define MA_MAXI 1U
#define MA_MAXOUT 350U

extern fp32 fabsf(fp32);

static pid_t PID_MotX = {0};
static pid_t PID_MotY = {0};
static pid_t PID_MotA = {0};

static bool_t mutex = 0;
static int8_t mutex_cnt = 0;
static int8_t direction = 0;

/// 底盘初始化
///
///! 外设自检失败会导致底盘无法正常工作
void chassis_init(void) {
  hwt101_init();
  motor_init();

  const fp32 pids_xy[4] = {MT_KP, 0, 0, MT_KS};
  const fp32 pids_an[4] = {MA_KP, 0, 0, MA_KS};
  const fp32 pids_out[2] = {MT_MAXI, MT_MAXOUT};

  /// 初始化 位置环 角度环 PID
  pid_init(&PID_MotX, pids_xy, pids_out);
  pid_init(&PID_MotY, pids_xy, pids_out);
  pid_init(&PID_MotA, pids_an, pids_out);
}

/// 控制相对位置, 单位为 mm
/// 坐标为相对坐标系下的位置
/// !!! 不要在中断里使用
void chassis_control_dest(int16_t x, int16_t y) {
  PID_MotX.target += (fp32)x, PID_MotY.target += (fp32)y;

  uint32_t tick = HAL_GetTick();

  /// 等待位置环稳定
  mutex = 1, mutex_cnt = 7;
  while (mutex && HAL_GetTick() - tick < 50000U);
}

/// 控制绝对位置, 右手坐标系, 单位为 mm
///
/// 以底盘初始方向为基准:
///
/// D: 0 底盘朝上
/// D: 1 底盘朝左
/// D: 2 底盘朝下
/// D: 3 底盘朝右
///
/// 坐标为绝对坐标系下的位置
///
/// !!! 不要在中断里使用
void chassis_control_point(int16_t x, int16_t y, int16_t d) {
  static int16_t lx = 0, ly = 0;  // 上一次的状态
  int16_t dx = 0, dy = 0, dd = 0, tx = 0, ty = 0;

  tx = x - lx, ty = y - ly, dd = d - direction;
  lx = x, ly = y, direction = d;  // 记录上一次状态

  if (direction % 4 == 0)       //* 朝上
    dx = tx, dy = ty;           //
  else if (direction % 4 == 1)  //* 朝左
    dx = ty, dy = -tx;          //
  else if (direction % 4 == 2)  //* 朝下
    dx = -tx, dy = -ty;         //
  else if (direction % 4 == 3)  //* 朝右
    dx = -ty, dy = tx;

  PID_MotX.target += (fp32)dx;
  PID_MotY.target += (fp32)dy;
  PID_MotA.target += (fp32)dd * ANGLE_90DEG;

  /// 等待位置环稳定
  mutex = 1, mutex_cnt = 7;
  uint32_t tick = HAL_GetTick();
  while (mutex && HAL_GetTick() - tick < 5000U);
}

/// 电机事件回调函数, 控制速度更新频率
/// 原则上应该与陀螺仪更新速度一致
void motor_event_callback(void) {
  /// OutPut[4] 为电机速度输出:
  /// [0]: Motor A    CW  左前
  /// [1]: Motor B    CW  左后
  /// [2]: Motor C   CCW  右后
  /// [3]: Motor D   CCW  右前
  static fp32 Output[4] = {0}, dest[4] = {0};

  motor_addup_get(dest);
  const fp32 angle = hwt101_get_angle();

  /// 运动解算: cnt -> mm
  /// X = (+ A + B + C + D) / 4;                cnt
  /// Y = (- A + B - C + D) / 4;                cnt
  /// W = (- A - B + C + D) / 4;                cnt
  ///
  /// Line Velocity (线速度): cnt -> mm
  /// N = N * 2π / 65536.f                      rad
  /// N = N * WHEEL_RADIUS;                     mm
  /// SO:
  ///   N = N * π * WHEEL_RADIUS / 131072.f;
  ///
  /// 单位与 `WHEEL_RADIUS` `CHASSIS_RW` 的单位相同
  const fp32 destX = (dest[0] + dest[1] + dest[2] + dest[3]) * ADUPX_PROP;
  const fp32 destY = (-dest[0] + dest[1] - dest[2] + dest[3]) * ADUPY_PROP;

  pid_update(&PID_MotX, (int)destX);
  pid_update(&PID_MotY, (int)destY);

  const fp32 SpeedX = PID_MotX.output, SpeedY = PID_MotY.output;

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

  const fp32 vm = PID_MotA.output * CHASSIS_RW / (WHEEL_RADIUS * 6.f);
  Output[0] = -vm, Output[1] = -vm, Output[2] = vm, Output[3] = vm;

  /// 运动解算: mm/s -> rpm X: V(mm/s); Y: V(mm/s); T: 60s/min
  ///
  /// Line Velocity (线速度): mm/s -> rpm
  /// V = X - Y                                 mm/s
  /// V = V * T / (2π * WHEEL_RADIUS)           rpm
  /// SO:
  ///   V = (X - Y) * 30.f / (WHEEL_RADIUS * __PI);
  ///
  /// 单位与 `WHEEL_RADIUS` `CHASSIS_RW` 的单位相同
  Output[0] += (SpeedX - SpeedY) * OPUT_PROP;
  Output[1] += (SpeedX + SpeedY) * OPUT_PROP;
  Output[2] += (SpeedX - SpeedY) * OPUT_PROP;
  Output[3] += (SpeedX + SpeedY) * OPUT_PROP;

  /// 并集 PID 输出
  motor_speed_ctrl(Output);

  /// 位置环稳定后解锁
  if (mutex_cnt < 3) {
    if (fabsf(PID_MotA.output) < 10.f && fabsf(PID_MotX.output) < 5.f &&
        fabsf(PID_MotY.output) < 5.f)
      mutex = 0;
  } else {
    mutex_cnt--;
  }
}
