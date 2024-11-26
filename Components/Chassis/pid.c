#include "pid.h"

/// PID 初始化
/// pid: PID 结构体指针
/// p: 比例系数, i: 积分系数, d: 微分系数, s: 步进系数
/// mI: 最大积分, mO: 最大输出
///
/// s 为 0 时, 不使用步进模式; s 不为 0 时, 使用步进模式
///
/// 步进模式下, 用户设置 pid->target,
/// 内部会逐步逼近 target, 逼近速度为 ks
///
void pid_init(pid_t *pid, fp32 p, fp32 i, fp32 d, fp32 s, fp32 mI, fp32 mO) {
  pid->kp = p, pid->ki = i, pid->kd = d, pid->ks = s;
  pid->maxIntegral = mI, pid->maxOutput = mO;
  pid->step_target = pid->target = 0;
}

void pid_update(pid_t *pid, fp32 feedback) {
  // update step target
  if (!pid->ks) {
    pid->step_target = pid->target;
  } else if (pid->step_target > pid->target) {
    pid->step_target -= pid->ks;
    if (pid->step_target < pid->target) pid->step_target = pid->target;
  } else if (pid->step_target < pid->target) {
    pid->step_target += pid->ks;
    if (pid->step_target > pid->target) pid->step_target = pid->target;
  }

  // update error and last error
  pid->lastError = pid->error;
  pid->error = pid->step_target - feedback;

  // update p, i, d outputs
  fp32 p_out = pid->error * pid->kp;
  fp32 i_out = pid->error * pid->ki;
  fp32 d_out = (pid->error - pid->lastError) * pid->kd;

  pid->integral += i_out;
  if (pid->integral > pid->maxIntegral)
    pid->integral = pid->maxIntegral;
  else if (pid->integral < -(pid->maxIntegral))
    pid->integral = -(pid->maxIntegral);

  pid->output = p_out + d_out + pid->integral;
  if (pid->output > pid->maxOutput)
    pid->output = pid->maxOutput;
  else if (pid->output < -(pid->maxOutput))
    pid->output = -(pid->maxOutput);
}

void pid_clear(pid_t *pid) {
  pid->error = pid->lastError = 0;
  pid->integral = pid->output = 0;
  pid->step_target = pid->target = 0;
}
