#ifndef __PID_H
#define __PID_H

#include "type_def.h"

/// kp: 比例系数
/// ki: 积分系数
/// kd: 微分系数
/// ks: 步进系数
typedef struct PID {
  fp32 kp, ki, kd, ks;
  fp32 error, lastError;
  fp32 integral, maxIntegral;
  fp32 output, maxOutput;
  fp32 step_target, target;
} pid_t;

#ifdef __cplusplus
extern "C" {
#endif

void pid_update(pid_t *pid, fp32 feedback);
void pid_init(pid_t *pid, const fp32 *pids, const fp32 *max);
void pid_clear(pid_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
