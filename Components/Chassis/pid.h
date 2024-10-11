#ifndef __PID_H
#define __PID_H

typedef struct PID {
  float kp, ki, kd;
  float error, lastError;
  float integral, maxIntegral;
  float output, maxOutput;
  float target;
} pid_t;

void pid_update(pid_t *pid, float feedback);
void pid_init(pid_t *pid, float p, float i, float d, float maxI, float maxOut);
void pid_clear(pid_t *pid);

#endif /* __PID_H */
