#include <type_def.h>

#include "motor.h"

#define CW 0x00   // 电机正转   CW
#define CCW 0x01  // 电机反转   CCW

void motor_data_send(const uint8_t *a, const uint8_t *b, const uint8_t *c,
                     const uint8_t *d, uint8_t len);

static uint8_t MotSpeed[4][8] = {
    {[0] = MotIdA, [1] = 0xF6, [6] = 0x00, [7] = 0x6B},
    {[0] = MotIdB, [1] = 0xF6, [6] = 0x00, [7] = 0x6B},
    {[0] = MotIdC, [1] = 0xF6, [6] = 0x00, [7] = 0x6B},
    {[0] = MotIdD, [1] = 0xF6, [6] = 0x00, [7] = 0x6B},
};

/// 电机速度控制, 单位 rpm, 0 ~ ±5000
void motor_speed_ctrl(int16_t a, int16_t b, int16_t c, int16_t d) {
  // 电机正反设置
  MotSpeed[0][2] = CW, MotSpeed[2][2] = CCW;
  MotSpeed[1][2] = CW, MotSpeed[3][2] = CCW;

  if (a < 0) MotSpeed[0][2] = CCW, a *= -1;
  if (b < 0) MotSpeed[1][2] = CCW, b *= -1;
  if (c < 0) MotSpeed[2][2] = CW, c *= -1;
  if (d < 0) MotSpeed[3][2] = CW, d *= -1;

  // 电机加速度设置
  MotSpeed[0][5] = MotSpeed[1][5] = MotSpeed[2][5] = MotSpeed[3][5] = 0;

  // 电机速度设置
  MotSpeed[0][3] = (a & 0xFF00) >> 8, MotSpeed[0][4] = a & 0xFF;
  MotSpeed[1][3] = (b & 0xFF00) >> 8, MotSpeed[1][4] = b & 0xFF;
  MotSpeed[2][3] = (c & 0xFF00) >> 8, MotSpeed[2][4] = c & 0xFF;
  MotSpeed[3][3] = (d & 0xFF00) >> 8, MotSpeed[3][4] = d & 0xFF;

  motor_data_send(MotSpeed[0], MotSpeed[1], MotSpeed[2], MotSpeed[3],
                  sizeof(MotSpeed[0]));
}
