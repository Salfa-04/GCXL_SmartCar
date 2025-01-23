#include "motor.h"

static uint8_t MotAupA[8] = {0};
static uint8_t MotAupB[8] = {0};
static uint8_t MotAupC[8] = {0};
static uint8_t MotAupD[8] = {0};

void motor_data_send(const uint8_t *a, const uint8_t *b, const uint8_t *c,
                     const uint8_t *d, uint8_t len);

void motor_data_read(const uint8_t *ta, const uint8_t *tb, const uint8_t *tc,
                     const uint8_t *td, uint8_t tlen, uint8_t *ra, uint8_t *rb,
                     uint8_t *rc, uint8_t *rd, uint8_t rlen);

static const uint8_t MotCnt[4][3] = {
    {MotIdA, 0x36, 0x6B},
    {MotIdB, 0x36, 0x6B},
    {MotIdC, 0x36, 0x6B},
    {MotIdD, 0x36, 0x6B},
};

/// 电机脉冲数获取, 一圈 65536 cnt
///
///! 注意, 电机最大值溢出之后底盘可能会疯跑乱跑
///! 注意, 当底盘行为异常的时候请检查电机线路是否正常
void motor_addup_get(fp32 *dest) {  // destX len 4
  fp32 AdupA = 0.f, AdupB = 0.f, AdupC = 0.f, AdupD = 0.f;

  motor_data_read(MotCnt[0], MotCnt[1], MotCnt[2], MotCnt[3], sizeof(MotCnt[0]),
                  MotAupA, MotAupB, MotAupC, MotAupD, sizeof(MotAupA));

  AdupA = MotAupA[3] << 24 | MotAupA[4] << 16 | MotAupA[5] << 8 | MotAupA[6];
  AdupB = MotAupB[3] << 24 | MotAupB[4] << 16 | MotAupB[5] << 8 | MotAupB[6];
  AdupC = MotAupC[3] << 24 | MotAupC[4] << 16 | MotAupC[5] << 8 | MotAupC[6];
  AdupD = MotAupD[3] << 24 | MotAupD[4] << 16 | MotAupD[5] << 8 | MotAupD[6];

  if (MotAupA[2]) AdupA = -AdupA;
  if (MotAupB[2]) AdupB = -AdupB;
  if (!MotAupC[2]) AdupC = -AdupC;
  if (!MotAupD[2]) AdupD = -AdupD;

  dest[0] = AdupA, dest[1] = AdupB, dest[2] = AdupC, dest[3] = AdupD;
}
