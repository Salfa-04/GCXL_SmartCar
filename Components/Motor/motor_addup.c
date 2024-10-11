#include "motor.h"

void motor_data_send(const uint8_t *a, const uint8_t *b, const uint8_t *c,
                     const uint8_t *d, uint8_t len);

void motor_data_read(const uint8_t *ta, const uint8_t *tb, const uint8_t *tc,
                     const uint8_t *td, uint8_t tlen, uint8_t *ra, uint8_t *rb,
                     uint8_t *rc, uint8_t *rd, uint8_t rlen);

static uint8_t MotAupA[8], MotAupB[8], MotAupC[8], MotAupD[8];

static const uint8_t MotCnt[4][3] = {
    {MotIdA, 0x36, 0x6B},
    {MotIdB, 0x36, 0x6B},
    {MotIdC, 0x36, 0x6B},
    {MotIdD, 0x36, 0x6B},
};

static const uint8_t MotCntClr[4][4] = {
    {MotIdA, 0x0A, 0x6D, 0x6B},
    {MotIdB, 0x0A, 0x6D, 0x6B},
    {MotIdC, 0x0A, 0x6D, 0x6B},
    {MotIdD, 0x0A, 0x6D, 0x6B},
};

/// 电机脉冲数获取, 一圈 65536 cnt
void motor_addup_get(int64_t *a, int64_t *b, int64_t *c, int64_t *d) {
  motor_data_read(MotCnt[0], MotCnt[1], MotCnt[2], MotCnt[3], sizeof(MotCnt[0]),
                  MotAupA, MotAupB, MotAupC, MotAupD, sizeof(MotAupA));

  *a = MotAupA[3] << 24 | MotAupA[4] << 16 | MotAupA[5] << 8 | MotAupA[6];
  *b = MotAupB[3] << 24 | MotAupB[4] << 16 | MotAupB[5] << 8 | MotAupB[6];
  *c = MotAupC[3] << 24 | MotAupC[4] << 16 | MotAupC[5] << 8 | MotAupC[6];
  *d = MotAupD[3] << 24 | MotAupD[4] << 16 | MotAupD[5] << 8 | MotAupD[6];

  if (MotAupA[2]) *a = -*a;
  if (MotAupB[2]) *b = -*b;
  if (!MotAupC[2]) *c = -*c;
  if (!MotAupD[2]) *d = -*d;
}

void motor_addup_clear(void) {
  motor_data_send(MotCntClr[0], MotCntClr[1], MotCntClr[2], MotCntClr[3],
                  sizeof(MotCntClr[0]));
}
