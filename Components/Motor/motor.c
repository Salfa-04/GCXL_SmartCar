#include "motor.h"

#include "type_def.h"

//   UART-X       TX   RX
//   huart2       D5  D6
//   huart5       C12 D2
//   huart3       D8  D9
//   huart6       C6  C7

#define MotPortA huart2  // Motor A    CW  左前
#define MotPortB huart5  // Motor B    CW  左后
#define MotPortC huart3  // Motor C   CCW  右后
#define MotPortD huart6  // Motor D   CCW  右前

/// 使用上位机更改设置：
/// 电机控制模式        FOC
/// 通讯端口复用        UART
/// 串口波特率          512000
/// 通讯校验方式        0x6B
/// 堵转保护、细分插补  关闭
/// 自动息屏           开启

extern MotPort MotPortA;
extern MotPort MotPortB;
extern MotPort MotPortC;
extern MotPort MotPortD;

void motor_uart_init(void);
void motor_gpio_init(void);
void motor_dma_init(void);
void motor_delay_short(void);
void motor_delay_long(void);

bool_t motor_wait_ok(MotPort *mot, uint8_t id, unsigned timeout) {
  uint8_t rx_buffer[24], offsize = 0;
  uint8_t tx_buffer[3] = {id, 0x1F, 0x6B};
  HAL_UART_Transmit(mot, tx_buffer, sizeof(tx_buffer), timeout);
  HAL_UART_Receive(mot, rx_buffer, sizeof(rx_buffer), timeout);
  for (; offsize < sizeof(rx_buffer) - 4; offsize++)
    if (rx_buffer[offsize] == id && rx_buffer[offsize + 1] == 0x1F &&
        rx_buffer[offsize + 4] == 0x6B)
      return 1;

  return 0;
}

void motor_init(void) {
  motor_gpio_init();
  motor_dma_init();
  motor_uart_init();

  motor_delay_long();

  // 等待上线
  while (!motor_wait_ok(&MotPortA, MotIdA, 10)) motor_delay_long();
  while (!motor_wait_ok(&MotPortB, MotIdB, 10)) motor_delay_long();
  while (!motor_wait_ok(&MotPortC, MotIdC, 10)) motor_delay_long();
  while (!motor_wait_ok(&MotPortD, MotIdD, 10)) motor_delay_long();
}

/// 发送数据到电机
void motor_data_send(const uint8_t *a, const uint8_t *b, const uint8_t *c,
                     const uint8_t *d, uint8_t len) {
  if (MotPortA.hdmatx->Lock || MotPortA.hdmarx->Lock)
    HAL_UART_DMAStop(&MotPortA);
  if (MotPortB.hdmatx->Lock || MotPortB.hdmarx->Lock)
    HAL_UART_DMAStop(&MotPortB);
  if (MotPortC.hdmatx->Lock || MotPortC.hdmarx->Lock)
    HAL_UART_DMAStop(&MotPortC);
  if (MotPortD.hdmatx->Lock || MotPortD.hdmarx->Lock)
    HAL_UART_DMAStop(&MotPortD);

  HAL_UART_Transmit_DMA(&MotPortA, a, len);
  HAL_UART_Transmit_DMA(&MotPortB, b, len);
  HAL_UART_Transmit_DMA(&MotPortC, c, len);
  HAL_UART_Transmit_DMA(&MotPortD, d, len);
}

void motor_data_read(const uint8_t *ta, const uint8_t *tb, const uint8_t *tc,
                     const uint8_t *td, uint8_t tlen, uint8_t *ra, uint8_t *rb,
                     uint8_t *rc, uint8_t *rd, uint8_t rlen) {
  if (MotPortA.hdmatx->Lock || MotPortA.hdmarx->Lock)
    HAL_UART_DMAStop(&MotPortA);
  if (MotPortB.hdmatx->Lock || MotPortB.hdmarx->Lock)
    HAL_UART_DMAStop(&MotPortB);
  if (MotPortC.hdmatx->Lock || MotPortC.hdmarx->Lock)
    HAL_UART_DMAStop(&MotPortC);
  if (MotPortD.hdmatx->Lock || MotPortD.hdmarx->Lock)
    HAL_UART_DMAStop(&MotPortD);

  HAL_UART_Transmit_DMA(&MotPortA, ta, tlen);
  HAL_UART_Receive_DMA(&MotPortA, ra, rlen);
  HAL_UART_Transmit_DMA(&MotPortB, tb, tlen);
  HAL_UART_Receive_DMA(&MotPortB, rb, rlen);
  HAL_UART_Transmit_DMA(&MotPortC, tc, tlen);
  HAL_UART_Receive_DMA(&MotPortC, rc, rlen);
  HAL_UART_Transmit_DMA(&MotPortD, td, tlen);
  HAL_UART_Receive_DMA(&MotPortD, rd, rlen);

  /// 等待接收完成
  while (MotPortA.hdmarx->Lock != HAL_UNLOCKED &&
         MotPortB.hdmarx->Lock != HAL_UNLOCKED &&
         MotPortC.hdmarx->Lock != HAL_UNLOCKED &&
         MotPortD.hdmarx->Lock != HAL_UNLOCKED);
}
