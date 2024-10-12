#include "hwt101.h"

#include <stdint.h>

// TX = PC10, RX = PC11
#define UART_USED huart4
void hwt101_uart_init(void);
void hwt101_gpio_init(void);
void hwt101_dma_init(void);
void hwt101_delay_long(void);
void hwt101_delay_short(void);

extern UART_HandleTypeDef UART_USED;
static uint8_t buffer[22], temp[25];
static uint16_t len = 0;

static const uint8_t clear_cmd[5] = {
    0xFF, 0xAA, 0x76, 0x00, 0x00,
};

_Bool hwt101_read_ok(unsigned timeout) {
  uint8_t rx_buffer[24] = {0}, offsize = 0;

  HAL_UART_Receive(&UART_USED, rx_buffer, sizeof(rx_buffer), timeout);
  for (; offsize < sizeof(rx_buffer) - 10; offsize++)
    if (rx_buffer[offsize] == 0x55 && ((rx_buffer[offsize + 1] & ~1) == 0x52))
      return 1;

  return 0;
}

void hwt101_init(void) {
  hwt101_gpio_init();
  hwt101_dma_init();
  hwt101_uart_init();

  hwt101_delay_long();
  HAL_UART_Transmit(&UART_USED, clear_cmd, sizeof(clear_cmd), 6);
  hwt101_delay_long();

  while (!hwt101_read_ok(10));

  /// 接收数据, 并过滤掉无效数据
  HAL_UARTEx_ReceiveToIdle(&UART_USED, temp, sizeof(temp), &len, HAL_MAX_DELAY);
  HAL_UART_Receive_DMA(&UART_USED, buffer, sizeof(buffer));
}

void hwt101_angle_clear(void) {
  // 停止接收 DMA
  HAL_UART_DMAStop(&UART_USED);

  // 发送清零指令
  HAL_UART_Transmit(&UART_USED, clear_cmd, sizeof(clear_cmd), 6);

  // 重新接收数据
  HAL_UARTEx_ReceiveToIdle(&UART_USED, temp, sizeof(temp), &len, HAL_MAX_DELAY);
  HAL_UART_Receive_DMA(&UART_USED, buffer, sizeof(buffer));
}

void hwt101_event_callback(void) {
  ///  0 ~ 10 : 0x55 0x52 L8 SUM : 角速度 dps
  /// 11 ~ 21 : 0x55 0x53 L8 SUM : 偏航角 d(°)
  HAL_UART_Receive_DMA(&UART_USED, buffer, sizeof(buffer));

  uint8_t sum_a = 0x55, sum_b = 0x55;
  for (uint8_t i = 1; i < 10; i++) sum_a += buffer[i];
  for (uint8_t i = 12; i < 21; i++) sum_b += buffer[i];

  if (sum_a == buffer[10] && sum_b == buffer[21]) {
    short temp_a = (short)buffer[7] << 8 | buffer[6];
    short temp_b = (short)buffer[18] << 8 | buffer[17];
    if (buffer[12] == 0x53)  // 偏航角: d(°)
      hwt101_angle_callback((float)temp_b * 180.f / 32768.f);
    else
      hwt101_angle_callback((float)temp_a * 180.f / 32768.f);
  } else {
    HAL_UART_DMAStop(&UART_USED);
    HAL_UARTEx_ReceiveToIdle(&UART_USED, temp, sizeof(temp), &len,
                             HAL_MAX_DELAY);
    HAL_UART_Receive_DMA(&UART_USED, buffer, sizeof(buffer));
  }
}

/// 串口接收回调函数
__weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef *U) {
  if (U->Instance == UART_USED.Instance) hwt101_event_callback();
}

/// 偏航角数据接收回调函数: d(°)
__weak void hwt101_angle_callback(float angle) { (void)angle; }
