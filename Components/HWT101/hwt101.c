#include "hwt101.h"

#include "stm32f4xx_hal.h"

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
static fp32 angle_g = 0;

bool_t hwt101_read_ok(unsigned timeout) {
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

  while (!hwt101_read_ok(10));

  /// 接收数据, 并过滤掉无效数据
  HAL_UARTEx_ReceiveToIdle(&UART_USED, temp, sizeof(temp), &len, HAL_MAX_DELAY);
  HAL_UARTEx_ReceiveToIdle_DMA(&UART_USED, buffer, sizeof(buffer));
}

fp32 hwt101_get_angle(void) { return angle_g; }

static void hwt101_angle_callback(fp32 angle);

void hwt101_event_callback(uint16_t size) {
  ///  0 ~ 10 : 0x55 0x52 L8 SUM : 角速度 dps
  /// 11 ~ 21 : 0x55 0x53 L8 SUM : 偏航角 d(°)
  HAL_UARTEx_ReceiveToIdle_DMA(&UART_USED, buffer, sizeof(buffer));
  static short temp_a = 0, temp_b = 0;

  uint8_t sum_a = 0x55, sum_b = 0x55;
  for (uint8_t i = 1; i < 10; i++) sum_a += buffer[i];
  for (uint8_t i = 12; i < 21; i++) sum_b += buffer[i];

  if (size != sizeof(buffer) || (sum_a == buffer[10] && sum_b == buffer[21])) {
    temp_a = (short)buffer[7] << 8 | buffer[6];
    temp_b = (short)buffer[18] << 8 | buffer[17];
    if (buffer[12] == 0x53)  // 偏航角: d(°)
      hwt101_angle_callback((fp32)temp_b * 180.f / 32768.f);
    else
      hwt101_angle_callback((fp32)temp_a * 180.f / 32768.f);
  }
}

inline void hwt101_angle_callback(fp32 angle) {
  static fp32 cumulative_angle = 0;
  static fp32 previous_angle = 0;
  cumulative_angle += angle - previous_angle;
  if (angle - previous_angle > 180) cumulative_angle -= 360;
  if (angle - previous_angle < -180) cumulative_angle += 360;
  previous_angle = angle, angle_g = cumulative_angle;
}

/// 串口接收回调函数
__weak void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,
                                       uint16_t Size) {
  if (huart->Instance == huart4.Instance) hwt101_event_callback(Size);
}
