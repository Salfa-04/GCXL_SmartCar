#include "stm32f4xx_hal.h"

#define __va(x) __builtin_va_##x

extern UART_HandleTypeDef huart1;
extern void SystemClock_Config(void);
int vsnprintf(char *, unsigned int, const char *, __va(list));

void hal_clock_init(void) {
  HAL_Init();
  SystemClock_Config();
}

void uprintf(const char *format, ...) {
  if (huart1.hdmatx->Lock) return;

  __va(list) arg;
  __va(start)(arg, format);
  static uint8_t buffer[0xFF] = {0};
  uint8_t len = vsnprintf((char *)buffer, sizeof(buffer), format, arg);
  __va(end)(arg);

  HAL_UART_Transmit_DMA(&huart1, buffer, len);
}
