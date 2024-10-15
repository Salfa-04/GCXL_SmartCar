#include "stm32f4xx_hal.h"

#define Serial huart1
#define __va(x) __builtin_va_##x

extern UART_HandleTypeDef Serial;
extern void SystemClock_Config(void);
int vsnprintf(char *, unsigned int, const char *, __va(list));

void hal_clock_init(void) {
  HAL_Init();
  SystemClock_Config();
}

void uprint(const uint8_t *data, uint8_t len) {
  if (Serial.hdmatx->Lock) return;
  HAL_UART_Transmit_DMA(&Serial, data, len);
}

void uprintf(const char *format, ...) {
  if (Serial.hdmatx->Lock) return;

  __va(list) arg;
  __va(start)(arg, format);
  static uint8_t buffer[0xFF] = {0};
  uint8_t len = vsnprintf((char *)buffer, sizeof(buffer), format, arg);
  __va(end)(arg);

  uprint(buffer, len);
}
