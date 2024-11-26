#include "board_led.h"
#include "type_def.h"

#define Serial huart1
#define __va(x) __builtin_va_##x

extern UART_HandleTypeDef Serial;
extern void SystemClock_Config(void);
int vsnprintf(char *, unsigned int, const char *, __va(list));

void system_init(void) {
  HAL_Init();

  SystemClock_Config();
}

bool_t uprint(const uint8_t *data, uint8_t len) {
  return HAL_UART_Transmit_DMA(&Serial, data, len);
}

bool_t uprintf(const char *format, ...) {
  if (Serial.hdmatx->Lock) return 1;

  __va(list) arg;
  __va(start)(arg, format);
  static uint8_t buffer[0xFF] = {0};
  uint8_t len = vsnprintf((char *)buffer, sizeof(buffer), format, arg);
  __va(end)(arg);

  return uprint(buffer, len);
}

void Error_Handler(void) {
  BLED_ON();

  __disable_irq();
  for (;;);
}
