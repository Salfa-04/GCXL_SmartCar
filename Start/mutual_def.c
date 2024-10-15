#include "stm32f4xx_hal.h"

void mutual_gpio_init(void);

/// @brief Mutual UART handle
/// !!!串口1
UART_HandleTypeDef U_Mutual;

void mutual_init(void) {
  U_Mutual.Instance = USART1;
  U_Mutual.Init.BaudRate = 115200;
  U_Mutual.Init.WordLength = UART_WORDLENGTH_8B;
  U_Mutual.Init.StopBits = UART_STOPBITS_1;
  U_Mutual.Init.Parity = UART_PARITY_NONE;
  U_Mutual.Init.Mode = UART_MODE_TX_RX;
  U_Mutual.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  U_Mutual.Init.OverSampling = UART_OVERSAMPLING_16;

  mutual_gpio_init();
  HAL_UART_Init(&U_Mutual);
}

void mutual_gpio_init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USART1 clock enable */
  __HAL_RCC_USART1_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**USART1 GPIO Configuration
  PA9     ------> USART1_TX
  PA10     ------> USART1_RX

  PB6     ------> USART1_TX
  PB7     ------> USART1_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
