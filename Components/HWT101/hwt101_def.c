#include "stm32f4xx_hal.h"

// TX = PC10, RX = PC11
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;

void hwt101_dma_init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* UART4_RX DMA Init */
  hdma_uart4_rx.Instance = DMA1_Stream2;
  hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
  hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_uart4_rx.Init.Mode = DMA_NORMAL;
  hdma_uart4_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

  HAL_DMA_Init(&hdma_uart4_rx);
  __HAL_LINKDMA(&huart4, hdmarx, hdma_uart4_rx);

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}

void hwt101_uart_init(void) {
  /* UART4 handler init */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_RX;  // 仅接收
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;

  /* UART4 init */
  HAL_UART_Init(&huart4);

  /* USART1 interrupt Init */
  HAL_NVIC_SetPriority(UART4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
}

void hwt101_gpio_init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* UART4 clock enable */
  __HAL_RCC_UART4_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**UART4 GPIO Configuration
  PC10     ------> UART4_TX
  PC11     ------> UART4_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void UART4_IRQHandler(void) {
  /* USART1 global interrupt handler */
  HAL_UART_IRQHandler(&huart4);
}

void DMA1_Stream2_IRQHandler(void) {
  /* DMA1 stream2 global interrupt handler */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
}

void hwt101_delay_short(void) { HAL_Delay(1); }
void hwt101_delay_long(void) { HAL_Delay(80); }
