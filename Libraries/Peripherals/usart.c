#include "usart.h"

UART_HandleTypeDef huart1;
static DMA_HandleTypeDef hdma_usart1_tx;
static DMA_HandleTypeDef hdma_usart1_rx;

void uart1_gpio_init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**USART1 GPIO Configuration
  PA9     ------> USART1_TX
  PA10     ------> USART1_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void uart1_dma_init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* USART1_RX DMA Init */
  hdma_usart1_rx.Instance = DMA2_Stream5;
  hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
  hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart1_rx.Init.Mode = DMA_NORMAL;
  hdma_usart1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_usart1_rx);
  __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

  /* USART1_TX DMA Init */
  hdma_usart1_tx.Instance = DMA2_Stream7;
  hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
  hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart1_tx.Init.Mode = DMA_NORMAL;
  hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_usart1_tx);
  __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

  /* DMA Interrupt Init */
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

void uart1_init(void) {
  /* USART1 clock enable */
  __HAL_RCC_USART1_CLK_ENABLE();

  /* UART1 handler init */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  /* UART1 init, and call UART MSP init */
  uart1_gpio_init(); /* UART1 MSP Init */
  uart1_dma_init();  /* UART1 DMA Init */
  HAL_UART_Init(&huart1);

  /* USART1 interrupt Init */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void) {
  /* USART1 global interrupt handler */
  HAL_UART_IRQHandler(&huart1);
}

void DMA2_Stream5_IRQHandler(void) {
  /* DMA2 stream5 global interrupt handler */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void DMA2_Stream7_IRQHandler(void) {
  /* DMA2 stream7 global interrupt handler */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}
