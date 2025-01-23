#include "stm32f4xx_hal.h"
#include "type_def.h"

/// 中断优先级
#define PrePriority_GLOBAL 5
#define PrePriority_DMA 4

#define MotPort UART_HandleTypeDef  // Motor X   DIR        TX   RX
#define MotPortA huart2             // Motor A    CW  左前   D5  D6
#define MotPortB huart5             // Motor B    CW  左后   C12 D2
#define MotPortC huart3             // Motor C   CCW  右后   D8  D9
#define MotPortD huart6             // Motor D   CCW  右前   C6  C7

#define UART_DEF_BUILDER(id, x, n)    \
  MotPort MotPort##id;                \
  DMA_HandleTypeDef hdma_##x##n##_rx; \
  DMA_HandleTypeDef hdma_##x##n##_tx;

/// Motor A CW 左上
UART_DEF_BUILDER(A, usart, 2)
/// Motor B CW 左下
UART_DEF_BUILDER(B, uart, 5)
/// Motor C CCW 右下
UART_DEF_BUILDER(C, usart, 3)
/// Motor D CCW 右上
UART_DEF_BUILDER(D, usart, 6)

#define UART_INIT_BUILDER(x, n)                      \
  huart##x.Instance = n;                             \
  huart##x.Init.BaudRate = 512000;                   \
  huart##x.Init.WordLength = UART_WORDLENGTH_8B;     \
  huart##x.Init.StopBits = UART_STOPBITS_1;          \
  huart##x.Init.Parity = UART_PARITY_NONE;           \
  huart##x.Init.Mode = UART_MODE_TX_RX;              \
  huart##x.Init.HwFlowCtl = UART_HWCONTROL_NONE;     \
  huart##x.Init.OverSampling = UART_OVERSAMPLING_16; \
  if (HAL_UART_Init(&huart##x) != HAL_OK) Error_Handler();

#define UART_NVIC_BUILDER(n)                             \
  HAL_NVIC_SetPriority(n##_IRQn, PrePriority_GLOBAL, 0); \
  HAL_NVIC_EnableIRQ(n##_IRQn);

#define DMA_NVIC_BUILDER(dr, dt)                       \
  HAL_NVIC_SetPriority(dr##_IRQn, PrePriority_DMA, 0); \
  HAL_NVIC_EnableIRQ(dr##_IRQn);                       \
  HAL_NVIC_SetPriority(dt##_IRQn, PrePriority_DMA, 0); \
  HAL_NVIC_EnableIRQ(dt##_IRQn);

void motor_uart_init(void) {
  /* U(s)ARTx clock enable */
  __HAL_RCC_UART5_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_USART6_CLK_ENABLE();

  UART_INIT_BUILDER(2, USART2)
  UART_INIT_BUILDER(5, UART5)
  UART_INIT_BUILDER(3, USART3)
  UART_INIT_BUILDER(6, USART6)

  UART_NVIC_BUILDER(USART2)
  UART_NVIC_BUILDER(UART5)
  UART_NVIC_BUILDER(USART3)
  UART_NVIC_BUILDER(USART6)
}

void motor_gpio_init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO clock enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /**USART GPIO Configuration*/
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  /**USART2 GPIO Configuration
  PD5     ------> USART2_TX
  PD6     ------> USART2_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /**UART5 GPIO Configuration
  PC12     ------> UART5_TX
  PD2     ------> UART5_RX
  */
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /**USART3 GPIO Configuration
  PD8     ------> USART3_TX
  PD9     ------> USART3_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /**USART6 GPIO Configuration
  PC6     ------> USART6_TX
  PC7     ------> USART6_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

#define DMA_INIT_BUILDER(x, n, dr, dt, ch)                         \
  hdma_##x##n##_rx.Instance = dr;                                  \
  hdma_##x##n##_rx.Init.Channel = DMA_CHANNEL_##ch;                \
  hdma_##x##n##_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;          \
  hdma_##x##n##_rx.Init.PeriphInc = DMA_PINC_DISABLE;              \
  hdma_##x##n##_rx.Init.MemInc = DMA_MINC_ENABLE;                  \
  hdma_##x##n##_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; \
  hdma_##x##n##_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    \
  hdma_##x##n##_rx.Init.Mode = DMA_NORMAL;                         \
  hdma_##x##n##_rx.Init.Priority = DMA_PRIORITY_HIGH;              \
  hdma_##x##n##_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;           \
  if (HAL_DMA_Init(&hdma_##x##n##_rx) != HAL_OK) Error_Handler();  \
  __HAL_LINKDMA(&huart##n, hdmarx, hdma_##x##n##_rx);              \
  hdma_##x##n##_tx.Instance = dt;                                  \
  hdma_##x##n##_tx.Init.Channel = DMA_CHANNEL_##ch;                \
  hdma_##x##n##_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;          \
  hdma_##x##n##_tx.Init.PeriphInc = DMA_PINC_DISABLE;              \
  hdma_##x##n##_tx.Init.MemInc = DMA_MINC_ENABLE;                  \
  hdma_##x##n##_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE; \
  hdma_##x##n##_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;    \
  hdma_##x##n##_tx.Init.Mode = DMA_NORMAL;                         \
  hdma_##x##n##_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;         \
  hdma_##x##n##_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;           \
  if (HAL_DMA_Init(&hdma_##x##n##_tx) != HAL_OK) Error_Handler();  \
  __HAL_LINKDMA(&huart##n, hdmatx, hdma_##x##n##_tx);

void motor_dma_init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  DMA_INIT_BUILDER(usart, 2, DMA1_Stream5, DMA1_Stream6, 4)
  DMA_INIT_BUILDER(uart, 5, DMA1_Stream0, DMA1_Stream7, 4)
  DMA_INIT_BUILDER(usart, 3, DMA1_Stream1, DMA1_Stream3, 4)
  DMA_INIT_BUILDER(usart, 6, DMA2_Stream2, DMA2_Stream6, 5)

  DMA_NVIC_BUILDER(DMA1_Stream5, DMA1_Stream6)  // UART2
  DMA_NVIC_BUILDER(DMA1_Stream0, DMA1_Stream7)  // UART5
  DMA_NVIC_BUILDER(DMA1_Stream1, DMA1_Stream3)  // UART3
  DMA_NVIC_BUILDER(DMA2_Stream2, DMA2_Stream6)  // UART6
}

#define IRQH_BUILDER(n, f, x) \
  void n##_IRQHandler(void) { HAL_##f##_IRQHandler(&x); }

/* USART2 global interrupt handler */
IRQH_BUILDER(USART2, UART, huart2)
/* DMA1 stream5 global interrupt handler */
IRQH_BUILDER(DMA1_Stream5, DMA, hdma_usart2_rx)
/* DMA1 stream6 global interrupt handler */
IRQH_BUILDER(DMA1_Stream6, DMA, hdma_usart2_tx)

/* UART5 global interrupt handler */
IRQH_BUILDER(UART5, UART, huart5)
/* DMA1 stream0 global interrupt handler */
IRQH_BUILDER(DMA1_Stream0, DMA, hdma_uart5_rx)
/* DMA1 stream7 global interrupt handler */
IRQH_BUILDER(DMA1_Stream7, DMA, hdma_uart5_tx)

/* USART3 global interrupt handler */
IRQH_BUILDER(USART3, UART, huart3)
/* DMA1 stream1 global interrupt handler */
IRQH_BUILDER(DMA1_Stream1, DMA, hdma_usart3_rx)
/* DMA1 stream3 global interrupt handler */
IRQH_BUILDER(DMA1_Stream3, DMA, hdma_usart3_tx)

/* USART6 global interrupt handler */
IRQH_BUILDER(USART6, UART, huart6)
/* DMA2 stream2 global interrupt handler */
IRQH_BUILDER(DMA2_Stream2, DMA, hdma_usart6_rx)
/* DMA2 stream3 global interrupt handler */
IRQH_BUILDER(DMA2_Stream6, DMA, hdma_usart6_tx)

void motor_delay_short(void) { HAL_Delay(1); }
void motor_delay_long(void) { HAL_Delay(20); }
