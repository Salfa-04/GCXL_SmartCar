#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

TIM_HandleTypeDef htim7;

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
  RCC_ClkInitTypeDef clkconfig;
  uint32_t uwTimclock, uwAPB1Prescaler = 0U;

  uint32_t uwPrescalerValue = 0U;
  uint32_t pFLatency;
  HAL_StatusTypeDef status;

  /* Enable TIM7 clock */
  __HAL_RCC_TIM7_CLK_ENABLE();

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Get APB1 prescaler */
  uwAPB1Prescaler = clkconfig.APB1CLKDivider;
  /* Compute TIM7 clock */
  if (uwAPB1Prescaler == RCC_HCLK_DIV1) {
    uwTimclock = HAL_RCC_GetPCLK1Freq();
  } else {
    uwTimclock = 2UL * HAL_RCC_GetPCLK1Freq();
  }

  /* Compute the prescaler value to have TIM7 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t)((uwTimclock / 1000000U) - 1U);

  /* Initialize TIM7 */
  htim7.Instance = TIM7;

  /* Initialize TIMx peripheral as follow:

  + Period = [(TIM7CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
  htim7.Init.Period = (1000000U / 1000U) - 1U;
  htim7.Init.Prescaler = uwPrescalerValue;
  htim7.Init.ClockDivision = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  status = HAL_TIM_Base_Init(&htim7);
  if (status == HAL_OK) {
    /* Start the TIM time Base generation in interrupt mode */
    status = HAL_TIM_Base_Start_IT(&htim7);
    if (status == HAL_OK) {
      /* Enable the TIM7 global Interrupt */
      HAL_NVIC_EnableIRQ(TIM7_IRQn);
      /* Configure the SysTick IRQ priority */
      if (TickPriority < (1UL << __NVIC_PRIO_BITS)) {
        /* Configure the TIM IRQ priority */
        HAL_NVIC_SetPriority(TIM7_IRQn, TickPriority, 0U);
        uwTickPrio = TickPriority;
      } else {
        status = HAL_ERROR;
      }
    }
  }

  /* Return function status */
  return status;
}

void TIM7_IRQHandler(void) {
  /* Handles TIM7 global interrupt */
  HAL_TIM_IRQHandler(&htim7);
}

void HAL_SuspendTick(void) {
  /* Disable TIM7 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void) {
  /* Enable TIM7 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
}
