#ifndef __BOARD_LED_H
#define __BOARD_LED_H

#include "stm32f4xx_hal.h"

void bled_gpio_init(void);

#define BLED_ON() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define BLED_OFF() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

#endif /* __BOARD_LED_H */
