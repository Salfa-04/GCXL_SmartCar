#include "main.h"

#include "chassis.h"
#include "gpio.h"
#include "hwt101.h"
#include "motor.h"
#include "usart.h"

static unsigned char buffer[5];
static _Bool flag = 0;

int main(void) {
  hal_clock_init();
  led_gpio_init();
  uart1_init();

  uprintf("Hello, world!\r\n");

  chassis_init();

  HAL_Delay(30);
  uprintf("OKOKOKOK\r\n");
  motor_addup_clear();

  HAL_UART_Receive_IT(&huart1, buffer, sizeof(buffer));

  flag = 1;

  for (;;) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(300);
  }

  /* We should never get here */
}

static unsigned char count = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM7) {
    HAL_IncTick();
    if (count++ > 10 && flag) motor_event_callback(), count = 0;
  } /* @type tick = ms */
}

void angle_pid_set(float p, float i, float d);
void dest_pid_set(float p, float i, float d);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == UART4) hwt101_event_callback();
  if (huart->Instance == USART1) {
    // chassis_control_dest(buffer[1] * 10, buffer[2] * 10);
    // chassis_control_angu(buffer[0]);

    // angle_pid_set(buffer[0], buffer[1], buffer[2]);
    // chassis_control_angu(buffer[3]);

    dest_pid_set(buffer[0], buffer[1], buffer[2]);
    chassis_control_dest(buffer[3] * 10, buffer[4] * 10);

    HAL_UART_Receive_IT(huart, buffer, sizeof(buffer));
  }
}
