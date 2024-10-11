#include "main.h"

#include "chassis.h"
#include "gpio.h"
#include "hwt101.h"
#include "motor.h"
#include "usart.h"

void chassis_pid_set(float kp, float ki, float kd);

static unsigned char buffer[3];
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == UART4) hwt101_event_callback();
  if (huart->Instance == USART1) {
    /// bit[6]   Kp  Ki  Kd (/1024)  destX destY  (*256) ACCEL
    // chassis_control_dest(buffer[0] * 10, buffer[1] * 10);
    chassis_pid_set((float)buffer[0] / 1024, (float)buffer[1] / 1024,
                    (float)buffer[2] / 1024);

    //!: acceleration
    /// 我想的是加一个控制加速度的算法，
    /// 比如，如果 DestX < PID.target * S
    /// 那么设置小一点的加速度，反之，加速度就设为0
    /// 通过对系数比例S的调节，来控制加速度的大小
    /// 从而精确的控制速度和目标位置
    ///
    /// 尝试之后发现，加速度的调节效果不是很好
    /// 实现之后会发现小车速度不稳定
    /// 并且在位置不同时的效果也不一样，需要重新调整

    //!: Acceleration 新想法
    /// 目前我想使用的是通过比例来控制加速度的大小
    /// 比如，当在一个阈值内时，加速度就设为一个较小的数值(需调试)
    /// 反之，加速度随比例增大，设为较大的数值，从而精确控制小车

    HAL_UART_Receive_IT(huart, buffer, sizeof(buffer));
  }
}
