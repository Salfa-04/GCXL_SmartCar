#include "main.h"

#include "board_led.h"
#include "chassis.h"
#include "hwt101.h"
#include "motor.h"
#include "mutual.h"
#include "usart.h"

//! bit2]: 1成功 2失败
static uint8_t mutual_buff[4] = {
    [0] = 0x07,
    [1] = 0x23,
    [3] = 0xC8,
};

static uint8_t buffer[6] = {0}, temp[64] = {0};
extern UART_HandleTypeDef huart4;
static _Bool is_start = 0;

int main(void) {
  hal_clock_init();
  bled_gpio_init();
  uart1_init();
  // uprintf("Hello, world!\r\n");

  chassis_init();

  HAL_Delay(30);
  // uprintf("OKOKOKOK\r\n");
  motor_addup_clear();

  BLED_OFF();
  is_start = 1;

  for (;;) {
    uint8_t* data = NULL;

    for (;;) {
      uint16_t len = 0;
      HAL_UARTEx_ReceiveToIdle(&huart1, buffer, sizeof(buffer), &len, 7);
      data = mutual_handle(buffer);

      if (len == 0) continue;
      if (len == sizeof(buffer) && data[0] <= 0x03) break;

      mutual_buff[2] = 0x02;  // 失败
      uprint(mutual_buff, sizeof(mutual_buff));
    }

    if (data == NULL) continue;

    //!   0]   状态位 len1 : 0xFF失败,
    //!                     0x01位移X方向, 0x02位移Y方向
    //!                     0x03角度, 0x00控制小幅度位移
    //!   1|2] 数据位 len2 : DataH, DataL

    mutual_buff[2] = 0x01;  // 成功
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, temp, sizeof(temp));

    if (data[0] == 0x00) {
      chassis_control_dest((int8_t)data[1], (int8_t)data[2]);
    }

    if (data[0] == 0x01) {
      int16_t disp = (int16_t)data[1] << 8 | data[2];
      chassis_control_dest(disp, 0);
    }

    if (data[0] == 0x02) {
      int16_t disp = (int16_t)data[1] << 8 | data[2];
      chassis_control_dest(0, disp);
    }

    if (data[0] == 0x03) {
      int8_t angle = data[2];
      chassis_control_angu(angle);
    }

    /// 为避免数据竞争, 请单线程使用 `mutual_buff` 变量
    HAL_UART_DMAStop(&huart1);
    uprint(mutual_buff, sizeof(mutual_buff));
  }

  /* We should never get here */
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
  if (huart->Instance == USART1) {
    static uint8_t error_cmd[4] = {0x07, 0x23, 0x03, 0xC8};
    uprint(error_cmd, sizeof(error_cmd));  // 发送未完成命令
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, temp, sizeof(temp));
  } else if (huart->Instance == huart4.Instance)
    hwt101_event_callback(Size);
}

static unsigned char count = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM7) {
    HAL_IncTick();
    if (++count > 10 && is_start) motor_event_callback(), count = 0;
  } /* @type tick = ms */
}

void Error_Handler(void) {
  BLED_ON();
  __disable_irq();

  for (;;);
}
