#include "main.h"

// #define DEBUG_MODE
// #define DEBUG_MOT

#include "board_led.h"
#include "chassis.h"
#include "hwt101.h"
#include "mutual.h"
#include "type_def.h"
#include "usart.h"

//! bit2]: 1成功 2失败
static uint8_t mutual_buff[4] = {
    [0] = 0x07,
    [1] = 0x23,
    [3] = 0xC8,
};

static uint8_t buffer[8] = {0}, temp[64] = {0};
extern UART_HandleTypeDef huart4;
static bool_t is_start = 0;

#ifdef DEBUG_MOT

#define MotPort UART_HandleTypeDef  // Motor X   DIR        TX   RX
#define MotPortA huart2             // Motor A    CW  左前   D5  D6
#define MotPortB huart5             // Motor B    CW  左后   C12 D2
#define MotPortC huart3             // Motor C   CCW  右后   D8  D9
#define MotPortD huart6             // Motor D   CCW  右前   C6  C7

extern MotPort MotPortA;
extern MotPort MotPortB;
extern MotPort MotPortC;
extern MotPort MotPortD;

#define MotUsed MotPortC
uint16_t host_len = 0, mot_len = 0;

#endif

int main(void) {
  system_init();
  bled_gpio_init();
  uart1_init();

#ifdef DEBUG_MOT
  {
    void motor_uart_init(void);
    void motor_gpio_init(void);
    void motor_dma_init(void);

    motor_gpio_init();
    motor_dma_init();
    motor_uart_init();

    static uint8_t temp1[64] = {0}, temp2[64] = {0};

    HAL_UARTEx_ReceiveToIdle_IT(&huart1, temp1, sizeof(temp1));
    HAL_UARTEx_ReceiveToIdle_IT(&MotUsed, temp2, sizeof(temp2));

    for (;;) {
      if (host_len) {
        HAL_UART_Transmit(&MotUsed, temp1, host_len, HAL_MAX_DELAY);
        // HAL_UART_Transmit(&huart1, temp1, host_len, HAL_MAX_DELAY);

        HAL_UARTEx_ReceiveToIdle_IT(&huart1, temp1, sizeof(temp1));
        host_len = 0;
      }

      if (mot_len) {
        HAL_UART_Transmit(&huart1, temp2, mot_len, HAL_MAX_DELAY);
        HAL_UARTEx_ReceiveToIdle_IT(&MotUsed, temp2, sizeof(temp2));
        mot_len = 0;
      }
    }
  }
#endif

  chassis_init();

  HAL_Delay(30);

  BLED_OFF();
  is_start = 1;

#ifdef DEBUG_MODE
  {
    static int16_t x = 0, y = 0, a = 0;

    for (;;) {
      chassis_control_point(x, y, a);
    }
  }
#endif

  for (;;) {
    const int16_t* data = getp_mutual();

    for (;;) {
      uint16_t len = 0;
      HAL_UARTEx_ReceiveToIdle(&huart1, buffer, sizeof(buffer), &len, 7);
      mutual_handle(buffer);  // get data

      if (len == 0) continue;
      if (len == sizeof(buffer)) break;

      mutual_buff[2] = 0x02;  // 失败
      uprint(mutual_buff, sizeof(mutual_buff));
    }

    //!   0]   状态位 len1 : 0xFF失败,
    //!                     0x00正向旋转, 0x01逆向旋转,
    //!                     0x02控制相对坐标系下的绝对位置
    //!   1|2] 数据位 len2 : DataH, DataL

    if (data == NULL) continue;

    mutual_buff[2] = 0x01;  // 成功
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, temp, sizeof(temp));

    if (data[0] == 0x80) {  // 相对模式
      chassis_control_dest(data[1], data[2]);
    } else {  // 坐标模式
      chassis_control_point(data[1], data[2], data[0]);
    }

    /// 为避免数据竞争, 请单线程使用 `mutual_buff` 变量
    HAL_UART_DMAStop(&huart1);
    uprint(mutual_buff, sizeof(mutual_buff));
  }

  /* We should never get here */
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
#ifdef DEBUG_MOT
  {
    if (huart->Instance == USART1) {
      host_len = Size;
    } else if (huart->Instance == MotUsed.Instance) {
      mot_len = Size;
    }

    return;
  }
#endif

  if (huart->Instance == USART1) {
    static const uint8_t error_cmd[4] = {0x07, 0x23, 0x03, 0xC8};
    uprint(error_cmd, sizeof(error_cmd));  // 发送未完成命令
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, temp, sizeof(temp));
  } else if (huart->Instance == UART4)
    hwt101_event_callback(Size);
}

static unsigned char count = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM7) {
    HAL_IncTick();
    if (++count > 10 && is_start) motor_event_callback(), count = 0;
  } /* @type tick = ms */
}
