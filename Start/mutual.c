#include "mutual.h"

/* 通讯交互协议:

! 接收数据格式: 0x07 0x23 0xXX 0xXX 0xXX 0xXX 0xXX 0xC8
    帧头: 0x07 0x23
    Data: Len5
    帧尾: 0xC8

    *Data: bit[5]
        0]    角度和模式位 len1
        1|2]  X 位        len2
        3|4]  Y 位        len2

    *Data 数据位:
        Data[0] & 0x80: 0, 坐标模式; 1, 相对模式

! 返回数据格式:
    帧头: 0x07 0x23
    Data: Len1
    帧尾: 0xC8

    *Data: bit
        0] 状态位 len1:
    ?        0x01 成功, 0x02 失败, 0x03 正在运行
*/

//! 请确保单线程调用，避免数据冲突 `static`
//? data: 接收数据 len8
//? ret : 返回数据 len4
///
//* ret: bit, len4[u16]
//!   0]    角度和模式位    len1
//!   1]    X 位           len1
//!   2]    Y 位           len1
int16_t* mutual_handle(uint8_t* data) {
  static int16_t mutual_data[3] = {0};

  mutual_data[0] = mutual_data[1] = mutual_data[2] = 0;

  if (data[0] != 0x07 || data[1] != 0x23 || data[7] != 0xC8)
  /// 判断帧头帧尾
  {
    return mutual_data;
  }

  if (data[2] & 0x80) {  // 相对模式
    mutual_data[0] = 0x10;
  } else {  // 坐标模式
    mutual_data[0] = data[2];
  }

  mutual_data[1] = (int16_t)(((int16_t)data[3] << 8) | data[4]);
  mutual_data[2] = (int16_t)(((int16_t)data[5] << 8) | data[6]);

  return mutual_data;
}
