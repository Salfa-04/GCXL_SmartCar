#include "mutual.h"

/* 通讯交互协议:

! 接收数据格式: 0x07 0x23 0xXX 0xXX 0xXX 0xXX 0xXX 0xC8
    帧头: 0x07 0x23
    Data: Len3
    帧尾: 0xC8

    *Data: bit[3]
        0]    类型位 len1
        1|2]  信息位 len2

    *类型位:
        0x00 控制正向旋转 90°
        0x01 控制逆向旋转 90°
        0x02 控制相对坐标系下的绝对位置

    *信息位: bit
    ?    角度模式: 0x00 正向旋转, 0x01 逆向旋转
            ! 0xXX 0xXX 0xXX:
            !   0x00 0x00 0x00 0x00 0x00: 正向旋转
            !   0x01 0x00 0x00 0x00 0x00: 逆向旋转
            ! 右手坐标系, 逆时针为正
    ?    位置模式: [PointX, PointY]: [i16, i16] : 单位 mm

! 返回数据格式:
    帧头: 0x07 0x23
    Data: Len1
    帧尾: 0xC8

    *Data: bit
        0] 状态位 len1:
    ?        0x01 成功, 0x02 失败, 0x03 正在运行
*/

static uint8_t mutual_data[5] = {0};

//! 请确保单线程调用，避免数据冲突 `static`
//? data: 接收数据 len6
//? ret : 返回数据 len3
///
//* ret: bit
//!   0]   状态位 len1 : 0xFF失败,
//!                     0x00正向旋转, 0x01逆向旋转,
//!                     0x02控制相对坐标系下的绝对位置
//!   1|2] 数据位 len2 : DataH, DataL
uint8_t* mutual_handle(uint8_t* data) {
  mutual_data[0] = mutual_data[1] = mutual_data[2] = mutual_data[3] =
      mutual_data[4] = 0xFF;

  if (data[0] != 0x07 || data[1] != 0x23 || data[7] != 0xC8)
  /// 判断帧头帧尾
  {
    return mutual_data;
  }

  if ((data[2] == 0x00 || data[2] == 0x01) &&
      (data[3] != 0 || data[4] != 0 || data[5] != 0 || data[6] != 0))
  /// 判断数据位
  {
    return mutual_data;
  }

  if (data[2] > 0x02) {
    return mutual_data;
  }

  mutual_data[0] = data[2];
  mutual_data[1] = data[3];
  mutual_data[2] = data[4];
  mutual_data[3] = data[5];
  mutual_data[4] = data[6];

  return mutual_data;
}
