#include "mutual.h"

/* 通讯交互协议:

! 接收数据格式: 0x07 0x23 0xXX 0xXX 0xXX 0xC8
    帧头: 0x07 0x23
    Data: Len3
    帧尾: 0xC8

    *Data: bit[3]
        0]    类型位 len1
        1|2]  信息位 len2

    *类型位:
        0x01 控制X轴移动
        0x02 控制Y轴移动
        0x03 控制相对角度

    *信息位: bit
    ?    角度: i8[0, angle] : 单位 度(°)
            ! i8 范围: [-128, 127]
            ! 右手坐标系, 逆时针为正
    ?    位移: u8[DispH, DispL] : 单位 cm

! 返回数据格式:
    帧头: 0x07 0x23
    Data: Len1
    帧尾: 0xC8

    *Data: bit
        0] 状态位 len1:
    ?        0x01 成功, 0x02 失败
*/

static uint8_t mutual_data[3] = {0};

//! 请确保单线程调用，避免数据冲突 `static`
//? data: 接收数据 len6
//? ret : 返回数据 len3
///
//* ret: bit
//!   0]   状态位 len1 : 0xFF失败, 0x01位移X方向, 0x02位移Y方向, 0x03角度
//!   1|2] 数据位 len2 : DataH, DataL
uint8_t* mutual_handle(uint8_t* data) {
  mutual_data[0] = mutual_data[1] = mutual_data[2] = 0xFF;

  if (data[0] != 0x07 || data[1] != 0x23 || data[5] != 0xC8) {
    return mutual_data;
  }

  if (data[2] == 0x03 && data[3] != 0) {
    return mutual_data;
  }

  mutual_data[0] = data[2];
  mutual_data[1] = data[3];
  mutual_data[2] = data[4];

  return mutual_data;
}
