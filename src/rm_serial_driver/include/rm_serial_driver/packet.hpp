// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct Header
{
  uint8_t header = 0xA5;
  uint16_t data_length = 15;
  uint8_t seq = 0;
  uint8_t crc8 = 0;
  uint16_t cmd_id;
} __attribute__((packed, aligned(1)));

struct IMUPacket
{
  Header header;
  float pitch;
  float roll;
  float yaw;
  // float p;
  // float vp;
  // float y;
  // float vy;
  uint8_t roboid;
  uint8_t id;
  // uint8_t timeseries; //测试时间戳
  uint16_t crc16;
} __attribute__((packed, aligned(1)));

struct classisPacket
{
  Header header;
  float vx;
  float vy;
  float vz;
  uint8_t enable;
  uint16_t crc16;
} __attribute__((packed, aligned(1)));

struct bottonPacket
{
  Header header;
  bool start;
  uint16_t crc16;
} __attribute__((packed, aligned(1)));

struct bassPacket
{
  Header header;
  uint8_t roboid;
  bool reset;
  uint8_t position[2];
  uint8_t HP;
  uint8_t robomode;
  bool canshoot;
  uint8_t shootmode;
  bool superpower;
  bool online;
  bool two;
  uint16_t crc16;
} __attribute__((packed, aligned(1)));

struct ReceivePacket
{
  uint8_t header = 0xA5;
  uint16_t size;
  uint8_t id;
  uint8_t crc8;
  uint16_t cmd_id;
  // uint8_t detect_color : 1;  // 0-red 1-blue
  // bool reset_tracker : 1;
  // uint8_t reserved : 6;
  float pitch;
  float roll;
  float yaw;
  // float aim_x;
  // float aim_y;
  // float aim_z;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket  //0x0402
{
  Header header;
  uint8_t id = 0;
  uint8_t robo_id = 0;
  float pitch;
  float yaw;
  uint8_t accuracy = 50;
  uint8_t shoot;
  uint8_t timeseries;
  uint16_t checksum = 0;
} __attribute__((packed));

// 导航控制指令包 (cmd_id = 0x405)
struct NavigationPacket
{
  Header header;
  float vx;  // 线速度x
  float vy;  // 线速度y
  float wz;  // 角速度z
  uint16_t crc16;
} __attribute__((packed, aligned(1)));

// // 导航状态反馈包 (cmd_id = 0x0505)
// struct NavigationStatePacket
// {
//   Header header;
//   float pitch;   // 俯仰角
//   float yaw;     // 偏航角
//   float
//   uint16_t crc16;
// } __attribute__((packed, aligned(1)));

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
