#ifndef PACKET_HPP_
#define PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace doorlock_sniper {

// 固定大小的串口发送包
struct SerialSendPacket {
  uint8_t data[320]; // 用于存放 Protobuf 序列化数据 + 尾部填充 0x00
} __attribute__((packed));

// 将结构体转换为可发送的 vector（固定长度）
inline std::vector<uint8_t> toVector(const SerialSendPacket &pkt) {
  std::vector<uint8_t> vec(sizeof(SerialSendPacket));
  std::memcpy(vec.data(), pkt.data, sizeof(SerialSendPacket));
  return vec;
}

} // namespace doorlock_sniper

#endif // PACKET_HPP_