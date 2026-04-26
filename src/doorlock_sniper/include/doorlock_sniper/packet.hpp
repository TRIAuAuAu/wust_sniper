#ifndef PACKET_HPP_
#define PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>
#include <cstring> 
namespace doorlock_sniper {

struct SerialSendPacket {
  uint16_t length;                        // 有效负载长度 (≤ 320的有效数据)
  uint8_t  data[320];                     // Protobuf 序列化数据
} __attribute__((packed));

inline std::vector<uint8_t> toVector(const SerialSendPacket &pkt) {
  // 计算实际需要发送的字节数：length 字段(2) + 有效数据(length)
  size_t total_size = sizeof(uint16_t) + pkt.length;
  std::vector<uint8_t> vec(total_size);
  std::memcpy(vec.data(), &pkt.length, sizeof(uint16_t));
  std::memcpy(vec.data() + sizeof(uint16_t), pkt.data, pkt.length);
  return vec;
}

} // namespace doorlock_sniper
#endif