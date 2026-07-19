#pragma once

#include "WiFi.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

class WiFiUDP {
public:
  bool begin(uint16_t) { return true; }
  void stop() { packets.clear(); }
  int beginPacket(IPAddress, uint16_t) { return 1; }
  size_t write(const uint8_t* data, size_t length) {
    writeCalls++;
    writes.emplace_back(data, data + length);
    return length;
  }
  int endPacket() { return 1; }

  int parsePacket() { return packets.empty() ? 0 : static_cast<int>(packets.front().size()); }
  int read(uint8_t* destination, size_t capacity) {
    if (packets.empty()) return 0;
    const std::vector<uint8_t> packet = packets.front();
    packets.pop_front();
    const size_t length = std::min(capacity, packet.size());
    std::copy(packet.begin(), packet.begin() + length, destination);
    return static_cast<int>(length);
  }

  void receive(const std::vector<uint8_t>& packet) { packets.push_back(packet); }
  size_t writeCalls = 0;
  std::vector<std::vector<uint8_t>> writes;

private:
  std::deque<std::vector<uint8_t>> packets;
};
