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

  int parsePacket() {
    if (packets.empty()) return 0;
    remote_ = packets.front().from;
    return static_cast<int>(packets.front().data.size());
  }
  int read(uint8_t* destination, size_t capacity) {
    if (packets.empty()) return 0;
    const std::vector<uint8_t> packet = packets.front().data;
    packets.pop_front();
    const size_t length = std::min(capacity, packet.size());
    std::copy(packet.begin(), packet.begin() + length, destination);
    return static_cast<int>(length);
  }
  IPAddress remoteIP() const { return remote_; }

  // Source defaults to 0.0.0.0, which matches a default-constructed radioIP, so
  // tests that never call begin() are unaffected by the sender-identity guard.
  void receive(const std::vector<uint8_t>& packet) { receive(IPAddress(0), packet); }
  void receive(IPAddress from, const std::vector<uint8_t>& packet) { packets.push_back({from, packet}); }
  size_t writeCalls = 0;
  std::vector<std::vector<uint8_t>> writes;

private:
  struct Datagram { IPAddress from; std::vector<uint8_t> data; };
  std::deque<Datagram> packets;
  IPAddress remote_{0};
};
