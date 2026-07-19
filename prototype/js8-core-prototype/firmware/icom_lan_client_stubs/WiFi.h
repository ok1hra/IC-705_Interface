#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#define HEX 16

class IPAddress {
public:
  IPAddress(uint32_t value = 0) : value_(value) {}
  operator uint32_t() const { return value_; }

private:
  uint32_t value_;
};

struct WiFiStub {
  IPAddress localIP() const { return IPAddress(0x3401A8C0u); }
};

extern WiFiStub WiFi;

struct SerialStub {
  template <typename T> void print(const T&) {}
  template <typename T> void print(const T&, int) {}
  template <typename T> void println(const T&) {}
  template <typename T> void println(const T&, int) {}
  void println() {}
};

extern SerialStub Serial;
extern uint32_t testMillis;

inline uint32_t millis() { return testMillis; }
inline uint32_t esp_random() { return 0x12345678u; }

inline size_t strlcpy(char* destination, const char* source, size_t size) {
  const size_t length = std::strlen(source);
  if (size > 0) {
    const size_t copied = std::min(length, size - 1);
    std::memcpy(destination, source, copied);
    destination[copied] = '\0';
  }
  return length;
}
