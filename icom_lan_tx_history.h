#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Small bounded replay buffer for the reliability layer used by Icom LAN UDP.
// The radio requests a missing tracked sequence with a 0x01 control packet; the
// response must be the byte-identical original packet, without allocating a new
// sequence number.
template <size_t MaxPacketSize, size_t Capacity>
class IcomLanTxHistory {
 public:
  IcomLanTxHistory() { clear(); }

  void clear() {
    next_ = 0;
    for (size_t i = 0; i < Capacity; ++i) entries_[i].valid = false;
  }

  bool remember(uint16_t sequence, const uint8_t* packet, size_t length) {
    if (!packet || length == 0 || length > MaxPacketSize || Capacity == 0)
      return false;
    Entry& entry = entries_[next_];
    entry.sequence = sequence;
    entry.length = length;
    memcpy(entry.packet, packet, length);
    entry.valid = true;
    next_ = (next_ + 1) % Capacity;
    return true;
  }

  const uint8_t* find(uint16_t sequence, size_t& length) const {
    for (size_t i = 0; i < Capacity; ++i) {
      const Entry& entry = entries_[i];
      if (entry.valid && entry.sequence == sequence) {
        length = entry.length;
        return entry.packet;
      }
    }
    length = 0;
    return nullptr;
  }

  const uint8_t* find(uint16_t sequence) const {
    size_t ignored = 0;
    return find(sequence, ignored);
  }

 private:
  struct Entry {
    uint16_t sequence;
    size_t length;
    uint8_t packet[MaxPacketSize];
    bool valid;
  };

  Entry entries_[Capacity];
  size_t next_;
};
