#pragma once

#include <stdint.h>

enum Aud1TxState : uint8_t {
  AUD1_TX_IDLE,
  AUD1_TX_READY,
  AUD1_TX_PREBUFFER,
  AUD1_TX_STREAM,
  AUD1_TX_DRAINED,
  AUD1_TX_FAULT
};

inline bool aud1TxNeedsDisconnectAbort(Aud1TxState state, bool pttKeyed) {
  return pttKeyed || state == AUD1_TX_READY || state == AUD1_TX_PREBUFFER ||
         state == AUD1_TX_STREAM;
}
