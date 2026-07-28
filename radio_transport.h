#pragma once

#include <stdint.h>

// Numeric values intentionally preserve the legacy TRX2/TRX3 EEPROM encoding:
// 0 = TrxNet, 1 = serial CI-V. LAN is the new third value.
enum RadioTransport : uint8_t {
  RADIO_TRXNET = 0,
  RADIO_CIV    = 1,
  RADIO_LAN    = 2
};

enum RadioCapability : uint8_t {
  RADIO_CAP_FREQUENCY = 1u << 0,
  RADIO_CAP_MODE      = 1u << 1,
  RADIO_CAP_TUNE      = 1u << 2,
  RADIO_CAP_FULL_CAT  = 1u << 3,
  RADIO_CAP_AUDIO     = 1u << 4
};

static inline uint8_t radioCapabilities(uint8_t slot, RadioTransport transport) {
  const uint8_t limited = RADIO_CAP_FREQUENCY | RADIO_CAP_MODE | RADIO_CAP_TUNE;
  if (transport == RADIO_TRXNET) return limited;
  // LAN brings its own CI-V and audio sub-streams, so it is fully capable in
  // whichever slot the operator gave it -- the LAN radio is not required to be
  // TRX1. Serial CI-V is polled thinly (frequency/mode only) outside slot 0,
  // which is exactly where its capabilities stop.
  if (transport == RADIO_LAN) return limited | RADIO_CAP_FULL_CAT | RADIO_CAP_AUDIO;
  if (slot == 0) return limited | RADIO_CAP_FULL_CAT;
  return limited;
}

static inline bool radioHasCapability(uint8_t slot, RadioTransport transport,
                                      RadioCapability capability) {
  return (radioCapabilities(slot, transport) & (uint8_t)capability) != 0;
}

static inline const char* radioTransportName(RadioTransport transport) {
  switch (transport) {
    case RADIO_LAN:    return "lan";
    case RADIO_CIV:    return "civ";
    case RADIO_TRXNET:
    default:           return "trxnet";
  }
}

static inline RadioTransport radioTransportFromName(const char* value,
                                                    RadioTransport fallback) {
  if (!value) return fallback;
  if (value[0] == 'l' || value[0] == 'L') return RADIO_LAN;
  if (value[0] == 'c' || value[0] == 'C') return RADIO_CIV;
  if (value[0] == 't' || value[0] == 'T') return RADIO_TRXNET;
  return fallback;
}

static inline uint16_t radioLanLocalControlPort(uint8_t slot) {
  return (uint16_t)(50001u + (uint16_t)slot * 10u);
}
