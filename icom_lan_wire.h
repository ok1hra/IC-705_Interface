// icom_lan_wire.h — byte-level primitives of the Icom LAN (RS-BA1) protocol.
//
// Split out of icomLanClient.h so the discovery scanner can build a byte-exact
// AreYouThere packet without duplicating the layout. Two independent copies of
// a wire format drift; one of them is then wrong on a radio nobody tests.
//
// Adapted in 2026 from wfview. Copyright 2017-2026 Elliott H. Liggett (W6EL)
// and Phil Taylor (M0VSE). wfview and this modified port are licensed under
// GNU GPL v3. Source: https://gitlab.com/eliggett/wfview/
// See docs/icom-lan-implementace.md for the packet tables.
#pragma once
#include <Arduino.h>
#include <WiFi.h>

namespace IcomWire {

// ---- little/big-endian accessors ----
static inline void putLE16(uint8_t*p,uint16_t v){p[0]=v;p[1]=v>>8;}
static inline void putLE32(uint8_t*p,uint32_t v){p[0]=v;p[1]=v>>8;p[2]=v>>16;p[3]=v>>24;}
static inline void putBE16(uint8_t*p,uint16_t v){p[0]=v>>8;p[1]=v;}
static inline void putBE32(uint8_t*p,uint32_t v){p[0]=v>>24;p[1]=v>>16;p[2]=v>>8;p[3]=v;}
static inline uint16_t getLE16(const uint8_t*p){return p[0]|(p[1]<<8);}
static inline uint32_t getLE32(const uint8_t*p){return (uint32_t)p[0]|(p[1]<<8)|(p[2]<<16)|((uint32_t)p[3]<<24);}
static inline uint16_t getBE16(const uint8_t*p){return (p[0]<<8)|p[1];}

// Session identity: last two octets of our address plus the local port. The
// radio echoes it back, so it only has to be stable within one exchange.
static inline uint32_t mkId(IPAddress localIP, uint16_t localPort) {
  uint32_t a = (uint32_t)localIP;   // stored little-endian (a = oct1..oct4)
  uint8_t o3 = (a >> 16) & 0xff, o4 = (a >> 24) & 0xff;
  return ((uint32_t)o3 << 24) | ((uint32_t)o4 << 16) | (localPort & 0xffff);
}

// The 0x10-byte control header every channel opens with. Returns the length so
// callers can write it straight out.
static inline size_t hdr16(uint8_t* buf, uint32_t myId, uint32_t remoteId,
                           uint16_t type, uint16_t seq) {
  memset(buf, 0, 0x10);
  putLE32(buf + 0, 0x10);
  putLE16(buf + 4, type);
  putLE16(buf + 6, seq);
  putLE32(buf + 8, myId);
  putLE32(buf + 12, remoteId);
  return 0x10;
}

// Control packet types used outside the client itself.
static const uint16_t TYPE_ARE_YOU_THERE = 0x03;
static const uint16_t TYPE_I_AM_HERE     = 0x04;

}  // namespace IcomWire
