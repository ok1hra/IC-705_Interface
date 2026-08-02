// icom_lan_discovery.h — find Icom LAN radios on the local /24.
//
// The Icom LAN protocol has no discovery of its own (wfview has none either;
// its discoveredRigID() is CI-V model detection over an already-open link). The
// one unauthenticated primitive it does offer is the control handshake's first
// step, so that is what this sweeps with:
//
//   -> AreYouThere (0x03, 16 bytes) to <host>:50001
//   <- IAmHere     (0x04, 16 bytes) carrying the radio's remoteId
//
// It deliberately stops there. Sending AreYouReady/Login would start a session,
// and the IC-705 accepts exactly one -- a scan that logs in would lock out the
// operator's own wfview, which is a far worse bug than typing an IP by hand.
//
// Non-blocking: start() then tick() from the main loop. The sweep is paced so a
// single loop iteration never emits more than a handful of datagrams; the loop
// this runs in is also carrying JS8/WSPR audio.
//
// Port note: the socket binds 50001 because the radio's reply lands on the port
// it was asked from, and the documented "local port == remote port" deviation
// (docs/icom-lan-implementace.md, deviation #4) makes any other choice a guess.
// WiFiUDP sets SO_REUSEADDR, so binding 50001 while IcomLanClient holds it
// SUCCEEDS and then silently steals its control packets -- the caller must stop
// the LAN client first. That is why the scan is gated, not opportunistic.
#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>
#include "icom_lan_wire.h"

#define ICOM_SCAN_MAX_FOUND      8
#define ICOM_SCAN_PROBES_PER_TICK 8     // ~8 * 16 B per loop pass
#define ICOM_SCAN_LISTEN_MS      2000   // grace after the last probe
#define ICOM_SCAN_PORT           50001

class IcomLanDiscovery {
public:
  enum State : uint8_t { IDLE, SWEEP, LISTEN, DONE };

  struct Found {
    IPAddress ip;
    uint32_t  id;
  };

  // Returns false when there is no usable station link. `localPort` exists so
  // the on-radio experiment "does the radio answer a probe from an ephemeral
  // port?" can be run without editing code; leave it at the default otherwise.
  bool start(uint16_t localPort = ICOM_SCAN_PORT) {
    if (WiFi.status() != WL_CONNECTED) return false;
    IPAddress ip   = WiFi.localIP();
    IPAddress mask = WiFi.subnetMask();
    if ((uint32_t)ip == 0) return false;

    stop();
    if (!udp.begin(localPort)) return false;

    selfIp   = ip;
    myId     = IcomWire::mkId(ip, localPort);
    // Octets are stored little-endian in the packed form (a = oct1..oct4), so
    // the /24 base is the address with its last octet cleared.
    uint32_t a = (uint32_t)ip;
    base3[0] = a & 0xff; base3[1] = (a >> 8) & 0xff; base3[2] = (a >> 16) & 0xff;
    // Anything wider than /24 is only partially covered; say so rather than
    // pretending the sweep was exhaustive.
    truncated = ((uint32_t)mask & 0x00FFFFFFu) != 0x00FFFFFFu;

    foundCount = 0;
    cursor     = 1;
    state      = SWEEP;
    startedAt  = millis();
    listenUntil = 0;
    sendBroadcastProbes();
    return true;
  }

  void stop() {
    if (state != IDLE) udp.stop();
    state = IDLE;
  }

  void tick() {
    if (state == IDLE || state == DONE) return;
    drainReplies();
    if (state == SWEEP) {
      for (uint8_t n = 0; n < ICOM_SCAN_PROBES_PER_TICK && cursor <= 254; n++) {
        IPAddress target(base3[0], base3[1], base3[2], cursor++);
        if (target == selfIp) continue;      // no point probing ourselves
        sendProbe(target);
      }
      if (cursor > 254) {
        state = LISTEN;
        listenUntil = millis() + ICOM_SCAN_LISTEN_MS;
      }
    } else if (state == LISTEN) {
      if ((long)(millis() - listenUntil) >= 0) {
        udp.stop();
        state = DONE;
      }
    }
  }

  State        scanState()  const { return state; }
  bool         busy()       const { return state == SWEEP || state == LISTEN; }
  uint8_t      count()      const { return foundCount; }
  const Found& entry(uint8_t i) const { return found[i]; }
  bool         wasTruncated() const { return truncated; }
  // 0..254 probes issued; the UI turns this into a progress bar.
  uint16_t     progress()   const {
    if (state == IDLE) return 0;
    return state == SWEEP ? cursor : 254;
  }
  void         subnetPrefix(char* out, size_t len) const {
    snprintf(out, len, "%u.%u.%u", base3[0], base3[1], base3[2]);
  }

private:
  void sendProbe(IPAddress target) {
    uint8_t pkt[0x10];
    IcomWire::hdr16(pkt, myId, 0, IcomWire::TYPE_ARE_YOU_THERE, 0);
    udp.beginPacket(target, ICOM_SCAN_PORT);
    udp.write(pkt, sizeof(pkt));
    udp.endPacket();
  }

  // Free shot before the sweep: a radio that honours a broadcast probe answers
  // in ~100 ms. Icom is not documented to do this, so it is a bonus, not a plan.
  void sendBroadcastProbes() {
    sendProbe(IPAddress(255, 255, 255, 255));
    sendProbe(IPAddress(base3[0], base3[1], base3[2], 255));
  }

  void drainReplies() {
    int n;
    while ((n = udp.parsePacket()) > 0) {
      uint8_t r[0x20];
      int got = udp.read(r, sizeof(r));
      if (got < 0x10) continue;
      if (IcomWire::getLE16(r + 4) != IcomWire::TYPE_I_AM_HERE) continue;
      record(udp.remoteIP(), IcomWire::getLE32(r + 8));
    }
  }

  void record(IPAddress ip, uint32_t id) {
    for (uint8_t i = 0; i < foundCount; i++) {
      if (found[i].ip == ip) return;            // broadcast + unicast both reply
    }
    if (foundCount >= ICOM_SCAN_MAX_FOUND) return;
    found[foundCount].ip = ip;
    found[foundCount].id = id;
    foundCount++;
    Serial.print("SCAN| Icom LAN answer from ");
    Serial.println(ip);
  }

  WiFiUDP   udp;
  State     state = IDLE;
  IPAddress selfIp;
  uint8_t   base3[3] = {0, 0, 0};
  uint32_t  myId = 0;
  uint16_t  cursor = 1;
  uint32_t  startedAt = 0, listenUntil = 0;
  bool      truncated = false;
  uint8_t   foundCount = 0;
  Found     found[ICOM_SCAN_MAX_FOUND];
};
