#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#define private public
#include "../../../icomLanClient.h"
#undef private

WiFiStub WiFi;
SerialStub Serial;
uint32_t testMillis = 0;

static int parsedFrames = 0;
void lanCivFrameHandler(const uint8_t*, size_t) { parsedFrames++; }
void lanAudioHandler(const uint8_t*, size_t, uint16_t) {}

static void putLE16(std::vector<uint8_t>& packet, size_t offset, uint16_t value) {
  packet[offset] = value;
  packet[offset + 1] = value >> 8;
}

static void putLE32(std::vector<uint8_t>& packet, size_t offset, uint32_t value) {
  packet[offset] = value;
  packet[offset + 1] = value >> 8;
  packet[offset + 2] = value >> 16;
  packet[offset + 3] = value >> 24;
}

static void putBE16(std::vector<uint8_t>& packet, size_t offset, uint16_t value) {
  packet[offset] = value >> 8;
  packet[offset + 1] = value;
}

static std::vector<uint8_t> controlPingReply() {
  std::vector<uint8_t> packet(0x15, 0);
  putLE32(packet, 0, packet.size());
  putLE16(packet, 4, 0x07);
  packet[0x10] = 0x01;
  return packet;
}

static std::vector<uint8_t> controlPacket(uint16_t type, uint16_t sequence,
                                          uint32_t sentId = 0x12345678) {
  std::vector<uint8_t> packet(0x10, 0);
  putLE32(packet, 0, packet.size());
  putLE16(packet, 4, type);
  putLE16(packet, 6, sequence);
  putLE32(packet, 8, sentId);
  return packet;
}

static std::vector<uint8_t> retransmitRange(uint16_t first, uint16_t last) {
  std::vector<uint8_t> packet(0x18, 0);
  putLE32(packet, 0, packet.size());
  putLE16(packet, 4, 0x01);
  putLE16(packet, 0x10, first);
  putLE16(packet, 0x12, last);
  // The protocol duplicates retransmit requests on the wire.
  putLE16(packet, 0x14, first);
  putLE16(packet, 0x16, last);
  return packet;
}

static std::vector<uint8_t> civPacket(const std::vector<uint8_t>& frame) {
  std::vector<uint8_t> packet(0x15 + frame.size(), 0);
  putLE32(packet, 0, packet.size());
  putLE16(packet, 4, 0x00);
  packet[0x10] = 0xC1;
  putLE16(packet, 0x11, frame.size());
  std::copy(frame.begin(), frame.end(), packet.begin() + 0x15);
  return packet;
}

static std::vector<uint8_t> controlStatus(bool disconnected, uint16_t civPort) {
  std::vector<uint8_t> packet(0x50, 0);
  putLE32(packet, 0, packet.size());
  packet[0x40] = disconnected ? 1 : 0;
  putBE16(packet, 0x42, civPort);
  putBE16(packet, 0x46, 50003);
  return packet;
}

static std::vector<uint8_t> authResponse(uint32_t error) {
  std::vector<uint8_t> packet(0x40, 0);
  putLE32(packet, 0, packet.size());
  packet[0x14] = 0x02;
  packet[0x15] = 0x05;
  putLE32(packet, 0x30, error);
  return packet;
}

static std::vector<uint8_t> loginResponse(uint16_t request, uint32_t error) {
  std::vector<uint8_t> packet(0x60, 0);
  putLE32(packet, 0, packet.size());
  putLE16(packet, 0x1A, request);
  putLE32(packet, 0x1C, 0x12345678);
  putLE32(packet, 0x30, error);
  return packet;
}

// Advance the clock the way a HEALTHY loop would: many short, serviced ticks so
// the client's loop-stall compensation credits the elapsed time as genuine link
// silence (a single big jump would look like an unserviced stall and be
// forgiven). Keeps the control channel fresh, as the radio's pings would, so
// these CI-V-only tests isolate CAT health from the control-loss watchdog.
static void idleHealthy(IcomLanClient& client, uint32_t ms) {
  uint32_t target = testMillis + ms;
  while (testMillis < target) {
    uint32_t remaining = target - testMillis;
    testMillis += remaining < 100 ? remaining : 100;
    // Source the keepalive from the configured radio so the sender-identity
    // guard accepts it (radioIP defaults to 0, matching packets injected as 0).
    client.ctrlUdp.receive(client.radioIP, controlPingReply());
    client.loop();
    if (client.failed()) return;
  }
}

int main() {
  bool ok = true;
  {
    IcomLanClient client;
    client.state = IcomLanClient::LAN_CONNECTED;
    client.civPort = 50002;
    client.civGotHere = true;
    client.civGotReady = true;
    client.civOpenSent = true;
    client.civGotData = true;
    client.lastCtrlRxMs = 0;
    client.lastCivDataMs = 0;

    // The control channel is alive, but CI-V has delivered nothing for 7 s.
    // First probe a known-supported command on the existing stream. Several
    // unsupported telemetry reads may legitimately produce a >2 s reply gap;
    // reopening immediately would create a permanent CAT silent/restored loop.
    testMillis = 7000;
    client.ctrlUdp.receive(controlPingReply());
    client.loop();
    bool resentOpen = false;
    bool sentFrequencyProbe = false;
    const std::vector<uint8_t> frequencyProbe = {
        0xFE, 0xFE, 0xA4, 0xE1, 0x03, 0xFD};
    for (const auto& packet : client.civUdp.writes) {
      if (packet.size() == 0x16 && packet[0x15] == 0x05) resentOpen = true;
      if (std::search(packet.begin(), packet.end(), frequencyProbe.begin(),
                      frequencyProbe.end()) != packet.end()) {
        sentFrequencyProbe = true;
      }
    }
    if (client.failed() || resentOpen || !sentFrequencyProbe) {
      std::fprintf(stderr, "CAT silence did not use a non-destructive frequency probe first\n");
      ok = false;
    }

    // Only if that liveness probe also times out may recovery reopen the CI-V
    // sub-stream, while retaining the authenticated control/audio session.
    client.civUdp.writes.clear();
    client.civUdp.writeCalls = 0;
    idleHealthy(client, 1000);   // probe reply times out over a healthy loop
    resentOpen = false;
    for (const auto& packet : client.civUdp.writes) {
      if (packet.size() == 0x16 && packet[0x15] == 0x05) resentOpen = true;
    }
    if (client.failed() || !resentOpen) {
      std::fprintf(stderr, "failed CAT liveness probe did not reopen CI-V in place\n");
      ok = false;
    }

    // The first valid radio reply ends recovery; do not keep injecting open
    // packets into an already healthy command stream.
    testMillis += 100;
    client.civUdp.receive(civPacket({0xFE, 0xFE, 0xE1, 0xA4, 0x03,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0xFD}));
    client.loop();
    client.civUdp.writes.clear();
    client.civUdp.writeCalls = 0;
    testMillis += 500;
    client.lastCtrlRxMs = testMillis;
    client.loop();
    bool openAfterReply = false;
    for (const auto& packet : client.civUdp.writes) {
      if (packet.size() == 0x16 && packet[0x15] == 0x05) openAfterReply = true;
    }
    if (client.civRecovering || openAfterReply) {
      std::fprintf(stderr, "valid CAT reply did not stop CI-V recovery\n");
      ok = false;
    }
  }

  {
    IcomLanClient client;
    parsedFrames = 0;
    client.state = IcomLanClient::LAN_CIV_OPEN;
    client.civPort = 50002;
    client.civGotHere = true;
    client.civGotReady = true;
    client.civOpenSent = true;
    client.civGotData = false;
    client.lastCtrlRxMs = testMillis;

    // This is our own command echoed by the radio: from=E1, not from=A4.
    // It proves UDP transport only; it must not prove a usable CAT reply path.
    client.civUdp.receive(civPacket({0xFE, 0xFE, 0xA4, 0xE1, 0x04, 0xFD}));
    client.loop();
    if (client.connected() || client.civGotData || parsedFrames != 0) {
      std::fprintf(stderr, "outgoing CI-V echo promoted the link to connected\n");
      ok = false;
    }
  }

  {
    IcomLanClient client;
    parsedFrames = 0;
    client.state = IcomLanClient::LAN_CIV_OPEN;
    client.civPort = 50002;
    client.civGotHere = true;
    client.civGotReady = true;
    client.civOpenSent = true;
    client.civGotData = false;
    client.lastCtrlRxMs = testMillis;

    // CI-V transceive broadcasts frequency to address 0 while the tuning knob
    // moves. Parse it for UI state, but it is not a reply to our E1 poll and
    // therefore must not establish or restore CAT request/reply health.
    client.civUdp.receive(civPacket({0xFE, 0xFE, 0x00, 0xA4, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0xFD}));
    client.loop();
    if (client.connected() || client.civGotData || parsedFrames != 1) {
      std::fprintf(stderr, "transceive broadcast falsely established CAT poll health\n");
      ok = false;
    }
  }

  {
    IcomLanClient client;
    client.civPort = 50002;
    const uint8_t readFrequency[] = {0x03};
    for (int i = 0; i < 100; ++i) client.sendCiv(readFrequency, 1);

    // Both wfview and the real-radio probe retain ten seconds or more. A brief
    // blocked ESP loop must not evict a packet before the radio asks for it.
    if (!client.resendTracked(client.civUdp, 1)) {
      std::fprintf(stderr, "CI-V retransmit history is shorter than ten-second window\n");
      ok = false;
    }
  }

  {
    IcomLanClient client;
    client.civPort = 50002;
    for (uint16_t sequence = 0x00BE; sequence <= 0x00C5; ++sequence) {
      std::vector<uint8_t> packet(0x10, 0);
      putLE32(packet, 0, packet.size());
      putLE16(packet, 6, sequence);
      client.civTxHistory.remember(sequence, packet.data(), packet.size());
    }

    // A 0x18 retransmit packet contains inclusive start/end ranges. The live
    // BE,C5 -> BF,C4 -> C0,C3 pattern was the radio repeatedly shrinking one
    // BE..C5 hole because firmware replayed only the two endpoints.
    // Called directly (not via loop()), so provide the per-iteration retransmit
    // budget loop() would normally reset; a high value isolates the unroll logic
    // from the rate limit (which its own test below covers).
    client.retransmitBudget = 1000;
    const std::vector<uint8_t> request = retransmitRange(0x00BE, 0x00C5);
    client.handleRetransmitRequest(client.civUdp, request.data(), request.size());
    if (client.civUdp.writeCalls != 16) {
      std::fprintf(stderr, "CI-V retransmit range restored %zu packets instead of two 8-packet copies\n",
                   client.civUdp.writeCalls);
      ok = false;
    }
  }

  {
    // Retransmit rate limit: a single loop() iteration answers at most
    // LAN_RETRANSMIT_BUDGET requested sequences (each reply is a blocking UDP
    // send), so a huge or stale range cannot freeze the cooperative loop. The
    // rest is deferred for the radio to re-request next tick.
    IcomLanClient client;
    client.civPort = 50002;
    // A 16-wide all-miss range (no tracked history) is duplicated on the wire to
    // 32 replies -> more than the budget, all fills. (Stays under
    // MAX_RETRANSMIT_RANGE so each copy is processed, not rejected wholesale.)
    const std::vector<uint8_t> request = retransmitRange(0x1000, 0x100F);
    client.retransmitBudget = IcomLanClient::LAN_RETRANSMIT_BUDGET;
    client.rtxResent = client.rtxFilled = client.rtxDeferred = 0;
    client.handleRetransmitRequest(client.civUdp, request.data(), request.size());
    if (client.civUdp.writeCalls != IcomLanClient::LAN_RETRANSMIT_BUDGET) {
      std::fprintf(stderr, "retransmit rate limit sent %zu packets instead of the %d budget\n",
                   client.civUdp.writeCalls, IcomLanClient::LAN_RETRANSMIT_BUDGET);
      ok = false;
    }
    if (client.rtxDeferred == 0) {
      std::fprintf(stderr, "retransmit rate limit did not defer the overflow\n");
      ok = false;
    }
  }

  {
    IcomLanClient client;
    client.state = IcomLanClient::LAN_CIV_OPEN;
    client.civPort = 50002;
    client.civGotHere = true;
    client.civOpenSent = false;
    client.lastCtrlRxMs = testMillis;
    client.lastPing = client.lastIdle = testMillis;
    client.civLastPing = client.civLastIdle = testMillis;

    // IC-705 opens client->radio CI-V only after the Ready exchange. If Ready
    // is lost, retry that handshake; an open sent before Ready produces the
    // observed ping/broadcast-only half-connection.
    testMillis += 500;
    client.lastCtrlRxMs = testMillis;
    client.loop();
    bool prematureOpen = false;
    for (const auto& packet : client.civUdp.writes) {
      if (packet.size() == 0x16 && packet[0x15] == 0x05) prematureOpen = true;
    }
    if (prematureOpen) {
      std::fprintf(stderr, "CI-V stream opened before Ready handshake\n");
      ok = false;
    }

    client.civUdp.writes.clear();
    client.civUdp.writeCalls = 0;
    client.civUdp.receive(controlPacket(0x06, 1));
    client.loop();
    bool openAfterReady = false;
    for (const auto& packet : client.civUdp.writes) {
      if (packet.size() == 0x16 && packet[0x15] == 0x05) openAfterReady = true;
    }
    if (!openAfterReady) {
      std::fprintf(stderr, "CI-V Ready did not open stream\n");
      ok = false;
    }
  }

  {
    IcomLanClient client;
    client.state = IcomLanClient::LAN_CONNECTED;
    client.civPort = 50002;
    client.civGotHere = true;
    client.civGotReady = true;
    client.civOpenSent = true;
    client.civGotData = true;
    client.scopeOff = true;
    client.lastCtrlRxMs = testMillis;
    client.lastCivDataMs = testMillis;
    client.lastPing = client.civLastPing = testMillis;
    client.lastIdle = client.civLastIdle = 0;
    client.lastFreqPoll = 0;

    // A real CI-V data packet resets the channel's idle timer. When a poll is
    // due on the same tick, do not send an idle packet as well; wfview resets
    // its idle timer after every tracked non-idle packet.
    testMillis += 100;
    client.lastCtrlRxMs = client.lastCivDataMs = testMillis;
    client.loop();
    if (client.civUdp.writeCalls != 1) {
      std::fprintf(stderr, "CI-V poll did not suppress redundant idle packet (%zu writes)\n",
                   client.civUdp.writeCalls);
      ok = false;
    }
  }

  {
    IcomLanClient client;
    client.state = IcomLanClient::LAN_CIV_OPEN;
    client.civPort = 50002;
    client.civGotHere = true;
    client.civGotReady = true;
    client.civOpenSent = true;
    client.civGotData = false;
    client.lastCtrlRxMs = testMillis;
    parsedFrames = 0;

    // A normal IC-705 selected-mode reply is addressed to LAN controller E1
    // and comes from the radio A4. It must establish CAT health, reach the
    // shared parser and disable the data-mode-blind legacy fallback.
    client.civUdp.receive(civPacket({0xFE, 0xFE, 0xE1, 0xA4, 0x26, 0x00,
                                     0x01, 0x01, 0x01, 0xFD}));
    client.loop();
    if (!client.connected() || !client.civGotData || !client.civSelectedModeSeen
        || parsedFrames != 1) {
      std::fprintf(stderr, "valid selected-mode reply did not establish CAT mode health\n");
      ok = false;
    }
  }

  {
    IcomLanClient client;
    client.state = IcomLanClient::LAN_CONNECTED;
    client.civPort = 50002;
    client.civGotHere = true;
    client.civGotReady = true;
    client.civOpenSent = true;
    client.civGotData = true;
    client.scopeOff = true;
    client.lastCtrlRxMs = testMillis;
    client.lastCivDataMs = testMillis;
    client.lastPing = client.lastIdle = testMillis;
    client.civLastPing = client.civLastIdle = testMillis;
    client.lastFreqPoll = 0;

    // CI-V is a serial command stream inside UDP. Keep exactly one request
    // outstanding; advancing the rotation every 100 ms without a reply makes
    // the initial frequency command win and the following mode read disappear.
    client.loop();
    if (client.civUdp.writeCalls != 1) {
      std::fprintf(stderr, "CI-V poller burst %zu packets in one tick\n",
                   client.civUdp.writeCalls);
      ok = false;
    }

    client.civUdp.writes.clear();
    client.civUdp.writeCalls = 0;
    testMillis += 100;
    client.lastCtrlRxMs = client.lastCivDataMs = testMillis;
    client.lastPing = client.lastIdle = testMillis;
    client.civLastPing = client.civLastIdle = testMillis;
    client.loop();
    const std::vector<uint8_t> expected = {0xFE, 0xFE, 0xA4, 0xE1, 0x26, 0x00, 0xFD};
    bool selectedModePoll = false;
    for (const auto& packet : client.civUdp.writes) {
      if (std::search(packet.begin(), packet.end(), expected.begin(), expected.end()) != packet.end())
        selectedModePoll = true;
    }
    if (selectedModePoll) {
      std::fprintf(stderr, "poller sent mode before frequency reply\n");
      ok = false;
    }

    // The addressed frequency reply releases the scheduler; mode must be the
    // very next CAT request, not be skipped in favour of telemetry.
    client.civUdp.writes.clear();
    client.civUdp.writeCalls = 0;
    client.civUdp.receive(civPacket({0xFE, 0xFE, 0xE1, 0xA4, 0x03,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0xFD}));
    client.loop();
    selectedModePoll = false;
    for (const auto& packet : client.civUdp.writes) {
      if (std::search(packet.begin(), packet.end(), expected.begin(), expected.end()) != packet.end())
        selectedModePoll = true;
    }
    if (!selectedModePoll) {
      std::fprintf(stderr, "frequency reply did not release selected-mode poll\n");
      ok = false;
    }

    // Some IC-705 configurations do not answer selected-mode 0x26. After its
    // bounded timeout, request legacy mode 0x04 so JS8LAN still gets USB/CW/etc.
    client.civUdp.writes.clear();
    client.civUdp.writeCalls = 0;
    testMillis += 500;
    client.lastCtrlRxMs = testMillis;
    client.lastPing = client.lastIdle = testMillis;
    client.civLastPing = client.civLastIdle = testMillis;
    client.loop();
    const std::vector<uint8_t> legacyMode = {0xFE, 0xFE, 0xA4, 0xE1, 0x04, 0xFD};
    bool legacyModePoll = false;
    for (const auto& packet : client.civUdp.writes) {
      if (std::search(packet.begin(), packet.end(), legacyMode.begin(), legacyMode.end()) != packet.end())
        legacyModePoll = true;
    }
    if (!legacyModePoll) {
      std::fprintf(stderr, "selected-mode timeout did not fall back to legacy mode\n");
      ok = false;
    }
  }

  {
    IcomLanClient client;
    client.state = IcomLanClient::LAN_CONNECTED;
    client.civPort = 50002;
    client.civGotHere = true;
    client.civGotReady = true;
    client.civOpenSent = true;
    client.civGotData = true;
    client.lastCtrlRxMs = testMillis;
    client.lastCivDataMs = testMillis;

    // Exact field failure: after the radio/control session disappears, stale
    // CI-V retransmit requests must not refresh whole-session health and leave
    // /state reporting CONNECTED forever.
    testMillis += 7000;
    client.civUdp.receive(retransmitRange(0x0938, 0x0939));
    client.loop();
    if (!client.failed()) {
      std::fprintf(stderr, "CI-V retransmit-only traffic masked dead control session\n");
      ok = false;
    }
  }

  {
    IcomLanClient client;
    client.state = IcomLanClient::LAN_CONNECTED;
    client.civPort = 50002;
    client.civGotHere = true;
    client.civGotReady = true;
    client.civOpenSent = true;
    client.civGotData = true;
    client.lastCtrlRxMs = testMillis;
    client.lastCivDataMs = testMillis;

    // Even with a healthy control channel, an unrecoverable CAT sub-stream may
    // not remain CONNECTED indefinitely. Bound recovery, then establish a new
    // authenticated session so firmware and JS8LAN agree that radio is offline.
    // A healthy loop reaches civSilent -> probe -> recovery -> reconnect in
    // ~9 s; 12 s of budget covers it (idleHealthy returns early on failure).
    idleHealthy(client, 12000);
    if (!client.failed()) {
      std::fprintf(stderr, "failed CAT recovery never escalated to session reconnect\n");
      ok = false;
    }
  }

  {
    // Loop-stall compensation: a single multi-second unserviced gap on an
    // established session is a stall (the socket was not read), not link
    // silence, and must not trip civSilent's CAT reopen or the control-loss
    // drop. The healthy-loop path above still fails on the same wall-clock
    // silence, so this only forgives genuine stalls, never a dead link.
    IcomLanClient client;
    client.state = IcomLanClient::LAN_CONNECTED;
    client.civPort = 50002;
    client.civGotHere = true;
    client.civGotReady = true;
    client.civOpenSent = true;
    client.civGotData = true;
    client.lastCtrlRxMs = testMillis;
    client.lastCivDataMs = testMillis;

    // One healthy tick establishes the service baseline...
    testMillis += 50;
    client.ctrlUdp.receive(controlPingReply());
    client.loop();
    // ...then a single 5 s stall: one loop() after a 5 s jump.
    client.civUdp.writes.clear();
    client.civUdp.writeCalls = 0;
    testMillis += 5000;
    client.loop();
    bool reopened = false;
    for (const auto& packet : client.civUdp.writes) {
      if (packet.size() == 0x16 && packet[0x15] == 0x05) reopened = true;
    }
    if (client.failed() || client.civRecovering || reopened) {
      std::fprintf(stderr, "loop stall misjudged as link silence\n");
      ok = false;
    }
  }

  {
    IcomLanClient client;
    client.state = IcomLanClient::LAN_CONNECTED;
    client.streamOpened = true;
    client.civPort = 50002;
    client.lastCtrlRxMs = testMillis;

    // The radio can explicitly revoke an otherwise responsive session. Port
    // fields may remain populated, so disc=1 itself must be authoritative.
    client.ctrlUdp.receive(controlStatus(true, 50002));
    client.loop();
    if (!client.failed() || client.civUdp.writeCalls != 0) {
      std::fprintf(stderr, "explicit radio disconnect did not stop session traffic\n");
      ok = false;
    }
  }

  {
    IcomLanClient client;
    client.state = IcomLanClient::LAN_CONNECTED;
    client.authOk = true;
    client.lastCtrlRxMs = testMillis;

    // A rejected token renewal invalidates the authenticated session even if
    // pings or retransmit traffic continue for a while.
    client.ctrlUdp.receive(authResponse(0xFFFFFFFF));
    client.loop();
    if (!client.failed()) {
      std::fprintf(stderr, "rejected token renewal left session CONNECTED\n");
      ok = false;
    }
  }

  {
    IcomLanClient client;
    client.state = IcomLanClient::LAN_LOGIN;
    client.tokRequest = 0x4321;
    client.lastCtrlRxMs = testMillis;

    // No nonzero login error may progress into token authentication merely
    // because the echoed request id matches.
    client.ctrlUdp.receive(loginResponse(client.tokRequest, 0xFFFFFFFF));
    client.loop();
    if (!client.failed()) {
      std::fprintf(stderr, "generic login error progressed into authentication\n");
      ok = false;
    }
  }

  {
    // Sender identity: only datagrams from the configured radio IP drive session
    // health. A foreign packet on our control port must be ignored, a genuine one
    // accepted.
    IcomLanClient client;
    client.state = IcomLanClient::LAN_CONNECTED;
    client.radioIP = IPAddress(0x0102030Au);
    client.stateSince = testMillis;
    client.lastCtrlRxMs = testMillis;
    client.lastCivDataMs = testMillis;

    uint32_t healthy = testMillis;
    testMillis += 100;
    client.ctrlUdp.receive(IPAddress(0x0BADCAFEu), controlPingReply());  // foreign
    client.loop();
    if (client.lastCtrlRxMs != healthy) {
      std::fprintf(stderr, "foreign control packet refreshed session health\n");
      ok = false;
    }
    testMillis += 100;
    client.ctrlUdp.receive(IPAddress(0x0102030Au), controlPingReply());  // radio
    client.loop();
    if (client.lastCtrlRxMs != testMillis) {
      std::fprintf(stderr, "control packet from the radio was rejected\n");
      ok = false;
    }
  }

  {
    // audioReady() = linked audio sub-stream with fresh payload. Payload silence
    // (normal on a quiet channel) makes it false but must NOT reopen the stream
    // or drop the session — the radio simply streams nothing until there is AF.
    IcomLanClient client;
    client.state = IcomLanClient::LAN_CONNECTED;
    client.radioIP = IPAddress(0x0102030Au);
    client.audioPort = 50003;
    client.audioOpened = true;
    client.audioGotHere = true;
    client.audioRemoteId = 0x1234;
    client.stateSince = testMillis;
    client.lastCtrlRxMs = testMillis;
    client.audioLastDataMs = testMillis;
    if (!client.audioReady()) {
      std::fprintf(stderr, "audioReady false right after fresh payload\n");
      ok = false;
    }
    client.audioUdp.writeCalls = 0;

    idleHealthy(client, 8000);   // long payload silence, healthy loop

    if (client.failed()) {
      std::fprintf(stderr, "audio payload silence dropped the whole session\n");
      ok = false;
    }
    if (!client.audioGotHere || !client.audioOpened) {
      std::fprintf(stderr, "audio payload silence disturbed the sub-stream\n");
      ok = false;
    }
    if (client.audioReady()) {
      std::fprintf(stderr, "audioReady stayed true through long payload silence\n");
      ok = false;
    }
  }

  if (!ok) return 1;
  std::puts("ICOM LAN CLIENT HEALTH PASS");
  return 0;
}
