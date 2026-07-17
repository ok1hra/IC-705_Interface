// icomLanClient.h — ICOM LAN (RS-BA1) CI-V client for ESP32.
//
// Step-3 minimal client: control + CI-V UDP channels, no audio, reads the
// operating frequency and prints it. Protocol verified against a real IC-705
// with tools/icom-lan-login-test.py — see docs/icom-lan-implementace.md §2 for
// the six IC-705 deviations from the wfview flow that this code follows.
//
// Non-blocking: call begin() once, then loop() often. Drives itself through a
// state machine. Designed to grow into the real transport (BT vs LAN) module.
#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>

// Implemented in the .ino — routes a received CI-V frame (FE FE..FD) through the
// shared parser so LAN gets the same full CAT state (freq, mode, meters, TX...).
extern void lanCivFrameHandler(const uint8_t *frame, size_t len);

class IcomLanClient {
public:
  enum State {
    LAN_IDLE, LAN_AYT, LAN_LOGIN, LAN_AUTH, LAN_STREAM,   // control channel
    LAN_CIV_AYT, LAN_CIV_OPEN, LAN_CONNECTED, LAN_FAILED
  };

  // civAddr = radio CI-V address (IC-705 = 0xA4). name is filled from caps.
  void begin(IPAddress radioIp, uint16_t controlPort,
             const char* user, const char* pass, uint8_t civAddr) {
    stop();
    radioIP = radioIp;
    ctrlPort = controlPort ? controlPort : 50001;
    strlcpy(username, user, sizeof(username));
    strlcpy(password, pass, sizeof(password));
    radioCivAddr = civAddr;

    localIP = WiFi.localIP();
    ctrlUdp.begin(50001);
    ctrlMyId = mkId(50001);
    ctrlRemoteId = 0;
    ctrlSendSeq = 1;
    authInnerSeq = 0x30;
    tokRequest = (uint16_t)esp_random();
    token = 0;
    haveCaps = false; authOk = false; authAnnounced = false; streamReqSent = false; streamOpened = false;
    civPort = 0;

    state = LAN_AYT;
    stateSince = millis();
    lastRxMs = millis();
    lastAyt = 0; lastPing = 0; lastIdle = 0;
    Serial.print("LAN | begin -> "); Serial.print(radioIP);
    Serial.print(":"); Serial.println(ctrlPort);
  }

  void stop() {
    if (state == LAN_IDLE) return;
    if (streamOpened) sendCivOpenClose(true);
    if (token) sendToken(0x01);        // release
    sendCtrl(ctrlUdp, ctrlMyId, ctrlRemoteId, 0x05, 0);  // disconnect
    civUdp.stop();
    ctrlUdp.stop();
    state = LAN_IDLE;
  }

  bool connected() const { return state == LAN_CONNECTED; }
  bool failed() const    { return state == LAN_FAILED; }
  State status() const   { return state; }

  // Send a CI-V command body (cmd + payload, WITHOUT the FE FE <to><from> .. FD
  // wrapper — sendCiv adds it with the LAN controller address 0xE1). Used by
  // catWriteFrame so CW/tune/set-freq/set-mode all work over LAN.
  bool sendCommand(const uint8_t* body, size_t len) {
    if (state != LAN_CONNECTED || civPort == 0) return false;
    sendCiv(body, len);
    return true;
  }

  void loop() {
    if (state == LAN_IDLE || state == LAN_FAILED) return;
    uint32_t now = millis();

    pumpControl();
    if (civPort) pumpCiv();

    // control-channel periodic sends
    if (state == LAN_AYT) {
      if (now - lastAyt >= 500) { sendCtrl(ctrlUdp, ctrlMyId, ctrlRemoteId, 0x03, 0); lastAyt = now; }
    } else {
      if (now - lastPing >= 500) { sendPing(ctrlUdp, ctrlMyId, ctrlRemoteId, ctrlPingSeq++); lastPing = now; }
      if (now - lastIdle >= 100) { sendTracked(ctrlUdp, ctrlPkt(0x10, 0x00), 0x10); lastIdle = now; }
      reauthMaybe(now);
    }

    // CI-V channel periodic sends
    if (civPort) {
      if (!civGotHere) {
        if (now - civLastAyt >= 500) { sendCtrl(civUdp, civMyId, civRemoteId, 0x03, 0); civLastAyt = now; }
      } else {
        if (now - civLastPing >= 500) { sendPing(civUdp, civMyId, civRemoteId, civPingSeq++); civLastPing = now; }
        if (now - civLastIdle >= 100) { sendTracked(civUdp, civPkt0(0x00), 0x10); civLastIdle = now; }
        // radio sends no "ready" on CI-V — open 0.5 s after IAmHere, then retry
        if (!civOpenSent && now - civHereTime >= 500) {
          Serial.println("LAN | civ: no ready, opening anyway");
          civOpenSent = true; sendCivOpenClose(false); civNextOpen = now + 500;
        }
        if (civOpenSent && !civGotData && (int32_t)(now - civNextOpen) >= 0) {
          sendCivOpenClose(false); civNextOpen = now + 500;
        }
        if (civOpenSent && now - lastFreqPoll >= 1000) {
          if (!scopeOff) { sendCivFrame3(0x27, 0x11, 0x00); scopeOff = true; }  // disable scope stream
          uint8_t f[] = {0x03}; sendCiv(f, 1);   // read frequency
          uint8_t m[] = {0x04}; sendCiv(m, 1);   // read mode
          lastFreqPoll = now;
        }
        // Aux telemetry for full CAT parity: S-meter + power every tick (live
        // metering), one rotating item from the rest — no BT coex to throttle.
        if (civOpenSent && scopeOff && now - lastAuxPoll >= 200) {
          uint8_t sm[] = {0x15, 0x02}; sendCiv(sm, 2);   // S-meter (RX)
          uint8_t pw[] = {0x15, 0x11}; sendCiv(pw, 2);   // power meter (TX)
          sendAuxRot(auxRot);
          auxRot = (auxRot + 1) % 10;
          lastAuxPoll = now;
        }
      }
    }

    if (state != LAN_CONNECTED && millis() - stateSince > 12000) {
      Serial.println("LAN | timeout in state, giving up");
      state = LAN_FAILED;
    }
    // liveness watchdog: base it on ANY received packet, not CI-V data. The
    // radio pings us ~1/s independently of CI-V, and CI-V is silent when idle
    // (transceive only fires on change). 6 s of total silence = link dead
    // (WiFi drop / radio off). Frequency is held at its last known value.
    if (state == LAN_CONNECTED && millis() - lastRxMs > 6000) {
      Serial.println("LAN | no packets 6s, link lost");
      state = LAN_FAILED;
    }
  }

private:
  // ---- config / session state ----
  IPAddress radioIP, localIP;
  uint16_t ctrlPort = 50001;
  char username[24] = {0}, password[24] = {0};
  uint8_t radioCivAddr = 0xA4;

  WiFiUDP ctrlUdp, civUdp;
  uint32_t ctrlMyId = 0, ctrlRemoteId = 0;
  uint32_t civMyId = 0, civRemoteId = 0;
  uint16_t ctrlSendSeq = 1, civSendSeq = 1;
  // CI-V channel needs TWO counters: civSendSeq = tracked seq (bytes 6-7, bumped
  // by every packet incl. idles); civDataSeq = CI-V sendseq (field 0x13, starts 0,
  // bumped only by open/close+data). Sharing them makes 0x13 non-contiguous and
  // the radio then ignores civ-open (verified: symptom = civ opens, no CI-V data).
  uint16_t civDataSeq = 0;
  uint16_t authInnerSeq = 0x30;
  uint16_t tokRequest = 0;
  uint32_t token = 0;
  uint16_t ctrlPingSeq = 0, civPingSeq = 0;

  // radio identity from capabilities
  uint8_t radioMac[6] = {0};
  uint16_t commonCap = 0x8010;
  char radioName[16] = "IC-705";
  bool haveCaps = false, authOk = false, authAnnounced = false, streamReqSent = false, streamOpened = false;
  uint16_t civPort = 0, audioPort = 0;

  // CI-V channel progress
  bool civGotHere = false, civOpenSent = false, civGotData = false, scopeOff = false;
  uint32_t civHereTime = 0, civNextOpen = 0, lastFreqPoll = 0, lastAuxPoll = 0, lastCivDataMs = 0, lastRxMs = 0;
  uint32_t civLastAyt = 0, civLastPing = 0, civLastIdle = 0;
  uint8_t auxRot = 0;

  State state = LAN_IDLE;
  uint32_t stateSince = 0, lastAyt = 0, lastPing = 0, lastIdle = 0, lastReauth = 0;

  uint8_t buf[1500];

  // ---- little/big-endian writers ----
  static void putLE16(uint8_t*p,uint16_t v){p[0]=v;p[1]=v>>8;}
  static void putLE32(uint8_t*p,uint32_t v){p[0]=v;p[1]=v>>8;p[2]=v>>16;p[3]=v>>24;}
  static void putBE16(uint8_t*p,uint16_t v){p[0]=v>>8;p[1]=v;}
  static void putBE32(uint8_t*p,uint32_t v){p[0]=v>>24;p[1]=v>>16;p[2]=v>>8;p[3]=v;}
  static uint16_t getLE16(const uint8_t*p){return p[0]|(p[1]<<8);}
  static uint32_t getLE32(const uint8_t*p){return (uint32_t)p[0]|(p[1]<<8)|(p[2]<<16)|((uint32_t)p[3]<<24);}
  static uint16_t getBE16(const uint8_t*p){return (p[0]<<8)|p[1];}

  uint32_t mkId(uint16_t localPort) {
    uint32_t a = (uint32_t)localIP;   // stored little-endian (a=oct1..oct4)
    uint8_t o3 = (a >> 16) & 0xff, o4 = (a >> 24) & 0xff;
    return ((uint32_t)o3 << 24) | ((uint32_t)o4 << 16) | (localPort & 0xffff);
  }

  // ---- passcode substitution (icomudpbase.h) ----
  static uint8_t pcSeq(uint8_t i) {
    static const uint8_t s[] = {
      0x47,0x5d,0x4c,0x42,0x66,0x20,0x23,0x46,0x4e,0x57,0x45,0x3d,0x67,0x76,0x60,0x41,
      0x62,0x39,0x59,0x2d,0x68,0x7e,0x7c,0x65,0x7d,0x49,0x29,0x72,0x73,0x78,0x21,0x6e,
      0x5a,0x5e,0x4a,0x3e,0x71,0x2c,0x2a,0x54,0x3c,0x3a,0x63,0x4f,0x43,0x75,0x27,0x79,
      0x5b,0x35,0x70,0x48,0x6b,0x56,0x6f,0x34,0x32,0x6c,0x30,0x61,0x6d,0x7b,0x2f,0x4b,
      0x64,0x38,0x2b,0x2e,0x50,0x40,0x3f,0x55,0x33,0x37,0x25,0x77,0x24,0x26,0x74,0x6a,
      0x28,0x53,0x4d,0x69,0x22,0x5c,0x44,0x31,0x36,0x58,0x3b,0x7a,0x51,0x5f,0x52};
    return (i >= 32 && i < 32 + sizeof(s)) ? s[i - 32] : 0;
  }
  static void passcode(const char* in, uint8_t* out16) {
    memset(out16, 0, 16);
    for (int i = 0; in[i] && i < 16; i++) {
      int p = (uint8_t)in[i] + i;
      if (p > 126) p = 32 + p % 127;
      out16[i] = pcSeq(p);
    }
  }

  // ---- packet primitives ----
  // Build a 0x10 control packet into buf; returns length.
  size_t hdr16(uint32_t myId, uint32_t rid, uint16_t type, uint16_t seq) {
    memset(buf, 0, 0x10);
    putLE32(buf+0, 0x10); putLE16(buf+4, type); putLE16(buf+6, seq);
    putLE32(buf+8, myId); putLE32(buf+12, rid);
    return 0x10;
  }
  size_t ctrlPkt(uint16_t /*len*/, uint16_t type) { return hdr16(ctrlMyId, ctrlRemoteId, type, 0); }
  size_t civPkt0(uint16_t type) { return hdr16(civMyId, civRemoteId, type, 0); }

  // remote port depends on channel
  uint16_t currentRemote(WiFiUDP& u) { return (&u == &ctrlUdp) ? ctrlPort : civPort; }

  void sendCtrl(WiFiUDP& u, uint32_t myId, uint32_t rid, uint16_t type, uint16_t seq) {
    hdr16(myId, rid, type, seq);
    u.beginPacket(radioIP, currentRemote(u)); u.write(buf, 0x10); u.endPacket();
  }

  // tracked packet: stamp seq into bytes 6-7 (we don't buffer for retransmit in
  // this minimal client — the IC-705 rarely asks and freq is re-polled anyway)
  void sendTracked(WiFiUDP& u, size_t len, uint16_t /*hint*/) {
    uint16_t& seq = (&u == &ctrlUdp) ? ctrlSendSeq : civSendSeq;
    putLE16(buf+6, seq++);
    u.beginPacket(radioIP, currentRemote(u)); u.write(buf, len); u.endPacket();
  }

  void sendPing(WiFiUDP& u, uint32_t myId, uint32_t rid, uint16_t seq) {
    memset(buf, 0, 0x15);
    putLE32(buf+0, 0x15); putLE16(buf+4, 0x07); putLE16(buf+6, seq);
    putLE32(buf+8, myId); putLE32(buf+12, rid);
    buf[0x10] = 0x00;                       // request
    putLE32(buf+0x11, millis());            // our uptime ms
    u.beginPacket(radioIP, currentRemote(u)); u.write(buf, 0x15); u.endPacket();
  }
  void sendPingReply(WiFiUDP& u, uint32_t myId, uint32_t rid, uint16_t seq, uint32_t t) {
    memset(buf, 0, 0x15);
    putLE32(buf+0, 0x15); putLE16(buf+4, 0x07); putLE16(buf+6, seq);
    putLE32(buf+8, myId); putLE32(buf+12, rid);
    buf[0x10] = 0x01; putLE32(buf+0x11, t);
    u.beginPacket(radioIP, currentRemote(u)); u.write(buf, 0x15); u.endPacket();
  }

  void sendLogin() {
    memset(buf, 0, 0x80);
    putLE32(buf+0, 0x80);
    putLE32(buf+8, ctrlMyId); putLE32(buf+12, ctrlRemoteId);
    putBE32(buf+0x10, 0x70);
    buf[0x14] = 0x01; buf[0x15] = 0x00;
    putBE16(buf+0x16, authInnerSeq++);
    putLE16(buf+0x1a, tokRequest);
    uint8_t u[16], w[16];
    passcode(username, u); passcode(password, w);
    memcpy(buf+0x40, u, 16); memcpy(buf+0x50, w, 16);
    const char* nm = "esp705if";
    memcpy(buf+0x60, nm, strlen(nm));
    sendTracked(ctrlUdp, 0x80, 0x80);
    Serial.println("LAN | login sent");
  }

  // magic: 0x02 confirm, 0x05 auth/renew, 0x01 release. No resetcap (IC-705).
  void sendToken(uint8_t magic) {
    memset(buf, 0, 0x40);
    putLE32(buf+0, 0x40);
    putLE32(buf+8, ctrlMyId); putLE32(buf+12, ctrlRemoteId);
    putBE32(buf+0x10, 0x30);
    buf[0x14] = 0x01; buf[0x15] = magic;
    putBE16(buf+0x16, authInnerSeq++);
    putLE16(buf+0x1a, tokRequest);
    putLE32(buf+0x1c, token);
    sendTracked(ctrlUdp, 0x40, 0x40);
  }

  void sendStreamRequest() {
    memset(buf, 0, 0x90);
    putLE32(buf+0, 0x90);
    putLE32(buf+8, ctrlMyId); putLE32(buf+12, ctrlRemoteId);
    putBE32(buf+0x10, 0x80);
    buf[0x14] = 0x01; buf[0x15] = 0x03;
    putBE16(buf+0x16, authInnerSeq++);
    putLE16(buf+0x1a, tokRequest);
    putLE32(buf+0x1c, token);
    putLE16(buf+0x27, commonCap);          // 0x8010 + mac identity
    memcpy(buf+0x2a, radioMac, 6);
    memcpy(buf+0x40, radioName, strnlen(radioName, 15));
    uint8_t u[16]; passcode(username, u); memcpy(buf+0x60, u, 16);
    // no-audio: rxenable/txenable/codecs/samples all zero
    putBE32(buf+0x7c, 50002);              // civ local port
    putBE32(buf+0x80, 50003);              // audio local port (unused)
    putBE32(buf+0x84, 150);                // txbuffer
    buf[0x88] = 1;                         // convert
    sendTracked(ctrlUdp, 0x90, 0x90);
    Serial.println("LAN | stream request sent (no audio)");
  }

  void sendCivOpenClose(bool close) {
    memset(buf, 0, 0x16);
    putLE32(buf+0, 0x16);
    putLE32(buf+8, civMyId); putLE32(buf+12, civRemoteId);
    putLE16(buf+0x10, 0x01c0);
    putBE16(buf+0x13, civDataSeq++);       // CI-V sendseq (BE), separate counter
    buf[0x15] = close ? 0x00 : 0x05;       // IC-705 open magic is 0x05
    sendTracked(civUdp, 0x16, 0x16);
  }

  // send a CI-V frame (payload only, without FE FE .. FD wrapper) to the radio
  void sendCiv(const uint8_t* civBody, size_t bodyLen) {
    // full CI-V frame: FE FE <radio> E1 <body...> FD
    uint8_t fr[32];
    size_t fl = 0;
    fr[fl++] = 0xFE; fr[fl++] = 0xFE; fr[fl++] = radioCivAddr; fr[fl++] = 0xE1;
    for (size_t i = 0; i < bodyLen && fl < sizeof(fr)-1; i++) fr[fl++] = civBody[i];
    fr[fl++] = 0xFD;
    // wrap in data packet
    memset(buf, 0, 0x15);
    putLE32(buf+0, 0x15 + fl);
    putLE32(buf+8, civMyId); putLE32(buf+12, civRemoteId);
    buf[0x10] = 0xC1;
    putLE16(buf+0x11, fl);
    putBE16(buf+0x13, civDataSeq++);       // CI-V sendseq (BE), separate counter
    memcpy(buf+0x15, fr, fl);
    sendTracked(civUdp, 0x15 + fl, 0x15);
  }
  void sendCivFrame3(uint8_t a, uint8_t b, uint8_t c) { uint8_t f[]={a,b,c}; sendCiv(f,3); }

  // rotating aux telemetry poll (mirrors the BT pollRadio aux set, minus the
  // S-meter/power that are polled every tick). Feeds the same state globals.
  void sendAuxRot(uint8_t i) {
    switch (i) {
      case 0: { uint8_t b[]={0x15,0x12}; sendCiv(b,2); break; } // SWR
      case 1: { uint8_t b[]={0x15,0x15}; sendCiv(b,2); break; } // supply voltage
      case 2: { uint8_t b[]={0x14,0x01}; sendCiv(b,2); break; } // AF gain
      case 3: { uint8_t b[]={0x14,0x0C}; sendCiv(b,2); break; } // key speed
      case 4: { uint8_t b[]={0x14,0x0A}; sendCiv(b,2); break; } // RF power
      case 5: { uint8_t b[]={0x1C,0x00}; sendCiv(b,2); break; } // TX state (PTT)
      case 6: { uint8_t b[]={0x21,0x00}; sendCiv(b,2); break; } // RIT offset
      case 7: { uint8_t b[]={0x11};      sendCiv(b,1); break; } // ATT
      case 8: { uint8_t b[]={0x16,0x02}; sendCiv(b,2); break; } // preamp
      case 9: { uint8_t b[]={0x16,0x47}; sendCiv(b,2); break; } // VOX
    }
  }

  // send the stream request once, only after BOTH auth 0x05 ack and caps arrived
  void maybeRequestStream() {
    if (authOk && haveCaps && !streamReqSent) {
      streamReqSent = true;
      sendStreamRequest();
      setState(LAN_STREAM);
    }
  }

  void reauthMaybe(uint32_t now) {
    // renew the token well before its ~60 s expiry (safety margin against the
    // radio dropping the session on token timeout)
    if (authOk && now - lastReauth >= 30000) { sendToken(0x05); lastReauth = now; }
  }

  // ---- receive ----
  void pumpControl() {
    int n;
    while ((n = ctrlUdp.parsePacket()) > 0) {
      int r = ctrlUdp.read(buf, sizeof(buf));
      if (r < 0x10) continue;
      lastRxMs = millis();          // any packet proves the link is alive
      handleControl(buf, r);
    }
  }
  void pumpCiv() {
    int n;
    while ((n = civUdp.parsePacket()) > 0) {
      int r = civUdp.read(buf, sizeof(buf));
      if (r < 0x10) continue;
      lastRxMs = millis();
      handleCiv(buf, r);
    }
  }

  void handleControl(uint8_t* r, int n) {
    uint16_t type = getLE16(r+4);
    uint32_t sentid = getLE32(r+8);

    // ping request -> reply
    if (n == 0x15 && type == 0x07 && r[0x10] == 0x00) {
      sendPingReply(ctrlUdp, ctrlMyId, ctrlRemoteId, getLE16(r+6), getLE32(r+0x11));
      return;
    }
    if (n == 0x10) {
      if (type == 0x04 && state == LAN_AYT) {          // IAmHere
        ctrlRemoteId = sentid;
        Serial.println("LAN | ctrl: I am here");
        sendCtrl(ctrlUdp, ctrlMyId, ctrlRemoteId, 0x06, 1);  // AreYouReady
        setState(LAN_LOGIN);
      } else if (type == 0x06 && state == LAN_LOGIN) {  // Ready
        sendLogin();
      }
      return;
    }
    if (n == 0x60) {                                   // login response
      uint32_t err = getLE32(r+0x30);
      uint16_t tr = getLE16(r+0x1a);
      Serial.print("LAN | login response err=0x"); Serial.println(err, HEX);
      if (err == 0xFEFFFFFF) { Serial.println("LAN | BAD USERNAME/PASSWORD"); state = LAN_FAILED; return; }
      if (tr == tokRequest) {
        token = getLE32(r+0x1c);
        sendToken(0x02); sendToken(0x05);              // confirm + auth
        setState(LAN_AUTH);
      }
      return;
    }
    if (n == 0x40) {                                   // auth response
      if (r[0x14] == 0x02 && r[0x15] == 0x05) {
        authOk = true; lastReauth = millis();
        if (!authAnnounced) { Serial.println("LAN | auth OK"); authAnnounced = true; }
        maybeRequestStream();
      }
      return;
    }
    if (n == 0x50) {                                   // status (stream ports)
      uint32_t err = getLE32(r+0x30);
      if (err == 0xFFFFFFFF) { Serial.println("LAN | stream refused (reboot radio)"); state = LAN_FAILED; return; }
      uint16_t cp = getBE16(r+0x42);
      if (cp == 0) {
        // radio still holding a prior session — fast-fail so we back off and retry
        Serial.println("LAN | stream civ port=0 (stale session), retrying");
        state = LAN_FAILED;
        return;
      }
      civPort = cp;
      audioPort = getBE16(r+0x46);
      Serial.print("LAN | stream ok, civ port="); Serial.println(civPort);
      openCivChannel();
      return;
    }
    if (n >= 0x42 && (n - 0x42) % 0x66 == 0) {         // capabilities
      // one radio: identity at first entry
      int base = 0x42;
      commonCap = getLE16(r+base+0x07);
      memcpy(radioMac, r+base+0x0a, 6);
      memcpy(radioName, r+base+0x10, 15); radioName[15] = 0;
      radioCivAddr = r[base+0x52];
      haveCaps = true;
      Serial.print("LAN | caps: "); Serial.print(radioName);
      Serial.print(" civ=0x"); Serial.println(radioCivAddr, HEX);
      maybeRequestStream();
      return;
    }
    // n==0x90 conninfo: ignored in minimal client
  }

  void openCivChannel() {
    if (streamOpened) return;
    streamOpened = true;
    civUdp.begin(50002);
    civMyId = mkId(50002);
    civRemoteId = 0;
    civGotHere = civOpenSent = civGotData = scopeOff = false;
    civSendSeq = 1;
    civDataSeq = 0;
    lastCivDataMs = millis();
    lastFreqPoll = lastAuxPoll = 0;
    auxRot = 0;
    civLastAyt = civLastPing = civLastIdle = 0;
    setState(LAN_CIV_AYT);
  }

  void handleCiv(uint8_t* r, int n) {
    uint16_t type = getLE16(r+4);
    uint32_t sentid = getLE32(r+8);

    if (n == 0x15 && type == 0x07 && r[0x10] == 0x00) {
      sendPingReply(civUdp, civMyId, civRemoteId, getLE16(r+6), getLE32(r+0x11));
      return;
    }
    if (n == 0x10) {
      if (type == 0x04 && !civGotHere) {
        civGotHere = true; civRemoteId = sentid; civHereTime = millis();
        Serial.println("LAN | civ: I am here");
        sendCtrl(civUdp, civMyId, civRemoteId, 0x06, 1);
        setState(LAN_CIV_OPEN);
        maybeConnected();            // data may have raced ahead of this handshake
      } else if (type == 0x06) {                       // ready (rare on IC-705)
        civRemoteId = sentid;
        if (!civOpenSent) { civOpenSent = true; sendCivOpenClose(false); civNextOpen = millis()+500; }
      }
      return;
    }
    if (n > 0x15 && type != 0x01) {                    // data packet
      uint16_t plen = getLE32(r+0) & 0xFFFF;
      uint16_t dlen = getLE16(r+0x11);
      if (((dlen + 0x15) & 0xFFFF) != plen) return;
      civGotData = true;
      lastCivDataMs = millis();
      // split possibly-multiple CI-V frames FE FE .. FD in payload
      int i = 0x15;
      while (i + 5 <= n) {
        if (r[i] != 0xFE || r[i+1] != 0xFE) { i++; continue; }
        int e = i + 2;
        while (e < n && r[e] != 0xFD) e++;
        if (e >= n) break;
        parseCivFrame(r + i, e - i + 1);
        i = e + 1;
      }
      maybeConnected();              // first data frame after the handshake -> CONNECTED
      return;
    }
  }

  void parseCivFrame(const uint8_t* f, int len) {
    if (len < 6) return;
    // CI-V frame: FE FE <to> <from> <cmd> ... FD. Only process frames FROM the
    // radio (from-addr == radioCivAddr); the radio echoes our own commands back
    // (from == 0xE1 controller), which must not be parsed as replies.
    if (f[3] != radioCivAddr) return;
    if (f[4] == 0x27) return;                            // scope data, ignore
    lanCivFrameHandler(f, (size_t)len);                  // full CAT parse in the .ino
    // NOTE: state is intentionally NOT driven here — see maybeConnected(). A radio
    // that streams CI-V data before finishing its civ handshake used to flip the
    // state (and the log) CIV_OPEN<->CONNECTED on every frame.
  }

  // Single promotion point to LAN_CONNECTED: reached only once the civ handshake
  // is done (civGotHere -> LAN_CIV_OPEN) AND at least one CI-V data frame has been
  // received. Called from both the handshake and the data path so whichever
  // completes the pair does the promotion, exactly once.
  void maybeConnected() {
    if (state == LAN_CIV_OPEN && civGotData) {
      Serial.println("LAN | CONNECTED");
      setState(LAN_CONNECTED);
    }
  }

  void setState(State s) { state = s; stateSince = millis(); }
};
