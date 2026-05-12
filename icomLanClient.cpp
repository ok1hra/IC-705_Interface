#include "icomLanClient.h"
#include <WiFi.h>

// Packet size constants (from packettypes.h / kappanhang)
#define CTRL_SZ       16   // 0x10  control packet
#define PING_SZ       21   // 0x15  ping / CIV data header
#define TOKEN_SZ      64   // 0x40  token packet
#define STATUS_SZ     80   // 0x50  status (civPort in it)
#define LOGIN_RESP_SZ 96   // 0x60  login response
#define CONNINFO_SZ   144  // 0x90  conninfo (stream request / radio status)
#define LOGIN_SZ      128  // 0x80  login
#define OPENCLOSE_SZ  22   // 0x16  open/close CIV stream
#define CAPS_HDR_SZ   66   // 0x42  capabilities header
#define CAPS_RADIO_SZ 102  // 0x66  capabilities per radio

// ── Byte helpers ────────────────────────────────────────────────────────────

void IcomLanClient::putU32LE(uint8_t* b, uint32_t v) {
    b[0]=v; b[1]=v>>8; b[2]=v>>16; b[3]=v>>24;
}
void IcomLanClient::putU16LE(uint8_t* b, uint16_t v) {
    b[0]=v; b[1]=v>>8;
}
void IcomLanClient::putU32BE(uint8_t* b, uint32_t v) {
    b[0]=v>>24; b[1]=v>>16; b[2]=v>>8; b[3]=v;
}
void IcomLanClient::putU16BE(uint8_t* b, uint16_t v) {
    b[0]=v>>8; b[1]=v;
}
uint32_t IcomLanClient::getU32LE(const uint8_t* b) {
    return (uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16)|((uint32_t)b[3]<<24);
}
uint16_t IcomLanClient::getU16LE(const uint8_t* b) {
    return (uint16_t)b[0]|((uint16_t)b[1]<<8);
}
uint16_t IcomLanClient::getU16BE(const uint8_t* b) {
    return ((uint16_t)b[0]<<8)|(uint16_t)b[1];
}

// ── passcode() from icomudpbase.h ────────────────────────────────────────────

void IcomLanClient::icomPasscode(const char* input, uint8_t* out, uint8_t maxLen) {
    static const uint8_t seq[128] = {
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0x47,0x5d,0x4c,0x42,0x66,0x20,0x23,0x46,0x4e,0x57,0x45,0x3d,0x67,0x76,0x60,0x41,
        0x62,0x39,0x59,0x2d,0x68,0x7e,0x7c,0x65,0x7d,0x49,0x29,0x72,0x73,0x78,0x21,0x6e,
        0x5a,0x5e,0x4a,0x3e,0x71,0x2c,0x2a,0x54,0x3c,0x3a,0x63,0x4f,0x43,0x75,0x27,0x79,
        0x5b,0x35,0x70,0x48,0x6b,0x56,0x6f,0x34,0x32,0x6c,0x30,0x61,0x6d,0x7b,0x2f,0x4b,
        0x64,0x38,0x2b,0x2e,0x50,0x40,0x3f,0x55,0x33,0x37,0x25,0x77,0x24,0x26,0x74,0x6a,
        0x28,0x53,0x4d,0x69,0x22,0x5c,0x44,0x31,0x36,0x58,0x3b,0x7a,0x51,0x5f,0x52,0
    };
    uint8_t len = (uint8_t)strlen(input);
    if (len > maxLen) len = maxLen;
    memset(out, 0, maxLen);
    for (uint8_t i = 0; i < len; i++) {
        int p = (uint8_t)input[i] + i;
        if (p > 126) p = 32 + p % 127;
        out[i] = seq[p];
    }
}

// ── Public API ───────────────────────────────────────────────────────────────

void IcomLanClient::begin(IPAddress radioIp, uint16_t controlPort,
                          const char* username, const char* password,
                          const char* clientName, uint8_t civAddress) {
    _radioIp   = radioIp;
    _ctrlPort  = controlPort;
    strncpy(_username,   username,   16); _username[16]   = '\0';
    strncpy(_password,   password,   16); _password[16]   = '\0';
    strncpy(_clientName, clientName, 16); _clientName[16] = '\0';
    _civAddress = civAddress;
    _state       = S_DISCONNECTED;
    _stateMs     = millis();
    _reconnectDelay = 5000;
}

void IcomLanClient::stop() {
    _ctrl.stop();
    _civ.stop();
    _state = S_DISCONNECTED;
    _stateMs = millis();
}

const char* IcomLanClient::statusString() const {
    switch (_state) {
        case S_DISCONNECTED: return "Odpojen";
        case S_AYT:          return "Hledam...";
        case S_LOGIN:        return "Login...";
        case S_TOKEN:        return "Auth...";
        case S_STREAM:       return "Stream...";
        case S_CIV_AYT:      return "CIV...";
        case S_CONNECTED:    return "Pripojeno";
    }
    return "?";
}

void IcomLanClient::setState(State s) {
    _state    = s;
    _stateMs  = millis();
    _aytCount = 0;
    _lastAytMs = 0;
    _civLastAytMs = 0;
    if (s == S_DISCONNECTED) {
        _keepAlive = false;
    }
}

// ── Low-level UDP send ───────────────────────────────────────────────────────

void IcomLanClient::ctrlSend(const uint8_t* data, size_t len) {
    _ctrl.beginPacket(_radioIp, _ctrlPort);
    _ctrl.write(data, len);
    _ctrl.endPacket();
}

void IcomLanClient::ctrlSendTracked(const uint8_t* pkt, size_t len) {
    if (len > 160) return;
    uint8_t buf[160];
    memcpy(buf, pkt, len);
    putU16LE(&buf[6], _ctrlSeq++);
    _ctrl.beginPacket(_radioIp, _ctrlPort);
    _ctrl.write(buf, len);
    _ctrl.endPacket();
    _idleMs = millis();
}

void IcomLanClient::civSend(const uint8_t* data, size_t len) {
    _civ.beginPacket(_radioIp, _civRadioPort);
    _civ.write(data, len);
    _civ.endPacket();
}

void IcomLanClient::civSendTracked(const uint8_t* pkt, size_t len) {
    if (len > 64) return;
    uint8_t buf[64];
    memcpy(buf, pkt, len);
    putU16LE(&buf[6], _civOuterSeq++);
    _civ.beginPacket(_radioIp, _civRadioPort);
    _civ.write(buf, len);
    _civ.endPacket();
    _civIdleMs = millis();
}

// ── Packet builders ──────────────────────────────────────────────────────────

// 16-byte untracked control packet (AreYouThere / AreYouReady / Idle)
void IcomLanClient::sendAreYouThere(bool civ) {
    uint8_t p[16] = {};
    putU32LE(p, 16);
    p[4] = 0x03; // type = AreYouThere
    putU32LE(&p[8], civ ? _civMyId : _myId);
    // rcvdid = 0 (unknown yet)
    if (civ) civSend(p, 16);
    else     ctrlSend(p, 16);
}

// AreYouReady: type=0x06, seq=1, rcvdid filled
void IcomLanClient::sendAreYouReady(bool civ) {
    uint8_t p[16] = {};
    putU32LE(p, 16);
    p[4] = 0x06;
    p[6] = 0x01; // seq=1
    putU32LE(&p[8],  civ ? _civMyId    : _myId);
    putU32LE(&p[12], civ ? _civRemoteId : _remoteId);
    if (civ) civSend(p, 16);
    else     ctrlSend(p, 16);
}

// 128-byte login packet
void IcomLanClient::sendLogin(bool newToken) {
    uint8_t p[128] = {};
    putU32LE(p, 128);
    // type = 0
    putU32LE(&p[8],  _myId);
    putU32LE(&p[12], _remoteId);
    putU32BE(&p[16], 0x70);             // payloadsize = BE(0x70)
    p[20] = 0x01;                       // requestreply
    p[21] = 0x00;                       // requesttype = login
    putU16BE(&p[22], _authSeq++);       // innerseq BE
    // Nový token jen při prvním volání — retransmit musí zachovat původní _tokReq
    if (newToken) _tokReq = (uint16_t)((millis() ^ (_myId >> 16) ^ _myId) & 0xFFFF);
    putU16LE(&p[26], _tokReq);          // tokrequest LE
    // token = 0 at [28..31]
    icomPasscode(_username, &p[64], 16);
    icomPasscode(_password, &p[80], 16);
    strncpy((char*)&p[96], _clientName, 15);
    Serial.print("LAN | Login seq="); Serial.print(_ctrlSeq);
    Serial.print(" tok=0x"); Serial.println(_tokReq, HEX);
    ctrlSendTracked(p, 128);
}

// 64-byte token packet (magic: 0x02=confirm, 0x05=renewal)
void IcomLanClient::sendToken(uint8_t magic) {
    uint8_t p[64] = {};
    putU32LE(p, 64);
    // type = 0
    putU32LE(&p[8],  _myId);
    putU32LE(&p[12], _remoteId);
    putU32BE(&p[16], 0x30);             // payloadsize = BE(0x30)
    p[20] = 0x01;                       // requestreply
    p[21] = magic;                      // requesttype
    putU16BE(&p[22], _authSeq++);       // innerseq BE
    putU16LE(&p[26], _tokReq);
    putU32LE(&p[28], _token);
    if (magic == 0x05) {
        // renewal: resetcap at bytes[36..37] = BE(0x0798)
        putU16BE(&p[36], 0x0798);
    }
    ctrlSendTracked(p, 64);
}

// 144-byte stream request (conninfo)
void IcomLanClient::sendStreamReq() {
    uint8_t p[144] = {};
    putU32LE(p, 144);
    // type = 0
    putU32LE(&p[8],  _myId);
    putU32LE(&p[12], _remoteId);
    putU32BE(&p[16], 0x80);             // payloadsize = BE(0x80)
    p[20] = 0x01;                       // requestreply
    p[21] = 0x03;                       // requesttype = stream
    putU16BE(&p[22], _authSeq++);       // innerseq BE
    putU16LE(&p[26], _tokReq);
    putU32LE(&p[28], _token);

    // guid or mac at bytes[32..47]
    if (_capsGot) {
        if (!_useGuid) {
            // commoncap = LE(0x8010) at [39..40], mac at [42..47]
            p[39] = 0x10; p[40] = 0x80;
            memcpy(&p[42], &_guidOrMac[10], 6);
        } else {
            memcpy(&p[32], _guidOrMac, 16);
        }
        // radio name at [64..95]
        strncpy((char*)&p[64], _radioName, 32);
    }

    // encoded username at [96..111]
    icomPasscode(_username, &p[96], 16);
    p[112] = 0x01;                      // rxenable = 1
    // txenable=0, codecs=0, samples=0
    putU32BE(&p[124], (uint32_t)_civLocalPort); // civport BE uint32
    // audioport=0, txbuffer=0
    p[136] = 0x01;                      // convert = 1
    ctrlSendTracked(p, 144);
}

// 21-byte ping (untracked, goes directly to ctrl)
void IcomLanClient::sendPing() {
    uint8_t p[21] = {};
    putU32LE(p, 21);
    p[4] = 0x07;                        // type = ping
    putU32LE(&p[8],  _myId);
    putU32LE(&p[12], _remoteId);
    p[16] = 0x00;                       // reply = request
    putU32LE(&p[17], (uint32_t)millis());
    ctrlSend(p, 21);
}

// 16-byte idle (tracked) on ctrl port
void IcomLanClient::sendIdle() {
    uint8_t p[16] = {};
    putU32LE(p, 16);
    // type = 0 (idle)
    putU32LE(&p[8],  _myId);
    putU32LE(&p[12], _remoteId);
    ctrlSendTracked(p, 16);
}

// 16-byte idle on CIV port
void IcomLanClient::sendCivIdle() {
    uint8_t p[16] = {};
    putU32LE(p, 16);
    putU32LE(&p[8],  _civMyId);
    putU32LE(&p[12], _civRemoteId);
    // use civSendTracked – but that has 64-byte limit, use civSend directly
    uint8_t buf[16];
    memcpy(buf, p, 16);
    putU16LE(&buf[6], _civOuterSeq++);
    _civ.beginPacket(_radioIp, _civRadioPort);
    _civ.write(buf, 16);
    _civ.endPacket();
    _civIdleMs = millis();
}

// 22-byte OpenClose on CIV port (tracked via civSendTracked)
void IcomLanClient::sendCivOpenClose(bool close) {
    uint8_t p[22] = {};
    putU32LE(p, 22);
    // type = 0
    putU32LE(&p[8],  _civMyId);
    putU32LE(&p[12], _civRemoteId);
    p[16] = 0xc0; p[17] = 0x01;        // data = LE(0x01c0)
    // [18] = 0 (unused)
    putU16BE(&p[19], _civInnerSeq++);   // sendseq BE
    p[21] = close ? 0x00 : 0x04;       // magic: 0x04=open, 0x00=close
    civSendTracked(p, 22);
}

// Send CI-V frame wrapped in data_packet header
bool IcomLanClient::sendCiv(const uint8_t* frame, size_t len) {
    if (_state != S_CONNECTED) return false;
    if (len > 235) return false;        // max safe for 256-byte stack buf
    size_t total = 21 + len;
    uint8_t buf[256];
    memset(buf, 0, 21);
    putU32LE(buf, (uint32_t)total);     // total len LE
    // type = 0
    putU16LE(&buf[6], _civOuterSeq++);  // outer seq LE (tracked inline)
    putU32LE(&buf[8],  _civMyId);
    putU32LE(&buf[12], _civRemoteId);
    buf[16] = 0xc1;                     // reply = CIV data magic
    putU16LE(&buf[17], (uint16_t)len);  // datalen LE
    putU16BE(&buf[19], _civInnerSeq++); // sendseq BE
    memcpy(&buf[21], frame, len);

    _civ.beginPacket(_radioIp, _civRadioPort);
    _civ.write(buf, total);
    _civ.endPacket();
    _civIdleMs = millis();
    return true;
}

// ── Packet processors ────────────────────────────────────────────────────────

void IcomLanClient::processCtrlPkt(const uint8_t* d, size_t n) {
    if (n < 16) return;
    uint16_t pktType = getU16LE(&d[4]);
    uint32_t sentId  = getU32LE(&d[8]);

    // Ping request from radio → reflect back
    if (n == PING_SZ && pktType == 0x07 && d[16] == 0x00) {
        uint8_t resp[PING_SZ];
        memcpy(resp, d, PING_SZ);
        putU32LE(&resp[8],  _myId);
        putU32LE(&resp[12], _remoteId);
        resp[16] = 0x01; // reply
        ctrlSend(resp, PING_SZ);
        return;
    }
    // Ping reply (odpověď na naše pings) — normální keepalive, tiše ignorovat
    if (n == PING_SZ && pktType == 0x07 && d[16] == 0x01) return;
    // Idle ack (type=0x00, 16 B) — rádio potvrzuje naše idle pakety
    if (n == CTRL_SZ && pktType == 0x00) return;

    // ── Capabilities packet — zpracovat bez ohledu na stav ──────────────────────
    // IC-705 posílá Caps PŘED Login_Response (wfview to zpracovává v default: case,
    // tj. nezávisle na stavu). V naší state-machine by Caps v S_LOGIN byl ignorován.
    if (n >= (size_t)(CAPS_HDR_SZ + CAPS_RADIO_SZ) &&
        ((n - CAPS_HDR_SZ) % CAPS_RADIO_SZ == 0)) {
        uint16_t numRadios = getU16LE(&d[64]);
        if (numRadios >= 1) {
            const uint8_t* cap = &d[CAPS_HDR_SZ];
            uint16_t commoncap = getU16LE(&cap[7]);
            if (commoncap == 0x8010) {
                _useGuid = false;
                memset(_guidOrMac, 0, 16);
                _guidOrMac[7] = cap[7]; _guidOrMac[8] = cap[8];
                memcpy(&_guidOrMac[10], &cap[10], 6);
            } else {
                _useGuid = true;
                memcpy(_guidOrMac, cap, 16);
            }
            strncpy(_radioName, (const char*)&cap[16], 32);
            _radioName[32] = '\0';
            _capsGot = true;
            Serial.print("LAN | Caps: "); Serial.println(_radioName);
        }
        return;
    }

    switch (_state) {
    case S_AYT:
        if (pktType == 0x04) {
            Serial.println("LAN | IAmHere->AYR");
            _remoteId  = sentId;
            _aytCount  = 255;
            _keepAlive = true;
            _pingMs    = millis();
            _idleMs    = millis();
            sendAreYouReady(false);
        } else if (pktType == 0x06) {
            Serial.println("LAN | IAmReady->Login");
            sendLogin();
            setState(S_LOGIN);
        }
        break;

    case S_LOGIN:
        if (n == LOGIN_RESP_SZ) {
            uint32_t err    = getU32LE(&d[48]);
            uint16_t tokRsp = getU16LE(&d[26]);
            Serial.print("LAN | LoginResp err=0x"); Serial.print(err, HEX);
            Serial.print(" tok=0x"); Serial.print(tokRsp, HEX);
            Serial.print(" req=0x"); Serial.println(_tokReq, HEX);
            if (err == 0xFEFFFFFF || err == 0xFFFFFFFF) {
                Serial.println("LAN | Login odmitnut – spatny username/password");
                _reconnectDelay = 30000;
                setState(S_DISCONNECTED);
                return;
            }
            if (tokRsp == _tokReq || err == 0x00000000) {
                _token = getU32LE(&d[28]);
                _tokReq = tokRsp;  // synchronizovat — Token packet musí echo to co radio poslalo
                sendToken(0x02);
                setState(S_TOKEN);
            } else {
                Serial.println("LAN | Login: neshoda tokenu, zkousim znovu");
                setState(S_DISCONNECTED);
                _reconnectDelay = min(_reconnectDelay * 2, 30000UL);
            }
        } else if (n == CTRL_SZ && pktType == 0x06) {
            Serial.println("LAN | Login: IAmReady retransmit, posilam Login znovu");
            _remoteId = sentId;
            sendLogin(false);  // zachovat původní _tokReq!
        }
        // ostatní délky jsou zachyceny dumem výše — žádný další log
        break;

    case S_TOKEN: {
        // Caps jsou zpracovány state-agnosticky výše (před switch).
        // Zde čekáme jen na Conninfo (144B) → spustit stream request.
        if (n == CONNINFO_SZ) {
            sendStreamReq();
            setState(S_STREAM);
        }
        break;
    }

    case S_STREAM:
        if (n == STATUS_SZ) {
            uint32_t err  = getU32LE(&d[48]);
            uint8_t  disc = d[64];
            if (err == 0xFFFFFFFF) {
                Serial.println("LAN | Status chyba");
                setState(S_DISCONNECTED);
                _reconnectDelay = min(_reconnectDelay * 2, 30000UL);
                return;
            }
            if (disc == 0x01) {
                Serial.println("LAN | Radio odpojeno");
                setState(S_DISCONNECTED);
                _reconnectDelay = min(_reconnectDelay * 2, 30000UL);
                return;
            }
            _civRadioPort = getU16BE(&d[66]); // civport BE uint16
            if (_civRadioPort == 0) _civRadioPort = 50002;
            Serial.print("LAN | CIV port: "); Serial.println(_civRadioPort);

            // Open CIV socket — pick a pseudo-random local port
            _civ.stop();
            _civLocalPort = 50200 + (uint16_t)(millis() & 0x3FF); // 50200–50823
            _civ.begin(_civLocalPort);
            IPAddress localIp = WiFi.localIP();
            _civMyId = ((uint32_t)localIp[2] << 24) |
                       ((uint32_t)localIp[3] << 16) |
                       (_civLocalPort & 0xFFFF);
            _civRemoteId = 0;
            _lastCivDataMs = millis();
            setState(S_CIV_AYT);
            sendAreYouThere(true);
        }
        break;

    case S_CONNECTED:
        // Token renewal response
        if (n == TOKEN_SZ && d[21] == 0x05 && d[20] == 0x02) {
            _tokenRenewMs = millis();
        }
        break;

    default:
        break;
    }
}

void IcomLanClient::processCivPkt(const uint8_t* d, size_t n) {
    if (n < 16) return;
    uint16_t pktType = getU16LE(&d[4]);
    uint32_t sentId  = getU32LE(&d[8]);

    // Ping request on CIV port → reflect
    if (n == PING_SZ && pktType == 0x07 && d[16] == 0x00) {
        uint8_t resp[PING_SZ];
        memcpy(resp, d, PING_SZ);
        putU32LE(&resp[8],  _civMyId);
        putU32LE(&resp[12], _civRemoteId);
        resp[16] = 0x01;
        civSend(resp, PING_SZ);
        return;
    }

    switch (_state) {
    case S_CIV_AYT:
        if (pktType == 0x04) {
            // IAmHere on CIV → send AreYouReady
            _civRemoteId = sentId;
            sendAreYouReady(true);
        } else if (pktType == 0x06) {
            // IAmReady on CIV → open CI-V stream
            _civRemoteId = sentId;
            sendCivOpenClose(false);
            setState(S_CONNECTED);
            _reconnectDelay = 5000;
            _lastCivDataMs = millis();
            _pingMs        = millis();
            _idleMs        = millis();
            _civIdleMs     = millis();
            _tokenRenewMs  = millis();
            Serial.println("LAN | Pripojeno!");
        }
        break;

    case S_CONNECTED:
        if (n > 21 && d[16] == 0xc1) {
            // CI-V data packet
            uint16_t civLen = getU16LE(&d[17]);
            if (civLen > 0 && civLen <= (n - 21)) {
                _lastCivDataMs = millis();
                if (_civCb) _civCb(&d[21], civLen);
            }
        } else if (pktType == 0x04) {
            _civRemoteId = sentId;
            sendAreYouReady(true);
        } else if (pktType == 0x06) {
            _civRemoteId = sentId;
            sendCivOpenClose(false);
        }
        break;

    default:
        break;
    }
}

// ── Main loop ────────────────────────────────────────────────────────────────

void IcomLanClient::loop() {
    unsigned long now = millis();

    // Receive and process control packets
    {
        uint8_t buf[200];
        int plen;
        while ((plen = _ctrl.parsePacket()) > 0) {
            size_t n = (plen <= (int)sizeof(buf)) ? (size_t)plen : sizeof(buf);
            _ctrl.read(buf, n);
            if ((size_t)plen > sizeof(buf)) { /* discard remainder */ }
            processCtrlPkt(buf, n);
        }
    }

    // Receive and process CIV packets (only when CIV socket is open)
    if (_state >= S_CIV_AYT) {
        uint8_t buf[512]; // up to 21 + 475 scope data
        int plen;
        while ((plen = _civ.parsePacket()) > 0) {
            size_t n = (plen <= (int)sizeof(buf)) ? (size_t)plen : sizeof(buf);
            _civ.read(buf, n);
            processCivPkt(buf, n);
        }
    }

    // Ping od IAmHere; idle jen po S_CONNECTED (jako v pracující session)
    if (_keepAlive && now - _pingMs >= 500) { _pingMs = now; sendPing(); }
    if (_state == S_CONNECTED && now - _idleMs >= 100) { sendIdle(); }

    // State machine timers
    switch (_state) {

    case S_DISCONNECTED:
        if (now - _stateMs >= _reconnectDelay) {
            // Poslat Disconnect aby rádio vědělo, že předchozí session skončila.
            // Bez toho IC-7610 může nové Login ignorovat (myslí si, že session stále běží).
            if (_remoteId != 0) {
                uint8_t disc[16] = {};
                putU32LE(disc, 16);
                disc[4] = 0x05;  // type = Disconnect
                putU32LE(&disc[8],  _myId);
                putU32LE(&disc[12], _remoteId);
                ctrlSend(disc, 16);
            }
            // Reset and try to connect — pick a pseudo-random local control port
            _ctrl.stop();
            _civ.stop();
            IPAddress localIp = WiFi.localIP();
            if (localIp == IPAddress(0,0,0,0)) break; // WiFi not ready yet
            uint16_t ctrlLocalPort = 50100 + (uint16_t)(millis() & 0x3FF); // 50100–50823
            _ctrl.begin(ctrlLocalPort);
            _myId = ((uint32_t)localIp[2] << 24) |
                    ((uint32_t)localIp[3] << 16) |
                    (ctrlLocalPort & 0xFFFF);
            _remoteId    = 0;
            _civRemoteId = 0;
            _token       = 0;
            _ctrlSeq     = 1;
            _authSeq     = 0x30;
            _civOuterSeq = 1;
            _civInnerSeq = 0;
            _capsGot     = false;
            memset(_radioName, 0, sizeof(_radioName));
            Serial.print("LAN | Pripojuji k "); Serial.println(_radioIp);
            setState(S_AYT);
            sendAreYouThere(false);
        }
        break;

    case S_AYT:
        if (_aytCount < 255 && now - _lastAytMs >= 500) {
            _lastAytMs = now;
            sendAreYouThere(false);
            if (_aytCount < 254) _aytCount++;
        }
        if (now - _stateMs >= 30000) {
            Serial.println("LAN | AYT timeout");
            setState(S_DISCONNECTED);
            _reconnectDelay = min(_reconnectDelay * 2, 30000UL);
        }
        break;

    case S_LOGIN:
        if (now - _stateMs >= 20000) {
            Serial.println("LAN | Login timeout");
            setState(S_DISCONNECTED);
            _reconnectDelay = min(_reconnectDelay * 2, 30000UL);
        }
        break;

    case S_TOKEN:
        // Fallback: if caps received but no conninfo after 8s, try stream
        if (_capsGot && now - _stateMs >= 8000) {
            Serial.println("LAN | Token: posilam stream pozadavek (caps, no conninfo)");
            sendStreamReq();
            setState(S_STREAM);
        } else if (!_capsGot && now - _stateMs >= 15000) {
            Serial.println("LAN | Token timeout (bez caps), zkousim stream");
            sendStreamReq();
            setState(S_STREAM);
        }
        break;

    case S_STREAM:
        if (now - _stateMs >= 10000) {
            Serial.println("LAN | Stream timeout");
            setState(S_DISCONNECTED);
            _reconnectDelay = min(_reconnectDelay * 2, 30000UL);
        }
        break;

    case S_CIV_AYT:
        if (now - _civLastAytMs >= 500) {
            _civLastAytMs = now;
            sendAreYouThere(true);
        }
        if (now - _stateMs >= 10000) {
            Serial.println("LAN | CIV AYT timeout");
            setState(S_DISCONNECTED);
            _reconnectDelay = min(_reconnectDelay * 2, 30000UL);
        }
        break;

    case S_CONNECTED:
        // ping + idle handled by _keepAlive block above
        // Idle every 100ms on CIV port
        if (now - _civIdleMs >= 100) {
            sendCivIdle();
        }
        // Token renewal every 60s
        if (now - _tokenRenewMs >= 60000) {
            _tokenRenewMs = now;
            sendToken(0x05);
        }
        // CI-V watchdog: no data in 5s → resend OpenClose
        if (now - _lastCivDataMs >= 5000) {
            _lastCivDataMs = now;
            Serial.println("LAN | CIV watchdog: OpenClose");
            sendCivOpenClose(false);
        }
        break;
    }
}
