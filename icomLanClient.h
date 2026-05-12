#pragma once
#include <Arduino.h>
#include <WiFiUdp.h>
#include <functional>

// ICOM proprietární UDP protokol (RS-BA1 kompatibilní)
// Referenece: wfview-master/src/radio/icomudphandler.cpp + icomudpcivdata.cpp

class IcomLanClient {
public:
    void begin(IPAddress radioIp, uint16_t controlPort,
               const char* username, const char* password,
               const char* clientName, uint8_t civAddress);
    void stop();
    void loop();

    bool sendCiv(const uint8_t* frame, size_t len);
    bool isConnected() const { return _state == S_CONNECTED; }
    void onCivReceived(std::function<void(const uint8_t*, size_t)> cb) { _civCb = cb; }
    const char* statusString() const;

private:
    enum State : uint8_t {
        S_DISCONNECTED = 0,
        S_AYT,       // AreYouThere → IAmHere → AreYouReady → IAmReady
        S_LOGIN,     // Login → LoginResponse
        S_TOKEN,     // Token(0x02) → Capabilities + Conninfo
        S_STREAM,    // StreamRequest → Status(civPort)
        S_CIV_AYT,   // CIV port handshake
        S_CONNECTED,
    };

    // Low-level send
    void ctrlSend(const uint8_t* data, size_t len);
    void ctrlSendTracked(const uint8_t* pkt, size_t len);
    void civSend(const uint8_t* data, size_t len);
    void civSendTracked(const uint8_t* pkt, size_t len);

    // Packet builders
    void sendAreYouThere(bool civ = false);
    void sendAreYouReady(bool civ = false);
    void sendLogin(bool newToken = true);
    void sendToken(uint8_t magic);
    void sendStreamReq();
    void sendPing();
    void sendIdle();
    void sendCivIdle();
    void sendCivOpenClose(bool close);

    // Packet processors
    void processCtrlPkt(const uint8_t* d, size_t n);
    void processCivPkt(const uint8_t* d, size_t n);

    void setState(State s);
    void icomPasscode(const char* input, uint8_t* out, uint8_t maxLen);

    // Byte order helpers
    static void putU32LE(uint8_t* b, uint32_t v);
    static void putU16LE(uint8_t* b, uint16_t v);
    static void putU32BE(uint8_t* b, uint32_t v);
    static void putU16BE(uint8_t* b, uint16_t v);
    static uint32_t getU32LE(const uint8_t* b);
    static uint16_t getU16LE(const uint8_t* b);
    static uint16_t getU16BE(const uint8_t* b);

    WiFiUDP  _ctrl;
    WiFiUDP  _civ;
    IPAddress _radioIp;
    uint16_t _ctrlPort    = 50001;
    uint16_t _civRadioPort = 50002; // remote CIV port (from status packet)
    uint16_t _civLocalPort = 0;     // our local CIV socket port

    uint32_t _myId      = 0; // control port identity
    uint32_t _civMyId   = 0; // CIV port identity
    uint32_t _remoteId  = 0;
    uint32_t _civRemoteId = 0;
    uint32_t _token     = 0;
    uint16_t _tokReq    = 0;
    uint16_t _ctrlSeq   = 1;
    uint16_t _authSeq   = 0x30;
    uint16_t _civOuterSeq = 1;
    uint16_t _civInnerSeq = 0;
    uint8_t  _civAddress  = 0xA4;

    char _username[17]   = {};
    char _password[17]   = {};
    char _clientName[17] = {};
    char _radioName[33]  = {}; // from capabilities packet
    bool _useGuid        = false;
    uint8_t _guidOrMac[16] = {}; // guid[16] OR mac[6] at [10..15] with commoncap at [7..8]
    bool _capsGot        = false;

    State         _state          = S_DISCONNECTED;
    unsigned long _stateMs        = 0;
    unsigned long _reconnectDelay = 5000;
    unsigned long _pingMs         = 0;
    unsigned long _idleMs         = 0;
    unsigned long _civIdleMs      = 0;
    unsigned long _tokenRenewMs   = 0;
    bool          _keepAlive      = false; // true po IAmHere — posílat ping+idle i v login fázi
    unsigned long _lastAytMs        = 0;
    unsigned long _civLastAytMs     = 0;
    unsigned long _lastCivDataMs    = 0;
    unsigned long _loginPendingMs   = 0; // >0 = čas posledního IAmReady, Login odesíláme po 50ms delay
    uint16_t      _aytCount       = 0; // 255 = received IAmHere, stop sending AYT

    std::function<void(const uint8_t*, size_t)> _civCb;
};
