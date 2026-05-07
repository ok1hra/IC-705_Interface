/*
  Based on ESP32 BT CAT for IC-705 by OK1CDJ ondra@ok1cdj.com
  https://github.com/ok1cdj/IC705-BT-CIV
  -----------------------------------------------------------
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  -----------------------------------------------------------

  Compile for "ESP32 Dev Module" board with Partition Scheme: "No OTA (2MB APP/2MB SPIFFS)"
  With libraries
  Using library EEPROM at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/EEPROM 
  Using library BluetoothSerial at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/BluetoothSerial 
  Using library WiFi at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/WiFi 
  Using library ESPmDNS at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/ESPmDNS 
  Using library WebServer at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/WebServer 
  Using library PubSubClient at version 2.8 in folder: /home/dan/Arduino/libraries/PubSubClient 
  Using library FS at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/FS 

  Features
  + Connecting the IC-705 via Bluetooth and sending the frequency to MQTT
  + Frequency and mode for PHP log available on http port 81 (address http://ic705.local:81)
  + UDP port 89 receives ascii characters, which it sends via Bluetooth to the IC-705, and transmit them as a CW message
  + UDP port 89 receives ascii characters, which it sends in RTTY mode by keying FSK and PTT TRX inputs
  + Status LED
    - Fade in/out - WiFi in AP mode
    - WiFi in client mode
      - ON waiting connected to WiFi
      - OFF Wifi connected to AP
      - FLASH send MQTT freq
      - DOUBLE FLASH receive CW via UDP
      - FLASH+PTT receive RTTY via UDP
  + mDNS - to easily find IP devices in the network, using the command "ping ic705.local"
  + Watchdog - resets the device after more than 73 seconds of inactivity
  + Output signal POWER-OUT (13.8V/0.5A) with LED activates after connecting BT (can turn on your hamshack)
  + Galvanically isolated CI-V output for connecting PA or other devices
  + CIV-MUTE on gpio16 allow send to CI-V output only commands with frequency (not debug messages)
  + UDP port for CAT command (clear RIT) from log
  + after BT connect, set TRX to enable: CI-V transceive + enable RIT + enable BK-IN
  + support external shift register control switch by frequency (not tested)
  + Detect PCB hardware ID
  + postponed MQTT
  + BT name to configure
  + detect HW rev 02
  + after connect WiFi, send assigned IP address with CW on TRX
  + send ? in serial terminal, answer interface status
  + add AP mode - status LED signal AP mode by slowly turning on and off (fade in / fade out)
  + add setup http web form on port 80
  + add HW rev 3 detection
  + optional reset after diconnect
  + add Debug to CLI

//--------------------------------------------------------------------*/

String SSID         = "";
String PSWD         = "";
String SSID2        = "";
String PSWD2        = "";
byte mqttBroker[4]  = {0,0,0,0};
int MQTT_PORT       = 0;
String MQTT_TOPIC   = "";
String MQTT_TOPIC_RX = "";
int HTTP_CAT_PORT   = 0;
int udpPort         = 0; // UDP port | echo -n "cq de ok1hra ok1hra test k;" | nc -u -w1 192.168.1.19 89
int udpCatPort      = 0;
int BaudRate        = 9600;
// char* BTname        = "";
const char* BTname  = "IC705-interface";
bool Debug          = false;

#define REV 20260505
#define WIFI
#define MQTT
#define UDP_TO_CW
#define UDP_TO_FSK
#define SELECT_ANT  // enable external shift registrer antenna switching (not tested)
#define HTTP        // http propagation freq|mode|
#define WDT         // watchdog timer
#define CIV_OUT     // send freq to CIV out with BaudRate
#define UDP_TO_CAT  // data command from udpCatPort send to TRX | FE FE A4 E0 <command> FD
#define BLUETOOTH   // BT
// #define RTLE     // not work now | credit OK2CQR https://github.com/ok2cqr/rtle/tree/master
// #define RESET_AFTER_DISCONNECT  // enable reset after each disconnect + short CW msg

#include "EEPROM.h"
#define EEPROM_SIZE 136
/*
  0|Byte    1|128
  1|Char    1|A
  2|UChar   1|255
  3|Short   2|-32768
  5|UShort  2|65535
  7|Int     4|-2147483648
  11|Uint    4|4294967295
  15|Long    4|-2147483648
  19|Ulong   4|4294967295
  23|Long64  8|0x00FFFF8000FF4180
  31|Ulong64 8|0x00FFFF8000FF4180
  39|Float   4|1234.1234
  43|Double  8|123456789.12345679
  51|Bool    1|1

  0 APmode
  1-20 SSID1
  22-39 PSWD1
  41-44 mqttBroker[4]
  45-46 MQTT_PORT
  47-67 MQTT_TOPIC
  115-135 MQTT_TOPIC_RX
  68-69 HTTP_CAT_PORT
  70-71 udpPort
  72-73 udpCatPort
  74-75 BaudRate
  76-95 SSID2
  97-114 PSWD2

  !! Increment EEPROM_SIZE #define !!
*/

#if defined(SELECT_ANT)
  unsigned long ANTrange[8][2] = {/* TRXfreq[0]
  Freq Hz from       to        ANT
  */  {1810000,   2000000},  // #1
      {3500000,   3800000},  // #2
      {7000000,   7200000},  // #3
      {10100000, 10150000},  // #4
      {14000000, 14350000},  // #5
      {21000000, 21450000},  // #6
      {28000000, 29700000},  // #7
      {50000000, 52000000}   // #8
  };
  int TRXselectANT = 42; // how antenna selcted - blocked for second TRX - don't be the same!
  const int ShiftOutClockPin = 12;
  const int ShiftOutDataPin  = 13;
  const int ShiftOutLatchPin = 14;
  byte ShiftOutByte;
#endif

#if defined(BLUETOOTH)
  #include "BluetoothSerial.h"
  //#define DEBUG 1
  //#define MIRRORCAT 1
  //#define MIRRORCAT_SPEED 9600
  #define BROADCAST_ADDRESS    0x00 //Broadcast address
  #define CONTROLLER_ADDRESS   0xE0 //Controller address

  #define START_BYTE           0xFE //Start byte
  #define STOP_BYTE            0xFD //Stop byte

  #define CMD_TRANS_FREQ       0x00 //Transfers operating frequency data
  #define CMD_TRANS_MODE       0x01 //Transfers operating mode data

  #define CMD_READ_FREQ        0x03 //Read operating frequency data
  #define CMD_READ_MODE        0x04 //Read operating mode data

  #define CMD_WRITE_FREQ       0x05 //Write operating frequency data
  #define CMD_WRITE_MODE       0x06 //Write operating mode data

  #define CMD_SEND_CW_MSG      0x17 //Write operating mode data

  #define IF_PASSBAND_WIDTH_WIDE   0x01
  #define IF_PASSBAND_WIDTH_MEDIUM   0x02
  #define IF_PASSBAND_WIDTH_NARROW   0x03

  const uint32_t decMulti[]    = {1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1};

  #define BAUD_RATES_SIZE 4
  const uint16_t baudRates[BAUD_RATES_SIZE]       = {19200, 9600, 4800, 1200};

  uint8_t  radio_address;     //Transiever address
  uint16_t  baud_rate;        //Current baud speed
  uint32_t readtimeout = 2000; // BUGFIX Serial port read timeout
  uint8_t  read_buffer[12];   //Read buffer
  // uint8_t  read_buffer_snapshot[12];   //Buffer snapshot
  uint32_t  frequency;        //Current frequency in Hz
  uint32_t  frequencyTmp;        //Current frequency in Hz
  uint32_t  timer;

  const char* mode[] = {"LSB", "USB", "AM", "CW", "FSK", "FM", "WFM", "DV"};
  #define MODE_TYPE_LSB   0x00
  #define MODE_TYPE_USB   0x01
  #define MODE_TYPE_AM    0x02
  #define MODE_TYPE_CW    0x03
  #define MODE_TYPE_RTTY  0x04
  #define MODE_TYPE_FM    0x05
  #define MODE_TYPE_DV    0x17

  String modes;
  BluetoothSerial CAT;
  bool btClientConnected = false;
#endif

short HardwareRev = 99;
const int HWidPin       = 34;  // analog
int HWidValue           = 0;

int pwmChannel          = 0; // Selects channel 0
int frequence           = 1000; // PWM frequency of 1 KHz
int resolution          = 8; // 8-bit resolution, 256 possible values

const int StatusPin     = 5;
const int PowerOnPin    = 4;
long powerTimer         = 0;
bool statusPower        = 0;
const int CIVmutePin    = 16;
bool TrxNeedSet         = 0;
bool TrxSetupDone       = false;
volatile bool btDisconnectPending = false;
long mqttPostponeTimer  = 0;
bool mqttPostponeStatus = 1;

#if defined(WDT)
  // 73 seconds WDT (WatchDogTimer)
  #include <esp_task_wdt.h>
  #define WDT_TIMEOUT 73
  long WdtTimer=0;
#endif

#if defined(WIFI)
  #include <WiFi.h>
  #include <FS.h>
  #include <SPIFFS.h>
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <mbedtls/sha1.h>
  // #include <ETH.h>
  // int SsidPassSize = (sizeof(SsidPass)/sizeof(char *))/2; //array size
  // int SelectSsidPass = -1;
  #define wifi_max_try 40             // Number of try
  unsigned long WifiTimer = 0;
  unsigned long WifiReconnect = 30000;
  byte ActiveWifiProfile = 0;
  String MACString;

  #include <ESPmDNS.h>

  const char* ssidAP     = "IC705-if";
  const char* passwordAP = "remoteqth";
  bool APmode = false;
  AsyncWebServer ajaxserver(80);
  AsyncWebSocket ws("/ws");
#endif
  #if defined(RTLE)
    #include <WebServer.h>
    #include "rtle.h"  //Web page header file
    WebServer rtleserver(88);
  #endif

#if defined(HTTP)
  WiFiServer server; //(HTTP_CAT_PORT);
  char linebuf[80];
  int charcount=0;

  WiFiServer serverForm(80);
  String header;
#endif

#if defined(UDP_TO_CW)
  #include <WiFiUdp.h>
  WiFiUDP udp;
#endif

#if defined(UDP_TO_CAT)
  WiFiUDP udpCat;
#endif

// uint8_t CwMsg[36] = "";
char CwMsg[37] = "";
uint8_t CwCat[36] = "";

#if defined(UDP_TO_FSK)
  #define FSK_OUT  33                      // TTL LEVEL pin OUTPUT
  #define PTT      32                      // PTT pin OUTPUT
  #define FSK_MARK_LEVEL   LOW             // FSK mark level [LOW/HIGH]
  #define FSK_SPACE_LEVEL  HIGH            // FSK space level [LOW/HIGH]
  #define BaudRateFSK 45.45                   // RTTY baud rate
  #define StopBit  1.5                     // stop bit long
  #define PTTlead  400                     // PTT lead delay ms
  #define PTTtail  200                     // PTT tail delay ms
  int     OneBit = 1/BaudRateFSK*1000;
  boolean d1;
  boolean d2;
  boolean d3;
  boolean d4;
  boolean d5;
  boolean space;
  boolean fig1;
  int     fig2;
  char    ch;
#endif

#if defined(UDP_TO_CAT)
  // uint8_t CatMsg[11] = "";
  byte CatMsg[10];
#endif

#if defined(MQTT)
  bool mqttEnable = true;
  #include <PubSubClient.h>
  IPAddress mqtt_server_ip(mqttBroker[0], mqttBroker[1], mqttBroker[2], mqttBroker[3]);       // MQTT broker IP address
  // #include "PubSubClient.h" // lokalni verze s upravou #define MQTT_MAX_PACKET_SIZE 128
  WiFiClient espClient;
  PubSubClient mqttClient(espClient);
  // PubSubClient mqttClient(server, 1883, callback, ethClient);
  long lastMqttReconnectAttempt = 0;
  boolean MQTT_LOGIN      = 0;          // enable MQTT broker login
  // char MQTT_USER= 'login';    // MQTT broker user login
  // char MQTT_PASS= 'passwd';   // MQTT broker password
  const int MqttBuferSize = 1000; // 1000
  char mqttTX[MqttBuferSize];
  char mqttPath[MqttBuferSize];
  char mqttClientId[32];
  long MqttStatusTimer[2]{1500,1000};
#endif

int incomingByte = 0;   // for incoming serial data

#if defined(WIFI)
typedef struct {
  uint32_t state[5];
  uint32_t count[2];
  unsigned char buffer[64];
} SHA1_CTX;

extern "C" void SHA1Init(SHA1_CTX* context){
  mbedtls_sha1_init(reinterpret_cast<mbedtls_sha1_context*>(context));
  mbedtls_sha1_starts_ret(reinterpret_cast<mbedtls_sha1_context*>(context));
}

extern "C" void SHA1Update(SHA1_CTX* context, const unsigned char* data, uint32_t len){
  mbedtls_sha1_update_ret(reinterpret_cast<mbedtls_sha1_context*>(context), data, len);
}

extern "C" void SHA1Final(unsigned char digest[20], SHA1_CTX* context){
  mbedtls_sha1_finish_ret(reinterpret_cast<mbedtls_sha1_context*>(context), digest);
  mbedtls_sha1_free(reinterpret_cast<mbedtls_sha1_context*>(context));
}
#endif

#if defined(WIFI)
  const uint8_t WS_QUEUE_DEPTH = 4;
  const uint8_t WS_SPLIT_PROBE_STEPS = 4;

  enum WsPendingMatchType : uint8_t {
    WS_MATCH_NONE = 0,
    WS_MATCH_ANY_FRAME = 1,
    WS_MATCH_COMMAND = 2,
    WS_MATCH_ACK = 3
  };

  struct WsPendingRequest {
    bool active;
    uint32_t clientId;
    unsigned long deadlineMs;
    String reqId;
    WsPendingMatchType matchType;
    uint8_t expectedCommand;
    uint8_t expectedPayloadLen;
    uint8_t expectedPayload[4];
  };

  struct WsQueuedRequest {
    bool used;
    uint32_t clientId;
    unsigned long timeoutMs;
    String reqId;
    WsPendingMatchType matchType;
    uint8_t expectedCommand;
    uint8_t expectedPayloadLen;
    uint8_t expectedPayload[4];
    size_t frameLen;
    uint8_t frame[70];
  };

  struct WsSplitProbeState {
    bool active;
    uint32_t clientId;
    String reqId;
    uint8_t step;
  };

  WsPendingRequest wsPendingRequest{false, 0, 0, "", WS_MATCH_NONE, 0, 0, {0, 0, 0, 0}};
  WsQueuedRequest wsQueuedRequests[WS_QUEUE_DEPTH]{};
  WsSplitProbeState wsSplitProbe{false, 0, "", 0};
  String setupSsidErr = "";
  String setupPswdErr = "";
  String setupSsid2Err = "";
  String setupPswd2Err = "";
  String setupMqttErr = "";
  String setupMqttErr1 = "";
  String setupMqttErr2 = "";
  String setupMqttErr3 = "";
  String setupMqttErr4 = "";
  String setupMqttPortErr = "";
  String setupMqttTopicErr = "";
  String setupMqttTopicRxErr = "";
  String setupHttpCatPortErr = "";
  String setupUdpPortErr = "";
  String setupUdpCatPortErr = "";
  String requestArg(AsyncWebServerRequest *request, const char *name);
  bool requestHasArg(AsyncWebServerRequest *request, const char *name);
  String jsonEscape(const String &value);
  String civFrameToHex(const uint8_t *frame, size_t frameLen);
  uint32_t decodeCivFrequencyBytes(const uint8_t *bytes, size_t byteCount);
  uint32_t decodeCivBcdBytes(const uint8_t *bytes, size_t byteCount);
  String decodeModeName(uint8_t modeId);
  String wsDecodeCivFrameJson(const uint8_t *frame, size_t frameLen);
  String wsBuildStateFields(void);
  String wsBuildStateJson(const char *type, const String &reqId = "");
  void wsBroadcastRigState(void);
  void wsBroadcastCivFrame(const char *type, const uint8_t *frame, size_t frameLen, const String &reqId = "");
  void wsSendError(AsyncWebSocketClient *client, const String &code, const String &message, const String &reqId = "");
  void wsSendErrorById(uint32_t clientId, const String &code, const String &message, const String &reqId);
  void wsSendHello(AsyncWebSocketClient *client);
  String extractJsonString(const String &json, const char *key);
  bool extractJsonBool(const String &json, const char *key, bool defaultValue);
  bool parseHexPayload(const String &hex, uint8_t *buffer, size_t &bufferLen, size_t maxLen);
  bool catWriteFrame(const uint8_t *frame, size_t frameLen, bool broadcastTx);
  void wsSendQueued(AsyncWebSocketClient *client, const String &reqId, uint8_t depth);
  bool wsStartRequest(uint32_t clientId, const String &reqId, WsPendingMatchType matchType, uint8_t expectedCommand, const uint8_t *expectedPayload, uint8_t expectedPayloadLen, unsigned long timeoutMs, const uint8_t *frame, size_t frameLen);
  bool wsQueuePendingRequest(AsyncWebSocketClient *client, const String &reqId, WsPendingMatchType matchType, uint8_t expectedCommand, const uint8_t *expectedPayload, uint8_t expectedPayloadLen, unsigned long timeoutMs, const uint8_t *frame, size_t frameLen);
  void wsClearPendingRequest(void);
  void wsClearQueuedRequest(uint8_t index);
  void wsFlushQueuedRequests(const String &code, const String &message);
  void wsDropQueuedRequestsForClient(uint32_t clientId);
  void wsTryStartNextQueuedRequest(void);
  void wsProcessPendingTimeout(void);
  bool wsResolvePendingRequest(const uint8_t *frame, size_t frameLen, String *matchedReqId = nullptr);
  bool parseModeId(const String &modeName, uint8_t &modeId);
  uint8_t parseFilterWidth(const String &filterName);
  size_t buildSimpleCatFrame(uint8_t command, const uint8_t *payload, size_t payloadLen, uint8_t *frame, size_t frameMaxLen);
  size_t buildSetFrequencyFrame(uint32_t freqHz, uint8_t *frame, size_t frameMaxLen);
  size_t buildSetModeFrame(uint8_t modeId, uint8_t modeWidth, uint8_t *frame, size_t frameMaxLen);
  size_t buildReadQuickSplitFrame(uint8_t *frame, size_t frameMaxLen);
  size_t buildReadSplitProbeFrame(uint8_t step, uint8_t *frame, size_t frameMaxLen);
  bool wsStartSplitProbe(AsyncWebSocketClient *client, const String &reqId);
  void wsClearSplitProbe(void);
  void resetSetupMessages(void);
  String setupTemplateProcessor(const String &key);
  void renderSetupPage(AsyncWebServerRequest *request);
  void wsHandleTextMessage(AsyncWebSocketClient *client, const String &msg);
  void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
  void setupAsyncHttpServer(void);
  void handleSet(AsyncWebServerRequest *request);
#endif

//-------------------------------------------------------------------------------------------------------

#if defined(WIFI)
String requestArg(AsyncWebServerRequest *request, const char *name){
  AsyncWebParameter *param = request->getParam(name, true);
  if (param != nullptr) {
    return param->value();
  }
  param = request->getParam(name);
  if (param != nullptr) {
    return param->value();
  }
  return String();
}

bool requestHasArg(AsyncWebServerRequest *request, const char *name){
  return request->hasParam(name, true) || request->hasParam(name);
}

String jsonEscape(const String &value){
  String out;
  out.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); i++) {
    char c = value.charAt(i);
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out += c; break;
    }
  }
  return out;
}

String civFrameToHex(const uint8_t *frame, size_t frameLen){
  String hex;
  hex.reserve(frameLen * 2);
  for (size_t i = 0; i < frameLen; i++) {
    if (frame[i] < 16) {
      hex += "0";
    }
    hex += String(frame[i], HEX);
  }
  return hex;
}

uint32_t decodeCivFrequencyBytes(const uint8_t *bytes, size_t byteCount){
  uint32_t value = 0;
  for (size_t i = 0; i < byteCount; i++) {
    size_t src = byteCount - 1 - i;
    value += (bytes[src] >> 4) * decMulti[i * 2];
    value += (bytes[src] & 0x0F) * decMulti[i * 2 + 1];
  }
  return value;
}

uint32_t decodeCivBcdBytes(const uint8_t *bytes, size_t byteCount){
  uint32_t value = 0;
  for (size_t i = 0; i < byteCount; i++) {
    value = value * 100 + ((bytes[i] >> 4) * 10) + (bytes[i] & 0x0F);
  }
  return value;
}

String decodeModeName(uint8_t modeId){
  switch (modeId) {
    case 0x00: return "LSB";
    case 0x01: return "USB";
    case 0x02: return "AM";
    case 0x03: return "CW";
    case 0x04: return "RTTY";
    case 0x05: return "FM";
    case 0x06: return "WFM";
    case 0x07: return "CW-R";
    case 0x08: return "RTTY-R";
    case 0x17: return "DV";
    default: return "UNK";
  }
}

String decodeFilterName(uint8_t filterId){
  switch (filterId) {
    case 0x01: return "FIL1";
    case 0x02: return "FIL2";
    case 0x03: return "FIL3";
    default: return "UNK";
  }
}

String wsDecodeCivFrameJson(const uint8_t *frame, size_t frameLen){
  if (frameLen < 6) {
    return String();
  }

  const uint8_t command = frame[4];
  const size_t payloadLen = frameLen > 6 ? frameLen - 6 : 0;
  const uint8_t *payload = frame + 5;

  String decoded;
  if ((command == CMD_READ_FREQ || command == CMD_TRANS_FREQ) && payloadLen >= 5) {
    decoded = "\"kind\":\"frequency\",\"frequency\":";
    decoded += String(decodeCivFrequencyBytes(payload, 5));
    return decoded;
  }

  if ((command == CMD_READ_MODE || command == CMD_TRANS_MODE) && payloadLen >= 1) {
    decoded = "\"kind\":\"mode\",\"mode\":\"";
    decoded += decodeModeName(payload[0]);
    decoded += "\"";
    if (payloadLen >= 2) {
      decoded += ",\"filter\":";
      decoded += String(payload[1]);
      decoded += ",\"filterName\":\"";
      decoded += decodeFilterName(payload[1]);
      decoded += "\"";
    }
    return decoded;
  }

  if (command == 0x25 && payloadLen >= 6) {
    decoded = "\"kind\":\"vfoFrequency\",\"selected\":";
    decoded += (payload[0] == 0x00 ? "true" : "false");
    decoded += ",\"frequency\":";
    decoded += String(decodeCivFrequencyBytes(payload + 1, 5));
    return decoded;
  }

  if (command == 0x26 && payloadLen >= 2) {
    decoded = "\"kind\":\"vfoMode\",\"selected\":";
    decoded += (payload[0] == 0x00 ? "true" : "false");
    decoded += ",\"mode\":\"";
    decoded += decodeModeName(payload[1]);
    decoded += "\"";
    if (payloadLen >= 3) {
      decoded += ",\"dataMode\":";
      decoded += String(payload[2]);
    }
    if (payloadLen >= 4) {
      decoded += ",\"filter\":";
      decoded += String(payload[3]);
      decoded += ",\"filterName\":\"";
      decoded += decodeFilterName(payload[3]);
      decoded += "\"";
    }
    return decoded;
  }

  if (command == 0x1A && payloadLen >= 4 && payload[0] == 0x05) {
    uint16_t settingId = ((uint16_t)payload[1] << 8) | payload[2];
    if (settingId == 0x0045 && payloadLen >= 4) {
      decoded = "\"kind\":\"split\",\"source\":\"quickSplit\",\"enabled\":";
      decoded += (payload[3] == 0x00 ? "false" : "true");
      decoded += ",\"code\":\"";
      decoded += (payload[3] == 0x00 ? "OFF" : "ON");
      decoded += "\",\"value\":";
      decoded += String(payload[3]);
      decoded += ",\"duplex\":\"off\"";
      return decoded;
    }
    if (settingId == 0x0047 && payloadLen >= 4) {
      decoded = "\"kind\":\"splitLock\",\"enabled\":";
      decoded += (payload[3] == 0x00 ? "false" : "true");
      decoded += ",\"value\":";
      decoded += String(payload[3]);
      return decoded;
    }
  }

  if (command == 0x0F && payloadLen >= 1) {
    decoded = "\"kind\":\"split\",\"code\":\"";
    if (payload[0] == 0x00) decoded += "OFF";
    else if (payload[0] == 0x01) decoded += "ON";
    else if (payload[0] == 0x10) decoded += "SIMPLEX";
    else if (payload[0] == 0x11) decoded += "DUP-";
    else if (payload[0] == 0x12) decoded += "DUP+";
    else decoded += "UNKNOWN";
    decoded += "\",\"value\":";
    decoded += String(payload[0]);
    decoded += ",\"enabled\":";
    decoded += (payload[0] == 0x00 || payload[0] == 0x10) ? "false" : "true";
    decoded += ",\"duplex\":\"";
    if (payload[0] == 0x11) decoded += "-";
    else if (payload[0] == 0x12) decoded += "+";
    else decoded += "off";
    decoded += "\"";
    return decoded;
  }

  if (command == 0x15 && payloadLen >= 2) {
    uint8_t subCommand = payload[0];
    uint32_t rawValue = decodeCivBcdBytes(payload + 1, payloadLen - 1);
    if (subCommand == 0x02) decoded = "\"kind\":\"sMeter\"";
    else if (subCommand == 0x11) decoded = "\"kind\":\"powerMeter\"";
    else if (subCommand == 0x12) decoded = "\"kind\":\"swrMeter\"";
    else if (subCommand == 0x13) decoded = "\"kind\":\"alcMeter\"";
    else if (subCommand == 0x15) decoded = "\"kind\":\"vdMeter\"";
    else if (subCommand == 0x16) decoded = "\"kind\":\"idMeter\"";
    if (decoded.length() > 0) {
      decoded += ",\"raw\":";
      decoded += String(rawValue);
      if (subCommand == 0x02) {
        decoded += ",\"s9Scale\":";
        decoded += String((float)rawValue / 120.0f, 2);
        if (rawValue > 120) {
          decoded += ",\"dbOverS9Approx\":";
          decoded += String(((float)(rawValue - 120) * 60.0f) / 121.0f, 1);
        }
      } else if (subCommand == 0x11) {
        decoded += ",\"percentApprox\":";
        decoded += String(min(100.0f, ((float)rawValue * 100.0f) / 213.0f), 1);
      } else if (subCommand == 0x12) {
        float swrApprox = 1.0f;
        if (rawValue <= 48) {
          swrApprox = 1.0f + ((float)rawValue * 0.5f / 48.0f);
        } else if (rawValue <= 80) {
          swrApprox = 1.5f + ((float)(rawValue - 48) * 0.5f / 32.0f);
        } else if (rawValue <= 120) {
          swrApprox = 2.0f + ((float)(rawValue - 80) * 1.0f / 40.0f);
        } else {
          swrApprox = 3.0f + ((float)(rawValue - 120) / 40.0f);
        }
        decoded += ",\"ratioApprox\":";
        decoded += String(swrApprox, 2);
      } else if (subCommand == 0x13) {
        decoded += ",\"percentApprox\":";
        decoded += String(min(100.0f, ((float)rawValue * 100.0f) / 120.0f), 1);
      } else if (subCommand == 0x15) {
        decoded += ",\"voltsApprox\":";
        decoded += String(((float)rawValue * 16.0f) / 241.0f, 2);
      } else if (subCommand == 0x16) {
        decoded += ",\"ampsApprox\":";
        decoded += String(((float)rawValue * 4.0f) / 241.0f, 2);
      }
      return decoded;
    }
  }

  if (command == 0x21 && payloadLen >= 1) {
    decoded = "\"kind\":\"rit\",\"subCommand\":";
    decoded += String(payload[0]);
    if (payloadLen > 1) {
      decoded += ",\"raw\":";
      decoded += String(decodeCivBcdBytes(payload + 1, payloadLen - 1));
      decoded += ",\"dataHex\":\"";
      decoded += civFrameToHex(payload + 1, payloadLen - 1);
      decoded += "\"";
    }
    return decoded;
  }

  if (command == 0x14 && payloadLen >= 2) {
    decoded = "\"kind\":\"control\",\"subCommand\":";
    decoded += String(payload[0]);
    decoded += ",\"raw\":";
    decoded += String(decodeCivBcdBytes(payload + 1, payloadLen - 1));

    if (payload[0] == 0x01) {
      decoded += ",\"name\":\"afGain\"";
    } else if (payload[0] == 0x0A) {
      decoded += ",\"name\":\"rfPower\"";
    } else if (payload[0] == 0x0C) {
      decoded += ",\"name\":\"keySpeed\"";
    }
    return decoded;
  }

  if (command == 0x1C && payloadLen >= 2 && payload[0] == 0x00) {
    decoded = "\"kind\":\"txState\",\"tx\":";
    decoded += (payload[1] == 0x01 ? "true" : "false");
    decoded += ",\"state\":\"";
    decoded += (payload[1] == 0x01 ? "TX" : "RX");
    decoded += "\"";
    return decoded;
  }

  if (command == 0xFB) {
    return "\"kind\":\"ack\",\"status\":\"OK\"";
  }

  if (command == 0xFA) {
    return "\"kind\":\"ack\",\"status\":\"NG\"";
  }

  return String();
}

String wsBuildStateFields(void){
  String json;
  json += "\"connected\":";
  json += (btClientConnected ? "true" : "false");
  json += ",\"btStatus\":\"";
  if (!btClientConnected) {
    json += "BT idle";
  } else if (radio_address == 0x00) {
    json += "BT linked | searching CI-V";
  } else if (TrxNeedSet == true || TrxSetupDone == false) {
    json += "BT linked | TRX setup";
  } else {
    json += "BT linked";
  }
  json += "\"";
  json += ",\"wifiStatus\":\"";
  if (APmode == true) {
    json += "WiFi AP";
  } else if (WiFi.status() == WL_CONNECTED) {
    json += "WiFi STA";
  } else {
    json += "WiFi down";
  }
  json += "\"";
  json += ",\"wifiRssi\":";
  if (APmode == true || WiFi.status() != WL_CONNECTED) {
    json += "-999";
  } else {
    json += String(WiFi.RSSI());
  }
  json += ",\"power\":";
  json += (statusPower == 1 ? "true" : "false");
  json += ",\"frequency\":";
  json += String(frequency);
  json += ",\"mode\":\"";
  json += jsonEscape(modes);
  json += "\",\"radioAddress\":\"0x";
  if (radio_address < 16) {
    json += "0";
  }
  json += String(radio_address, HEX);
  json += "\"";
  return json;
}

String extractJsonString(const String &json, const char *key){
  String token = "\"" + String(key) + "\":";
  int start = json.indexOf(token);
  if (start < 0) {
    return String();
  }
  start += token.length();
  while (start < (int)json.length() && (json.charAt(start) == ' ' || json.charAt(start) == '\t')) {
    start++;
  }
  if (start >= (int)json.length()) {
    return String();
  }

  if (json.charAt(start) == '"') {
    start++;
    String out;
    bool escaped = false;
    for (int i = start; i < (int)json.length(); i++) {
      char c = json.charAt(i);
      if (escaped) {
        out += c;
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        return out;
      } else {
        out += c;
      }
    }
    return String();
  }

  int end = start;
  while (end < (int)json.length() && json.charAt(end) != ',' && json.charAt(end) != '}') {
    end++;
  }
  String out = json.substring(start, end);
  out.trim();
  return out;
}

bool extractJsonBool(const String &json, const char *key, bool defaultValue){
  String value = extractJsonString(json, key);
  if (value.length() == 0) {
    return defaultValue;
  }
  value.toLowerCase();
  if (value == "true" || value == "1") {
    return true;
  }
  if (value == "false" || value == "0") {
    return false;
  }
  return defaultValue;
}

bool parseHexPayload(const String &hex, uint8_t *buffer, size_t &bufferLen, size_t maxLen){
  bufferLen = 0;
  int nibble = -1;
  for (size_t i = 0; i < hex.length(); i++) {
    char c = hex.charAt(i);
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
      continue;
    }

    int value;
    if (c >= '0' && c <= '9') {
      value = c - '0';
    } else if (c >= 'a' && c <= 'f') {
      value = 10 + c - 'a';
    } else if (c >= 'A' && c <= 'F') {
      value = 10 + c - 'A';
    } else {
      return false;
    }

    if (nibble < 0) {
      nibble = value;
    } else {
      if (bufferLen >= maxLen) {
        return false;
      }
      buffer[bufferLen++] = (uint8_t)((nibble << 4) | value);
      nibble = -1;
    }
  }
  return nibble < 0 && bufferLen > 0;
}

bool parseModeId(const String &modeName, uint8_t &modeId){
  String normalized = modeName;
  normalized.toUpperCase();
  if (normalized == "LSB") {
    modeId = MODE_TYPE_LSB;
    return true;
  }
  if (normalized == "USB") {
    modeId = MODE_TYPE_USB;
    return true;
  }
  if (normalized == "AM") {
    modeId = MODE_TYPE_AM;
    return true;
  }
  if (normalized == "CW") {
    modeId = MODE_TYPE_CW;
    return true;
  }
  if (normalized == "RTTY" || normalized == "FSK") {
    modeId = MODE_TYPE_RTTY;
    return true;
  }
  if (normalized == "FM") {
    modeId = MODE_TYPE_FM;
    return true;
  }
  if (normalized == "DV") {
    modeId = MODE_TYPE_DV;
    return true;
  }
  return false;
}

uint8_t parseFilterWidth(const String &filterName){
  String normalized = filterName;
  normalized.toUpperCase();
  if (normalized == "FIL2" || normalized == "MID" || normalized == "MEDIUM") {
    return IF_PASSBAND_WIDTH_MEDIUM;
  }
  if (normalized == "FIL3" || normalized == "NAR" || normalized == "NARROW") {
    return IF_PASSBAND_WIDTH_NARROW;
  }
  return IF_PASSBAND_WIDTH_WIDE;
}

size_t buildSimpleCatFrame(uint8_t command, const uint8_t *payload, size_t payloadLen, uint8_t *frame, size_t frameMaxLen){
  if (radio_address == 0x00 || frameMaxLen < payloadLen + 6) {
    return 0;
  }

  size_t frameLen = 0;
  frame[frameLen++] = START_BYTE;
  frame[frameLen++] = START_BYTE;
  frame[frameLen++] = radio_address;
  frame[frameLen++] = CONTROLLER_ADDRESS;
  frame[frameLen++] = command;
  for (size_t i = 0; i < payloadLen; i++) {
    frame[frameLen++] = payload[i];
  }
  frame[frameLen++] = STOP_BYTE;
  return frameLen;
}

size_t buildSetFrequencyFrame(uint32_t freqHz, uint8_t *frame, size_t frameMaxLen){
  String strFreq = IntToTenString(freqHz);
  String splitFreq[5];
  SplitString(strFreq, splitFreq);

  uint8_t payload[5] = {
    stringToByte(splitFreq[4]),
    stringToByte(splitFreq[3]),
    stringToByte(splitFreq[2]),
    stringToByte(splitFreq[1]),
    stringToByte(splitFreq[0])
  };
  return buildSimpleCatFrame(CMD_WRITE_FREQ, payload, sizeof(payload), frame, frameMaxLen);
}

size_t buildSetModeFrame(uint8_t modeId, uint8_t modeWidth, uint8_t *frame, size_t frameMaxLen){
  uint8_t payload[2] = {modeId, modeWidth};
  return buildSimpleCatFrame(CMD_WRITE_MODE, payload, sizeof(payload), frame, frameMaxLen);
}

size_t buildReadQuickSplitFrame(uint8_t *frame, size_t frameMaxLen){
  uint8_t payload[3] = {0x05, 0x00, 0x45};
  return buildSimpleCatFrame(0x1A, payload, sizeof(payload), frame, frameMaxLen);
}

size_t buildReadSplitProbeFrame(uint8_t step, uint8_t *frame, size_t frameMaxLen){
  static const uint8_t probeValues[] = {0x00, 0x01, 0x11, 0x12};
  if (step >= sizeof(probeValues)) {
    return 0;
  }
  uint8_t payload[1] = {probeValues[step]};
  return buildSimpleCatFrame(0x0F, payload, sizeof(payload), frame, frameMaxLen);
}

void wsClearSplitProbe(void){
  wsSplitProbe.active = false;
  wsSplitProbe.clientId = 0;
  wsSplitProbe.reqId = "";
  wsSplitProbe.step = 0;
}

void resetSetupMessages(void){
  setupSsidErr = "";
  setupPswdErr = "";
  setupSsid2Err = "";
  setupPswd2Err = "";
  setupMqttErr = mqttEnable ? "" : "MQTT disabled";
  setupMqttErr1 = "";
  setupMqttErr2 = "";
  setupMqttErr3 = "";
  setupMqttErr4 = "";
  setupMqttPortErr = "";
  setupMqttTopicErr = "";
  setupMqttTopicRxErr = "";
  setupHttpCatPortErr = "";
  setupUdpPortErr = "";
  setupUdpCatPortErr = "";
}

String setupTemplateProcessor(const String &key){
  if (key == "APMODE_TEXT") return APmode ? "AP mode ON" : "AP mode OFF";
  if (key == "MAC") return MACString;
  if (key == "REV") return String(REV);
  if (key == "HWREV") return String(HardwareRev);
  if (key == "SSID") return SSID;
  if (key == "PSWD") return PSWD;
  if (key == "SSID2") return SSID2;
  if (key == "PSWD2") return PSWD2;
  if (key == "HTTP_CAT_PORT") return String(HTTP_CAT_PORT);
  if (key == "UDP_PORT") return String(udpPort);
  if (key == "UDP_CAT_PORT") return String(udpCatPort);
  if (key == "MQTT_IP0") return String(mqttBroker[0]);
  if (key == "MQTT_IP1") return String(mqttBroker[1]);
  if (key == "MQTT_IP2") return String(mqttBroker[2]);
  if (key == "MQTT_IP3") return String(mqttBroker[3]);
  if (key == "MQTT_PORT") return String(MQTT_PORT);
  if (key == "MQTT_TOPIC") return MQTT_TOPIC;
  if (key == "MQTT_TOPIC_RX") return MQTT_TOPIC_RX;
  if (key == "SSID_ERR") return setupSsidErr;
  if (key == "PSWD_ERR") return setupPswdErr;
  if (key == "SSID2_ERR") return setupSsid2Err;
  if (key == "PSWD2_ERR") return setupPswd2Err;
  if (key == "MQTT_ERR") return setupMqttErr;
  if (key == "MQTT_ERR1") return setupMqttErr1;
  if (key == "MQTT_ERR2") return setupMqttErr2;
  if (key == "MQTT_ERR3") return setupMqttErr3;
  if (key == "MQTT_ERR4") return setupMqttErr4;
  if (key == "MQTT_PORT_ERR") return setupMqttPortErr;
  if (key == "MQTT_TOPIC_ERR") return setupMqttTopicErr;
  if (key == "MQTT_TOPIC_RX_ERR") return setupMqttTopicRxErr;
  if (key == "HTTP_CAT_PORT_ERR") return setupHttpCatPortErr;
  if (key == "UDP_PORT_ERR") return setupUdpPortErr;
  if (key == "UDP_CAT_PORT_ERR") return setupUdpCatPortErr;
  if (key == "BAUD1200_SEL") return BaudRate == 1200 ? "selected" : "";
  if (key == "BAUD2400_SEL") return BaudRate == 2400 ? "selected" : "";
  if (key == "BAUD4800_SEL") return BaudRate == 4800 ? "selected" : "";
  if (key == "BAUD9600_SEL") return BaudRate == 9600 ? "selected" : "";
  if (key == "BAUD115200_SEL") return BaudRate == 115200 ? "selected" : "";
  return String();
}

void renderSetupPage(AsyncWebServerRequest *request){
  if (SPIFFS.exists("/setup.html")) {
    request->send(SPIFFS, "/setup.html", "text/html", false, setupTemplateProcessor);
    return;
  }
  request->send(500, "text/plain", "Missing /setup.html in SPIFFS");
}

String wsBuildStateJson(const char *type, const String &reqId){
  String json = "{\"type\":\"";
  json += type;
  json += "\",";
  json += wsBuildStateFields();
  if (reqId.length() > 0) {
    json += ",\"reqId\":\"";
    json += jsonEscape(reqId);
    json += "\"";
  }
  json += "}";
  return json;
}

void wsBroadcastRigState(void){
  ws.textAll(wsBuildStateJson("state"));
}

void wsBroadcastCivFrame(const char *type, const uint8_t *frame, size_t frameLen, const String &reqId){
  String hex = civFrameToHex(frame, frameLen);
  String decoded = wsDecodeCivFrameJson(frame, frameLen);

  String json = "{\"type\":\"";
  json += type;
  json += "\",\"frame\":\"";
  json += hex;
  json += "\"";
  if (decoded.length() > 0) {
    json += ",\"decoded\":{";
    json += decoded;
    json += "}";
  }
  if (reqId.length() > 0) {
    json += ",\"reqId\":\"";
    json += jsonEscape(reqId);
    json += "\"";
  }
  json += "}";
  ws.textAll(json);
}

void wsSendError(AsyncWebSocketClient *client, const String &code, const String &message, const String &reqId){
  String json = "{\"type\":\"error\",\"code\":\"";
  json += jsonEscape(code);
  json += "\",\"message\":\"";
  json += jsonEscape(message);
  json += "\"";
  if (reqId.length() > 0) {
    json += ",\"reqId\":\"";
    json += jsonEscape(reqId);
    json += "\"";
  }
  json += "}";
  client->text(json);
}

void wsSendErrorById(uint32_t clientId, const String &code, const String &message, const String &reqId){
  AsyncWebSocketClient *client = ws.client(clientId);
  if (client != nullptr && client->status() == WS_CONNECTED) {
    wsSendError(client, code, message, reqId);
  }
}

void wsSendHello(AsyncWebSocketClient *client){
  String json = "{\"type\":\"hello\",\"proto\":1,\"transport\":\"bt-civ\",\"connected\":";
  json += (btClientConnected ? "true" : "false");
  json += ",\"state\":{";
  json += wsBuildStateFields();
  json += "}";
  json += "}";
  client->text(json);
}

bool catWriteFrame(const uint8_t *frame, size_t frameLen, bool broadcastTx){
  #if defined(BLUETOOTH)
    if (!btClientConnected) {
      return false;
    }
    for (size_t i = 0; i < frameLen; i++) {
      CAT.write(frame[i]);
    }
    if (broadcastTx) {
      wsBroadcastCivFrame("civ.tx", frame, frameLen);
    }
    return true;
  #endif
  return false;
}

void wsSendQueued(AsyncWebSocketClient *client, const String &reqId, uint8_t depth){
  String json = "{\"type\":\"queued\",\"reqId\":\"";
  json += jsonEscape(reqId);
  json += "\",\"depth\":";
  json += String(depth);
  json += "}";
  client->text(json);
}

bool wsStartRequest(uint32_t clientId, const String &reqId, WsPendingMatchType matchType, uint8_t expectedCommand, const uint8_t *expectedPayload, uint8_t expectedPayloadLen, unsigned long timeoutMs, const uint8_t *frame, size_t frameLen){
  if (!catWriteFrame(frame, frameLen, false)) {
    return false;
  }

  if (reqId.length() > 0) {
    wsBroadcastCivFrame("civ.tx", frame, frameLen, reqId);
  }

  wsPendingRequest.active = true;
  wsPendingRequest.clientId = clientId;
  wsPendingRequest.deadlineMs = millis() + timeoutMs;
  wsPendingRequest.reqId = reqId;
  wsPendingRequest.matchType = matchType;
  wsPendingRequest.expectedCommand = expectedCommand;
  wsPendingRequest.expectedPayloadLen = min<uint8_t>(expectedPayloadLen, sizeof(wsPendingRequest.expectedPayload));
  memset(wsPendingRequest.expectedPayload, 0, sizeof(wsPendingRequest.expectedPayload));
  if (expectedPayload != nullptr && wsPendingRequest.expectedPayloadLen > 0) {
    memcpy(wsPendingRequest.expectedPayload, expectedPayload, wsPendingRequest.expectedPayloadLen);
  }
  return true;
}

bool wsStartSplitProbe(AsyncWebSocketClient *client, const String &reqId){
  if (wsPendingRequest.active || wsSplitProbe.active) {
    return false;
  }

  uint8_t frame[8];
  size_t frameLen = buildReadSplitProbeFrame(0, frame, sizeof(frame));
  if (frameLen == 0) {
    return false;
  }

  if (!wsStartRequest(client->id(), reqId, WS_MATCH_COMMAND, 0x0F, nullptr, 0, 1500, frame, frameLen)) {
    return false;
  }

  wsSplitProbe.active = true;
  wsSplitProbe.clientId = client->id();
  wsSplitProbe.reqId = reqId;
  wsSplitProbe.step = 0;
  return true;
}

bool wsQueuePendingRequest(AsyncWebSocketClient *client, const String &reqId, WsPendingMatchType matchType, uint8_t expectedCommand, const uint8_t *expectedPayload, uint8_t expectedPayloadLen, unsigned long timeoutMs, const uint8_t *frame, size_t frameLen){
  if (!wsPendingRequest.active) {
    return wsStartRequest(client->id(), reqId, matchType, expectedCommand, expectedPayload, expectedPayloadLen, timeoutMs, frame, frameLen);
  }

  for (uint8_t i = 0; i < WS_QUEUE_DEPTH; i++) {
    if (wsQueuedRequests[i].used) {
      continue;
    }
    wsQueuedRequests[i].used = true;
    wsQueuedRequests[i].clientId = client->id();
    wsQueuedRequests[i].timeoutMs = timeoutMs;
    wsQueuedRequests[i].reqId = reqId;
    wsQueuedRequests[i].matchType = matchType;
    wsQueuedRequests[i].expectedCommand = expectedCommand;
    wsQueuedRequests[i].expectedPayloadLen = min<uint8_t>(expectedPayloadLen, sizeof(wsQueuedRequests[i].expectedPayload));
    memset(wsQueuedRequests[i].expectedPayload, 0, sizeof(wsQueuedRequests[i].expectedPayload));
    if (expectedPayload != nullptr && wsQueuedRequests[i].expectedPayloadLen > 0) {
      memcpy(wsQueuedRequests[i].expectedPayload, expectedPayload, wsQueuedRequests[i].expectedPayloadLen);
    }
    wsQueuedRequests[i].frameLen = frameLen;
    memcpy(wsQueuedRequests[i].frame, frame, frameLen);
    wsSendQueued(client, reqId, i + 1);
    return true;
  }

  return false;
}

void wsClearPendingRequest(void){
  wsPendingRequest.active = false;
  wsPendingRequest.clientId = 0;
  wsPendingRequest.deadlineMs = 0;
  wsPendingRequest.reqId = "";
  wsPendingRequest.matchType = WS_MATCH_NONE;
  wsPendingRequest.expectedCommand = 0;
  wsPendingRequest.expectedPayloadLen = 0;
  memset(wsPendingRequest.expectedPayload, 0, sizeof(wsPendingRequest.expectedPayload));
}

void wsClearQueuedRequest(uint8_t index){
  wsQueuedRequests[index].used = false;
  wsQueuedRequests[index].clientId = 0;
  wsQueuedRequests[index].timeoutMs = 0;
  wsQueuedRequests[index].reqId = "";
  wsQueuedRequests[index].matchType = WS_MATCH_NONE;
  wsQueuedRequests[index].expectedCommand = 0;
  wsQueuedRequests[index].expectedPayloadLen = 0;
  memset(wsQueuedRequests[index].expectedPayload, 0, sizeof(wsQueuedRequests[index].expectedPayload));
  wsQueuedRequests[index].frameLen = 0;
}

void wsFlushQueuedRequests(const String &code, const String &message){
  for (uint8_t i = 0; i < WS_QUEUE_DEPTH; i++) {
    if (!wsQueuedRequests[i].used) {
      continue;
    }
    wsSendErrorById(wsQueuedRequests[i].clientId, code, message, wsQueuedRequests[i].reqId);
    wsClearQueuedRequest(i);
  }
}

void wsDropQueuedRequestsForClient(uint32_t clientId){
  for (uint8_t i = 0; i < WS_QUEUE_DEPTH; i++) {
    if (wsQueuedRequests[i].used && wsQueuedRequests[i].clientId == clientId) {
      wsClearQueuedRequest(i);
    }
  }
  if (wsSplitProbe.active && wsSplitProbe.clientId == clientId) {
    wsClearSplitProbe();
  }
}

void wsTryStartNextQueuedRequest(void){
  if (wsPendingRequest.active) {
    return;
  }

  for (uint8_t i = 0; i < WS_QUEUE_DEPTH; i++) {
    if (!wsQueuedRequests[i].used) {
      continue;
    }

    bool started = wsStartRequest(
      wsQueuedRequests[i].clientId,
      wsQueuedRequests[i].reqId,
      wsQueuedRequests[i].matchType,
      wsQueuedRequests[i].expectedCommand,
      wsQueuedRequests[i].expectedPayload,
      wsQueuedRequests[i].expectedPayloadLen,
      wsQueuedRequests[i].timeoutMs,
      wsQueuedRequests[i].frame,
      wsQueuedRequests[i].frameLen
    );

    if (!started) {
      wsSendErrorById(wsQueuedRequests[i].clientId, "tx_failed", "Queued CAT request could not be sent", wsQueuedRequests[i].reqId);
    }
    wsClearQueuedRequest(i);
    return;
  }
}

void wsProcessPendingTimeout(void){
  if (!wsPendingRequest.active) {
    return;
  }
  if ((long)(millis() - wsPendingRequest.deadlineMs) < 0) {
    return;
  }

  if (wsSplitProbe.active && wsPendingRequest.clientId == wsSplitProbe.clientId && wsPendingRequest.reqId == wsSplitProbe.reqId) {
    if (wsSplitProbe.step + 1 < WS_SPLIT_PROBE_STEPS) {
      wsPendingRequest.active = false;
      wsPendingRequest.clientId = 0;
      wsPendingRequest.deadlineMs = 0;
      wsPendingRequest.reqId = "";
      wsPendingRequest.matchType = WS_MATCH_NONE;
      wsPendingRequest.expectedCommand = 0;
      wsPendingRequest.expectedPayloadLen = 0;
      memset(wsPendingRequest.expectedPayload, 0, sizeof(wsPendingRequest.expectedPayload));

      wsSplitProbe.step++;
      uint8_t frame[8];
      size_t frameLen = buildReadSplitProbeFrame(wsSplitProbe.step, frame, sizeof(frame));
      if (frameLen > 0 && wsStartRequest(wsSplitProbe.clientId, wsSplitProbe.reqId, WS_MATCH_COMMAND, 0x0F, nullptr, 0, 1500, frame, frameLen)) {
        return;
      }
    }
    wsClearSplitProbe();
  }

  wsSendErrorById(wsPendingRequest.clientId, "timeout", "No CI-V reply received before timeout", wsPendingRequest.reqId);
  wsClearPendingRequest();
  wsTryStartNextQueuedRequest();
}

bool wsResolvePendingRequest(const uint8_t *frame, size_t frameLen, String *matchedReqId){
  if (!wsPendingRequest.active || frameLen < 6) {
    return false;
  }

  bool matches = false;
  switch (wsPendingRequest.matchType) {
    case WS_MATCH_ANY_FRAME:
      matches = true;
      break;
    case WS_MATCH_COMMAND:
      matches = frameLen > 4 && frame[4] == wsPendingRequest.expectedCommand;
      if (matches && wsPendingRequest.expectedPayloadLen > 0) {
        matches = frameLen >= (size_t)(5 + wsPendingRequest.expectedPayloadLen + 1)
          && memcmp(frame + 5, wsPendingRequest.expectedPayload, wsPendingRequest.expectedPayloadLen) == 0;
      }
      break;
    case WS_MATCH_ACK:
      matches = frame[4] == 0xFB || frame[4] == 0xFA;
      break;
    default:
      break;
  }

  if (!matches) {
    return false;
  }

  if (matchedReqId != nullptr) {
    *matchedReqId = wsPendingRequest.reqId;
  }

  AsyncWebSocketClient *client = ws.client(wsPendingRequest.clientId);
  if (client != nullptr && client->status() == WS_CONNECTED) {
    String decoded = wsDecodeCivFrameJson(frame, frameLen);
    String reply = "{\"type\":\"reply\",\"ok\":";
    reply += (frame[4] == 0xFA ? "false" : "true");
    reply += ",\"reqId\":\"";
    reply += jsonEscape(wsPendingRequest.reqId);
    reply += "\",\"frame\":\"";
    reply += civFrameToHex(frame, frameLen);
    if (decoded.length() > 0) {
      reply += "\",\"decoded\":{";
      reply += decoded;
      reply += "}";
    } else {
      reply += "\"";
    }
    reply += ",\"state\":{";
    reply += wsBuildStateFields();
    reply += "}";
    reply += "}";
    client->text(reply);
  }

  if (wsSplitProbe.active && wsPendingRequest.clientId == wsSplitProbe.clientId && wsPendingRequest.reqId == wsSplitProbe.reqId) {
    wsClearSplitProbe();
  }
  wsClearPendingRequest();
  wsTryStartNextQueuedRequest();
  return true;
}

void wsHandleTextMessage(AsyncWebSocketClient *client, const String &msg){
  String type = extractJsonString(msg, "type");
  String reqId = extractJsonString(msg, "reqId");

  if (type == "getState") {
    client->text(wsBuildStateJson("reply", reqId));
    return;
  }

  if (type == "setFrequency") {
    if (!btClientConnected || radio_address == 0x00) {
      wsSendError(client, "radio_disconnected", "Bluetooth CAT is not connected", reqId);
      return;
    }
    String frequencyText = extractJsonString(msg, "frequency");
    uint32_t nextFrequency = (uint32_t)frequencyText.toInt();
    if (nextFrequency == 0) {
      wsSendError(client, "invalid_frequency", "Expected frequency in Hz", reqId);
      return;
    }
    uint8_t frame[16];
    size_t frameLen = buildSetFrequencyFrame(nextFrequency, frame, sizeof(frame));
    if (frameLen == 0) {
      wsSendError(client, "tx_failed", "Unable to build frequency command", reqId);
      return;
    }
    if (!wsQueuePendingRequest(client, reqId, WS_MATCH_ACK, 0, nullptr, 0, 1500, frame, frameLen)) {
      wsSendError(client, "queue_full", "CAT request queue is full", reqId);
      return;
    }
    return;
  }

  if (type == "readFrequency") {
    if (!btClientConnected || radio_address == 0x00) {
      wsSendError(client, "radio_disconnected", "Bluetooth CAT is not connected", reqId);
      return;
    }
    uint8_t frame[8];
    size_t frameLen = buildSimpleCatFrame(CMD_READ_FREQ, nullptr, 0, frame, sizeof(frame));
    if (!wsQueuePendingRequest(client, reqId, WS_MATCH_COMMAND, CMD_READ_FREQ, nullptr, 0, 1500, frame, frameLen)) {
      wsSendError(client, "queue_full", "CAT request queue is full", reqId);
    }
    return;
  }

  if (type == "readMode") {
    if (!btClientConnected || radio_address == 0x00) {
      wsSendError(client, "radio_disconnected", "Bluetooth CAT is not connected", reqId);
      return;
    }
    uint8_t frame[8];
    size_t frameLen = buildSimpleCatFrame(CMD_READ_MODE, nullptr, 0, frame, sizeof(frame));
    if (!wsQueuePendingRequest(client, reqId, WS_MATCH_COMMAND, CMD_READ_MODE, nullptr, 0, 1500, frame, frameLen)) {
      wsSendError(client, "queue_full", "CAT request queue is full", reqId);
    }
    return;
  }

  if (type == "readVfoFrequency") {
    if (!btClientConnected || radio_address == 0x00) {
      wsSendError(client, "radio_disconnected", "Bluetooth CAT is not connected", reqId);
      return;
    }
    bool selected = !extractJsonBool(msg, "unselected", false);
    uint8_t payload[1] = {selected ? 0x00 : 0x01};
    uint8_t frame[8];
    size_t frameLen = buildSimpleCatFrame(0x25, payload, sizeof(payload), frame, sizeof(frame));
    if (!wsQueuePendingRequest(client, reqId, WS_MATCH_COMMAND, 0x25, nullptr, 0, 1500, frame, frameLen)) {
      wsSendError(client, "queue_full", "CAT request queue is full", reqId);
    }
    return;
  }

  if (type == "readVfoMode") {
    if (!btClientConnected || radio_address == 0x00) {
      wsSendError(client, "radio_disconnected", "Bluetooth CAT is not connected", reqId);
      return;
    }
    bool selected = !extractJsonBool(msg, "unselected", false);
    uint8_t payload[1] = {selected ? 0x00 : 0x01};
    uint8_t frame[8];
    size_t frameLen = buildSimpleCatFrame(0x26, payload, sizeof(payload), frame, sizeof(frame));
    if (!wsQueuePendingRequest(client, reqId, WS_MATCH_COMMAND, 0x26, nullptr, 0, 1500, frame, frameLen)) {
      wsSendError(client, "queue_full", "CAT request queue is full", reqId);
    }
    return;
  }

  if (type == "readRit") {
    if (!btClientConnected || radio_address == 0x00) {
      wsSendError(client, "radio_disconnected", "Bluetooth CAT is not connected", reqId);
      return;
    }
    uint8_t payload[1] = {0x00};
    uint8_t frame[8];
    size_t frameLen = buildSimpleCatFrame(0x21, payload, sizeof(payload), frame, sizeof(frame));
    uint8_t expectedPayload[1] = {0x00};
    if (!wsQueuePendingRequest(client, reqId, WS_MATCH_COMMAND, 0x21, expectedPayload, sizeof(expectedPayload), 1500, frame, frameLen)) {
      wsSendError(client, "queue_full", "CAT request queue is full", reqId);
    }
    return;
  }

  if (type == "readSplit") {
    if (!btClientConnected || radio_address == 0x00) {
      wsSendError(client, "radio_disconnected", "Bluetooth CAT is not connected", reqId);
      return;
    }
    uint8_t frame[12];
    size_t frameLen = buildReadQuickSplitFrame(frame, sizeof(frame));
    uint8_t expectedPayload[3] = {0x05, 0x00, 0x45};
    if (!wsQueuePendingRequest(client, reqId, WS_MATCH_COMMAND, 0x1A, expectedPayload, sizeof(expectedPayload), 1500, frame, frameLen)) {
      wsSendError(client, "queue_full", "CAT request queue is full", reqId);
    }
    return;
  }

  if (type == "readSmeter") {
    if (!btClientConnected || radio_address == 0x00) {
      wsSendError(client, "radio_disconnected", "Bluetooth CAT is not connected", reqId);
      return;
    }
    uint8_t payload[1] = {0x02};
    uint8_t frame[8];
    size_t frameLen = buildSimpleCatFrame(0x15, payload, sizeof(payload), frame, sizeof(frame));
    uint8_t expectedPayload[1] = {0x02};
    if (!wsQueuePendingRequest(client, reqId, WS_MATCH_COMMAND, 0x15, expectedPayload, sizeof(expectedPayload), 1500, frame, frameLen)) {
      wsSendError(client, "queue_full", "CAT request queue is full", reqId);
    }
    return;
  }

  if (type == "readPower") {
    if (!btClientConnected || radio_address == 0x00) {
      wsSendError(client, "radio_disconnected", "Bluetooth CAT is not connected", reqId);
      return;
    }
    uint8_t payload[1] = {0x11};
    uint8_t frame[8];
    size_t frameLen = buildSimpleCatFrame(0x15, payload, sizeof(payload), frame, sizeof(frame));
    uint8_t expectedPayload[1] = {0x11};
    if (!wsQueuePendingRequest(client, reqId, WS_MATCH_COMMAND, 0x15, expectedPayload, sizeof(expectedPayload), 1500, frame, frameLen)) {
      wsSendError(client, "queue_full", "CAT request queue is full", reqId);
    }
    return;
  }

  if (type == "readSwr") {
    if (!btClientConnected || radio_address == 0x00) {
      wsSendError(client, "radio_disconnected", "Bluetooth CAT is not connected", reqId);
      return;
    }
    uint8_t payload[1] = {0x12};
    uint8_t frame[8];
    size_t frameLen = buildSimpleCatFrame(0x15, payload, sizeof(payload), frame, sizeof(frame));
    uint8_t expectedPayload[1] = {0x12};
    if (!wsQueuePendingRequest(client, reqId, WS_MATCH_COMMAND, 0x15, expectedPayload, sizeof(expectedPayload), 1500, frame, frameLen)) {
      wsSendError(client, "queue_full", "CAT request queue is full", reqId);
    }
    return;
  }

  if (type == "readAlc") {
    if (!btClientConnected || radio_address == 0x00) {
      wsSendError(client, "radio_disconnected", "Bluetooth CAT is not connected", reqId);
      return;
    }
    uint8_t payload[1] = {0x13};
    uint8_t frame[8];
    size_t frameLen = buildSimpleCatFrame(0x15, payload, sizeof(payload), frame, sizeof(frame));
    uint8_t expectedPayload[1] = {0x13};
    if (!wsQueuePendingRequest(client, reqId, WS_MATCH_COMMAND, 0x15, expectedPayload, sizeof(expectedPayload), 1500, frame, frameLen)) {
      wsSendError(client, "queue_full", "CAT request queue is full", reqId);
    }
    return;
  }

  if (type == "setMode") {
    if (!btClientConnected || radio_address == 0x00) {
      wsSendError(client, "radio_disconnected", "Bluetooth CAT is not connected", reqId);
      return;
    }
    String modeName = extractJsonString(msg, "mode");
    String filterName = extractJsonString(msg, "filter");
    uint8_t modeId = 0;
    if (!parseModeId(modeName, modeId)) {
      wsSendError(client, "invalid_mode", "Supported modes: LSB, USB, AM, CW, RTTY/FSK, FM, DV", reqId);
      return;
    }
    uint8_t frame[12];
    size_t frameLen = buildSetModeFrame(modeId, parseFilterWidth(filterName), frame, sizeof(frame));
    if (frameLen == 0) {
      wsSendError(client, "tx_failed", "Unable to build mode command", reqId);
      return;
    }
    if (!wsQueuePendingRequest(client, reqId, WS_MATCH_ACK, 0, nullptr, 0, 1500, frame, frameLen)) {
      wsSendError(client, "queue_full", "CAT request queue is full", reqId);
    }
    return;
  }

  if (type == "setRitClear") {
    if (!btClientConnected || radio_address == 0x00) {
      wsSendError(client, "radio_disconnected", "Bluetooth CAT is not connected", reqId);
      return;
    }
    uint8_t payload[4] = {0x00, 0x00, 0x00, 0x00};
    uint8_t frame[12];
    size_t frameLen = buildSimpleCatFrame(0x21, payload, sizeof(payload), frame, sizeof(frame));
    if (!wsQueuePendingRequest(client, reqId, WS_MATCH_ACK, 0, nullptr, 0, 1500, frame, frameLen)) {
      wsSendError(client, "queue_full", "CAT request queue is full", reqId);
    }
    return;
  }

  if (type == "sendCw") {
    String text = extractJsonString(msg, "text");
    if (text.length() == 0) {
      wsSendError(client, "missing_text", "Expected CW text", reqId);
      return;
    }
    memset(CwMsg, 0, sizeof(CwMsg));
    text.toCharArray(CwMsg, sizeof(CwMsg));
    sendCW();
    String reply = "{\"type\":\"reply\",\"ok\":true";
    if (reqId.length() > 0) {
      reply += ",\"reqId\":\"";
      reply += jsonEscape(reqId);
      reply += "\"";
    }
    reply += "}";
    client->text(reply);
    return;
  }

  if (type == "civ.raw") {
    String data = extractJsonString(msg, "data");
    bool framed = extractJsonBool(msg, "framed", false);
    bool expectReply = extractJsonBool(msg, "expectReply", false);
    bool expectAck = extractJsonBool(msg, "expectAck", false);
    if (data.length() == 0) {
      wsSendError(client, "missing_data", "Expected hex payload in data", reqId);
      return;
    }
    if (!btClientConnected) {
      wsSendError(client, "radio_disconnected", "Bluetooth CAT is not connected", reqId);
      return;
    }
    if (!framed && radio_address == 0x00) {
      wsSendError(client, "radio_address_unknown", "Radio address is not known yet", reqId);
      return;
    }

    uint8_t payload[64];
    size_t payloadLen = 0;
    if (!parseHexPayload(data, payload, payloadLen, sizeof(payload))) {
      wsSendError(client, "invalid_hex", "Payload must be valid hex bytes", reqId);
      return;
    }

    uint8_t frame[70];
    size_t frameLen = 0;
    if (framed) {
      if (payloadLen < 5 || payloadLen > sizeof(frame)) {
        wsSendError(client, "invalid_frame", "Framed CI-V payload has invalid length", reqId);
        return;
      }
      memcpy(frame, payload, payloadLen);
      frameLen = payloadLen;
    } else {
      if (payloadLen + 5 > sizeof(frame)) {
        wsSendError(client, "frame_too_large", "CI-V payload is too large", reqId);
        return;
      }
      frame[frameLen++] = START_BYTE;
      frame[frameLen++] = START_BYTE;
      frame[frameLen++] = radio_address;
      frame[frameLen++] = CONTROLLER_ADDRESS;
      memcpy(frame + frameLen, payload, payloadLen);
      frameLen += payloadLen;
      frame[frameLen++] = STOP_BYTE;
    }

    uint8_t commandByte = frameLen > 4 ? frame[4] : 0x00;
    if (expectReply) {
      WsPendingMatchType matchType = expectAck ? WS_MATCH_ACK : WS_MATCH_COMMAND;
      if (!expectAck && commandByte == 0xFB) {
        matchType = WS_MATCH_ACK;
      }
      uint8_t expectedPayload[4] = {0, 0, 0, 0};
      uint8_t expectedPayloadLen = 0;
      if (!expectAck && frameLen > 5) {
        size_t availablePayloadLen = frameLen - 6;
        expectedPayloadLen = (uint8_t)min((size_t)4, availablePayloadLen);
        memcpy(expectedPayload, frame + 5, expectedPayloadLen);
      }
      if (!wsQueuePendingRequest(client, reqId, matchType, commandByte, expectedPayloadLen > 0 ? expectedPayload : nullptr, expectedPayloadLen, 1500, frame, frameLen)) {
        wsSendError(client, "queue_full", "CAT request queue is full", reqId);
        return;
      }
    } else if (!catWriteFrame(frame, frameLen, true)) {
      wsSendError(client, "tx_failed", "Unable to send frame to radio", reqId);
      return;
    }

    if (!expectReply) {
      String reply = "{\"type\":\"reply\",\"ok\":true";
      if (reqId.length() > 0) {
        reply += ",\"reqId\":\"";
        reply += jsonEscape(reqId);
        reply += "\"";
      }
      reply += "}";
      client->text(reply);
    }
    return;
  }

  wsSendError(client, "unsupported_type", "Supported messages: getState, readFrequency, readMode, readVfoFrequency, readVfoMode, readRit, readSplit, readSmeter, readPower, readSwr, readAlc, setFrequency, setMode, setRitClear, sendCw, civ.raw", reqId);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len){
  (void)server;
  if (type == WS_EVT_CONNECT) {
    wsSendHello(client);
  } else if (type == WS_EVT_DISCONNECT) {
    if (wsPendingRequest.active && wsPendingRequest.clientId == client->id()) {
      wsClearPendingRequest();
      wsTryStartNextQueuedRequest();
    }
    wsDropQueuedRequestsForClient(client->id());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) {
      wsSendError(client, "unsupported_frame", "Only single-frame text messages are supported", "");
      return;
    }

    String msg;
    msg.reserve(len);
    for (size_t i = 0; i < len; i++) {
      msg += (char)data[i];
    }
    wsHandleTextMessage(client, msg);
  }
}

void setupAsyncHttpServer(void){
  ajaxserver.on("/api/state", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", wsBuildStateJson("state"));
  });

  ajaxserver.on("/cat", HTTP_GET, [](AsyncWebServerRequest *request){
    if (APmode || !SPIFFS.exists("/index.html")) {
      handleSet(request);
      return;
    }
    request->send(SPIFFS, "/index.html", "text/html");
  });

  ajaxserver.on("/ws-cat", HTTP_GET, [](AsyncWebServerRequest *request){
    if (APmode || !SPIFFS.exists("/ws-cat.html")) {
      handleSet(request);
      return;
    }
    request->send(SPIFFS, "/ws-cat.html", "text/html");
  });

  ajaxserver.on("/setup", HTTP_GET, [](AsyncWebServerRequest *request){
    handleSet(request);
  });
  ajaxserver.on("/setup", HTTP_POST, [](AsyncWebServerRequest *request){
    handleSet(request);
  });

  ajaxserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    if (APmode || !SPIFFS.exists("/index.html")) {
      handleSet(request);
      return;
    }
    request->send(SPIFFS, "/index.html", "text/html");
  });

  ajaxserver.serveStatic("/", SPIFFS, "/");
  ws.onEvent(onWsEvent);
  ajaxserver.addHandler(&ws);
  ajaxserver.begin();
}
#endif

//-------------------------------------------------------------------------------------------------------

void setup(){

  if (!EEPROM.begin(EEPROM_SIZE)){
    Serial.begin(BaudRate);
    Serial.println("EEPROM failed to initialise");
    delay(1);
  }else{

    // // clear eeprom
    // if(EEPROM.read(0)==0x00){
    //   for (int i=0; i<EEPROM_SIZE; i++){
    //     EEPROM.writeByte(i, 0xff);
    //     EEPROM.commit();
    //     Serial.println("Clear eeprom with 0xff and reboot...");
    //     ESP.restart();
    //   }
    // }

    // 74-75 BaudRate
    if(EEPROM.read(74)==0xff){
      BaudRate=9600;
    }else{
      BaudRate = EEPROM.readUShort(74);
    }
    Serial.begin(BaudRate);

    // 0 APmode
    if(EEPROM.read(0)==0xff){
      APmode=true;
    }else{
      if(EEPROM.readBool(0)==1){
        APmode=true;
      }else{
        APmode=false;
      }
    }

    // 1-21 - SSID
    if(EEPROM.read(1)==0xff){
      // APmode=true;
    }else{
      for (int i=1; i<21; i++){
        if(EEPROM.read(i)!=0xff){
          SSID=SSID+char(EEPROM.read(i));
        }
      }
    }

    // 22-40 - PSWD
    if(EEPROM.read(22)==0xff){
      // nil
    }else{
      for (int i=22; i<40; i++){
        if(EEPROM.read(i)!=0xff){
          PSWD=PSWD+char(EEPROM.read(i));
        }
      }
    }

    // 76-95 SSID2
    if(EEPROM.read(76)!=0xff){
      for (int i=76; i<96; i++){
        if(EEPROM.read(i)!=0xff){
          SSID2=SSID2+char(EEPROM.read(i));
        }
      }
    }

    // 97-114 PSWD2
    if(EEPROM.read(97)!=0xff){
      for (int i=97; i<115; i++){
        if(EEPROM.read(i)!=0xff){
          PSWD2=PSWD2+char(EEPROM.read(i));
        }
      }
    }

    // 41-44 mqttBroker[4]
    #if defined(MQTT)
      mqttBroker[0]=EEPROM.readByte(41);
      mqttBroker[1]=EEPROM.readByte(42);
      mqttBroker[2]=EEPROM.readByte(43);
      mqttBroker[3]=EEPROM.readByte(44);
      if(EEPROM.read(41)==0xff && EEPROM.read(42)==0xff && EEPROM.read(43)==0xff && EEPROM.read(44)==0xff){
        mqttBroker[0]=0; // default disable //54;
        mqttBroker[1]=38;
        mqttBroker[2]=157;
        mqttBroker[3]=134;
      }
      if(mqttBroker[0]==0x00){
        mqttEnable = false;
      }
    #endif

    // 45-46 MQTT_PORT
    if(EEPROM.read(45)==0xff){
      MQTT_PORT=1883;
    }else{
      MQTT_PORT = EEPROM.readUShort(45);
    }

    // 47-67 MQTT_TOPIC
    if(EEPROM.read(47)==0xff){
      MQTT_TOPIC="CALL/IC705/1/hz";
    }else{
      for (int i=47; i<68; i++){
        if(EEPROM.read(i)!=0xff){
          MQTT_TOPIC=MQTT_TOPIC+char(EEPROM.read(i));
        }
      }
    }

    // 115-135 MQTT_TOPIC_RX
    if(EEPROM.read(115)==0xff){
      MQTT_TOPIC_RX="";
    }else{
      for (int i=115; i<136; i++){
        if(EEPROM.read(i)!=0xff){
          MQTT_TOPIC_RX=MQTT_TOPIC_RX+char(EEPROM.read(i));
        }
      }
    }

    // 68-69 HTTP_CAT_PORT
    if(EEPROM.read(68)==0xff){
      HTTP_CAT_PORT=81;
    }else{
      HTTP_CAT_PORT = EEPROM.readUShort(68);
    }

    // 70-71 udpPort
    if(EEPROM.read(70)==0xff){
      udpPort=89;
    }else{
      udpPort = EEPROM.readUShort(70);
    }

    // 72-73 udpCatPort
    if(EEPROM.read(72)==0xff){
      udpCatPort=90;
    }else{
      udpCatPort = EEPROM.readUShort(72);
    }

  }
//------------------------------------------

  Serial.println();
  Serial.print(" FW | ");
  Serial.println(REV);
  pinMode(HWidPin, INPUT);
    // HWidValue = readADC_Cal(analogRead(HWidPin));
    HWidValue = analogRead(HWidPin);
    if(HWidValue<=150){
      HardwareRev=1;
    }else if(HWidValue>150 && HWidValue<=406){
      HardwareRev=2;  // 253
     }else if(HWidValue>406 && HWidValue<=713){
       HardwareRev=3;  // 560
    // }else if(HWidValue>700 && HWidValue<=900){
    //   HardwareRev=5;  // 807
    // }else if(HWidValue>900){
    //   HardwareRev=6;  // 1036
    }
  Serial.print(" HW | ");
  Serial.print(HardwareRev);
  Serial.print(" [");
  Serial.print(HWidValue);
  Serial.println(" raw]");

  if(APmode==true){
    ledcSetup(pwmChannel, frequence, resolution);
    ledcAttachPin(StatusPin, pwmChannel);
    ledcWrite(pwmChannel, 0);
  }else{    
    pinMode(StatusPin, OUTPUT);
    digitalWrite(StatusPin, LOW);
  }

  pinMode(PowerOnPin, OUTPUT);
    digitalWrite(PowerOnPin, LOW);
  pinMode(CIVmutePin, OUTPUT);
    digitalWrite(CIVmutePin, HIGH);

  #if defined(SELECT_ANT)
    pinMode(ShiftOutLatchPin, OUTPUT);
      digitalWrite(ShiftOutLatchPin, HIGH);
    pinMode(ShiftOutClockPin, OUTPUT);
      digitalWrite(ShiftOutClockPin, LOW);
    pinMode(ShiftOutDataPin, OUTPUT);
      digitalWrite(ShiftOutDataPin, LOW);

    digitalWrite(ShiftOutLatchPin, LOW);
    shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, B10000000);
    shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, B00000000);
    digitalWrite(ShiftOutLatchPin, HIGH);
  #endif

  #if defined(WIFI)
    if (!SPIFFS.begin(true)) {
      Serial.println("SPIFFS| mount failed");
    } else {
      Serial.println("SPIFFS| mounted");
    }

    if(APmode==true){
      // WiFi.softAP(ssid, password); remove the password parameter, if you want the AP (Access Point) to be open
      WiFi.softAP(ssidAP, passwordAP);
      IPAddress IP = WiFi.softAPIP();
      MACString = WiFi.softAPmacAddress();
      Serial.print(" AP | IP address: ");
      Serial.println(IP);

      #if defined(RTLE)
        rtleserver.on("/", handleRTLE);      //This is display page
        rtleserver.begin();                  //Start server
        Serial.println("HTTP| RTLE server started");
      #endif

      if (!MDNS.begin("ic705")) {
        Serial.println("Error setting up MDNS responder!");
        while(1) {
          delay(1000);
        }
      }
      MDNS.addService("http", "tcp", 80);
      MDNS.addService("ws", "tcp", 80);

      setupAsyncHttpServer();
      Serial.println("HTTP| async server started");

      Serial.println("mDNS| responder started");
      APcliAlert();
      Serial.println("SETTINGS  press key to select");
      Serial.println("       ?  list refresh");
      Serial.println("       A  restart to AP mode");
      Serial.println("       E  erase whole eeprom and restart");
      Serial.println("       @  restart device");
      Serial.print( " > " );

    }else{
      ConnectWiFiAlternating();
      Serial.print("WIFI| connected with IP ");
      Serial.println(WiFi.localIP());
      Serial.print("WIFI| ");
      Serial.print(WiFi.RSSI());
      if(APmode==true){
        ledcWrite(pwmChannel, 255);
      }else{
        digitalWrite(StatusPin, HIGH);
      }
      MACString=WiFi.macAddress();
      Serial.print(" dBm, MAC ");
      Serial.println(MACString);

      #if defined(RTLE)
        rtleserver.on("/", handleRTLE);      //This is display page
        rtleserver.begin();                  //Start server
        Serial.println("HTTP| RTLE server started");
      #endif

      // Set up mDNS responder:
      // - first argument is the domain name, in this example
      //   the fully-qualified domain name is "esp32.local"
      // - second argument is the IP address to advertise
      //   we send our IP address on the WiFi network
      if (!MDNS.begin("ic705")) {
          Serial.println("Error setting up MDNS responder!");
          while(1) {
              delay(1000);
          }
      }
      MDNS.addService("ws", "tcp", 80);
      setupAsyncHttpServer();
      Serial.println("HTTP| async server started");
      Serial.println("mDNS| responder started");
    }
  #endif

  if(APmode==false){

    #if defined(HTTP)
      server = WiFiServer(HTTP_CAT_PORT);
      server.begin();

      // Add service to MDNS-SD
      MDNS.addService("http", "tcp", 81);
      MDNS.addService("http", "tcp", 80);
    #endif

    #if defined(UDP_TO_CW)
      udp.begin(udpPort);
    #endif

    #if defined(UDP_TO_CAT)
      udpCat.begin(udpCatPort);
    #endif

    #if defined(UDP_TO_FSK)
      pinMode(PTT,  OUTPUT);
        digitalWrite(PTT, LOW);
      pinMode(FSK_OUT,  OUTPUT);
        digitalWrite(FSK_OUT, LOW);
    #endif

    #if defined(MQTT)
      if(mqttEnable==true){
        if (MQTT_LOGIN == true){
        // if (mqttClient.connect("esp32gwClient", MQTT_USER, MQTT_PASS)){
          //   AfterMQTTconnect();
          // }
        }else{
            IPAddress mqtt_server_ip(mqttBroker[0], mqttBroker[1], mqttBroker[2], mqttBroker[3]);       // MQTT broker IP address
            mqttClient.setServer(mqtt_server_ip, MQTT_PORT);
            Serial.print("MQTT| Connect to ");
            Serial.print(mqtt_server_ip);
            Serial.print(":");
            Serial.println(MQTT_PORT);
            mqttClient.setCallback(MqttRx);
            mqttClient.setKeepAlive(60);
            mqttClient.setSocketTimeout(5);
            Serial.println("MQTT| Callback");
            lastMqttReconnectAttempt = 0;

            MqttBuildClientId();
              Serial.print("MQTT| maccharbuf ");
              Serial.println(mqttClientId);
              mqttReconnect();
        }
      }
    #endif

    // #if defined(BLUETOOTH) // BUGFIX
    //   while (radio_address == 0x00) {
    //     if (!searchRadio()) {
    //       if(Debug==true){
    //         Serial.println("Radio not found");
    //       }
    //     } else {
    //       if(Debug==true){
    //         Serial.print("Radio found at ");
    //         Serial.print(radio_address, HEX);
    //         Serial.println();
    //       }
    //     }
    //   }
    // #endif
    #if defined(BLUETOOTH)
      configRadioBaud(0);
    #endif

    #if defined(WDT)
      // WDT
      esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
      esp_task_wdt_add(NULL); //add current thread to WDT watch
      WdtTimer=millis();
    #endif
  }
}
//-------------------------------------------------------------------------------------------------------

void loop(){
  if(APmode==true){
    #if defined(RTLE)
      rtleserver.handleClient();
    #endif
    CLI();
    ws.cleanupClients();
    wsProcessPendingTimeout();

    // Status LED
    static int PwmValue = 0;
    static bool PwmDir = true;
    static long pwmTimer = 0;
    if(millis()-pwmTimer>3){
      if(PwmDir==true){
        PwmValue++;
        if(PwmValue>254){
          PwmDir=false;
        }
      }else{
        PwmValue--;
        if(PwmValue<1){
          PwmDir=true;
        }
      }
      ledcWrite(pwmChannel, PwmValue);
      pwmTimer=millis();
    }
  }else{
    Watchdog();
    Mqtt();
    httpCAT();
    UdpToCwFsk();
    UdpToCat();
    CLI();
    #if defined(RTLE)
      rtleserver.handleClient();
    #endif
    ws.cleanupClients();
    wsProcessPendingTimeout();
  }
}

// SUBROUTINES -------------------------------------------------------------------------------------------------------

void Watchdog(){
  if (btDisconnectPending == true) {
    btDisconnectPending = false;
    powerTimer = 0;
    frequency = 0;
    modes = "OFF";
    mqttPostponeStatus = 1;
    if (statusPower == 1 && mqttEnable == true) {
      MqttPubString(MQTT_TOPIC, "0", 0);
    }
    if (wsPendingRequest.active) {
      wsSendErrorById(wsPendingRequest.clientId, "radio_disconnected", "Radio disconnected before reply arrived", wsPendingRequest.reqId);
      wsClearPendingRequest();
    }
    wsClearSplitProbe();
    wsFlushQueuedRequests("radio_disconnected", "Radio disconnected before queued request could be sent");
    wsBroadcastRigState();
  }

  static unsigned long catPollTimer = 0;
  static unsigned long mqttFreqTimer = 0;
  if (btClientConnected == true) {
    if (!wsPendingRequest.active && millis() - catPollTimer > 500) {
      catPollTimer = millis();

      if (radio_address == 0x00) {
        if (Debug == true) Serial.println("CAT | searching radio...");
        searchRadio();
      } else {
        sendCatRequest(CMD_READ_FREQ);
        processCatMessages();
      }
    }
  }

  // set TRX after BT connect
  if(TrxNeedSet==1 && TrxSetupDone == false){
    TrxNeedSet=0;
    /*
    FE FE A4 E0 <command> FD
      - enable CI-V transceive after BT connect, 1A 05 01 31 01
      - enable RIT 21 01 01|clear RIT 21 00 00 00 00 (RIT freq to 0)
      - IP to CW after connect, 16 47 00 (BK-IN OFF), 16 47 01 (BK-IN ON)
    */

    // Enable CI-V transceive
    Serial.print("SET |CIV-tx-ON");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x1A;
    CatMsg[1] = 0x05;
    CatMsg[2] = 0x01;
    CatMsg[3] = 0x31;
    CatMsg[4] = 0x01;
    sendCat();
    ServiceBackgroundTasks(500);

    // Enable RIT
    Serial.print("|RIT-ON");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x21;
    CatMsg[1] = 0x01;
    CatMsg[2] = 0x01;
    sendCat();
    ServiceBackgroundTasks(500);

    // Disable BK-IN
    Serial.print("|BK-IN-OFF");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x16;
    CatMsg[1] = 0x47;
    CatMsg[2] = 0x00;
    sendCat();
    ServiceBackgroundTasks(500);

    // Set mode CW
    Serial.print("|MODE-CW");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x06;
    CatMsg[1] = 0x03;
    CatMsg[2] = 0x03;
    sendCat();
    ServiceBackgroundTasks(500);

    // Set CW 28 WPM - 0000=6 WPM ~ 0255=48 WPM (6 per one wpm)
    Serial.print("|CW-WPM-28");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x14;
    CatMsg[1] = 0x0C;
    CatMsg[2] = 0x01;
    CatMsg[3] = 0x38;
    sendCat();
    ServiceBackgroundTasks(500);

    // Set AFgain to 10
    Serial.print("|AFgain-10");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x14;
    CatMsg[1] = 0x01;
    CatMsg[2] = 0x00;
    CatMsg[3] = 0x25;
    sendCat();
    ServiceBackgroundTasks(500);

    // Set RFgain to 0
    Serial.print("|RFgain-0");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x14;
    CatMsg[1] = 0x02;
    CatMsg[2] = 0x00;
    CatMsg[3] = 0x00;
    sendCat();
    ServiceBackgroundTasks(500);

    // CW send IP
    Serial.print("|CWsend-IP..");
    #if defined(RESET_AFTER_DISCONNECT)
      String StrBuf = "="+String(WiFi.localIP()[3]) ;
    #else
      String StrBuf = "IP=  "+String(WiFi.localIP()[0])+"."+String(WiFi.localIP()[1])+"."+String(WiFi.localIP()[2])+"."+String(WiFi.localIP()[3]) ;
    #endif
    StrBuf.toCharArray(CwMsg, StrBuf.length()+1);
    modes="CW";
    sendCW();
    #if defined(RESET_AFTER_DISCONNECT)
      ServiceBackgroundTasks(2000);
    #else
      ServiceBackgroundTasks(11000);
    #endif

    // Enable BK-IN
    Serial.print("|BK-IN-ON");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x16;
    CatMsg[1] = 0x47;
    CatMsg[2] = 0x01;
    sendCat();
    ServiceBackgroundTasks(500);

    // Set RFgain to 100
    Serial.print("|RFgain-100");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x14;
    CatMsg[1] = 0x02;
    CatMsg[2] = 0x02;
    CatMsg[3] = 0x55;
    sendCat();
    ServiceBackgroundTasks(500);

    Serial.println();
    TrxSetupDone = true;
  }

  // BT CAT
  static long requestTimer = 0;
  if(millis()-requestTimer > 1000){
    // Serial.println("req "+String(millis()/1000));
    if (radio_address == 0x00) {  // BUGFIX
      if (millis() - requestTimer > 1000) {
        if (Debug == true) Serial.println("CAT | searching radio address...");
        searchRadio();
        requestTimer = millis();
      }
      processCatMessages();
      return;
    }
    sendCatRequest(CMD_READ_FREQ);
    // sendCatRequest(CMD_READ_MODE);   // BUGFIX?
    requestTimer=millis();
  }
  processCatMessages();

  // Power OUT/LED
  if(millis()-powerTimer > 3000){
    if(statusPower==1){
      digitalWrite(PowerOnPin, LOW);
      Serial.println(" PWR| OFF");
      statusPower = 0;
      frequency = 0;
      frequencyTmp = 0;
      modes = "OFF";
      #if defined(RESET_AFTER_DISCONNECT)
        Serial.println();
        Serial.println("** Interface will be restarted **");
        delay(3000);
        ESP.restart();
      #endif
    }
  }else{
    if(statusPower==0){
      digitalWrite(PowerOnPin, HIGH);
      Serial.println(" PWR| ON");
      statusPower = 1;
      mqttPostponeTimer = millis();
      mqttPostponeStatus = 0;
    }
  }

  // postponed MQTT
  if(statusPower==1 && btClientConnected==true && radio_address!=0x00 && millis()-mqttPostponeTimer > 11000 && mqttPostponeStatus==0){
    mqttPostponeStatus = 1;
    MqttPubString(MQTT_TOPIC, String(frequency), 0);
  }

  if(statusPower==1 && btClientConnected==true && radio_address!=0x00 && millis()-mqttFreqTimer > 2000 && frequencyTmp!=frequency){
  // MQTT
    if(mqttEnable==true){
      // Serial.println("MQTT>"+String(frequency)+"Hz "+String(modes) );
      MqttPubString(MQTT_TOPIC, String(frequency), 0);
    }
    frequencyTmp=frequency;
    mqttFreqTimer=millis();
    #if defined(SELECT_ANT)
      SelectANT();
    #endif

    // CAT out
    #if defined(CIV_OUT)
      // fefe560e03fd fefe0e56038079022800fd fefe560e04fd fefe0e56040301fd    request
      // fefe005600 90 79 02 28 00 fd fefe0056000080022800fd    CIV-TX-ON
      // FEFEE04203 00 00 58 45 01 FD  -145.580.000
      byte buffer[12];
      String StrFreq;
        StrFreq=IntToTenString(frequency);
        // Serial.println(StrFreq);
      String SplitStrFreq[5];
        SplitString(StrFreq, SplitStrFreq);

      buffer[0]=0xFE;
      buffer[1]=0xFE;
      buffer[2]=0x00;
      buffer[3]=0x42;
      buffer[4]=0x00;    
      for (int i = 5; i < 11; i++) {
        // buffer[i] = stringToByte(SplitStrFreq[4-(i-5)]);   // PANIC
      }
      buffer[10]=0xFD;

      digitalWrite(CIVmutePin, LOW);
      delay(2);
      for (int i = 0; i < 11; i++) {
        Serial.write(buffer[i]);
      }
      Serial.flush();
      delay(2);
      digitalWrite(CIVmutePin, HIGH);
      delay(2);
      Serial.println();
     #endif
  }

  // WIFI status
  #if defined(WIFI)
    unsigned long currentMillis = millis();
    // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
    // if ((WiFi.status() != WL_CONNECTED) && (currentMillis - WifiTimer >=WifiReconnect)) {
    //   Serial.print(millis());
    //   Serial.println("cReconnecting...");
    //   WiFi.disconnect();
    //   WiFi.reconnect();
    //   WifiTimer = currentMillis;
    // }
    if (!APmode && (currentMillis - WifiTimer >= WifiReconnect)) {  // BUGFIX
      wl_status_t st = WiFi.status();

      if (st == WL_DISCONNECTED || st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL) {
        Serial.print(currentMillis);
        Serial.println(" WIFI| reconnecting...");
        ActiveWifiProfile = NextWifiProfile(ActiveWifiProfile);
        if (WifiProfileConfigured(ActiveWifiProfile) == false) {
          ActiveWifiProfile = 0;
        }
        String wifiSsid = WifiProfileSSID(ActiveWifiProfile);
        String wifiPswd = WifiProfilePSWD(ActiveWifiProfile);
        WiFi.disconnect(false, false);
        delay(100);
        WiFi.begin(wifiSsid.c_str(), wifiPswd.c_str());
        Serial.print("WIFI| retry ssid ");
        Serial.println(wifiSsid);
        WifiTimer = currentMillis;
      }
    }
  #endif

  // WDT
  #if defined(WDT)
    if(millis()-WdtTimer > 60000){
      esp_task_wdt_reset();
      WdtTimer=millis();
    }
  #endif

}

//-------------------------------------------------------------------------------------------------------

void ServiceBackgroundTasks(unsigned long waitMs) {
  unsigned long start = millis();
  while (millis() - start < waitMs) {
    #if defined(MQTT)
      if (mqttEnable == true && mqttClient.connected() == true) {
        mqttClient.loop();
      }
    #endif
    ws.cleanupClients();
    wsProcessPendingTimeout();
    delay(10);
  }
}

//-------------------------------------------------------------------------------------------------------
bool WifiProfileConfigured(byte profile) {
  if (profile == 0) {
    return SSID.length() > 0 && PSWD.length() > 0;
  }
  if (profile == 1) {
    return SSID2.length() > 0 && PSWD2.length() > 0;
  }
  return false;
}

//-------------------------------------------------------------------------------------------------------
String WifiProfileSSID(byte profile) {
  if (profile == 1) {
    return SSID2;
  }
  return SSID;
}

//-------------------------------------------------------------------------------------------------------
String WifiProfilePSWD(byte profile) {
  if (profile == 1) {
    return PSWD2;
  }
  return PSWD;
}

//-------------------------------------------------------------------------------------------------------
byte NextWifiProfile(byte profile) {
  if (profile == 0 && WifiProfileConfigured(1)) {
    return 1;
  }
  return 0;
}

//-------------------------------------------------------------------------------------------------------
bool ConnectWiFiProfile(byte profile, int maxTryCount) {
  if (WifiProfileConfigured(profile) == false) {
    return false;
  }

  String wifiSsid = WifiProfileSSID(profile);
  String wifiPswd = WifiProfilePSWD(profile);

  WiFi.disconnect(false, false);
  delay(100);
  WiFi.begin(wifiSsid.c_str(), wifiPswd.c_str());
  Serial.print("WIFI| Connecting ssid ");
  Serial.print(wifiSsid);
  Serial.print(" ");

  for (int count_try = 0; count_try < maxTryCount; count_try++) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      ActiveWifiProfile = profile;
      return true;
    }
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  print_wifi_error();
  return WiFi.status() == WL_CONNECTED;
}

//-------------------------------------------------------------------------------------------------------
void ConnectWiFiAlternating() {
  WiFi.mode(WIFI_STA);

  if (WifiProfileConfigured(0) == false && WifiProfileConfigured(1) == false) {
    Serial.println("WIFI| no configured SSID, staying in AP mode");
    APmode = true;
    EEPROM.writeBool(0, true);
    EEPROM.commit();
    delay(500);
    ESP.restart();
  }

  byte wifiProfile = 0;
  if (WifiProfileConfigured(0) == false && WifiProfileConfigured(1) == true) {
    wifiProfile = 1;
  }

  WifiTimer = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (ConnectWiFiProfile(wifiProfile, wifi_max_try)) {
      break;
    }
    Serial.println("WIFI| switching to next configured SSID");
    wifiProfile = NextWifiProfile(wifiProfile);
  }
}

//-------------------------------------------------------------------------------------------------------

String IntToTenString(int NR) {
  String str = String(NR);
  while (str.length() < 10) {
    str = "0" + str;
  }
  return str;
}

//-------------------------------------------------------------------------------------------------------
void SplitString(String ORIGINAL, String* SplitStrFreq) {
  for (int i = 0; i < 5; i++) {
    SplitStrFreq[i] = ORIGINAL.substring(i * 2, i * 2 + 2);
    // Serial.println(SplitStrFreq[i]);
  }
}

//-------------------------------------------------------------------------------------------------------

byte stringToByte(String str) {
  // Převod prvního znaku na číslo
  byte prvniZnak = str.charAt(0) - '0';

  // Převod druhého znaku na číslo a posun do vyššího číselného řádu
  byte druhyZnak = str.charAt(1) - '0';

  // Sestavení výsledné hodnoty
  byte vysledek = (prvniZnak << 4) | druhyZnak;

  return vysledek;
}

//-------------------------------------------------------------------------------------------------------
// void SelectANT(){
//   #if defined(SELECT_ANT)
//     TRXselectANT = 42;
//     for(int ant=0; ant<8; ant++){
//       if(ANTrange[ant][0]<frequency && frequency<ANTrange[ant][1]){
//         TRXselectANT=ant;
//       }else{
//       }
//     }
//     ShiftOutByte=0x00;
//     bitSet(ShiftOutByte, TRXselectANT);
//     digitalWrite(ShiftOutLatchPin, LOW);
//       shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte);
//     digitalWrite(ShiftOutLatchPin, HIGH);
//   #endif
// }
void SelectANT(){   // BUGFIX
  #if defined(SELECT_ANT)
    int foundAnt = -1;

    for (int ant = 0; ant < 8; ant++) {
      if (ANTrange[ant][0] < frequency && frequency < ANTrange[ant][1]) {
        foundAnt = ant;
        break;
      }
    }

    if (foundAnt < 0 || foundAnt > 7) {
      if (Debug == true) {
        Serial.print("ANT | no matching range for frequency ");
        Serial.println(frequency);
      }
      return;
    }

    TRXselectANT = foundAnt;
    ShiftOutByte = 0x00;
    bitSet(ShiftOutByte, TRXselectANT);

    digitalWrite(ShiftOutLatchPin, LOW);
    shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte);
    digitalWrite(ShiftOutLatchPin, HIGH);
  #endif
}

//-------------------------------------------------------------------------------------------------------
// uint32_t readADC_Cal(int ADC_Raw)
// {
//   esp_adc_cal_characteristics_t adc_chars;

//   esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
//   return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
// }

//-------------------------------------------------------------------------------------------------------
void print_wifi_error(){
  switch(WiFi.status())
  {
    case WL_IDLE_STATUS : Serial.println("WiFi| WL_IDLE_STATUS"); break;
    case WL_NO_SSID_AVAIL : Serial.println("WiFi| WL_NO_SSID_AVAIL"); break;
    case WL_CONNECT_FAILED : Serial.println("WiFi| WL_CONNECT_FAILED"); break;
    case WL_DISCONNECTED : Serial.println("WiFi| WL_DISCONNECTED"); break;
    default : Serial.printf("WiFi| No know WiFi error"); break;
  }
}
//-------------------------------------------------------------------------------------------------------
// void processCatMessages(){
//   #if defined(BLUETOOTH)
//     /*
//       <FE FE E0 42 04 00 01 FD  - LSB
//       <FE FE E0 42 03 00 00 58 45 01 FD  -145.580.000

//       FE FE - start bytes
//       00/E0 - target address (broadcast/controller)
//       42 - source address
//       00/03 - data type
//       <data>
//       FD - stop byte
//     */

//     while (CAT.available()) {
//       uint8_t knowncommand = 1;
//       uint8_t r;
//       if (readLine() > 0) {
//         if (read_buffer[0] == START_BYTE && read_buffer[1] == START_BYTE) {
//           if (read_buffer[3] == radio_address) {
//             if (read_buffer[2] == BROADCAST_ADDRESS) {
//               switch (read_buffer[4]) {
//                 case CMD_TRANS_FREQ:
//                   printFrequency();
//                   break;
//                 case CMD_TRANS_MODE:
//                   printMode();
//                   break;
//                 default:
//                   knowncommand = false;
//               }
//             } else if (read_buffer[2] == CONTROLLER_ADDRESS) {
//               switch (read_buffer[4]) {
//                 case CMD_READ_FREQ:
//                   printFrequency();
//                   break;
//                 case CMD_READ_MODE:
//                   printMode();
//                   break;
//                 default:
//                   knowncommand = false;
//               }
//             }
//           } else {
//             // if(Debug==true){
//             //   Serial.print(read_buffer[3]);
//             //   Serial.println(" also on-line?!");
//             // }
//           }
//         }
//         powerTimer=millis(); // RX BT

//       }

//       // if(Debug==true){
//       //   if(!knowncommand){
//       //   Serial.print("<");
//       //     if(read_buffer[10] == STOP_BYTE){
//       //       memcpy(read_buffer_snapshot, read_buffer, sizeof(read_buffer));
//       //       for (uint8_t i = 0; i < sizeof(read_buffer); i++){
//       //         if (read_buffer[i] < 16)Serial.print("0");
//       //         Serial.print(read_buffer[i], HEX);
//       //         Serial.print(" ");
//       //         if (read_buffer[i] == STOP_BYTE)break;
//       //       }
//       //       Serial.println();
//       //     }
//       //   }
//       // }
//     }

//   // #ifdef MIRRORCAT
//   //   while (Serial2.available()) {
//   //     CAT.print((byte)Serial2.read());
//   //   }
//   // #endif
//   #endif
// }
void processCatMessages(){  // BUGFIX
  #if defined(BLUETOOTH)
    while (CAT.available()) {
      uint8_t len = readLine();
      if (len == 0) return;

      if (Debug == true) {
        Serial.print("CAT RX < ");
        for (uint8_t i = 0; i < len; i++) {
          if (read_buffer[i] < 16) Serial.print("0");
          Serial.print(read_buffer[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }

      if (len < 6) continue;
      if (read_buffer[0] != START_BYTE || read_buffer[1] != START_BYTE) continue;
      if (read_buffer[len - 1] != STOP_BYTE) continue;
      // if (read_buffer[3] != radio_address) continue;
      if (radio_address == 0x00) {  // BUGFIX
        radio_address = read_buffer[3];
        if (Debug == true) {
          Serial.print("CAT | learned radio address 0x");
          Serial.println(radio_address, HEX);
        }
        if (TrxSetupDone == false) {
          TrxNeedSet = 1;
        }
      } else if (read_buffer[3] != radio_address) {
        continue;
      }

      String matchedReqId;
      bool matchedPendingRequest = wsResolvePendingRequest(read_buffer, len, &matchedReqId);
      wsBroadcastCivFrame("civ.rx", read_buffer, len, matchedPendingRequest ? matchedReqId : "");

      if (read_buffer[2] == BROADCAST_ADDRESS || read_buffer[2] == CONTROLLER_ADDRESS) {
        switch (read_buffer[4]) {
          case CMD_TRANS_FREQ:
          case CMD_READ_FREQ:
            if (len >= 11) printFrequency();
            break;

          case CMD_TRANS_MODE:
          case CMD_READ_MODE:
            if (len >= 7) printMode();
            break;
        }
      }

      powerTimer = millis();
      wsBroadcastRigState();
    }
  #endif
}

//-------------------------------------------------------------------------------------------------------
// call back to get info about connection
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){ // BUGFIX
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    btClientConnected = true;
    radio_address = 0xA4;
    frequency = 0;
    frequencyTmp = 0;
    if (TrxSetupDone == false) {
      TrxNeedSet = 1;
    }

    Serial.println("    | Client Connected");
    Serial.println("    | CHANGE frequency on IC-705, for initialize CAT");
    Serial.println("    | -----------------------------------------------------------------------------");
    wsBroadcastRigState();
  }

  if (event == ESP_SPP_CLOSE_EVT) {
    btClientConnected = false;
    radio_address = 0x00;
    TrxNeedSet = 0;
    btDisconnectPending = true;
    Serial.println("    | Client Disconnected");
    wsBroadcastRigState();
  }
}
//-------------------------------------------------------------------------------------------------------
// void configRadioBaud(uint16_t  baudrate){
//   #if defined(BLUETOOTH)
//     if (!CAT.begin(BTname)) //Bluetooth device name
//     {
//       Serial.println(" BT | An error occurred initializing Bluetooth");
//     } else {
//       CAT.register_callback(callback);
//       Serial.println(" BT | Initialized");
//       Serial.println("    | -----------------------------------------------------------------------------");
//       Serial.println(" BT | The device started, now you MUST PAIR it with Bluetooth name "+String(BTname));
//     }
//   #endif
// }
void configRadioBaud(uint16_t baudrate){
  #if defined(BLUETOOTH)
    static bool btInitDone = false;
    if (btInitDone) return;

    if (!CAT.begin(BTname)) {
      Serial.println(" BT | An error occurred initializing Bluetooth");
    } else {
      CAT.register_callback(callback);
      Serial.println(" BT | Initialized");
      Serial.println("    | -----------------------------------------------------------------------------");
      Serial.println(" BT | The device started, now you MUST PAIR it with Bluetooth name " + String(BTname));
      Serial.println("    | -----------------------------------------------------------------------------");
      btInitDone = true;
    }
  #endif
}

//-------------------------------------------------------------------------------------------------------
uint8_t readLine(void){
  #if defined(BLUETOOTH)
    uint8_t byte;
    uint8_t counter = 0;
    uint32_t ed = readtimeout;
    read_buffer[10] = 0x00;

    while (true)
    {
      while (!CAT.available()) {
        if (--ed == 0)return 0;
      }
      ed = readtimeout;
      byte = CAT.read();
      if (byte == 0xFF)continue; //TODO skip to start byte instead
      // #ifdef MIRRORCAT
      //     Serial2.write(byte); // !byte
      // #endif

      read_buffer[counter++] = byte;
      if (STOP_BYTE == byte) break;

      if (counter >= sizeof(read_buffer))return 0;
    }
    return counter;
  #endif
}
//-------------------------------------------------------------------------------------------------------
void radioSetMode(uint8_t modeid, uint8_t modewidth){
  #if defined(BLUETOOTH)
    uint8_t req[] = {START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS, CMD_WRITE_MODE, modeid, modewidth, STOP_BYTE};
    if(Debug==true){
      Serial.print("CAT TX >");
    }
    for (uint8_t i = 0; i < sizeof(req); i++) {
      if(Debug==true){
        if (req[i] < 16)Serial.print("0");
        Serial.print(req[i], HEX);
        Serial.print(" ");
      }
    }
    catWriteFrame(req, sizeof(req), true);
    if(Debug==true){
      Serial.println();
    }
  #endif
}
//-------------------------------------------------------------------------------------------------------
bool radioSetFrequency(uint32_t freqHz){
  #if defined(BLUETOOTH)
    if (btClientConnected == false || radio_address == 0x00) {
      return false;
    }

    String strFreq = IntToTenString(freqHz);
    String splitFreq[5];
    SplitString(strFreq, splitFreq);

    uint8_t req[] = {
      START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS, CMD_WRITE_FREQ,
      stringToByte(splitFreq[4]),
      stringToByte(splitFreq[3]),
      stringToByte(splitFreq[2]),
      stringToByte(splitFreq[1]),
      stringToByte(splitFreq[0]),
      STOP_BYTE
    };

    if(Debug==true){
      Serial.print("CAT TX >");
    }
    for (uint8_t i = 0; i < sizeof(req); i++) {
      if(Debug==true){
        if (req[i] < 16)Serial.print("0");
        Serial.print(req[i], HEX);
        Serial.print(" ");
      }
    }
    catWriteFrame(req, sizeof(req), true);
    if(Debug==true){
      Serial.println();
    }
    return true;
  #endif
  return false;
}
//-------------------------------------------------------------------------------------------------------
void sendCatRequest(uint8_t requestCode){
  #if defined(BLUETOOTH)
    uint8_t req[] = {START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS, requestCode, STOP_BYTE};
    if(Debug==true){
      Serial.print("CAT TX >");
    }
    for (uint8_t i = 0; i < sizeof(req); i++) {
      if(Debug==true){
        if (req[i] < 16)Serial.print("0");
        Serial.print(req[i], HEX);
        Serial.print(" ");
      }
    }
    catWriteFrame(req, sizeof(req), true);
    if(Debug==true){
      Serial.println();
    }
  #endif
}
//-------------------------------------------------------------------------------------------------------
// bool searchRadio(){
//   #if defined(BLUETOOTH)
//     for (uint8_t baud = 0; baud < BAUD_RATES_SIZE; baud++) {
//       if(Debug==true){
//         Serial.print("Try baudrate ");
//         Serial.println(baudRates[baud]);
//       }
//         configRadioBaud(baudRates[baud]);
//         sendCatRequest(CMD_READ_FREQ);

//         if (readLine() > 0)
//         {
//           if (read_buffer[0] == START_BYTE && read_buffer[1] == START_BYTE) {
//             radio_address = read_buffer[3];
//           }
//           return true;
//         }
//     }

//       // radio_address = 0xFF; // BUGFIX
//       return false;
//   #endif
// }

bool searchRadio(){ // BUGFIX
  #if defined(BLUETOOTH)
    sendCatRequest(CMD_READ_FREQ);

    uint8_t len = readLine();
    if (len > 0) {
      if (read_buffer[0] == START_BYTE && read_buffer[1] == START_BYTE) {
        radio_address = read_buffer[3];
        if (Debug == true) {
          Serial.print("CAT | found radio address 0x");
          Serial.println(radio_address, HEX);
        }
        return true;
      }
    }
    return false;
  #endif
}
  
//-------------------------------------------------------------------------------------------------------
void printFrequency(void){
  #if defined(BLUETOOTH)
      frequency = 0;
      //FE FE E0 42 03 <00 00 58 45 01> FD ic-820
      //FE FE 00 40 00 <00 60 06 14> FD ic-732
      for (uint8_t i = 0; i < 5; i++) {
        if (read_buffer[9 - i] == 0xFD)continue; //spike
        if(Debug==true){

          if (read_buffer[9 - i] < 16)Serial.print("0");
          Serial.print(read_buffer[9 - i], HEX);
        }
        frequency += (read_buffer[9 - i] >> 4) * decMulti[i * 2];
        frequency += (read_buffer[9 - i] & 0x0F) * decMulti[i * 2 + 1];
      }
      if(Debug==true){
        Serial.println();
      }
  #endif
}
//-------------------------------------------------------------------------------------------------------
// void printMode(void){
//   #if defined(BLUETOOTH)
//     //FE FE E0 42 04 <00 01> FD
//     if(Debug==true){
//       Serial.println(mode[read_buffer[5]]);
//     }
//     modes = mode[read_buffer[5]];
//     //read_buffer[6] -> 01 - Wide, 02 - Medium, 03 - Narrow
//   #endif
// }
void printMode(void){ // BUGFIX
  #if defined(BLUETOOTH)
    uint8_t modeId = read_buffer[5];

    switch (modeId) {
      case 0x00: modes = "LSB"; break;
      case 0x01: modes = "USB"; break;
      case 0x02: modes = "AM";  break;
      case 0x03: modes = "CW";  break;
      case 0x04: modes = "RTTY"; break;
      case 0x05: modes = "FM";  break;
      case 0x06: modes = "WFM"; break;
      case 0x17: modes = "DV";  break;
      default:
        modes = "UNK";
        if (Debug == true) {
          Serial.print("CAT | invalid/unknown mode id: 0x");
          if (modeId < 16) Serial.print("0");
          Serial.println(modeId, HEX);
        }
        return;
    }

    if (Debug == true) {
      Serial.print("CAT | mode=");
      Serial.println(modes);
    }
  #endif
}
//-------------------------------------------------------------------------------------------------------
void MqttBuildClientId(){
  #if defined(MQTT)
    uint8_t mac[6];
    WiFi.macAddress(mac);
    snprintf(mqttClientId, sizeof(mqttClientId), "IC705-%02X%02X%02X%02X%02X%02X",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  #endif
}
//-------------------------------------------------------------------------------------------------------
void Mqtt(){
  #if defined(MQTT)
    if(mqttEnable==true){
      if (millis()-MqttStatusTimer[0]>MqttStatusTimer[1]){
        if(!mqttClient.connected()){
          long now = millis();
          if (now - lastMqttReconnectAttempt > 10000) {
            lastMqttReconnectAttempt = now;
            Serial.print("MQTT| Attempt to MQTT reconnect | ");
            Serial.println(millis()/1000);
            if (mqttReconnect()) {
              lastMqttReconnectAttempt = 0;
            }
          }
        }else{
          // Serial.println("MQTT| Client connected");
          mqttClient.loop();
        }
        MqttStatusTimer[0]=millis();
      }
    }
  #endif
}

//-------------------------------------------------------------------------------------------------------

#if defined(MQTT)
  // if(mqttEnable==true){
    bool mqttReconnect() {
        MqttBuildClientId();
        if (mqttClient.connect(mqttClientId)) {
          Serial.println("MQTT| mqttReconnect-connected");
          if(MQTT_TOPIC_RX.length() > 0){
            MQTT_TOPIC_RX.toCharArray(mqttPath, 50);
            if(mqttClient.subscribe(mqttPath)==true){
              Serial.print("MQTT| subscribe ");
              Serial.println(mqttPath);
            }else{
              Serial.print("MQTT| subscribe failed ");
              Serial.println(mqttPath);
            }
          }
        }else{
          Serial.println("MQTT| mqttReconnect-not-connected");
        }
        return mqttClient.connected();
    }
  // }
#endif
//------------------------------------------------------------------------------------
void MqttRx(char *topic, byte *payload, unsigned int length) {
  #if defined(MQTT)
    if(mqttEnable==true){
      if (MQTT_TOPIC_RX.length() == 0) {
        return;
      }

      String rxTopic = String(topic);
      if (rxTopic != MQTT_TOPIC_RX) {
        return;
      }

      char payloadBuf[32];
      unsigned int copyLen = length;
      if (copyLen >= sizeof(payloadBuf)) {
        copyLen = sizeof(payloadBuf) - 1;
      }
      memcpy(payloadBuf, payload, copyLen);
      payloadBuf[copyLen] = '\0';

      uint32_t newFreq = strtoul(payloadBuf, NULL, 10);
      Serial.print("RXmqtt < ");
      Serial.print(rxTopic);
      Serial.print(" ");
      Serial.println(payloadBuf);

      if (newFreq == 0) {
        Serial.println("MQTT| RX freq ignored, invalid payload");
        return;
      }

      if (radioSetFrequency(newFreq)) {
        Serial.print("CAT | set VFO ");
        Serial.println(newFreq);
      } else {
        Serial.println("CAT | set VFO skipped, TRX not connected");
      }
    }
  #endif
} // MqttRx END
//-----------------------------------------------------------------------------------
void MqttPubString(String TOPICEND, String DATA, bool RETAIN){
  #if defined(MQTT)
    if(mqttEnable==true){
      if(APmode==true){
        ledcWrite(pwmChannel, 0);
      }else{
        digitalWrite(StatusPin, LOW);
      }
      if(mqttClient.connected()==true){
        mqttClient.loop();
        Serial.print("MQTT| ");
        String topic = String(TOPICEND);
        topic.toCharArray( mqttPath, 50 );
        DATA.toCharArray( mqttTX, 50 );
        if (mqttClient.publish(mqttPath, mqttTX, RETAIN)) {
          Serial.print(mqttPath);
          Serial.print(" ");
          Serial.println(mqttTX);
        } else {
          Serial.print("publish failed, state=");
          Serial.println(mqttClient.state());
        }
      }else{
        Serial.print("MQTT| skip publish, disconnected state=");
        Serial.println(mqttClient.state());
      }
      delay(100);
      if(APmode==true){
        ledcWrite(pwmChannel, 255);
      }else{
        digitalWrite(StatusPin, HIGH);
      }
    }
  #endif
}
//-----------------------------------------------------------------------------------

void httpCAT(){
  #if defined(HTTP)
  // listen for incoming clients
  WiFiClient webCatClient = server.available();
  if (webCatClient) {
      if(Debug==true){
        Serial.println("WebCat new client");
      }
      memset(linebuf,0,sizeof(linebuf));
      charcount=0;
      // an http request ends with a blank line
      boolean currentLineIsBlank = true;
      unsigned long idleStart = millis();
      while (webCatClient.connected()) {
        if (webCatClient.available()) {
          idleStart = millis();
          char c = webCatClient.read();
          if(Debug==true){
            Serial.print(c);
          }
          //read char by char HTTP request
          linebuf[charcount]=c;
          if (charcount<sizeof(linebuf)-1) charcount++;
          // if you've gotten to the end of the line (received a newline
          // character) and the line is blank, the http request has ended,
          // so you can send a reply
          if (c == '\n' && currentLineIsBlank) {
            // send a standard http response header
            webCatClient.println(F("HTTP/1.1 200 OK"));
            webCatClient.println(F("Content-Type: text/html"));
            webCatClient.println(F("Connection: close"));  // the connection will be closed after completion of the response
            webCatClient.println();
            if(statusPower==1){
              webCatClient.print(String(frequency)+"|"+String(modes)+"|");
            }else{
              webCatClient.print("0|OFF|");
            }
            break;
          }
          if (c == '\n') {
            // you're starting a new line
            currentLineIsBlank = true;
            // if (strstr(linebuf,"GET /h0 ") > 0){digitalWrite(GPIOS[0], HIGH);}else if (strstr(linebuf,"GET /l0 ") > 0){digitalWrite(GPIOS[0], LOW);}
            // else if (strstr(linebuf,"GET /h1 ") > 0){digitalWrite(GPIOS[1], HIGH);}else if (strstr(linebuf,"GET /l1 ") > 0){digitalWrite(GPIOS[1], LOW);}

            // you're starting a new line
            currentLineIsBlank = true;
            memset(linebuf,0,sizeof(linebuf));
            charcount=0;
          } else if (c != '\r') {
            // you've gotten a character on the current line
            currentLineIsBlank = false;
          }
        } else if (millis() - idleStart > 1000) {
          if(Debug==true){
            Serial.println("WebCat client timeout");
          }
          break;
        }
      }
      // give the web browser time to receive the data
      delay(1);
      // close the connection:
      webCatClient.stop();
      if(Debug==true){
        Serial.println("WebCat client disconnected");
      }
  }
  #endif
}
//-----------------------------------------------------------------------------------

void UdpToCat(){
  #if defined(UDP_TO_CAT)
    memset(CatMsg, 0xff, 10);

    udpCat.parsePacket();
    // int packetSize = udp.parsePacket();
    if(udpCat.read(CatMsg, 10) > 0){
      // Serial.print("UDPcat rx: ");
      // Serial.println((char *)CatMsg);

      // clear RIT only - SECURITY FEATURE
      if(statusPower==1 && CatMsg[0]==0x21){
        sendCat();
      }
    }

  #endif
}
//-----------------------------------------------------------------------------------

void UdpToCwFsk(){
  #if defined(UDP_TO_CW)
    // //data will be sent to server
    // uint8_t buffer[30] = "";
    // //send hello world to server
    // udp.beginPacket(udpAddress, udpPort);
    // udp.write(buffer, 11);
    // udp.endPacket();
    memset(CwMsg, 0, sizeof(CwMsg));

    //processing incoming packet, must be called before reading the buffer
    udp.parsePacket();
    //receive response from server, it will be HELLO WORLD
    int packetLen = udp.read(CwMsg, sizeof(CwMsg) - 1);
    if(packetLen > 0){
      CwMsg[packetLen] = '\0';
      if(APmode==true){
        ledcWrite(pwmChannel, 0);
      }else{
        digitalWrite(StatusPin, LOW);
      }
      Serial.print("UDP FSK rx: ");
      Serial.println((char *)CwMsg);
      if(statusPower==1){
        sendCW();
      }
    }

  #endif
}
//-------------------------------------------------------------------------------------------------------
void sendCat(){
  #if defined(UDP_TO_CAT)
    uint8_t frame[sizeof(CatMsg) + 5];
    uint8_t frameLen = 0;

    frame[frameLen++] = START_BYTE;
    frame[frameLen++] = START_BYTE;
    frame[frameLen++] = radio_address;
    frame[frameLen++] = CONTROLLER_ADDRESS;

    for (uint8_t i = 0; i < sizeof(CatMsg); i++) {
      if(CatMsg[i] == 0xff){
        continue;
      }
      if(frameLen >= sizeof(frame) - 1){
        break;
      }
      frame[frameLen++] = CatMsg[i];
    }

    frame[frameLen++] = STOP_BYTE;
    catWriteFrame(frame, frameLen, true);
  #endif
}
//-------------------------------------------------------------------------------------------------------
void sendCW(){
  int payloadLen = strnlen(CwMsg, sizeof(CwMsg) - 1);

  if(modes=="CW"){  // CAT -----------------
    uint8_t frame[sizeof(CwMsg) + 6];
    uint8_t frameLen = 0;

    frame[frameLen++] = START_BYTE;
    frame[frameLen++] = START_BYTE;
    frame[frameLen++] = radio_address;
    frame[frameLen++] = CONTROLLER_ADDRESS;
    frame[frameLen++] = CMD_SEND_CW_MSG;
    for (int i = 0; i < payloadLen && frameLen < sizeof(frame) - 1; i++) {
      frame[frameLen++] = static_cast<uint8_t>(CwMsg[i]);
    }
    frame[frameLen++] = STOP_BYTE;

    Serial.print("CW ");
    for (uint8_t i = 0; i < frameLen; i++) {
      Serial.print(frame[i], HEX);
      Serial.print(" ");
    }
    catWriteFrame(frame, frameLen, true);
    Serial.println();
    delay(100);
    if(APmode==true){
      ledcWrite(pwmChannel, 255);
      delay(100);
      ledcWrite(pwmChannel, 0);
      delay(100);
      ledcWrite(pwmChannel, 255);
    }else{
      digitalWrite(StatusPin, HIGH);
      delay(100);
      digitalWrite(StatusPin, LOW);
      delay(100);
      digitalWrite(StatusPin, HIGH);
    }
  }else if(modes=="FSK"){ // GPIO -----------------
    int TheEnd = payloadLen - 1;
    if(TheEnd < 0){
      return;
    }
    digitalWrite(PTT, HIGH);          // PTT ON
    delay(PTTlead);                   // PTT lead delay
    // ch = ' '; Serial.print(ch); chTable(); sendFsk();   // Space before sending
    // while (Serial.available()) {
    Serial.print("FSK ");
    for (int i = 0; i < TheEnd+1; i++) {
        ch = toUpperCase(static_cast<char>(CwMsg[i]));
        Serial.print(String(ch));
        Serial.print("|");
        chTable();
        if(fig1 == 0 && fig2 == 1){
                d1 = 1; d2 = 1; d3 = 0; d4 = 1; d5 = 1; //FIGURES
                sendFsk();
        }else if(fig1 == 1 && fig2 == 0){
                d1 = 1; d2 = 1; d3 = 1; d4 = 1; d5 = 1; //LETTERS
                sendFsk();
        }else if(space == 1 && fig2 == 1){
                d1 = 1; d2 = 1; d3 = 0; d4 = 1; d5 = 1; //FIGURES
                sendFsk();
        }
        if(fig2 == 0 || fig2 == 1){
                space = 0;
                fig1 = fig2;
        }
        chTable();
        sendFsk();
        delay(5);
    }
    Serial.println();
    // ch = ' '; Serial.print(ch); chTable(); sendFsk();   // Space after sending
    delay(PTTtail);
    digitalWrite(PTT, LOW);Serial.println();
    digitalWrite(FSK_OUT, LOW);
    if(APmode==true){
      ledcWrite(pwmChannel, 255);
    }else{
      digitalWrite(StatusPin, HIGH);
    }
    powerTimer=millis();
  }

//   uint8_t req[] = {START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS, CMD_SEND_CW_MSG, requestCode, STOP_BYTE};
// #ifdef DEBUG
//   Serial.print(">");
// #endif
//   for (uint8_t i = 0; i < sizeof(req); i++) {
//     CAT.write(req[i]);
//     #ifdef DEBUG
//       if (req[i] < 16)Serial.print("0");
//       Serial.print(req[i], HEX);
//       Serial.print(" ");
//     #endif
//     }
// #ifdef DEBUG
//   Serial.println();
// #endif
}

//-------------------------------------------------------------------------------------------------------
void sendFsk(){
  #if defined(UDP_TO_FSK)
        #if defined(serialECHO )
              Serial.print(d1);Serial.print(d2);Serial.print(d3);Serial.print(d4);Serial.print(d5);Serial.print(' '); // 5bit code serial echo
        //      Serial.print(OneBit);Serial.print('|');Serial.print(OneBit*StopBit);Serial.print(' ');                  // ms
        #endif
        //--start bit
        digitalWrite(FSK_OUT, FSK_SPACE_LEVEL); delay(OneBit);
        //--bit1
        if(d1 == 1){digitalWrite(FSK_OUT, FSK_MARK_LEVEL); }
        else       {digitalWrite(FSK_OUT, FSK_SPACE_LEVEL); } delay(OneBit);
        //--bit2
        if(d2 == 1){digitalWrite(FSK_OUT, FSK_MARK_LEVEL); }
        else       {digitalWrite(FSK_OUT, FSK_SPACE_LEVEL); } delay(OneBit);
        //--bit3
        if(d3 == 1){digitalWrite(FSK_OUT, FSK_MARK_LEVEL); }
        else       {digitalWrite(FSK_OUT, FSK_SPACE_LEVEL); } delay(OneBit);
        //--bit4
        if(d4 == 1){digitalWrite(FSK_OUT, FSK_MARK_LEVEL); }
        else       {digitalWrite(FSK_OUT, FSK_SPACE_LEVEL); } delay(OneBit);
        //--bit5
        if(d5 == 1){digitalWrite(FSK_OUT, FSK_MARK_LEVEL); }
        else       {digitalWrite(FSK_OUT, FSK_SPACE_LEVEL); } delay(OneBit);
        //--stop bit
        digitalWrite(FSK_OUT, FSK_MARK_LEVEL); delay(OneBit*StopBit);
  #endif
}

void chTable(){
  #if defined(UDP_TO_FSK)

        fig2 = -1;
        if(ch == ' ')
        {
                d1 = 0; d2 = 0; d3 = 1; d4 = 0; d5 = 0;
                space = 1;
        }
        else if(ch == 'A'){d1 = 1; d2 = 1; d3 = 0; d4 = 0; d5 = 0; fig2 = 0;}
        else if(ch == 'B'){d1 = 1; d2 = 0; d3 = 0; d4 = 1; d5 = 1; fig2 = 0;}
        else if(ch == 'C'){d1 = 0; d2 = 1; d3 = 1; d4 = 1; d5 = 0; fig2 = 0;}
        else if(ch == 'D'){d1 = 1; d2 = 0; d3 = 0; d4 = 1; d5 = 0; fig2 = 0;}
        else if(ch == 'E'){d1 = 1; d2 = 0; d3 = 0; d4 = 0; d5 = 0; fig2 = 0;}
        else if(ch == 'F'){d1 = 1; d2 = 0; d3 = 1; d4 = 1; d5 = 0; fig2 = 0;}
        else if(ch == 'G'){d1 = 0; d2 = 1; d3 = 0; d4 = 1; d5 = 1; fig2 = 0;}
        else if(ch == 'H'){d1 = 0; d2 = 0; d3 = 1; d4 = 0; d5 = 1; fig2 = 0;}
        else if(ch == 'I'){d1 = 0; d2 = 1; d3 = 1; d4 = 0; d5 = 0; fig2 = 0;}
        else if(ch == 'J'){d1 = 1; d2 = 1; d3 = 0; d4 = 1; d5 = 0; fig2 = 0;}
        else if(ch == 'K'){d1 = 1; d2 = 1; d3 = 1; d4 = 1; d5 = 0; fig2 = 0;}
        else if(ch == 'L'){d1 = 0; d2 = 1; d3 = 0; d4 = 0; d5 = 1; fig2 = 0;}
        else if(ch == 'M'){d1 = 0; d2 = 0; d3 = 1; d4 = 1; d5 = 1; fig2 = 0;}
        else if(ch == 'N'){d1 = 0; d2 = 0; d3 = 1; d4 = 1; d5 = 0; fig2 = 0;}
        else if(ch == 'O'){d1 = 0; d2 = 0; d3 = 0; d4 = 1; d5 = 1; fig2 = 0;}
        else if(ch == 'P'){d1 = 0; d2 = 1; d3 = 1; d4 = 0; d5 = 1; fig2 = 0;}
        else if(ch == 'Q'){d1 = 1; d2 = 1; d3 = 1; d4 = 0; d5 = 1; fig2 = 0;}
        else if(ch == 'R'){d1 = 0; d2 = 1; d3 = 0; d4 = 1; d5 = 0; fig2 = 0;}
        else if(ch == 'S'){d1 = 1; d2 = 0; d3 = 1; d4 = 0; d5 = 0; fig2 = 0;}
        else if(ch == 'T'){d1 = 0; d2 = 0; d3 = 0; d4 = 0; d5 = 1; fig2 = 0;}
        else if(ch == 'U'){d1 = 1; d2 = 1; d3 = 1; d4 = 0; d5 = 0; fig2 = 0;}
        else if(ch == 'V'){d1 = 0; d2 = 1; d3 = 1; d4 = 1; d5 = 1; fig2 = 0;}
        else if(ch == 'W'){d1 = 1; d2 = 1; d3 = 0; d4 = 0; d5 = 1; fig2 = 0;}
        else if(ch == 'X'){d1 = 1; d2 = 0; d3 = 1; d4 = 1; d5 = 1; fig2 = 0;}
        else if(ch == 'Y'){d1 = 1; d2 = 0; d3 = 1; d4 = 0; d5 = 1; fig2 = 0;}
        else if(ch == 'Z'){d1 = 1; d2 = 0; d3 = 0; d4 = 0; d5 = 1; fig2 = 0;}
        else if(ch == '0'){d1 = 0; d2 = 1; d3 = 1; d4 = 0; d5 = 1; fig2 = 1;}
        else if(ch == '1'){d1 = 1; d2 = 1; d3 = 1; d4 = 0; d5 = 1; fig2 = 1;}
        else if(ch == '2'){d1 = 1; d2 = 1; d3 = 0; d4 = 0; d5 = 1; fig2 = 1;}
        else if(ch == '3'){d1 = 1; d2 = 0; d3 = 0; d4 = 0; d5 = 0; fig2 = 1;}
        else if(ch == '4'){d1 = 0; d2 = 1; d3 = 0; d4 = 1; d5 = 0; fig2 = 1;}
        else if(ch == '5'){d1 = 0; d2 = 0; d3 = 0; d4 = 0; d5 = 1; fig2 = 1;}
        else if(ch == '6'){d1 = 1; d2 = 0; d3 = 1; d4 = 0; d5 = 1; fig2 = 1;}
        else if(ch == '7'){d1 = 1; d2 = 1; d3 = 1; d4 = 0; d5 = 0; fig2 = 1;}
        else if(ch == '8'){d1 = 0; d2 = 1; d3 = 1; d4 = 0; d5 = 0; fig2 = 1;}
        else if(ch == '9'){d1 = 0; d2 = 0; d3 = 0; d4 = 1; d5 = 1; fig2 = 1;}
        else if(ch == '-'){d1 = 1; d2 = 1; d3 = 0; d4 = 0; d5 = 0; fig2 = 1;}
        else if(ch == '?'){d1 = 1; d2 = 0; d3 = 0; d4 = 1; d5 = 1; fig2 = 1;}
        else if(ch == ':'){d1 = 0; d2 = 1; d3 = 1; d4 = 1; d5 = 0; fig2 = 1;}
        else if(ch == '('){d1 = 1; d2 = 1; d3 = 1; d4 = 1; d5 = 0; fig2 = 1;}
        else if(ch == ')'){d1 = 0; d2 = 1; d3 = 0; d4 = 0; d5 = 1; fig2 = 1;}
        else if(ch == '.'){d1 = 0; d2 = 0; d3 = 1; d4 = 1; d5 = 1; fig2 = 1;}
        else if(ch == ','){d1 = 0; d2 = 0; d3 = 1; d4 = 1; d5 = 0; fig2 = 1;}
        else if(ch == '/'){d1 = 1; d2 = 0; d3 = 1; d4 = 1; d5 = 1; fig2 = 1;}
        else if(ch == '+'){d1 = 1; d2 = 0; d3 = 0; d4 = 0; d5 = 1; fig2 = 1;} //ITA2
        else if(ch == '\n'){d1 = 0; d2 = 1; d3 = 0; d4 = 0; d5 = 0;} //LF
        else if(ch == '\r'){d1 = 0; d2 = 0; d3 = 0; d4 = 1; d5 = 0;} //CR
        else
        {
                ch = ' ';
                d1 = 0; d2 = 0; d3 = 1; d4 = 0; d5 = 0;
                space = 1;
        }
  #endif
}

//-------------------------------------------------------------------------------------------------------
void CLI(){
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    // ?
    if(incomingByte==63){
      ListCommands();

    // D
    }else if(incomingByte==68){
      if(Debug==false){
        Debug=true;
        Serial.println("   Debug ENABLED");
      }else{
        Debug=false;
        Serial.println("   Debug DISABLED");
      }

    // A
    }else if(incomingByte==65 || incomingByte==97){
      Serial.println("   Switch to AP mode and restart? (y/n)");
      EnterChar();
      if(incomingByte==89 || incomingByte==121){
        EEPROM.writeBool(0, true);
        EEPROM.commit();
        Serial.println("** Interface will be restarted to AP mode **");
        delay(3000);
        ESP.restart();
      }else{
        Serial.println("   AP mode switch aborted");
      }
 
    // E
    }else if(incomingByte==69){
      Serial.println("   Erase whole eeprom? (y/n)");
      EnterChar();
      if(incomingByte==89 || incomingByte==121){
        Serial.println("   Stop erase? (y/n)");
        EnterChar();
        if(incomingByte==78 || incomingByte==110){
          for(int i=0; i<EEPROM_SIZE; i++){
            EEPROM.write(i, 0xff);
            Serial.print(".");
          }
          EEPROM.commit();
          Serial.println("");
          Serial.println("   Eeprom erased done");
          Serial.println("** Interface will be restarted **");
          delay(3000);
          ESP.restart();
        }else{
          Serial.println("   Erase aborted");
        }
      }else{
        Serial.println("   Erase aborted");
      }

    // @
    }else if(incomingByte==64){
      Serial.println("   Restart Interface? (y/n)");
      EnterChar();
      if(incomingByte==89 || incomingByte==121){
        Serial.println("   Stop Restart? (y/n)");
        EnterChar();
        if(incomingByte==78 || incomingByte==110){
          Serial.println("** Interface will be restarted **");
          delay(3000);
          ESP.restart();
        }else{
          Serial.println("   Restart aborted");
        }
      }else{
        Serial.println("   Restart aborted");
      }

    // CR/LF
    }else if(incomingByte==13||incomingByte==10){
      // anykey
    }else{
      // Serial.print(" [");
      // Serial.print( String(incomingByte) ); //, DEC);
      // Serial.println("] unknown command");
    }
    incomingByte=0;
  }
}

//-------------------------------------------------------------------------------------------------------
void EnterChar(){
  incomingByte = 0;
  static byte byteBuf = 0x00;
  Serial.print(" > ");
  while (Serial.available() == 0) {
    // Wait
  }
  // byteBuf = Serial.read();
  // // CR/LF
  // if(byteBuf==13||byteBuf==10){
  //   //nil
  // }else{
    incomingByte = Serial.read();
    Serial.println( String(char(incomingByte)) );
  // }
}

//-------------------------------------------------------------------------------------------------------
void ListCommands(){
  Serial.println("");
  Serial.println("-------- DivaDroid International | IC-705 IP interface status  --------");
  Serial.print("  Uptime: ");
  if(millis() < 60000){
    Serial.print( String(millis()/1000) );
    Serial.println(" second");
  }else if(millis() > 60000 && millis() < 3600000){
    Serial.print( String(millis()/60000) );
    Serial.println(" minutes");
  }else if(millis() > 3600000 && millis() < 86400000){
    Serial.print( String(millis()/3600000) );
    Serial.println(" hours");
  }else{
    Serial.print( String(millis()/86400000) );
    Serial.println(" days");
  }
  Serial.println("  FW "+String(REV)+" | HW "+String(HardwareRev)+" ["+String(HWidValue)+" raw]" );
  if(APmode==true){
    Serial.println("  WIFI-AP mode ON" );
    APcliAlert();
  }else{
    Serial.println("  Bluetooth: "+String(BTname) );
    Serial.println("  WIFI-AP mode OFF" );
    Serial.println("  WIFI-SSID1 "+SSID );
    if (SSID2.length() > 0) {
      Serial.println("  WIFI-SSID2 "+SSID2 );
    } else {
      Serial.println("  WIFI-SSID2 DISABLE" );
    }
    Serial.println("  WIFI-connected with IP "+String(WiFi.localIP()[0])+"."+String(WiFi.localIP()[1])+"."+String(WiFi.localIP()[2])+"."+String(WiFi.localIP()[3]) );
    Serial.println("  WIFI-MAC "+String(MACString) );
    Serial.println("  WIFI-dBm: "+String(WiFi.RSSI()) );
    Serial.println("----------------------------------------------------------------------------" );
    Serial.println("  For setup OPEN url http://ic705.local or http://"+String(WiFi.localIP()[0])+"."+String(WiFi.localIP()[1])+"."+String(WiFi.localIP()[2])+"."+String(WiFi.localIP()[3]) );
    Serial.println("----------------------------------------------------------------------------" );
    Serial.println("  IP http-cat  http://"+String(WiFi.localIP()[0])+"."+String(WiFi.localIP()[1])+"."+String(WiFi.localIP()[2])+"."+String(WiFi.localIP()[3])+":"+String(HTTP_CAT_PORT) );
    Serial.println("  IP udp cw/rtty port: "+String(udpPort) );
    Serial.println("  IP udp cat port: "+String(udpCatPort) );
    if(mqttEnable==true){
      Serial.println("  IP MqttSubscribe: "+String(mqttBroker[0])+"."+String(mqttBroker[1])+"."+String(mqttBroker[2])+"."+String(mqttBroker[3])+":"+String(MQTT_PORT)+"/");
      Serial.println("  IP MqttTopic TX: "+String(MQTT_TOPIC));
      if (MQTT_TOPIC_RX.length() > 0) {
        Serial.println("  IP MqttTopic RX: "+String(MQTT_TOPIC_RX));
      } else {
        Serial.println("  IP MqttTopic RX: DISABLE");
      }
    }else{
      Serial.println("  IP mqtt DISABLE" );
    }
    #if defined(RESET_AFTER_DISCONNECT)
      Serial.println("     RESET after TRX disconnect - ENABLE" );
    #endif

    Serial.println(" CAT "+String(frequency)+"Hz "+String(modes) );
  }
  Serial.println("Commands  press key to select");
  Serial.println("       ?  list refresh");
  Serial.println("       A  restart to AP mode");
  if(Debug==false){
    Serial.println("       D  enable serial debug");
  }else{
    Serial.println("       D  disable serial debug");
  }
  Serial.println("       E  erase whole eeprom and restart");
  Serial.println("       @  restart device");
  Serial.print( " > " );
}

//-------------------------------------------------------------------------------------------------------
void APcliAlert(){
  IPAddress IP = WiFi.softAPIP();
  Serial.println();
  Serial.println("---------------------------------------------------------------");
  Serial.println("  PLEASE connect your PC to '"+String(ssidAP)+"' WiFi access-point");
  Serial.println("  with password 'remoteqth'");
  Serial.println("  and OPEN url http://ic705.local or http://"+String(IP[0])+"."+String(IP[1])+"."+String(IP[2])+"."+String(IP[3]) );
  Serial.println("  AP mode can be entered again only from the serial console");
  Serial.println("---------------------------------------------------------------");
  Serial.println();
}

//-------------------------------------------------------------------------------------------------------

// ajax rx
void handleSet(AsyncWebServerRequest *request) {
  resetSetupMessages();
  bool ERRdetect=0;

  if ( requestHasArg(request, "ssid") == false \
    && requestHasArg(request, "pswd") == false \
  ) {
    // Serial.println("Form NOT valid");
  }else{
    // Serial.println("Form VALID");

    // 1-20 - SSID1*
    if ( requestArg(request, "ssid").length()<1 || requestArg(request, "ssid").length()>20){
      setupSsidErr = "Out of range 1-20 characters";
      ERRdetect=1;
    }else{
      String str = requestArg(request, "ssid");
      if(SSID == str){
        setupSsidErr = "";
      }else{
        setupSsidErr = "Warning: SSID has changed.";
        SSID = requestArg(request, "ssid");

        int str_len = str.length();
        if(str_len > 20){
          str_len = 20;
        }
        char char_array[str_len + 1];
        str.toCharArray(char_array, str_len+1);
        for (int i=0; i<20; i++){
          if(i < str_len){
            EEPROM.write(1+i, char_array[i]);
          }else{
            EEPROM.write(1+i, 0xff);
          }
        }
        // EEPROM.commit();
      }
    }

    // 22-39 - PSWD1*
    if ( requestArg(request, "pswd").length()<1 || requestArg(request, "pswd").length()>18){
      setupPswdErr = "Out of range 1-18 characters";
      ERRdetect=1;
    }else{
      String str = requestArg(request, "pswd");
      if(PSWD == str){
        setupPswdErr = "";
      }else{
        setupPswdErr = "Warning: Password has changed.";
        PSWD = requestArg(request, "pswd");

        int str_len = str.length();
        if(str_len > 18){
          str_len = 18;
        }
        char char_array[str_len + 1];
        str.toCharArray(char_array, str_len+1);
        for (int i=0; i<18; i++){
          if(i < str_len){
            EEPROM.write(22+i, char_array[i]);
          }else{
            EEPROM.write(22+i, 0xff);
          }
        }
        // EEPROM.commit();
      }
    }

    // 76-95 - SSID2
    if ( requestArg(request, "ssid2").length()>20){
      setupSsid2Err = "Out of range 0-20 characters";
      ERRdetect=1;
    }else{
      String str = requestArg(request, "ssid2");
      if(SSID2 == str){
        setupSsid2Err = "";
      }else{
        setupSsid2Err = "Warning: SSID 2 has changed.";
        SSID2 = str;

        int str_len = str.length();
        if(str_len > 20){
          str_len = 20;
        }
        char char_array[str_len + 1];
        str.toCharArray(char_array, str_len+1);
        for (int i=0; i<20; i++){
          if(i < str_len){
            EEPROM.write(76+i, char_array[i]);
          }else{
            EEPROM.write(76+i, 0xff);
          }
        }
      }
    }

    // 97-114 - PSWD2
    if ( requestArg(request, "pswd2").length()>18){
      setupPswd2Err = "Out of range 0-18 characters";
      ERRdetect=1;
    }else{
      String str = requestArg(request, "pswd2");
      if(PSWD2 == str){
        setupPswd2Err = "";
      }else{
        setupPswd2Err = "Warning: Password 2 has changed.";
        PSWD2 = str;

        int str_len = str.length();
        if(str_len > 18){
          str_len = 18;
        }
        char char_array[str_len + 1];
        str.toCharArray(char_array, str_len+1);
        for (int i=0; i<18; i++){
          if(i < str_len){
            EEPROM.write(97+i, char_array[i]);
          }else{
            EEPROM.write(97+i, 0xff);
          }
        }
      }
    }

    if ((SSID2.length() == 0 && PSWD2.length() > 0) || (SSID2.length() > 0 && PSWD2.length() == 0)) {
      setupSsid2Err = "SSID2 and Password2 must be both filled or both empty";
      setupPswd2Err = "";
      ERRdetect = 1;
    }

    // 68-69 HTTP_CAT_PORT
    if ( requestArg(request, "httpcatport").length()<1 || requestArg(request, "httpcatport").toInt()<1 || requestArg(request, "httpcatport").toInt()>65534){
      setupHttpCatPortErr = "Out of range number 1-65534";
      ERRdetect=1;
    }else{
      if(HTTP_CAT_PORT == requestArg(request, "httpcatport").toInt()){
        setupHttpCatPortErr = "";
      }else{
        setupHttpCatPortErr = "";
        HTTP_CAT_PORT = requestArg(request, "httpcatport").toInt();
        EEPROM.writeUShort(68, HTTP_CAT_PORT);
      }
    }

    // 70-71 udpPort
    if ( requestArg(request, "udpport").length()<1 || requestArg(request, "udpport").toInt()<1 || requestArg(request, "udpport").toInt()>65534){
      setupUdpPortErr = "Out of range number 1-65534";
      ERRdetect=1;
    }else{
      if(udpPort == requestArg(request, "udpport").toInt()){
        setupUdpPortErr = "";
      }else{
        setupUdpPortErr = "";
        udpPort = requestArg(request, "udpport").toInt();
        EEPROM.writeUShort(70, udpPort);
      }
    }

    // 72-73 udpCatPort
    if ( requestArg(request, "udpcatport").length()<1 || requestArg(request, "udpcatport").toInt()<1 || requestArg(request, "udpcatport").toInt()>65534){
      setupUdpCatPortErr = "Out of range number 1-65534";
      ERRdetect=1;
    }else{
      if(udpCatPort == requestArg(request, "udpcatport").toInt()){
        setupUdpCatPortErr = "";
      }else{
        setupUdpCatPortErr = "";
        udpCatPort = requestArg(request, "udpcatport").toInt();
        EEPROM.writeUShort(72, udpCatPort);
      }
    }

    // 41-44 mqttBroker[4]
    if ( requestArg(request, "mqttip0").length()<1 || requestArg(request, "mqttip0").toInt()>255){
      setupMqttErr1 = "IP1: Out of range 0-255";
      ERRdetect=1;
    }else{
      if(mqttBroker[0] == byte(requestArg(request, "mqttip0").toInt()) ){
        setupMqttErr1 = "";
      }else{
        setupMqttErr1 = "";
        mqttBroker[0] = byte(requestArg(request, "mqttip0").toInt()) ;
        EEPROM.writeByte(41, mqttBroker[0]);
      }
      if(mqttBroker[0]==0x00){
        mqttEnable = false;
      }else{
        mqttEnable = true;
      }
    }

    if ( requestArg(request, "mqttip1").length()<1 || requestArg(request, "mqttip1").toInt()>255){
      setupMqttErr2 = "IP2: Out of range 0-255";
      ERRdetect=1;
    }else{
      if(mqttBroker[1] == byte(requestArg(request, "mqttip1").toInt()) ){
        setupMqttErr2 = "";
      }else{
        setupMqttErr2 = "";
        mqttBroker[1] = byte(requestArg(request, "mqttip1").toInt()) ;
        EEPROM.writeByte(42, mqttBroker[1]);
      }
    }

    if ( requestArg(request, "mqttip2").length()<1 || requestArg(request, "mqttip2").toInt()>255){
      setupMqttErr3 = "IP3: Out of range 0-255";
      ERRdetect=1;
    }else{
      if(mqttBroker[2] == byte(requestArg(request, "mqttip2").toInt()) ){
        setupMqttErr3 = "";
      }else{
        setupMqttErr3 = "";
        mqttBroker[2] = byte(requestArg(request, "mqttip2").toInt()) ;
        EEPROM.writeByte(43, mqttBroker[2]);
      }
    }

    if ( requestArg(request, "mqttip3").length()<1 || requestArg(request, "mqttip3").toInt()>255){
      setupMqttErr4 = "IP4: Out of range 0-255";
      ERRdetect=1;
    }else{
      if(mqttBroker[3] == byte(requestArg(request, "mqttip3").toInt()) ){
        setupMqttErr4 = "";
      }else{
        setupMqttErr4 = "";
        mqttBroker[3] = byte(requestArg(request, "mqttip3").toInt()) ;
        EEPROM.writeByte(44, mqttBroker[3]);
      }
    }

    // 45-46 MQTT_PORT
    if ( requestArg(request, "mqttport").length()<1 || requestArg(request, "mqttport").toInt()<1 || requestArg(request, "mqttport").toInt()>65534){
      setupMqttPortErr = "Out of range number 1-65534";
      ERRdetect=1;
    }else{
      if(MQTT_PORT == requestArg(request, "mqttport").toInt()){
        setupMqttPortErr = "";
      }else{
        setupMqttPortErr = "";
        MQTT_PORT = requestArg(request, "mqttport").toInt();
        EEPROM.writeUShort(45, MQTT_PORT);
      }
    }

    // 47-67 MQTT_TOPIC
    if ( requestArg(request, "mqtttopic").length()<1 || requestArg(request, "mqtttopic").length()>20){
      setupMqttTopicErr = "Out of range 1-20 characters";
      ERRdetect=1;
    }else{
      String str = requestArg(request, "mqtttopic");
      if(MQTT_TOPIC == str){
        setupMqttTopicErr = "";
      }else{
        setupMqttTopicErr = "";
        MQTT_TOPIC = requestArg(request, "mqtttopic");

        int str_len = str.length();
        if(str_len > 20){
          str_len = 20;
        }
        char char_array[str_len + 1];
        str.toCharArray(char_array, str_len+1);
        for (int i=0; i<20; i++){
          if(i < str_len){
            EEPROM.write(47+i, char_array[i]);
          }else{
            EEPROM.write(47+i, 0xff);
          }
        }
        // EEPROM.commit();
      }
    }

    // 115-135 MQTT_TOPIC_RX
    if ( requestArg(request, "mqtttopicrx").length()>20){
      setupMqttTopicRxErr = "Out of range 0-20 characters";
      ERRdetect=1;
    }else{
      String str = requestArg(request, "mqtttopicrx");
      if(MQTT_TOPIC_RX == str){
        setupMqttTopicRxErr = "";
      }else{
        setupMqttTopicRxErr = "";
        MQTT_TOPIC_RX = str;

        int str_len = str.length();
        if(str_len > 20){
          str_len = 20;
        }
        char char_array[str_len + 1];
        str.toCharArray(char_array, str_len+1);
        for (int i=0; i<21; i++){
          if(i < str_len){
            EEPROM.write(115+i, char_array[i]);
          }else{
            EEPROM.write(115+i, 0xff);
          }
        }
      }
    }

    // 74-75 BaudRate *
    static int BaudRateTmp=115200;
    switch (requestArg(request, "baud").toInt()) {
      case 0: {BaudRateTmp= 1200; break; }
      case 1: {BaudRateTmp= 2400; break; }
      case 2: {BaudRateTmp= 4800; break; }
      case 3: {BaudRateTmp= 9600; break; }
      case 4: {BaudRateTmp= 115200; break; }
    }
    if(BaudRateTmp!=BaudRate){
      BaudRate=(int)BaudRateTmp;
      EEPROM.writeUShort(74, BaudRate);
      // MqttPubString("USB-BaudRate", String(BaudRate), true);
      Serial.println("Baudrate change to "+String(BaudRate)+"...");
      Serial.flush();
      // Serial.end();
      delay(1000);
      Serial.begin(BaudRate);
      delay(500);
      Serial.println();
      Serial.println();
      Serial.println("New Baudrate "+String(BaudRate));
    }


    if(ERRdetect==0){
      // // APmode
      EEPROM.writeBool(0, false);
      EEPROM.commit();
      Serial.println("Interface will be restarted...");
      delay(3000);
      ESP.restart();
    }
    // Serial.println("ERRdetect = "+String(ERRdetect) );

  } // else form valid

  renderSetupPage(request);
}

#if defined(RTLE)
  void handleRTLE() {
    String HtmlSrc = "";
    String s = RTLE_page; //Read HTML contents
    HtmlSrc +=s;
    rtleserver.send(200, "text/html", HtmlSrc); //Send web page
  }
#endif
