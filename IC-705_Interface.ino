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

1. Increase REV value in this .ino
2. Arduino IDE 1.8.19 menu: Sketch/Export compiled Binary (for "ESP32 Dev Module" + Tools/Partition Scheme:"No OTA (2MB APP/2MB SPIFFS)")
3. generate all .bin and publish to GitHub web page: $ ./tools/all.sh --publish
4. git commit with comment Release number and push


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
bool cwIpOnConnect  = false;      // announce WiFi IP via CW on first BT connect
volatile bool cwIpSendPending = false;

#define REV 20260509
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
#define EEPROM_SIZE 302
// EEPROM mapa TRX1/2/3 (per-slot 55B):
//   TRX1 base=137, TRX2 base=192, TRX3 base=247
//   +0   mode: 0=disabled,1=IC-705-BT,2=IC-705-LAN,3=IC-7610-LAN,4=IC-7300-LAN,5=OI3
//   +1   ip[4]
//   +5   port LE (LAN: controlPort 50001 / OI3: httpPort 81)
//   +7   username[16]
//   +23  password[16]
//   +39  civAddr
//   +40  label[11]
//   +51  udpCwPort LE  (BT: local incoming / OI3: remote port)
//   +53  udpCatPort LE (BT: local incoming / OI3: remote port)

#define TRX_MODE_DISABLED   0
#define TRX_MODE_BT         1
#define TRX_MODE_IC705_LAN  2
#define TRX_MODE_IC7610_LAN 3
#define TRX_MODE_IC7300_LAN 4
#define TRX_MODE_OI3        5
#define TRX_COUNT           3

struct TrxConfig {
  uint8_t  mode       = TRX_MODE_DISABLED;
  uint8_t  ip[4]      = {0,0,0,0};
  uint16_t port       = 50001;
  char     username[17] = {};
  char     password[17] = {};
  uint8_t  civAddr    = 0xA4;
  char     label[12]  = {};
  uint16_t udpCwPort  = 89;
  uint16_t udpCatPort = 90;

  bool enabled() const { return mode != TRX_MODE_DISABLED; }
  bool isBt()    const { return mode == TRX_MODE_BT; }
  bool isLan()   const { return mode >= TRX_MODE_IC705_LAN && mode <= TRX_MODE_IC7300_LAN; }
  bool isOi3()   const { return mode == TRX_MODE_OI3; }
  uint8_t effectiveCivAddr() const {
    if (mode == TRX_MODE_IC705_LAN)  return 0xA4;
    if (mode == TRX_MODE_IC7610_LAN) return 0x98;
    if (mode == TRX_MODE_IC7300_LAN) return 0x94;
    return civAddr;
  }
  const char* modeStr() const {
    switch (mode) {
      case TRX_MODE_BT:         return "IC-705-BT";
      case TRX_MODE_IC705_LAN:  return "IC-705-LAN";
      case TRX_MODE_IC7610_LAN: return "IC-7610-LAN";
      case TRX_MODE_IC7300_LAN: return "IC-7300-LAN";
      case TRX_MODE_OI3:        return "OI3";
      default:                  return "";
    }
  }
  IPAddress ipAddr() const { return IPAddress(ip[0],ip[1],ip[2],ip[3]); }
};

struct TrxState {
  uint8_t  radioAddress = 0;
  uint32_t frequency    = 0;
  char     modes[12]    = "OFF";
  bool     connected    = false;
  bool     tx           = false;
  uint8_t  filter       = 1;
  uint32_t smeterRaw    = 0;
  uint32_t powerMeterRaw = 0;
  float    supplyVolts  = 0.0f;
  float    swr          = 1.0f;
  uint8_t  afGain       = 0;
  uint8_t  keySpeed     = 138;
  uint8_t  rfPower      = 205;
  uint8_t  preampMode   = 0;
  uint8_t  attOn        = 0;
  uint8_t  voxMode      = 0;
  uint32_t ritRaw       = 0;
  long     powerTimer   = 0;
  bool     trxSetupDone = false;
  bool     trxNeedSet   = false;
};

static const uint16_t TRX_EEPROM_BASE[TRX_COUNT] = {137, 192, 247};
static const uint8_t  TRX_EEPROM_SLOT = 55; // bytes per TRX slot
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
  uint32_t readtimeout = 2000;
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

  char modes[12] = "OFF";
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
volatile bool btStateBroadcastPending = false;
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
  #include <WebServer.h>
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
  #include <HTTPClient.h>
  #include <ESPAsyncWebServer.h>
  #include <AsyncWebSocket.h>

  const char* ssidAP     = "IC705-if";
  const char* passwordAP = "remoteqth";
  bool APmode = false;
  WebServer webServer(80);
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
typedef mbedtls_sha1_context SHA1_CTX;

extern "C" void SHA1Init(SHA1_CTX* context){
  mbedtls_sha1_init(context);
  mbedtls_sha1_starts_ret(context);
}

extern "C" void SHA1Update(SHA1_CTX* context, const unsigned char* data, uint32_t len){
  mbedtls_sha1_update_ret(context, data, len);
}

extern "C" void SHA1Final(unsigned char digest[20], SHA1_CTX* context){
  mbedtls_sha1_finish_ret(context, digest);
  mbedtls_sha1_free(context);
}
#endif

#if defined(WIFI)
  const uint8_t CW_MEMORY_COUNT = 4;
  const uint8_t FREQ_MEMORY_COUNT = 10;
  const size_t CW_MEMORY_MAX_LEN = 30;
  const size_t FREQ_MEMORY_MAX_LEN = 20;
  const uint8_t CIV_ADDRESS_DEFAULT = 0xA4;
  const char *MEMORY_CONFIG_PATH = "/memories.cfg";
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
  String setupCivAddrErr = "";

  // Per-TRX konfigurace a stav
  #include "icomLanClient.h"
  TrxConfig trxCfg[TRX_COUNT];
  TrxState  trxSt[TRX_COUNT];
  IcomLanClient lanClients[TRX_COUNT];
  int activeTrxIdx = 0;            // TRX zobrazovaný na CAT stránce

  // Pomocné funkce
  int btTrxIdx() {
    for (int i = 0; i < TRX_COUNT; i++) if (trxCfg[i].isBt()) return i;
    return -1;
  }
  // Zpětná kompatibilita — aliasy na TRX1 / aktivní TRX
  String transceiverType = "";     // nastavuje se z trxCfg[0].modeStr() po načtení
  uint8_t configuredCivAddress = CIV_ADDRESS_DEFAULT; // alias na trxCfg[0].effectiveCivAddr()
  bool    lanMode = false;         // true pokud aspoň jeden TRX je LAN
  // Legacy BT vars (přetrvávají pro kompatibilitu s printFrequency, printMode atd.)
  // radio_address, frequency, modes atd. jsou stále globální a odpovídají trxSt[activeTrxIdx]

  // WebSocket server pro vodopad (port 82)
  AsyncWebServer wfWsServer(82);
  AsyncWebSocket  wfWs("/ws");

  // Vodopad — stav assembleru
  struct WfFrame {
    uint32_t startHz;
    uint32_t endHz;
    uint8_t  mode;
    bool     oor;
    uint8_t  pixels[500];
    uint16_t pixelCount;
    bool     complete;
  };
  WfFrame wfBuf;
  bool scopeInitDone = false;  // příznak: scope ON byl odeslán
  String cwMemoryText[CW_MEMORY_COUNT];
  String freqMemoryText[FREQ_MEMORY_COUNT];
  portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

  // State variables updated from CIV frames, served via /state polling
  uint32_t stateRitRaw = 0;
  uint8_t stateFilter = 1;
  uint8_t stateAfGain = 0;
  uint8_t stateKeySpeed = 138;
  uint8_t stateRfPower = 205;
  bool stateTx = false;
  uint8_t statePreampMode = 0;  // 0=OFF, 1=AMP, 2=ATT
  uint8_t stateAttOn = 0;       // internal: tracks ATT separately for combine logic
  uint8_t stateVoxMode = 0;
  float stateSupplyVolts = 0.0f;
  float stateSwr = 1.0f;
  uint32_t stateSmeterRaw = 0;
  uint32_t statePowerMeterRaw = 0;

  String requestArg(const char *name);
  bool requestHasArg(const char *name);
  String trimMemoryValue(const String &value, size_t maxLen);
  bool parseHexByteString(const String &value, uint8_t &outValue);
  void loadMemoryConfig(void);
  void saveMemoryConfig(void);
  void processCivFrame(int trxIdx, const uint8_t* frame, size_t len);
  void civProcessScopePacket(int trxIdx, const uint8_t* d, size_t lenD);
  void sendWaterfallToWebSocket(const WfFrame& f);
  void sendScopeActivation(int trxIdx);
  String jsonEscape(const String &value);
  String civFrameToHex(const uint8_t *frame, size_t frameLen);
  uint32_t decodeCivFrequencyBytes(const uint8_t *bytes, size_t byteCount);
  uint32_t decodeCivBcdBytes(const uint8_t *bytes, size_t byteCount);
  String decodeModeName(uint8_t modeId);
  bool catWriteFrame(const uint8_t *frame, size_t frameLen, bool broadcastTx);
  void setModesText(const char *value);
  void copyModesText(char *dest, size_t destSize);
  String extractJsonString(const String &json, const char *key);
  bool extractJsonBool(const String &json, const char *key, bool defaultValue);
  bool parseHexPayload(const String &hex, uint8_t *buffer, size_t &bufferLen, size_t maxLen);
  bool parseModeId(const String &modeName, uint8_t &modeId);
  uint8_t parseFilterWidth(const String &filterName);
  size_t buildSimpleCatFrame(uint8_t command, const uint8_t *payload, size_t payloadLen, uint8_t *frame, size_t frameMaxLen);
  size_t buildSetFrequencyFrame(uint32_t freqHz, uint8_t *frame, size_t frameMaxLen);
  size_t buildSetModeFrame(uint8_t modeId, uint8_t modeWidth, uint8_t *frame, size_t frameMaxLen);
  size_t buildReadQuickSplitFrame(uint8_t *frame, size_t frameMaxLen);
  void buildStateJson(char *buf, size_t bufSize);
  void handleGetState(void);
  void handleTrxSelect(void);
  bool handleFileFromSPIFFS(const String &path);
  void handlePostCmd(void);
  void handleSet(void);
  void renderIndexPage(void);
  void handleSetupData(void);
  void resetSetupMessages(void);
  String setupTemplateProcessor(const String &key);
  void setupWebServer(void);
  void handleConfigDownload(void);
  void handleConfigUpload(void);
  void handleGetLogConfig(void);
  void handlePostLogConfig(void);
  void handleOi3State(void);
  void handleOi3Send(void);
#endif

//-------------------------------------------------------------------------------------------------------

#if defined(WIFI)
String requestArg(const char *name){
  return webServer.arg(name);
}

bool requestHasArg(const char *name){
  return webServer.hasArg(name);
}

String trimMemoryValue(const String &value, size_t maxLen){
  String trimmed = value;
  trimmed.trim();
  if (trimmed.length() > maxLen) {
    trimmed = trimmed.substring(0, maxLen);
    trimmed.trim();
  }
  return trimmed;
}

bool parseHexByteString(const String &value, uint8_t &outValue){
  String normalized = value;
  normalized.trim();
  normalized.toUpperCase();
  if (normalized.startsWith("0X")) {
    normalized = normalized.substring(2);
  }
  if (normalized.length() == 0 || normalized.length() > 2) {
    return false;
  }

  char buffer[3] = {0, 0, 0};
  normalized.toCharArray(buffer, sizeof(buffer));
  char *endPtr = nullptr;
  long parsed = strtol(buffer, &endPtr, 16);
  if (endPtr == nullptr || *endPtr != '\0' || parsed < 0 || parsed > 255) {
    return false;
  }
  outValue = (uint8_t)parsed;
  return true;
}

void loadMemoryConfig(void){
  for (uint8_t i = 0; i < CW_MEMORY_COUNT; i++) {
    cwMemoryText[i] = "";
  }
  for (uint8_t i = 0; i < FREQ_MEMORY_COUNT; i++) {
    freqMemoryText[i] = "";
  }

  if (!SPIFFS.exists(MEMORY_CONFIG_PATH)) {
    return;
  }

  File file = SPIFFS.open(MEMORY_CONFIG_PATH, FILE_READ);
  if (!file) {
    return;
  }

  for (uint8_t i = 0; i < CW_MEMORY_COUNT && file.available(); i++) {
    cwMemoryText[i] = trimMemoryValue(file.readStringUntil('\n'), CW_MEMORY_MAX_LEN);
  }
  for (uint8_t i = 0; i < FREQ_MEMORY_COUNT && file.available(); i++) {
    freqMemoryText[i] = trimMemoryValue(file.readStringUntil('\n'), FREQ_MEMORY_MAX_LEN);
  }

  if (file.available()) {
    String configuredType = trimMemoryValue(file.readStringUntil('\n'), 16);
    if (configuredType == "IC-7610-CI-V"  ||
        configuredType == "IC-705-LAN"    ||
        configuredType == "IC-7610-LAN"   ||
        configuredType == "IC-7300-LAN") {
      transceiverType = configuredType;
    } else {
      transceiverType = "IC-705-BT";
    }
  }

  if (file.available()) {
    uint8_t parsedAddress = CIV_ADDRESS_DEFAULT;
    if (parseHexByteString(file.readStringUntil('\n'), parsedAddress)) {
      configuredCivAddress = parsedAddress;
    }
  }

  file.close();
}

void saveMemoryConfig(void){
  File file = SPIFFS.open(MEMORY_CONFIG_PATH, FILE_WRITE);
  if (!file) {
    return;
  }

  for (uint8_t i = 0; i < CW_MEMORY_COUNT; i++) {
    file.println(trimMemoryValue(cwMemoryText[i], CW_MEMORY_MAX_LEN));
  }
  for (uint8_t i = 0; i < FREQ_MEMORY_COUNT; i++) {
    file.println(trimMemoryValue(freqMemoryText[i], FREQ_MEMORY_MAX_LEN));
  }
  file.println(transceiverType);
  if (configuredCivAddress < 16) {
    file.print("0");
  }
  file.println(String(configuredCivAddress, HEX));

  file.close();
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

// LSB-first variant used for RIT (bytes[0] = least significant pair)
uint32_t decodeCivBcdBytesLsb(const uint8_t *bytes, size_t byteCount){
  uint32_t value = 0;
  uint32_t mult = 1;
  for (size_t i = 0; i < byteCount; i++) {
    value += (((bytes[i] >> 4) * 10) + (bytes[i] & 0x0F)) * mult;
    mult *= 100;
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

void setModesText(const char *value){
  portENTER_CRITICAL(&stateMux);
  snprintf(modes, sizeof(modes), "%s", value != nullptr ? value : "");
  portEXIT_CRITICAL(&stateMux);
}

void copyModesText(char *dest, size_t destSize){
  if (dest == nullptr || destSize == 0) {
    return;
  }

  portENTER_CRITICAL(&stateMux);
  snprintf(dest, destSize, "%s", modes);
  portEXIT_CRITICAL(&stateMux);
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
  setupCivAddrErr = "";
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
  if (key == "MQTT_AP_NOTE") return APmode
    ? "<span class=\"ap-note\">MQTT is not active in AP mode — a WiFi network connection to the broker is required. Settings are saved and will apply after switching to station mode.</span>"
    : "";
  if (key == "MQTT_ERR") return setupMqttErr;
  if (key == "MQTT_ERR1") return setupMqttErr1;
  if (key == "MQTT_ERR2") return setupMqttErr2;
  if (key == "MQTT_ERR3") return setupMqttErr3;
  if (key == "MQTT_ERR4") return setupMqttErr4;
  if (key == "MQTT_PORT_ERR") return setupMqttPortErr;
  if (key == "MQTT_TOPIC_ERR") return setupMqttTopicErr;
  if (key == "MQTT_TOPIC_RX_ERR") return setupMqttTopicRxErr;
  if (key == "HTTP_CAT_PORT_ERR") return setupHttpCatPortErr;
  // Per-TRX template klíče: TRXn_FIELD (n=1..3)
  for (int t = 0; t < TRX_COUNT; t++) {
    String prefix = "TRX" + String(t + 1) + "_";
    if (!key.startsWith(prefix)) continue;
    String field = key.substring(prefix.length());
    const TrxConfig& c = trxCfg[t];
    if (field == "LABEL") return String(c.label);
    if (field == "MODE")  return String(c.modeStr());
    if (field == "CIV") {
      char buf[3]; snprintf(buf, sizeof(buf), "%02X", c.civAddr);
      return String(buf);
    }
    if (field == "IP")   return String(c.ip[0])+"."+String(c.ip[1])+"."+String(c.ip[2])+"."+String(c.ip[3]);
    if (field == "PORT") return String(c.port);
    if (field == "USER") return String(c.username);
    if (field == "PASS") return String(c.password);
    if (field == "UDPCW")  return String(c.udpCwPort);
    if (field == "UDPCAT") return String(c.udpCatPort);
    // Vybraná volba v dropdown
    if (field == "SEL_DIS")  return (c.mode == TRX_MODE_DISABLED)   ? "selected" : "";
    if (field == "SEL_BT")   return (c.mode == TRX_MODE_BT)         ? "selected" : "";
    if (field == "SEL_705L") return (c.mode == TRX_MODE_IC705_LAN)  ? "selected" : "";
    if (field == "SEL_7610") return (c.mode == TRX_MODE_IC7610_LAN) ? "selected" : "";
    if (field == "SEL_7300") return (c.mode == TRX_MODE_IC7300_LAN) ? "selected" : "";
    if (field == "SEL_OI3")  return (c.mode == TRX_MODE_OI3)        ? "selected" : "";
    // Zobrazit sekci LAN/OI3
    if (field == "SHOW_LAN") return c.isLan() ? "block" : "none";
    if (field == "SHOW_OI3") return c.isOi3() ? "block" : "none";
    if (field == "SHOW_BT")  return c.isBt()  ? "block" : "none";
    if (field == "SHOW_IP")  return (c.isLan() || c.isOi3()) ? "block" : "none";
    if (field == "SHOW_UDP") return (c.isBt()  || c.isOi3()) ? "block" : "none";
  }
  if (key == "CW_MEM1") return cwMemoryText[0];
  if (key == "CW_MEM2") return cwMemoryText[1];
  if (key == "CW_MEM3") return cwMemoryText[2];
  if (key == "CW_MEM4") return cwMemoryText[3];
  if (key == "FREQ_MEM1") return freqMemoryText[0];
  if (key == "FREQ_MEM2") return freqMemoryText[1];
  if (key == "FREQ_MEM3") return freqMemoryText[2];
  if (key == "FREQ_MEM4") return freqMemoryText[3];
  if (key == "FREQ_MEM5") return freqMemoryText[4];
  if (key == "FREQ_MEM6") return freqMemoryText[5];
  if (key == "FREQ_MEM7") return freqMemoryText[6];
  if (key == "FREQ_MEM8") return freqMemoryText[7];
  if (key == "FREQ_MEM9") return freqMemoryText[8];
  if (key == "FREQ_MEM10") return freqMemoryText[9];
  if (key == "BAUD1200_SEL") return BaudRate == 1200 ? "selected" : "";
  if (key == "BAUD2400_SEL") return BaudRate == 2400 ? "selected" : "";
  if (key == "BAUD4800_SEL") return BaudRate == 4800 ? "selected" : "";
  if (key == "BAUD9600_SEL") return BaudRate == 9600 ? "selected" : "";
  if (key == "BAUD115200_SEL") return BaudRate == 115200 ? "selected" : "";
  if (key == "CW_IP_CHK") return cwIpOnConnect ? "checked" : "";
  return String();
}



void renderIndexPage(){
  if (!SPIFFS.exists("/index.html")) {
    webServer.send(500, "text/plain", "Missing /index.html in SPIFFS");
    return;
  }
  File file = SPIFFS.open("/index.html", "r");
  if (!file) { webServer.send(500, "text/plain", "File open failed"); return; }
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(200, "text/html", "");
  String out;
  out.reserve(256);
  while (file.available()) {
    yield();
    String line = file.readStringUntil('\n');
    out = "";
    int start = 0;
    while (true) {
      int p1 = line.indexOf('%', start);
      if (p1 < 0) { out += line.substring(start); break; }
      int p2 = line.indexOf('%', p1 + 1);
      if (p2 < 0) { out += line.substring(start); break; }
      out += line.substring(start, p1);
      out += setupTemplateProcessor(line.substring(p1 + 1, p2));
      start = p2 + 1;
    }
    out += '\n';
    webServer.sendContent(out);
  }
  file.close();
  webServer.sendContent("");
}

void buildStateJson(char *buf, size_t bufSize){
  const TrxState& ts = trxSt[activeTrxIdx];
  const TrxConfig& tc = trxCfg[activeTrxIdx];
  char addrStr[5];
  snprintf(addrStr, sizeof(addrStr), "0x%02X", ts.radioAddress);

  // Status aktivního TRX
  const char *trxStat;
  if (tc.isBt()) {
    trxStat = !btClientConnected ? "BT idle" :
              (ts.radioAddress == 0 ? "BT linked | searching CI-V" : "BT linked");
  } else if (tc.isLan()) {
    trxStat = lanClients[activeTrxIdx].statusString();
  } else {
    trxStat = "OI3";
  }

  const char *wifiStat = APmode ? "WiFi AP" :
                         (WiFi.status() == WL_CONNECTED ? "WiFi STA" : "WiFi down");
  int rssi = (APmode || WiFi.status() != WL_CONNECTED) ? -999 : (int)WiFi.RSSI();

  // Hlavní pole z aktivního TRX
  int n = snprintf(buf, bufSize,
    "{\"connected\":%s,\"btStatus\":\"%s\",\"wifiStatus\":\"%s\","
    "\"wifiRssi\":%d,\"fwRev\":\"%u\",\"power\":%s,"
    "\"frequency\":%lu,\"mode\":\"%s\",\"filter\":%u,"
    "\"radioAddress\":\"%s\",\"transceiverType\":\"%s\",\"tx\":%s,\"ritRaw\":%lu,"
    "\"smeterRaw\":%lu,\"powerMeterRaw\":%lu,"
    "\"afGain\":%u,\"keySpeed\":%u,\"rfPower\":%u,"
    "\"supplyVolts\":%.2f,\"swr\":%.2f,"
    "\"preamp\":%u,\"vox\":%u,"
    "\"activeTrx\":%d,",
    ts.connected ? "true" : "false", trxStat, wifiStat,
    rssi, (unsigned)REV, statusPower ? "true" : "false",
    (unsigned long)ts.frequency, ts.modes, (unsigned)ts.filter,
    addrStr, tc.modeStr(), ts.tx ? "true" : "false", (unsigned long)ts.ritRaw,
    (unsigned long)ts.smeterRaw, (unsigned long)ts.powerMeterRaw,
    (unsigned)ts.afGain, (unsigned)ts.keySpeed, (unsigned)ts.rfPower,
    ts.supplyVolts, ts.swr,
    (unsigned)ts.preampMode, (unsigned)ts.voxMode,
    activeTrxIdx
  );

  // Per-TRX array pro chips
  if (n > 0 && n < (int)bufSize - 10) {
    strncat(buf, "\"trx\":[", bufSize - n - 1); n += 7;
    for (int t = 0; t < TRX_COUNT && n < (int)bufSize - 60; t++) {
      if (t > 0) { strncat(buf, ",", bufSize-n-1); n++; }
      char tmp[80];
      snprintf(tmp, sizeof(tmp),
        "{\"label\":\"%s\",\"mode\":\"%s\",\"freq\":%lu,\"conn\":%s}",
        trxCfg[t].label, trxSt[t].modes,
        (unsigned long)trxSt[t].frequency,
        trxSt[t].connected ? "true" : "false");
      strncat(buf, tmp, bufSize-n-1);
      n += strlen(tmp);
    }
    strncat(buf, "]}", bufSize-n-1);
  } else {
    strncat(buf, "\"trx\":[]}", bufSize-n-1);
  }
}

void handleGetState(){
  static char stateBuf[900];
  buildStateJson(stateBuf, sizeof(stateBuf));
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send(200, "application/json", stateBuf);
}

// Přepnout aktivní TRX (GET → vrátí index, POST {idx:N} → nastaví)
void handleTrxSelect(){
  if (webServer.method() == HTTP_POST) {
    String body = webServer.arg("plain");
    int idx = extractJsonString(body, "idx").toInt();
    if (idx >= 0 && idx < TRX_COUNT) {
      activeTrxIdx = idx;
      // Synchronizovat globals z nového aktivního TRX
      radio_address = trxSt[activeTrxIdx].radioAddress;
      frequency     = trxSt[activeTrxIdx].frequency;
      setModesText(trxSt[activeTrxIdx].modes);
    }
    webServer.send(200, "application/json", "{\"ok\":true}");
  } else {
    char tmp[24]; snprintf(tmp, sizeof(tmp), "{\"activeTrx\":%d}", activeTrxIdx);
    webServer.send(200, "application/json", tmp);
  }
}

// ── Pairing signalling buffer ─────────────────────────────────────────────────
static String  g_pairOffer;
static String  g_pairAnswer;
static uint32_t g_pairOfferMs = 0;
const  uint32_t PAIR_TTL_MS   = 300000UL; // 5 minutes

static void pairCors() {
  webServer.sendHeader("Access-Control-Allow-Origin",  "*");
  webServer.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  webServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  webServer.sendHeader("Cache-Control", "no-store");
}

void handlePairingOptions() { pairCors(); webServer.send(204); }

void handlePairingOfferPost() {
  pairCors();
  g_pairOffer   = webServer.arg("plain");
  g_pairAnswer  = "";
  g_pairOfferMs = millis();
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handlePairingOfferGet() {
  pairCors();
  if (g_pairOffer.isEmpty() || (millis() - g_pairOfferMs) > PAIR_TTL_MS) {
    g_pairOffer = ""; g_pairAnswer = "";
    webServer.send(200, "application/json", "{\"pending\":false}");
  } else {
    webServer.send(200, "application/json", g_pairOffer);
  }
}

void handlePairingAnswerPost() {
  pairCors();
  g_pairAnswer = webServer.arg("plain");
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handlePairingAnswerGet() {
  pairCors();
  if (g_pairAnswer.isEmpty()) {
    webServer.send(200, "application/json", "{\"pending\":false}");
  } else {
    String ans = g_pairAnswer;
    g_pairOffer = ""; g_pairAnswer = "";
    webServer.send(200, "application/json", ans);
  }
}

void handlePairingReject() {
  pairCors();
  g_pairOffer = ""; g_pairAnswer = "";
  webServer.send(200, "application/json", "{\"ok\":true}");
}

bool handleFileFromSPIFFS(const String &path){
  String contentType = "text/plain";
  if (path.endsWith(".html")) contentType = "text/html";
  else if (path.endsWith(".css"))  contentType = "text/css";
  else if (path.endsWith(".js"))   contentType = "application/javascript";
  else if (path.endsWith(".ico"))  contentType = "image/x-icon";
  else if (path.endsWith(".png"))  contentType = "image/png";
  if (!SPIFFS.exists(path)) return false;
  File f = SPIFFS.open(path, "r");
  if (!f) return false;
  webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  webServer.sendHeader("Pragma", "no-cache");
  webServer.setContentLength(f.size());
  webServer.send(200, contentType, "");
  // Manuální streaming po 512B blocích s yield() — log.js je 41KB, bez yield by WDT přerušil loopTask
  static uint8_t buf[512];
  while (f.available()) {
    yield();
    size_t len = f.read(buf, sizeof(buf));
    if (len > 0) webServer.client().write(buf, len);
  }
  f.close();
  return true;
}

void handlePostCmd(){
  String body = webServer.arg("plain");
  if (body.length() == 0) { webServer.send(400, "application/json", "{\"error\":\"empty body\"}"); return; }
  String type = extractJsonString(body, "type");

  const TrxConfig& atc = trxCfg[activeTrxIdx];
  bool radioReady = (radio_address != 0x00) &&
                    (atc.isLan() ? lanClients[activeTrxIdx].isConnected() : btClientConnected);
  if (!radioReady) {
    webServer.send(503, "application/json", "{\"error\":\"radio_disconnected\"}");
    return;
  }

  if (type == "setFrequency") {
    uint32_t freq = (uint32_t)extractJsonString(body, "frequency").toInt();
    if (freq == 0) { webServer.send(400, "application/json", "{\"error\":\"invalid_frequency\"}"); return; }
    uint8_t frame[16];
    size_t len = buildSetFrequencyFrame(freq, frame, sizeof(frame));
    if (len == 0 || !catWriteFrame(frame, len, true)) { webServer.send(500, "application/json", "{\"error\":\"tx_failed\"}"); return; }
    webServer.send(200, "application/json", "{\"ok\":true}");
    return;
  }

  if (type == "setMode") {
    uint8_t modeId, modeWidth;
    if (!parseModeId(extractJsonString(body, "mode"), modeId)) { webServer.send(400, "application/json", "{\"error\":\"invalid_mode\"}"); return; }
    modeWidth = parseFilterWidth(extractJsonString(body, "filter"));
    uint8_t frame[16];
    size_t len = buildSetModeFrame(modeId, modeWidth, frame, sizeof(frame));
    if (len == 0 || !catWriteFrame(frame, len, true)) { webServer.send(500, "application/json", "{\"error\":\"tx_failed\"}"); return; }
    webServer.send(200, "application/json", "{\"ok\":true}");
    return;
  }


  if (type == "sendCw") {
    String text = extractJsonString(body, "text");
    if (text.length() == 0) { webServer.send(400, "application/json", "{\"error\":\"missing_text\"}"); return; }
    text.toCharArray(CwMsg, sizeof(CwMsg));
    // mode is preserved — sendCW() routes to CW (CI-V) or RTTY (FSK GPIO) based on current mode
    sendCW();
    webServer.send(200, "application/json", "{\"ok\":true}");
    return;
  }

  if (type == "civ.raw") {
    String hexData = extractJsonString(body, "data");
    uint8_t payload[32];
    size_t payloadLen = 0;
    if (!parseHexPayload(hexData, payload, payloadLen, sizeof(payload))) { webServer.send(400, "application/json", "{\"error\":\"invalid_hex\"}"); return; }
    uint8_t frame[40];
    size_t frameLen = buildSimpleCatFrame(payload[0], payload + 1, payloadLen - 1, frame, sizeof(frame));
    if (frameLen == 0 || !catWriteFrame(frame, frameLen, true)) { webServer.send(500, "application/json", "{\"error\":\"tx_failed\"}"); return; }
    webServer.send(200, "application/json", "{\"ok\":true}");
    return;
  }

  webServer.send(400, "application/json", "{\"error\":\"unsupported_type\"}");
}

static String configJsonEscape(const String &s) {
  String out;
  out.reserve(s.length() + 4);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"')       out += "\\\"";
    else if (c == '\\') out += "\\\\";
    else                out += c;
  }
  return out;
}

static void eepromWriteStr(const String &str, int addr, int maxLen) {
  for (int i = 0; i < maxLen; i++) {
    EEPROM.write(addr + i, (i < (int)str.length()) ? (uint8_t)str[i] : 0xff);
  }
}

static void eepromReadStr(char* dst, int addr, int maxLen) {
  for (int i = 0; i < maxLen; i++) {
    uint8_t c = EEPROM.read(addr + i);
    dst[i] = (c != 0xFF) ? (char)c : '\0';
  }
  dst[maxLen] = '\0';
}

// Uložit TrxConfig do EEPROM slotu i (i=0..2)
static void saveTrxConfig(int i, const TrxConfig& cfg) {
  int base = TRX_EEPROM_BASE[i];
  EEPROM.writeByte(base + 0, cfg.mode);
  for (int j = 0; j < 4; j++) EEPROM.writeByte(base + 1 + j, cfg.ip[j]);
  EEPROM.writeUShort(base + 5, cfg.port);
  for (int j = 0; j < 16; j++) EEPROM.writeByte(base + 7 + j, j < 16 ? (uint8_t)cfg.username[j] : 0xFF);
  for (int j = 0; j < 16; j++) EEPROM.writeByte(base + 23 + j, (uint8_t)cfg.password[j]);
  EEPROM.writeByte(base + 39, cfg.civAddr);
  for (int j = 0; j < 11; j++) EEPROM.writeByte(base + 40 + j, j < 11 ? (uint8_t)cfg.label[j] : 0xFF);
  EEPROM.writeUShort(base + 51, cfg.udpCwPort);
  EEPROM.writeUShort(base + 53, cfg.udpCatPort);
}

// Načíst TrxConfig z EEPROM slotu i
static void loadTrxConfig(int i, TrxConfig& cfg) {
  int base = TRX_EEPROM_BASE[i];
  uint8_t mode = EEPROM.read(base + 0);
  cfg.mode = (mode <= TRX_MODE_OI3) ? mode : TRX_MODE_DISABLED;
  for (int j = 0; j < 4; j++) cfg.ip[j] = EEPROM.read(base + 1 + j);
  uint16_t port = EEPROM.readUShort(base + 5);
  cfg.port = (port == 0 || port == 0xFFFF) ? (cfg.isOi3() ? 81 : 50001) : port;
  eepromReadStr(cfg.username, base + 7,  16);
  eepromReadStr(cfg.password, base + 23, 16);
  uint8_t civ = EEPROM.read(base + 39);
  cfg.civAddr = (civ != 0xFF) ? civ : 0xA4;
  eepromReadStr(cfg.label, base + 40, 11);
  if (cfg.label[0] == '\0') {
    const char* defaults[] = {"TRX1","TRX2","TRX3"};
    strncpy(cfg.label, defaults[i], 11);
  }
  uint16_t cw = EEPROM.readUShort(base + 51);
  cfg.udpCwPort  = (cw  == 0 || cw  == 0xFFFF) ? 89 : cw;
  uint16_t cat = EEPROM.readUShort(base + 53);
  cfg.udpCatPort = (cat == 0 || cat == 0xFFFF) ? 90 : cat;
}

void handleSetupData() {
  // JSON s aktuální konfigurací + chybovými hlášeními pro /setup-data endpoint.
  // setup.html je statický soubor; hodnoty se načítají tímto endpointem přes JS fetch.
  String j;
  j.reserve(1200);
  j += "{\"rev\":";      j += REV;
  j += ",\"hwrev\":";    j += HardwareRev;
  j += ",\"mac\":\"";    j += MACString;          j += "\"";
  j += ",\"apmode\":";   j += APmode ? "true" : "false";
  j += ",\"ssid\":\"";   j += jsonEscape(SSID);   j += "\"";
  j += ",\"pswd\":\"";   j += jsonEscape(PSWD);   j += "\"";
  j += ",\"ssid2\":\"";  j += jsonEscape(SSID2);  j += "\"";
  j += ",\"pswd2\":\"";  j += jsonEscape(PSWD2);  j += "\"";
  j += ",\"httpcatport\":"; j += HTTP_CAT_PORT;
  j += ",\"mqttip\":[";
  j += mqttBroker[0]; j += ","; j += mqttBroker[1]; j += ",";
  j += mqttBroker[2]; j += ","; j += mqttBroker[3]; j += "]";
  j += ",\"mqttport\":"; j += MQTT_PORT;
  j += ",\"mqtttopic\":\"";   j += jsonEscape(MQTT_TOPIC);    j += "\"";
  j += ",\"mqtttopicrx\":\""; j += jsonEscape(MQTT_TOPIC_RX); j += "\"";
  j += ",\"baud\":";     j += BaudRate;
  j += ",\"cwIpOnConnect\":"; j += cwIpOnConnect ? "true" : "false";
  // Per-TRX
  j += ",\"trx\":[";
  for (int t = 0; t < TRX_COUNT; t++) {
    const TrxConfig& tc = trxCfg[t];
    char civBuf[3]; snprintf(civBuf, sizeof(civBuf), "%02X", tc.civAddr);
    if (t) j += ",";
    j += "{\"mode\":"; j += tc.mode;
    j += ",\"label\":\""; j += jsonEscape(String(tc.label)); j += "\"";
    j += ",\"ip\":\"";
    j += tc.ip[0]; j += "."; j += tc.ip[1]; j += "."; j += tc.ip[2]; j += "."; j += tc.ip[3];
    j += "\"";
    j += ",\"port\":"; j += tc.port;
    j += ",\"user\":\""; j += jsonEscape(String(tc.username)); j += "\"";
    j += ",\"pass\":\""; j += jsonEscape(String(tc.password)); j += "\"";
    j += ",\"civ\":\""; j += civBuf; j += "\"";
    j += ",\"udpcw\":"; j += tc.udpCwPort;
    j += ",\"udpcat\":"; j += tc.udpCatPort;
    j += "}";
  }
  j += "]";
  // CW + freq memories
  j += ",\"cwmem\":[";
  for (uint8_t i = 0; i < CW_MEMORY_COUNT; i++) {
    if (i) j += ",";
    j += "\""; j += jsonEscape(cwMemoryText[i]); j += "\"";
  }
  j += "]";
  j += ",\"freqmem\":[";
  for (uint8_t i = 0; i < FREQ_MEMORY_COUNT; i++) {
    if (i) j += ",";
    j += "\""; j += jsonEscape(freqMemoryText[i]); j += "\"";
  }
  j += "]";
  // Chybová hlášení (nastavují se v handleSet(), prázdné = žádná chyba)
  j += ",\"eSSID\":\"";     j += jsonEscape(setupSsidErr);       j += "\"";
  j += ",\"ePSWD\":\"";     j += jsonEscape(setupPswdErr);       j += "\"";
  j += ",\"eSSID2\":\"";    j += jsonEscape(setupSsid2Err);      j += "\"";
  j += ",\"ePSWD2\":\"";    j += jsonEscape(setupPswd2Err);      j += "\"";
  j += ",\"eHttpcatport\":\""; j += jsonEscape(setupHttpCatPortErr); j += "\"";
  j += ",\"eMqtt\":\"";     j += jsonEscape(setupMqttErr);       j += "\"";
  j += ",\"eMqtt1\":\"";    j += jsonEscape(setupMqttErr1);      j += "\"";
  j += ",\"eMqtt2\":\"";    j += jsonEscape(setupMqttErr2);      j += "\"";
  j += ",\"eMqtt3\":\"";    j += jsonEscape(setupMqttErr3);      j += "\"";
  j += ",\"eMqtt4\":\"";    j += jsonEscape(setupMqttErr4);      j += "\"";
  j += ",\"eMqttPort\":\""; j += jsonEscape(setupMqttPortErr);   j += "\"";
  j += ",\"eMqttTopic\":\""; j += jsonEscape(setupMqttTopicErr); j += "\"";
  j += ",\"eMqttTopicRx\":\""; j += jsonEscape(setupMqttTopicRxErr); j += "\"";
  j += ",\"mqttApNote\":\""; j += APmode
    ? "MQTT is not active in AP mode — a WiFi network connection to the broker is required."
    : "";
  j += "\"";
  j += "}";
  webServer.send(200, "application/json", j);
}

void handleConfigDownload() {
  String j;
  j.reserve(1024);
  j += "{\"ssid\":\"";          j += configJsonEscape(SSID);          j += "\"";
  j += ",\"pswd\":\"";          j += configJsonEscape(PSWD);          j += "\"";
  j += ",\"ssid2\":\"";         j += configJsonEscape(SSID2);         j += "\"";
  j += ",\"pswd2\":\"";         j += configJsonEscape(PSWD2);         j += "\"";
  j += ",\"mqttip0\":";         j += mqttBroker[0];
  j += ",\"mqttip1\":";         j += mqttBroker[1];
  j += ",\"mqttip2\":";         j += mqttBroker[2];
  j += ",\"mqttip3\":";         j += mqttBroker[3];
  j += ",\"mqttport\":";        j += MQTT_PORT;
  j += ",\"mqtttopic\":\"";     j += configJsonEscape(MQTT_TOPIC);    j += "\"";
  j += ",\"mqtttopicrx\":\"";   j += configJsonEscape(MQTT_TOPIC_RX); j += "\"";
  j += ",\"httpcatport\":";     j += HTTP_CAT_PORT;
  j += ",\"udpport\":";         j += udpPort;
  j += ",\"udpcatport\":";      j += udpCatPort;
  j += ",\"baudrate\":";        j += BaudRate;
  j += ",\"cwIpOnConnect\":";  j += cwIpOnConnect ? "true" : "false";
  // Per-TRX konfigurace
  for (int t = 0; t < TRX_COUNT; t++) {
    const TrxConfig& tc = trxCfg[t];
    int n = t + 1;
    j += ",\"trx" + String(n) + "mode\":" + String(tc.mode);
    j += ",\"trx" + String(n) + "label\":\"" + configJsonEscape(String(tc.label)) + "\"";
    j += ",\"trx" + String(n) + "ip\":\"" + String(tc.ip[0])+"."+String(tc.ip[1])+"."+String(tc.ip[2])+"."+String(tc.ip[3]) + "\"";
    j += ",\"trx" + String(n) + "port\":" + String(tc.port);
    j += ",\"trx" + String(n) + "user\":\"" + configJsonEscape(String(tc.username)) + "\"";
    char civBuf[3]; snprintf(civBuf, sizeof(civBuf), "%02X", tc.civAddr);
    j += ",\"trx" + String(n) + "civ\":\"" + civBuf + "\"";
    j += ",\"trx" + String(n) + "udpcw\":" + String(tc.udpCwPort);
    j += ",\"trx" + String(n) + "udpcat\":" + String(tc.udpCatPort);
  }
  for (uint8_t i = 0; i < CW_MEMORY_COUNT; i++) {
    j += ",\"cwmem"; j += (i + 1); j += "\":\""; j += configJsonEscape(cwMemoryText[i]); j += "\"";
  }
  for (uint8_t i = 0; i < FREQ_MEMORY_COUNT; i++) {
    j += ",\"freqmem"; j += (i + 1); j += "\":\""; j += configJsonEscape(freqMemoryText[i]); j += "\"";
  }
  j += "}";
  webServer.sendHeader("Content-Disposition", "attachment; filename=\"ic705-config.json\"");
  webServer.send(200, "application/json", j);
}

void handleConfigUpload() {
  String body = webServer.arg("plain");
  if (body.length() == 0) { webServer.send(400, "application/json", "{\"error\":\"empty\"}"); return; }

  // Strings to EEPROM
  String ssid = extractJsonString(body, "ssid");
  if (ssid.length() >= 1 && ssid.length() <= 20) { SSID = ssid; eepromWriteStr(SSID, 1, 20); }

  String pswd = extractJsonString(body, "pswd");
  if (pswd.length() >= 1 && pswd.length() <= 18) { PSWD = pswd; eepromWriteStr(PSWD, 22, 18); }

  String ssid2 = extractJsonString(body, "ssid2");
  if (ssid2.length() <= 20) { SSID2 = ssid2; eepromWriteStr(SSID2, 76, 20); }

  String pswd2 = extractJsonString(body, "pswd2");
  if (pswd2.length() <= 18) { PSWD2 = pswd2; eepromWriteStr(PSWD2, 97, 18); }

  String mqtttopic = extractJsonString(body, "mqtttopic");
  if (mqtttopic.length() >= 1 && mqtttopic.length() <= 20) { MQTT_TOPIC = mqtttopic; eepromWriteStr(MQTT_TOPIC, 47, 20); }

  String mqtttopicrx = extractJsonString(body, "mqtttopicrx");
  if (mqtttopicrx.length() <= 20) { MQTT_TOPIC_RX = mqtttopicrx; eepromWriteStr(MQTT_TOPIC_RX, 115, 21); }

  // Integers to EEPROM — lambda must be declared before per-TRX block
  auto parseField = [&](const char *key, int minVal, int maxVal) -> int {
    int idx = body.indexOf(String("\"") + key + "\":");
    if (idx < 0) return -1;
    int start = body.indexOf(':', idx) + 1;
    while (start < (int)body.length() && body[start] == ' ') start++;
    int end = start;
    while (end < (int)body.length() && (isDigit(body[end]) || body[end] == '-')) end++;
    int val = body.substring(start, end).toInt();
    return (val >= minVal && val <= maxVal) ? val : -1;
  };

  // Per-TRX konfigurace z JSON
  {
    bool btTaken = false;
    for (int t = 0; t < TRX_COUNT; t++) {
      int n = t + 1;
      String pfx = "trx" + String(n);
      int modeVal = parseField((pfx + "mode").c_str(), 0, (int)TRX_MODE_OI3);
      if (modeVal >= 0) {
        uint8_t mode = (uint8_t)modeVal;
        if (mode == TRX_MODE_BT && btTaken) mode = TRX_MODE_DISABLED;
        if (mode == TRX_MODE_BT) btTaken = true;
        trxCfg[t].mode = mode;
      }
      String lbl = extractJsonString(body, (pfx + "label").c_str());
      if (lbl.length() > 0) { if (lbl.length() > 11) lbl = lbl.substring(0, 11); lbl.toCharArray(trxCfg[t].label, sizeof(trxCfg[t].label)); }
      String ipStr = extractJsonString(body, (pfx + "ip").c_str());
      if (ipStr.length() > 0) { int b[4]={0,0,0,0}; sscanf(ipStr.c_str(),"%d.%d.%d.%d",&b[0],&b[1],&b[2],&b[3]); for(int j=0;j<4;j++) trxCfg[t].ip[j]=(uint8_t)b[j]; }
      int lp = parseField((pfx + "port").c_str(), 1, 65534); if (lp > 0) trxCfg[t].port = (uint16_t)lp;
      String usr = extractJsonString(body, (pfx + "user").c_str());
      if (usr.length() > 0) { if (usr.length() > 16) usr = usr.substring(0, 16); usr.toCharArray(trxCfg[t].username, sizeof(trxCfg[t].username)); }
      String civStr = extractJsonString(body, (pfx + "civ").c_str());
      uint8_t civ = trxCfg[t].civAddr;
      if (parseHexByteString(civStr, civ)) trxCfg[t].civAddr = civ;
      int cw = parseField((pfx + "udpcw").c_str(), 1, 65534); if (cw > 0) trxCfg[t].udpCwPort = (uint16_t)cw;
      int ct = parseField((pfx + "udpcat").c_str(), 1, 65534); if (ct > 0) trxCfg[t].udpCatPort = (uint16_t)ct;
      saveTrxConfig(t, trxCfg[t]);
    }
    transceiverType      = trxCfg[0].modeStr();
    configuredCivAddress = trxCfg[0].effectiveCivAddr();
    lanMode = false;
    for (int t = 0; t < TRX_COUNT; t++) if (trxCfg[t].isLan()) lanMode = true;
  }

  {
    int idx = body.indexOf("\"cwIpOnConnect\"");
    if (idx >= 0) {
      int colon = body.indexOf(':', idx);
      if (colon >= 0) {
        int vs = colon + 1;
        while (vs < (int)body.length() && body[vs] == ' ') vs++;
        bool v = body.substring(vs, vs + 4) == "true";
        cwIpOnConnect = v;
        EEPROM.writeBool(136, v);
      }
    }
  }

  int v;
  v = parseField("mqttip0", 0, 255); if (v >= 0) { mqttBroker[0] = v; EEPROM.writeByte(41, v); mqttEnable = (v != 0); }
  v = parseField("mqttip1", 0, 255); if (v >= 0) { mqttBroker[1] = v; EEPROM.writeByte(42, v); }
  v = parseField("mqttip2", 0, 255); if (v >= 0) { mqttBroker[2] = v; EEPROM.writeByte(43, v); }
  v = parseField("mqttip3", 0, 255); if (v >= 0) { mqttBroker[3] = v; EEPROM.writeByte(44, v); }
  v = parseField("mqttport", 1, 65534); if (v >= 0) { MQTT_PORT = v; EEPROM.writeUShort(45, v); }
  v = parseField("httpcatport", 1, 65534); if (v >= 0) { HTTP_CAT_PORT = v; EEPROM.writeUShort(68, v); }
  v = parseField("baudrate", 1200, 115200); if (v > 0) { BaudRate = v; EEPROM.writeUShort(74, v); }

  for (uint8_t i = 0; i < CW_MEMORY_COUNT; i++) {
    char key[10]; snprintf(key, sizeof(key), "cwmem%u", i + 1);
    cwMemoryText[i] = trimMemoryValue(extractJsonString(body, key), CW_MEMORY_MAX_LEN);
  }
  for (uint8_t i = 0; i < FREQ_MEMORY_COUNT; i++) {
    char key[12]; snprintf(key, sizeof(key), "freqmem%u", i + 1);
    freqMemoryText[i] = trimMemoryValue(extractJsonString(body, key), FREQ_MEMORY_MAX_LEN);
  }

  EEPROM.writeBool(0, false);
  EEPROM.commit();
  saveMemoryConfig();

  webServer.send(200, "application/json", "{\"ok\":true}");
}

static const char* LOG_CONFIG_PATH = "/log-config.json";

void handleGetLogConfig() {
  if (!SPIFFS.exists(LOG_CONFIG_PATH)) {
    webServer.send(200, "application/json", "{}");
    return;
  }
  File f = SPIFFS.open(LOG_CONFIG_PATH, "r");
  if (!f) { webServer.send(500, "application/json", "{\"error\":\"open\"}"); return; }
  webServer.streamFile(f, "application/json");
  f.close();
}

void handlePostLogConfig() {
  String body = webServer.arg("plain");
  body.trim();
  if (body.length() == 0 || body.length() > 2048) {
    webServer.send(400, "application/json", "{\"error\":\"bad_request\"}");
    return;
  }
  if (body[0] != '{' || body[body.length()-1] != '}') {
    webServer.send(400, "application/json", "{\"error\":\"not_json\"}");
    return;
  }
  File f = SPIFFS.open(LOG_CONFIG_PATH, "w");
  if (!f) { webServer.send(500, "application/json", "{\"error\":\"write\"}"); return; }
  f.print(body);
  f.close();
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleOi3State() {
  String ip = webServer.arg("ip");
  if (ip.length() == 0) {
    webServer.send(400, "application/json", "{\"error\":\"missing_ip\"}");
    return;
  }
  HTTPClient http;
  String url = "http://" + ip + ":81/";
  if (!http.begin(url)) {
    webServer.send(200, "application/json", "{\"connected\":false,\"frequency\":0,\"mode\":\"USB\"}");
    return;
  }
  http.setTimeout(1000);
  int code = http.GET();
  if (code != 200) {
    http.end();
    webServer.send(200, "application/json", "{\"connected\":false,\"frequency\":0,\"mode\":\"USB\"}");
    return;
  }
  String resp = http.getString();
  http.end();
  resp.trim();
  int p1 = resp.indexOf('|');
  if (p1 < 0) {
    webServer.send(200, "application/json", "{\"connected\":false,\"frequency\":0,\"mode\":\"USB\"}");
    return;
  }
  long freq = resp.substring(0, p1).toInt();
  int p2 = resp.indexOf('|', p1 + 1);
  String modeStr = (p2 > p1) ? resp.substring(p1 + 1, p2) : resp.substring(p1 + 1);
  modeStr.trim();
  String j = "{\"connected\":true,\"frequency\":";
  j += freq;
  j += ",\"mode\":\"";
  j += modeStr;
  j += "\"}";
  webServer.send(200, "application/json", j);
}

void handleOi3Send() {
  String body = webServer.arg("plain");
  String ip   = extractJsonString(body, "ip");
  String text = extractJsonString(body, "text");
  if (ip.length() == 0 || text.length() == 0) {
    webServer.send(400, "application/json", "{\"error\":\"missing\"}");
    return;
  }
  WiFiUDP oi3Udp;
  oi3Udp.beginPacket(ip.c_str(), 89);
  oi3Udp.print(text);
  oi3Udp.endPacket();
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void setupWebServer(void){
  webServer.on("/state", HTTP_GET,  handleGetState);
  webServer.on("/trx",   HTTP_GET,  handleTrxSelect);
  webServer.on("/trx",   HTTP_POST, handleTrxSelect);
  webServer.on("/cmd",   HTTP_POST, handlePostCmd);
  webServer.on("/config/download", HTTP_GET,  handleConfigDownload);
  webServer.on("/config/upload",   HTTP_POST, handleConfigUpload);
  webServer.on("/log-config", HTTP_GET,  handleGetLogConfig);
  webServer.on("/log-config", HTTP_POST, handlePostLogConfig);
  webServer.on("/oi3/state", HTTP_GET,  handleOi3State);
  webServer.on("/oi3/send",  HTTP_POST, handleOi3Send);

  webServer.on("/", HTTP_GET, [](){
    if (!SPIFFS.exists("/index.html")) { handleFileFromSPIFFS("/setup.html"); return; }
    renderIndexPage();
  });

  // setup.html je statický soubor — žádný template rendering, žádný WDT risk.
  // Hodnoty se načítají JS fetchem z /setup-data.
  // POST: handleSet() restartuje při úspěchu; při chybách vrátí 302 redirect na GET.
  webServer.on("/setup-data", HTTP_GET, [](){ handleSetupData(); });
  webServer.on("/setup",  HTTP_GET,  [](){ handleFileFromSPIFFS("/setup.html"); });
  webServer.on("/setup",  HTTP_POST, [](){
    handleSet();
    // Pokud jsme stále tady, handleSet() nenalezl chyby ale z nějakého důvodu nerestartoval,
    // nebo nalezl chyby (ERRdetect != 0). V obou případech přesměrovat zpět na GET /setup.
    webServer.sendHeader("Location", "/setup", true);
    webServer.send(302, "text/plain", "");
  });
  webServer.on("/ws-cat",   HTTP_GET,  [](){ handleFileFromSPIFFS("/ws-cat.html"); });
  webServer.on("/log",      HTTP_GET,  [](){ handleFileFromSPIFFS("/log.html"); });
  webServer.on("/datasync", HTTP_GET,  [](){ handleFileFromSPIFFS("/datasync.html"); });

  webServer.on("/pairing/offer",  HTTP_OPTIONS, handlePairingOptions);
  webServer.on("/pairing/offer",  HTTP_POST,    handlePairingOfferPost);
  webServer.on("/pairing/offer",  HTTP_GET,     handlePairingOfferGet);
  webServer.on("/pairing/answer", HTTP_OPTIONS, handlePairingOptions);
  webServer.on("/pairing/answer", HTTP_POST,    handlePairingAnswerPost);
  webServer.on("/pairing/answer", HTTP_GET,     handlePairingAnswerGet);
  webServer.on("/pairing/reject", HTTP_OPTIONS, handlePairingOptions);
  webServer.on("/pairing/reject", HTTP_POST,    handlePairingReject);

  webServer.onNotFound([](){
    String path = webServer.uri();
    if (!handleFileFromSPIFFS(path)) webServer.send(404, "text/plain", "Not found");
  });

  webServer.begin();

  // WebSocket pro vodopad na portu 82
  wfWs.onEvent([](AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType t,
                  void*, uint8_t*, size_t){
    // posílat hello při připojení by bylo hezké, ale stačí tiché připojení
    (void)t;
  });
  wfWsServer.addHandler(&wfWs);
  wfWsServer.begin();
  MDNS.addService("ws", "tcp", 82);
}


bool catWriteFrame(const uint8_t *frame, size_t frameLen, bool broadcastTx){
  #if defined(WIFI)
  if (trxCfg[activeTrxIdx].isLan() && lanClients[activeTrxIdx].isConnected()) {
    return lanClients[activeTrxIdx].sendCiv(frame, frameLen);
  }
  #endif
  #if defined(BLUETOOTH)
  if (trxCfg[activeTrxIdx].isBt() && btClientConnected) {
    for (size_t i = 0; i < frameLen; i++) CAT.write(frame[i]);
    return true;
  }
  #endif
  return false;
}

void wsClearSplitProbe(void){
  // stub retained for BT callback compatibility
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

    // 70-71 udpPort (legacy global — migrace do TRX1 udpCwPort při prvním startu)
    if(EEPROM.read(70)==0xff){
      udpPort=89;
    }else{
      udpPort = EEPROM.readUShort(70);
    }

    // 72-73 udpCatPort (legacy global)
    if(EEPROM.read(72)==0xff){
      udpCatPort=90;
    }else{
      udpCatPort = EEPROM.readUShort(72);
    }

    if(EEPROM.read(136) != 0xff){
      cwIpOnConnect = EEPROM.readBool(136);
    }

    // Načíst TRX1/2/3 konfigurace z EEPROM
    #if defined(WIFI)
    {
      for (int t = 0; t < TRX_COUNT; t++) {
        loadTrxConfig(t, trxCfg[t]);
        Serial.print(" TRX"); Serial.print(t+1);
        Serial.print("| "); Serial.print(trxCfg[t].label);
        Serial.print(" mode="); Serial.println(trxCfg[t].modeStr());
      }
      // Zpětná kompatibilita: migrate legacy IC-705-BT default (connMode 0xFF → BT)
      // Pokud TRX1 je DISABLED a TRX2/3 jsou taky disabled → defaultně BT
      bool anyEnabled = false;
      for (int t = 0; t < TRX_COUNT; t++) if (trxCfg[t].enabled()) { anyEnabled = true; break; }
      if (!anyEnabled) {
        trxCfg[0].mode = TRX_MODE_BT;
        trxCfg[0].civAddr = 0xA4;
        trxCfg[0].udpCwPort  = udpPort;
        trxCfg[0].udpCatPort = udpCatPort;
        strncpy(trxCfg[0].label, "IC-705", 11);
        Serial.println(" TRX | defaultni BT na TRX1");
      }
      // Synchronizovat legacy globals z TRX1
      transceiverType    = trxCfg[0].modeStr();
      configuredCivAddress = trxCfg[0].effectiveCivAddr();
      lanMode = false;
      for (int t = 0; t < TRX_COUNT; t++) if (trxCfg[t].isLan()) lanMode = true;
      // UDP porty z BT TRX (pro N1MM+/Win-Test)
      int bti = btTrxIdx();
      if (bti >= 0) {
        udpPort    = trxCfg[bti].udpCwPort;
        udpCatPort = trxCfg[bti].udpCatPort;
      }
    }
    #endif

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
      loadMemoryConfig();
    }

    if(APmode==true){
      WiFi.mode(WIFI_AP);
      WiFi.softAP(ssidAP, passwordAP, 1, 0, 4); // ch1, visible, max 4 clients, WPA2
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

      setupWebServer();
      Serial.println("HTTP| web server started");

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
      setupWebServer();
      Serial.println("HTTP| web server started");
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

    #if defined(BLUETOOTH)
      if (btTrxIdx() >= 0) configRadioBaud(0);
    #endif

    #if defined(WIFI)
      for (int t = 0; t < TRX_COUNT; t++) {
        if (!trxCfg[t].isLan()) continue;
        IPAddress ip = trxCfg[t].ipAddr();
        if (ip == IPAddress(0,0,0,0)) continue;
        int ti = t; // capture for lambda
        lanClients[t].begin(ip, trxCfg[t].port,
                            trxCfg[t].username, trxCfg[t].password,
                            "ic705if", trxCfg[t].effectiveCivAddr());
        lanClients[t].onCivReceived([ti](const uint8_t* d, size_t l){ processCivFrame(ti, d, l); });
        Serial.print(" LAN| TRX"); Serial.print(t+1);
        Serial.print(" "); Serial.print(ip); Serial.print(":"); Serial.println(trxCfg[t].port);
      }
    #endif

    #if defined(WDT)
      // WDT
      esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
      esp_task_wdt_add(NULL); //add current thread to WDT watch
      WdtTimer=millis();
    #endif

    // Flush any garbage bytes accumulated in Serial RX buffer during boot/USB re-enumeration
    while (Serial.available()) Serial.read();
  }
}
//-------------------------------------------------------------------------------------------------------

void loop(){
  Watchdog();
  Mqtt();
  httpCAT();
  UdpToCwFsk();
  UdpToCat();
  CLI();
  #if defined(RTLE)
    rtleserver.handleClient();
  #endif
  webServer.handleClient();

  // AP mode: status LED fade in/out
  if(APmode==true){
    static int PwmValue = 0;
    static bool PwmDir = true;
    static long pwmTimer = 0;
    if(millis()-pwmTimer>3){
      if(PwmDir==true){
        PwmValue++;
        if(PwmValue>254){ PwmDir=false; }
      }else{
        PwmValue--;
        if(PwmValue<1){ PwmDir=true; }
      }
      ledcWrite(pwmChannel, PwmValue);
      pwmTimer=millis();
    }
  }
}

// SUBROUTINES -------------------------------------------------------------------------------------------------------

void handleBtEvents(){
  if (btStateBroadcastPending == true) {
    btStateBroadcastPending = false;
  }

  if (btDisconnectPending == true) {
    btDisconnectPending = false;
    powerTimer = 0;
    frequency = 0;
    setModesText("OFF");
    mqttPostponeStatus = 1;
    if (statusPower == 1 && mqttEnable == true) {
      MqttPubString(MQTT_TOPIC, "0", 0);
    }
  }
}

void pollRadio(){
  static unsigned long catPollTimer = 0;
  static unsigned long auxPollTimer = 0;
  static uint8_t auxPollIndex = 0;

  // Polling aktivního TRX
  bool radioOnline = false;
  #if defined(BLUETOOTH)
  { int bti = btTrxIdx(); if (bti >= 0 && btClientConnected) radioOnline = true; }
  #endif
  #if defined(WIFI)
  if (trxCfg[activeTrxIdx].isLan() && lanClients[activeTrxIdx].isConnected()) radioOnline = true;
  #endif
  if (!radioOnline) return;

  if (millis() - catPollTimer > 200) {
    catPollTimer = millis();
    if (radio_address == 0x00) {
      #if defined(BLUETOOTH)
      if (trxCfg[activeTrxIdx].isBt()) {
        if (Debug) Serial.println("CAT | searching radio...");
        searchRadio();
      }
      #endif
    } else {
      sendCatRequest(CMD_READ_FREQ);
      sendCatRequest(CMD_READ_MODE);
      #if defined(BLUETOOTH)
      if (trxCfg[activeTrxIdx].isBt()) processCatMessages();
      #endif
    }
  }

  if (cwIpSendPending && radio_address != 0x00) {
    cwIpSendPending = false;
    TrxSetupDone = true;
    ServiceBackgroundTasks(800);
    uint8_t modeFrame[10];
    size_t modeFrameLen = buildSetModeFrame(0x03, 0x03, modeFrame, sizeof(modeFrame)); // CW, FIL3
    catWriteFrame(modeFrame, modeFrameLen, false);
    ServiceBackgroundTasks(400);
    String ipStr = "IP " + WiFi.localIP().toString();
    ipStr.toCharArray(CwMsg, sizeof(CwMsg));
    setModesText("CW");
    sendCW();
    ServiceBackgroundTasks(10000); // wait for radio to finish keying
  }

  if (radio_address != 0x00 && millis() - auxPollTimer > 80) {
    auxPollTimer = millis();
    uint8_t frame[10];
    size_t frameLen = 0;
    switch (auxPollIndex % 10) {
      case 0: { uint8_t p[] = {stateTx ? (uint8_t)0x11 : (uint8_t)0x02}; frameLen = buildSimpleCatFrame(0x15, p, 1, frame, sizeof(frame)); break; } // TX=power, RX=S-meter
      case 1: { uint8_t p[] = {stateTx ? (uint8_t)0x12 : (uint8_t)0x15}; frameLen = buildSimpleCatFrame(0x15, p, 1, frame, sizeof(frame)); break; } // TX=SWR, RX=supply
      case 2: { uint8_t p[] = {0x01}; frameLen = buildSimpleCatFrame(0x14, p, 1, frame, sizeof(frame)); break; } // AF gain
      case 3: { uint8_t p[] = {0x0C}; frameLen = buildSimpleCatFrame(0x14, p, 1, frame, sizeof(frame)); break; } // key speed
      case 4: { uint8_t p[] = {0x0A}; frameLen = buildSimpleCatFrame(0x14, p, 1, frame, sizeof(frame)); break; } // RF power
      case 5: { uint8_t p[] = {0x00}; frameLen = buildSimpleCatFrame(0x1C, p, 1, frame, sizeof(frame)); break; } // TX state
      case 6: { uint8_t p[] = {0x00}; frameLen = buildSimpleCatFrame(0x21, p, 1, frame, sizeof(frame)); break; } // RIT offset (sub 0x00)
      case 7: {                        frameLen = buildSimpleCatFrame(0x11, nullptr, 0, frame, sizeof(frame)); break; } // ATT
      case 8: { uint8_t p[] = {0x02}; frameLen = buildSimpleCatFrame(0x16, p, 1, frame, sizeof(frame)); break; } // PREAMP
      case 9: { uint8_t p[] = {0x47}; frameLen = buildSimpleCatFrame(0x16, p, 1, frame, sizeof(frame)); break; } // VOX/BKIN
    }
    if (frameLen > 0) catWriteFrame(frame, frameLen, false);
    #if defined(BLUETOOTH)
    if (!lanMode) processCatMessages();
    #endif
    auxPollIndex++;
  }
}

void Watchdog(){
  #if defined(WIFI)
  // Loop všech LAN klientů + scope aktivace
  for (int t = 0; t < TRX_COUNT; t++) {
    if (!trxCfg[t].isLan()) continue;
    lanClients[t].loop();
    bool nowConn = lanClients[t].isConnected();
    trxSt[t].connected = nowConn;
    if (nowConn && !scopeInitDone) {
      if (trxSt[t].radioAddress == 0) {
        trxSt[t].radioAddress = trxCfg[t].effectiveCivAddr();
      }
      if (t == activeTrxIdx) radio_address = trxSt[t].radioAddress;
    }
    if (nowConn && !scopeInitDone) {
      scopeInitDone = true;
      sendScopeActivation(t);
    }
    if (!nowConn && scopeInitDone) {
      scopeInitDone = false;
    }
  }
  // Synchronizovat connected flag z BT
  {
    int bti = btTrxIdx();
    if (bti >= 0) trxSt[bti].connected = btClientConnected;
  }
  // WebSocket cleanup
  static unsigned long wsCleanTimer = 0;
  if (millis() - wsCleanTimer > 2000) {
    wsCleanTimer = millis();
    wfWs.cleanupClients();
  }
  #endif
  handleBtEvents();
  pollRadio();

  static unsigned long mqttFreqTimer = 0;

  // Power OUT/LED
  if(millis()-powerTimer > 3000){
    if(statusPower==1){
      digitalWrite(PowerOnPin, LOW);
      Serial.println(" PWR| OFF");
      statusPower = 0;
      frequency = 0;
      frequencyTmp = 0;
      setModesText("OFF");
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
    if (!APmode && (currentMillis - WifiTimer >= WifiReconnect)) {
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
    webServer.handleClient();
    #if defined(WDT)
      esp_task_wdt_reset();
    #endif
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
void SelectANT(){
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
// ── Vodopad (fáze 5) ────────────────────────────────────────────────────────

#if defined(WIFI)

// Scope aktivace: scope ON, fixed mode, edge slot 1
void sendScopeActivation(int trxIdx) {
  if (trxSt[trxIdx].radioAddress == 0x00) return;
  uint8_t frame[12];
  size_t len;
  // FE FE <addr> E0 27 00 01 FD  — scope ON
  { uint8_t p[] = {0x00, 0x01}; len = buildSimpleCatFrame(0x27, p, 2, frame, sizeof(frame)); catWriteFrame(frame, len, false); }
  delay(20);
  // FE FE <addr> E0 27 14 00 01 FD  — fixed mode
  { uint8_t p[] = {0x14, 0x00, 0x01}; len = buildSimpleCatFrame(0x27, p, 3, frame, sizeof(frame)); catWriteFrame(frame, len, false); }
  delay(20);
  // FE FE <addr> E0 27 16 01 FD  — edge slot 1
  { uint8_t p[] = {0x16, 0x01}; len = buildSimpleCatFrame(0x27, p, 2, frame, sizeof(frame)); catWriteFrame(frame, len, false); }
  Serial.println(" WF | scope aktivovan");
}

// 5-bajtový BCD → Hz (LSB first, IC-705 formát)
static uint32_t bcd5ToHz(const uint8_t* b) {
  return (uint32_t)(b[4] >> 4) * 1000000000UL / 10
       + (uint32_t)(b[4] & 0xF) * 100000000UL / 10
       + (uint32_t)(b[3] >> 4) * 10000000UL  / 10
       + (uint32_t)(b[3] & 0xF) * 1000000UL   / 10
       + (uint32_t)(b[2] >> 4) * 100000UL     / 10
       + (uint32_t)(b[2] & 0xF) * 10000UL      / 10
       + (uint32_t)(b[1] >> 4) * 1000UL
       + (uint32_t)(b[1] & 0xF) * 100UL
       + (uint32_t)(b[0] >> 4) * 10UL
       + (uint32_t)(b[0] & 0xF);
}

static uint8_t bcdToUint8(uint8_t b) {
  return (uint8_t)((b >> 4) * 10 + (b & 0x0F));
}

// Odeslat kompletní vodopadový frame přes WebSocket
void sendWaterfallToWebSocket(const WfFrame& f) {
  if (wfWs.count() == 0) return; // žádný klient

  // JSON: {"t":"wf","s":...,"e":...,"v":...,"d":[...]}
  // Odhad délky: header ~60 B + pixely (max 500 * 4 B = 2000 B) + footer 2 B
  size_t bufSize = 80 + f.pixelCount * 4 + 4;
  char* buf = (char*)malloc(bufSize);
  if (!buf) return;

  int pos = snprintf(buf, bufSize,
    "{\"t\":\"wf\",\"s\":%lu,\"e\":%lu,\"v\":%lu,\"d\":[",
    (unsigned long)f.startHz, (unsigned long)f.endHz, (unsigned long)frequency);

  for (uint16_t i = 0; i < f.pixelCount && pos < (int)(bufSize - 6); i++) {
    if (i > 0) buf[pos++] = ',';
    pos += snprintf(&buf[pos], bufSize - pos, "%u", f.pixels[i]);
  }
  if (pos < (int)(bufSize - 3)) {
    buf[pos++] = ']';
    buf[pos++] = '}';
    buf[pos]   = '\0';
  }

  wfWs.textAll(buf);
  free(buf);
}

// Zpracování CI-V 0x27 scope segmentu.
// d = &read_buffer[4] (začíná bajtem 0x27), lenD = délka od 0x27 do FD včetně
void civProcessScopePacket(int trxIdx, const uint8_t* d, size_t lenD) {
  (void)trxIdx;
  if (lenD < 6) return;
  // d[0]=0x27, d[1]=0x00, d[2]=0x00, d[3]=seq_BCD, d[4]=seqMax_BCD
  uint8_t seq    = bcdToUint8(d[3]);
  uint8_t seqMax = bcdToUint8(d[4]);
  if (seq == 0 || seqMax == 0) return;

  if (seq == 1) {
    // Metadata segment: d[5]=mode, d[6..10]=fA, d[11..15]=fB, d[16]=OOR
    if (lenD < 18) return;
    wfBuf.mode = d[5];
    uint32_t fA = bcd5ToHz(&d[6]);
    uint32_t fB = bcd5ToHz(&d[11]);
    wfBuf.oor   = (d[16] != 0);
    if (wfBuf.mode == 0x00) { // center mode
      wfBuf.startHz = (fA > fB) ? fA - fB : 0;
      wfBuf.endHz   = fA + fB;
    } else {                   // fixed / scroll
      wfBuf.startHz = fA;
      wfBuf.endHz   = fB;
    }
    wfBuf.pixelCount = 0;
    wfBuf.complete   = false;
  } else {
    // Datový segment: pixely od d[5] do d[lenD-2] (lenD-1 je FD)
    if (lenD < 7) return;
    uint16_t pxLen = (uint16_t)(lenD - 5 - 1); // -5 header, -1 FD
    if (pxLen > 50) pxLen = 50;
    if (wfBuf.pixelCount + pxLen > sizeof(wfBuf.pixels)) {
      pxLen = sizeof(wfBuf.pixels) - wfBuf.pixelCount;
    }
    memcpy(&wfBuf.pixels[wfBuf.pixelCount], &d[5], pxLen);
    wfBuf.pixelCount += pxLen;
  }

  if (seq == seqMax && seq > 1) {
    wfBuf.complete = true;
    sendWaterfallToWebSocket(wfBuf);
  }
}

#endif // WIFI

// Zpracování CI-V rámce pro konkrétní TRX.
// Pracuje přes globální read_buffer — vždy zkopíruj frame před voláním.
static void processCivData(int trxIdx, uint8_t len) {
  if (trxIdx < 0 || trxIdx >= TRX_COUNT) return;
  if (len < 6) return;
  if (read_buffer[0] != START_BYTE || read_buffer[1] != START_BYTE) return;
  if (read_buffer[len - 1] != STOP_BYTE) return;

  TrxState& ts = trxSt[trxIdx];
  bool isActive = (trxIdx == activeTrxIdx);

  // Auto-detect CI-V adresa
  if (ts.radioAddress == 0x00) {
    ts.radioAddress = read_buffer[3];
    if (isActive) {
      radio_address = ts.radioAddress;
      if (Debug) {
        Serial.print("CAT | TRX"); Serial.print(trxIdx+1);
        Serial.print(" adresa 0x"); Serial.println(radio_address, HEX);
      }
      if (!ts.trxSetupDone) ts.trxNeedSet = true;
    }
  } else if (read_buffer[3] != ts.radioAddress) {
    return;
  }

  // Scope / vodopad (0x27)
  #if defined(WIFI)
  if (read_buffer[4] == 0x27 && len >= 6 &&
      read_buffer[5] == 0x00 && read_buffer[6] == 0x00) {
    civProcessScopePacket(trxIdx, &read_buffer[4], (size_t)(len - 4));
    return;
  }
  #endif

  if (read_buffer[2] == BROADCAST_ADDRESS || read_buffer[2] == CONTROLLER_ADDRESS) {
    switch (read_buffer[4]) {
      case CMD_TRANS_FREQ:
      case CMD_READ_FREQ:
        if (len >= 11) {
          // Dekódovat frekvenci do trxSt[trxIdx]
          ts.frequency = 0;
          for (uint8_t i = 0; i < 5; i++) {
            if (read_buffer[9-i] == STOP_BYTE) continue;
            ts.frequency += (read_buffer[9-i] >> 4) * decMulti[i*2];
            ts.frequency += (read_buffer[9-i] & 0x0F) * decMulti[i*2+1];
          }
          if (isActive) {
            frequency = ts.frequency;
          }
        }
        break;
      case CMD_TRANS_MODE:
      case CMD_READ_MODE:
        if (len >= 7) {
          // Přímé mapování bez heap alokace (dříve decodeModeName().c_str() = dangling pointer → crash)
          const char* mn;
          switch (read_buffer[5]) {
            case 0x00: mn = "LSB";    break;
            case 0x01: mn = "USB";    break;
            case 0x02: mn = "AM";     break;
            case 0x03: mn = "CW";     break;
            case 0x04: mn = "RTTY";   break;
            case 0x05: mn = "FM";     break;
            case 0x06: mn = "WFM";    break;
            case 0x07: mn = "CW-R";   break;
            case 0x08: mn = "RTTY-R"; break;
            case 0x17: mn = "DV";     break;
            default:   mn = "UNK";    break;
          }
          strncpy(ts.modes, mn, sizeof(ts.modes) - 1);
          ts.modes[sizeof(ts.modes) - 1] = '\0';
          if (len >= 8) ts.filter = read_buffer[6];
          if (isActive) {
            setModesText(ts.modes);
            stateFilter = ts.filter;
          }
        }
        break;
    }
  }

  if (len >= 7) {
    const uint8_t cmd = read_buffer[4];
    const uint8_t *pl = read_buffer + 5;
    const size_t plLen = len - 6;
    if (cmd == 0x15 && plLen >= 2) {
      uint32_t raw = decodeCivBcdBytes(pl + 1, plLen - 1);
      if (pl[0] == 0x02) ts.smeterRaw = raw;
      else if (pl[0] == 0x11) ts.powerMeterRaw = raw;
      else if (pl[0] == 0x12) ts.swr = 1.0f + ((float)raw * 3.0f / 120.0f);
      else if (pl[0] == 0x15) ts.supplyVolts = ((float)raw * 16.0f) / 241.0f;
    }
    if (cmd == 0x14 && plLen >= 2) {
      uint32_t raw = decodeCivBcdBytes(pl + 1, plLen - 1);
      if (pl[0] == 0x01) ts.afGain    = (uint8_t)raw;
      else if (pl[0] == 0x0C) ts.keySpeed = (uint8_t)raw;
      else if (pl[0] == 0x0A) ts.rfPower  = (uint8_t)raw;
    }
    if (cmd == 0x21 && plLen >= 4 && pl[0] == 0x00)
      ts.ritRaw = decodeCivBcdBytesLsb(pl + 1, 3);
    if (cmd == 0x1C && plLen >= 2 && pl[0] == 0x00)
      ts.tx = (pl[1] == 0x01);
    if (cmd == 0x04 && plLen >= 2)
      ts.filter = (uint8_t)pl[1];
    if (cmd == 0x11 && plLen >= 1) {
      ts.attOn = pl[0] ? 1 : 0;
      if (ts.attOn) ts.preampMode = 3;
      else if (ts.preampMode == 3) ts.preampMode = 0;
    }
    if (cmd == 0x16 && plLen >= 2) {
      if (pl[0] == 0x02 && !ts.attOn) ts.preampMode = pl[1];
      if (pl[0] == 0x47) ts.voxMode = pl[1];
    }
    // Synchronizovat stav aktivního TRX do globálních state vars
    if (isActive) {
      stateSmeterRaw    = ts.smeterRaw;
      statePowerMeterRaw = ts.powerMeterRaw;
      stateSwr          = ts.swr;
      stateSupplyVolts  = ts.supplyVolts;
      stateAfGain       = ts.afGain;
      stateKeySpeed     = ts.keySpeed;
      stateRfPower      = ts.rfPower;
      stateRitRaw       = ts.ritRaw;
      stateTx           = ts.tx;
      stateFilter       = ts.filter;
      statePreampMode   = ts.preampMode;
      stateAttOn        = ts.attOn;
      stateVoxMode      = ts.voxMode;
    }
  }
  ts.powerTimer = millis();
  if (isActive) powerTimer = ts.powerTimer;
}

// Vstupní bod pro LAN CI-V callback (s TRX indexem)
void processCivFrame(int trxIdx, const uint8_t* frame, size_t len) {
  if (len == 0 || len > sizeof(read_buffer)) return;
  memcpy(read_buffer, frame, len);
  processCivData(trxIdx, (uint8_t)len);
}

void processCatMessages(){
  #if defined(BLUETOOTH)
    int bti = btTrxIdx();
    if (bti < 0) return;
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

      processCivData(bti, len);
    }
  #endif
}

//-------------------------------------------------------------------------------------------------------
// call back to get info about connection
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  (void)param;
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    btClientConnected = true;
    radio_address = configuredCivAddress;
    // Přednastavit radioAddress v TrxState — jinak echo frame z IC-705 způsobí
    // špatnou auto-detekci (echo má read_buffer[3]=0xE0 = controller, ne radio adresu)
    // a skutečná odpověď s frekvencí pak filtrem neprojde.
    { int bti = btTrxIdx(); if (bti >= 0) trxSt[bti].radioAddress = configuredCivAddress; }
    frequency = 0;
    frequencyTmp = 0;
    if (TrxSetupDone == false) {
      TrxNeedSet = 1;
      if (cwIpOnConnect) cwIpSendPending = true;
    }

    Serial.println("    | Client Connected");
    btStateBroadcastPending = true;
  }

  if (event == ESP_SPP_CLOSE_EVT) {
    btClientConnected = false;
    radio_address = 0x00;
    // Reset radioAddress v TrxState pro čistý stav při příštím připojení
    { int bti = btTrxIdx(); if (bti >= 0) trxSt[bti].radioAddress = 0; }
    TrxNeedSet = 0;
    btDisconnectPending = true;
    Serial.println("    | Client Disconnected");
    btStateBroadcastPending = true;
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
  if (radio_address == 0x00) return;
  uint8_t req[] = {START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS, requestCode, STOP_BYTE};
  if(Debug==true){
    Serial.print("CAT TX >");
    for (uint8_t i = 0; i < sizeof(req); i++) {
      if (req[i] < 16) Serial.print("0");
      Serial.print(req[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  catWriteFrame(req, sizeof(req), true);
}
//-------------------------------------------------------------------------------------------------------
bool searchRadio(){
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
void printMode(void){
  #if defined(BLUETOOTH)
    uint8_t modeId = read_buffer[5];

    switch (modeId) {
      case 0x00: setModesText("LSB"); break;
      case 0x01: setModesText("USB"); break;
      case 0x02: setModesText("AM");  break;
      case 0x03: setModesText("CW");  break;
      case 0x04: setModesText("RTTY"); break;
      case 0x05: setModesText("FM");  break;
      case 0x06: setModesText("WFM"); break;
      case 0x17: setModesText("DV");  break;
      default:
        setModesText("UNK");
        if (Debug == true) {
          Serial.print("CAT | invalid/unknown mode id: 0x");
          if (modeId < 16) Serial.print("0");
          Serial.println(modeId, HEX);
        }
        return;
    }

    if (Debug == true) {
      char modesSnapshot[sizeof(modes)];
      copyModesText(modesSnapshot, sizeof(modesSnapshot));
      Serial.print("CAT | mode=");
      Serial.println(modesSnapshot);
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
    if(APmode) return;
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
        #if defined(WDT)
          esp_task_wdt_reset();
        #endif
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
	              char modesSnapshot[sizeof(modes)];
	              copyModesText(modesSnapshot, sizeof(modesSnapshot));
	              webCatClient.print(String(frequency)+"|"+String(modesSnapshot)+"|");
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
  char modesSnapshot[sizeof(modes)];
  copyModesText(modesSnapshot, sizeof(modesSnapshot));

  if(strcmp(modesSnapshot, "CW") == 0){  // CAT -----------------
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
  }else if(strcmp(modesSnapshot, "RTTY") == 0){ // GPIO FSK keying -----------------
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

    // B
    }else if(incomingByte==66 || incomingByte==98){
      #if defined(BLUETOOTH)
        Serial.println("   Clear all BT bonding data? (y/n)");
        EnterChar();
        if(incomingByte==89 || incomingByte==121){
          int numBonded = esp_bt_gap_get_bond_device_num();
          if(numBonded == 0){
            Serial.println("   No bonded devices found");
          } else {
            esp_bd_addr_t *bondedDevices = new esp_bd_addr_t[numBonded];
            esp_bt_gap_get_bond_device_list(&numBonded, bondedDevices);
            for(int i=0; i<numBonded; i++){
              esp_bt_gap_remove_bond_device(bondedDevices[i]);
              Serial.print("   Removed bond #");
              Serial.println(i+1);
            }
            delete[] bondedDevices;
            Serial.println("   BT bonding cleared - restart device and re-pair");
          }
        }else{
          Serial.println("   Aborted");
        }
      #else
        Serial.println("   BT not enabled");
      #endif

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
  Serial.print(" > ");
  unsigned long enterTimeout = millis();
  while (Serial.available() == 0) {
    #if defined(WDT)
      esp_task_wdt_reset();
    #endif
    if (millis() - enterTimeout > 30000) {
      Serial.println("(timeout)");
      return;
    }
    delay(10);
  }
  incomingByte = Serial.read();
  Serial.println( String(char(incomingByte)) );
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

    char modesSnapshot[sizeof(modes)];
    copyModesText(modesSnapshot, sizeof(modesSnapshot));
    Serial.println(" CAT "+String(frequency)+"Hz "+String(modesSnapshot) );
  }
  Serial.println("Commands  press key to select");
  Serial.println("       ?  list refresh");
  Serial.println("       A  restart to AP mode");
  #if defined(BLUETOOTH)
  Serial.println("       B  clear BT bonding data (use if re-pairing fails)");
  #endif
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

// setup form handler
void handleSet() {
  resetSetupMessages();
  bool ERRdetect=0;

  if ( requestHasArg("ssid") == false \
    && requestHasArg("pswd") == false \
  ) {
    // Serial.println("Form NOT valid");
  }else{
    // Serial.println("Form VALID");

    // 1-20 - SSID1*
    if ( requestArg("ssid").length()<1 || requestArg("ssid").length()>20){
      setupSsidErr = "Out of range 1-20 characters";
      ERRdetect=1;
    }else{
      String str = requestArg("ssid");
      if(SSID == str){
        setupSsidErr = "";
      }else{
        setupSsidErr = "Warning: SSID has changed.";
        SSID = requestArg("ssid");

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
    if ( requestArg("pswd").length()<1 || requestArg("pswd").length()>18){
      setupPswdErr = "Out of range 1-18 characters";
      ERRdetect=1;
    }else{
      String str = requestArg("pswd");
      if(PSWD == str){
        setupPswdErr = "";
      }else{
        setupPswdErr = "Warning: Password has changed.";
        PSWD = requestArg("pswd");

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
    if ( requestArg("ssid2").length()>20){
      setupSsid2Err = "Out of range 0-20 characters";
      ERRdetect=1;
    }else{
      String str = requestArg("ssid2");
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
    if ( requestArg("pswd2").length()>18){
      setupPswd2Err = "Out of range 0-18 characters";
      ERRdetect=1;
    }else{
      String str = requestArg("pswd2");
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
    if ( requestArg("httpcatport").length()<1 || requestArg("httpcatport").toInt()<1 || requestArg("httpcatport").toInt()>65534){
      setupHttpCatPortErr = "Out of range number 1-65534";
      ERRdetect=1;
    }else{
      if(HTTP_CAT_PORT == requestArg("httpcatport").toInt()){
        setupHttpCatPortErr = "";
      }else{
        setupHttpCatPortErr = "";
        HTTP_CAT_PORT = requestArg("httpcatport").toInt();
        EEPROM.writeUShort(68, HTTP_CAT_PORT);
      }
    }

    // 70-71 udpPort a 72-73 udpCatPort se synchronizují z BT TRX konfigurace níže

    // 41-44 mqttBroker[4]
    if ( requestArg("mqttip0").length()<1 || requestArg("mqttip0").toInt()>255){
      setupMqttErr1 = "IP1: Out of range 0-255";
      ERRdetect=1;
    }else{
      if(mqttBroker[0] == byte(requestArg("mqttip0").toInt()) ){
        setupMqttErr1 = "";
      }else{
        setupMqttErr1 = "";
        mqttBroker[0] = byte(requestArg("mqttip0").toInt()) ;
        EEPROM.writeByte(41, mqttBroker[0]);
      }
      if(mqttBroker[0]==0x00){
        mqttEnable = false;
      }else{
        mqttEnable = true;
      }
    }

    if ( requestArg("mqttip1").length()<1 || requestArg("mqttip1").toInt()>255){
      setupMqttErr2 = "IP2: Out of range 0-255";
      ERRdetect=1;
    }else{
      if(mqttBroker[1] == byte(requestArg("mqttip1").toInt()) ){
        setupMqttErr2 = "";
      }else{
        setupMqttErr2 = "";
        mqttBroker[1] = byte(requestArg("mqttip1").toInt()) ;
        EEPROM.writeByte(42, mqttBroker[1]);
      }
    }

    if ( requestArg("mqttip2").length()<1 || requestArg("mqttip2").toInt()>255){
      setupMqttErr3 = "IP3: Out of range 0-255";
      ERRdetect=1;
    }else{
      if(mqttBroker[2] == byte(requestArg("mqttip2").toInt()) ){
        setupMqttErr3 = "";
      }else{
        setupMqttErr3 = "";
        mqttBroker[2] = byte(requestArg("mqttip2").toInt()) ;
        EEPROM.writeByte(43, mqttBroker[2]);
      }
    }

    if ( requestArg("mqttip3").length()<1 || requestArg("mqttip3").toInt()>255){
      setupMqttErr4 = "IP4: Out of range 0-255";
      ERRdetect=1;
    }else{
      if(mqttBroker[3] == byte(requestArg("mqttip3").toInt()) ){
        setupMqttErr4 = "";
      }else{
        setupMqttErr4 = "";
        mqttBroker[3] = byte(requestArg("mqttip3").toInt()) ;
        EEPROM.writeByte(44, mqttBroker[3]);
      }
    }

    // 45-46 MQTT_PORT
    if ( requestArg("mqttport").length()<1 || requestArg("mqttport").toInt()<1 || requestArg("mqttport").toInt()>65534){
      setupMqttPortErr = "Out of range number 1-65534";
      ERRdetect=1;
    }else{
      if(MQTT_PORT == requestArg("mqttport").toInt()){
        setupMqttPortErr = "";
      }else{
        setupMqttPortErr = "";
        MQTT_PORT = requestArg("mqttport").toInt();
        EEPROM.writeUShort(45, MQTT_PORT);
      }
    }

    // 47-67 MQTT_TOPIC
    if ( requestArg("mqtttopic").length()<1 || requestArg("mqtttopic").length()>20){
      setupMqttTopicErr = "Out of range 1-20 characters";
      ERRdetect=1;
    }else{
      String str = requestArg("mqtttopic");
      if(MQTT_TOPIC == str){
        setupMqttTopicErr = "";
      }else{
        setupMqttTopicErr = "";
        MQTT_TOPIC = requestArg("mqtttopic");

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
    if ( requestArg("mqtttopicrx").length()>20){
      setupMqttTopicRxErr = "Out of range 0-20 characters";
      ERRdetect=1;
    }else{
      String str = requestArg("mqtttopicrx");
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
    switch (requestArg("baud").toInt()) {
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

    // Per-TRX konfigurace (TRX1–3)
    {
      bool btTaken = false;
      for (int t = 0; t < TRX_COUNT; t++) {
        int n = t + 1;
        String pfx = "trx" + String(n);
        String kmode  = pfx + "mode";
        String klabel = pfx + "label";
        String kip    = pfx + "ip";
        String kport  = pfx + "port";
        String kuser  = pfx + "user";
        String kpass  = pfx + "pass";
        String kciv   = pfx + "civ";
        String kudpcw = pfx + "udpcw";
        String kudpct = pfx + "udpcat";

        int modeRaw = requestArg(kmode.c_str()).toInt();
        uint8_t mode = (uint8_t)(modeRaw < 0 ? 0 : (modeRaw > (int)TRX_MODE_OI3 ? (int)TRX_MODE_OI3 : modeRaw));
        if (mode == TRX_MODE_BT && btTaken) mode = TRX_MODE_DISABLED;
        if (mode == TRX_MODE_BT) btTaken = true;
        trxCfg[t].mode = mode;

        String lbl = requestArg(klabel.c_str());
        if (lbl.length() > 11) lbl = lbl.substring(0, 11);
        lbl.toCharArray(trxCfg[t].label, sizeof(trxCfg[t].label));

        if (requestHasArg(kip.c_str())) {
          String ipStr = requestArg(kip.c_str());
          int b[4] = {0, 0, 0, 0};
          sscanf(ipStr.c_str(), "%d.%d.%d.%d", &b[0], &b[1], &b[2], &b[3]);
          for (int j = 0; j < 4; j++) trxCfg[t].ip[j] = (uint8_t)b[j];
        }

        if (requestHasArg(kport.c_str())) {
          int lp = requestArg(kport.c_str()).toInt();
          if (lp > 0 && lp <= 65534) trxCfg[t].port = (uint16_t)lp;
        }

        if (requestHasArg(kuser.c_str())) {
          String usr = requestArg(kuser.c_str());
          if (usr.length() > 16) usr = usr.substring(0, 16);
          usr.toCharArray(trxCfg[t].username, sizeof(trxCfg[t].username));
        }

        if (requestHasArg(kpass.c_str())) {
          String pw = requestArg(kpass.c_str());
          if (pw.length() > 0) {
            if (pw.length() > 16) pw = pw.substring(0, 16);
            pw.toCharArray(trxCfg[t].password, sizeof(trxCfg[t].password));
          }
        }

        if (requestHasArg(kciv.c_str()) && requestArg(kciv.c_str()).length() > 0) {
          uint8_t civ = trxCfg[t].civAddr;
          if (parseHexByteString(requestArg(kciv.c_str()), civ)) trxCfg[t].civAddr = civ;
        }

        if (requestHasArg(kudpcw.c_str())) {
          int cw = requestArg(kudpcw.c_str()).toInt();
          if (cw > 0 && cw <= 65534) trxCfg[t].udpCwPort = (uint16_t)cw;
        }
        if (requestHasArg(kudpct.c_str())) {
          int ct = requestArg(kudpct.c_str()).toInt();
          if (ct > 0 && ct <= 65534) trxCfg[t].udpCatPort = (uint16_t)ct;
        }

        saveTrxConfig(t, trxCfg[t]);
      }
      // Aktualizovat globální aliasy
      transceiverType      = trxCfg[0].modeStr();
      configuredCivAddress = trxCfg[0].effectiveCivAddr();
      lanMode = false;
      for (int t = 0; t < TRX_COUNT; t++) if (trxCfg[t].isLan()) lanMode = true;
      // Synchronizovat globální UDP porty z BT TRX
      int bi = btTrxIdx();
      if (bi >= 0) {
        udpPort    = trxCfg[bi].udpCwPort;
        udpCatPort = trxCfg[bi].udpCatPort;
        EEPROM.writeUShort(70, udpPort);
        EEPROM.writeUShort(72, udpCatPort);
      }
    }

    cwIpOnConnect = requestHasArg("cwIpOnConnect");
    EEPROM.writeBool(136, cwIpOnConnect);

    for (uint8_t i = 0; i < CW_MEMORY_COUNT; i++) {
      char fieldName[10];
      snprintf(fieldName, sizeof(fieldName), "cwmem%u", i + 1);
      cwMemoryText[i] = trimMemoryValue(requestArg(fieldName), CW_MEMORY_MAX_LEN);
    }

    for (uint8_t i = 0; i < FREQ_MEMORY_COUNT; i++) {
      char fieldName[12];
      snprintf(fieldName, sizeof(fieldName), "freqmem%u", i + 1);
      freqMemoryText[i] = trimMemoryValue(requestArg(fieldName), FREQ_MEMORY_MAX_LEN);
    }

    saveMemoryConfig();


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
}

#if defined(RTLE)
  void handleRTLE() {
    String HtmlSrc = "";
    String s = RTLE_page; //Read HTML contents
    HtmlSrc +=s;
    rtleserver.send(200, "text/html", HtmlSrc); //Send web page
  }
#endif
