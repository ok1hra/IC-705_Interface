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
3. generate all .bin and publish to GitHub web page: $ ./tools/gh-pages.sh --publish
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
String MQTT_TOPIC        = "";
String MQTT_TOPIC_RX     = "";
String TRX2_MQTT_ROOT    = "";
String TRX3_MQTT_ROOT    = "";
int HTTP_CAT_PORT   = 0;
int udpPort         = 0; // UDP port | echo -n "cq de ok1hra ok1hra test k;" | nc -u -w1 192.168.1.19 89
int udpCatPort      = 0;
int BaudRate        = 9600;
// char* BTname        = "";
// const char* BTname  = "IC705-interface";
String BT_NAME;  // loaded from EEPROM; default IC705-XXXXXX from MAC
bool Debug          = false;
bool cwIpOnConnect  = true;       // announce WiFi IP via CW on first BT connect
volatile bool cwIpSendPending = false;

#define REV 20260517
#define WIFI
#define MQTT
#define UDP_TO_CW
#define UDP_TO_FSK
#define HTTP        // http propagation freq|mode|
#define WDT         // watchdog timer
#define CIV_OUT     // send freq to CIV out with BaudRate
#define UDP_TO_CAT  // data command from udpCatPort send to TRX | FE FE A4 E0 <command> FD
#define BLUETOOTH   // BT
// #define RTLE     // not work now | credit OK2CQR https://github.com/ok2cqr/rtle/tree/master
// #define RESET_AFTER_DISCONNECT  // enable reset after each disconnect + short CW msg

#include "EEPROM.h"
#define EEPROM_SIZE 288
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
  136 cwIpOnConnect
  137-200 DXC host (64B)
  201-202 DXC port (UShort)
  203-218 DXC callsign (16B)
  219-224 DXC locator (6B)
  225-245 TRX2 MQTT root topic (21B)
  246-266 TRX3 MQTT root topic (21B)
  267-287 BT_NAME (21B)

  !! Increment EEPROM_SIZE #define !!
*/

#if defined(BLUETOOTH)
  #include "BluetoothSerial.h"
  #include <esp_bt.h>
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
  volatile bool btClientConnected = false;
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
volatile bool btConnectPending = false;
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
  WiFiServer dxcRawServer(82);
  WiFiClient DxcTelnetClient;
  WiFiClient DxcWsClient;
  String DxcHost = "";
  uint16_t DxcPort = 7300;
  String DxcCallsign = "";
  String DxcLocator = "";

  static const char* LOG_CONFIG_PATH = "/log-config.json";

  // In-memory cache of log-config.json fields used by setupTemplateProcessor.
  String g_lcTrx1Label = "IC-705";
  String g_lcTrx2Label = "TRX2";
  String g_lcTrx3Label = "TRX3";
  String g_lcTrx2Civ   = "";
  String g_lcTrx3Civ   = "";
  String g_lcTrx2Ip    = "";
  String g_lcTrx3Ip    = "";
  bool   g_lcTrx2Oi3   = false;
  bool   g_lcTrx3Oi3   = false;
  String g_lcRstSsb    = "59";
  String g_lcRstCwRtty = "599";
  bool   g_lcManualModeForPhone = true;
  String g_lcBlockedDxcc = "";

  // OI3 MQTT cache for TRX2 (index 0) and TRX3 (index 1) — written by MqttRx, read by handleOi3State
  static volatile long g_mqttFreq[2]    = {0, 0};
  static char          g_mqttMode[2][8] = {"USB", "USB"};
  static volatile bool g_mqttHasData[2] = {false, false};

// Band Decoder
#define BD_CLOCK_PIN  15
#define BD_LATCH_PIN  13
#define BD_DATA_PIN   14
#define BD_ROWS       16
#define BD_CONFIG_PATH "/bd-config.json"

struct BdRow {
    uint32_t fMin;
    uint32_t fMax;
    uint16_t outputs;
};

BdRow     bdRows[BD_ROWS];
int       bdSource         = 1;
uint16_t  bdCurrentOutputs = 0;
bool      bdEnabled        = false;

  bool DxcTelnetStatus = false;
  bool DxcWsStatus = false;
  bool DxcTelnetLoginPending = false;
  unsigned long DxcReconnectTimer = 0;
#endif

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
  String setupUdpPortErr = "";
  String setupUdpCatPortErr = "";
  String setupCivAddrErr = "";
  bool setupSaveOk = false;
  String transceiverType = "IC-705-BT";
  uint8_t configuredCivAddress = CIV_ADDRESS_DEFAULT;
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
  String jsonEscape(const String &value);
  String civFrameToHex(const uint8_t *frame, size_t frameLen);
  uint32_t decodeCivFrequencyBytes(const uint8_t *bytes, size_t byteCount);
  uint32_t decodeCivBcdBytes(const uint8_t *bytes, size_t byteCount);
  String decodeModeName(uint8_t modeId);
  bool catWriteFrame(const uint8_t *frame, size_t frameLen, bool broadcastTx);
  void setModesText(const char *value);
  void copyModesText(char *dest, size_t destSize);
  String extractJsonString(const String &json, const char *key);
  String extractJsonObject(const String &json, const char *key);
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
  bool handleFileFromSPIFFS(const String &path);
  void handlePostCmd(void);
  void handleSet(void);
  void renderSetupPage(void);
  void renderIndexPage(void);
  void resetSetupMessages(void);
  String setupTemplateProcessor(const String &key);
  void setupWebServer(void);
  void handleConfigDownload(void);
  void handleConfigUpload(void);
  void handleGetLogConfig(void);
  void handlePostLogConfig(void);
  void handleOi3State(void);
  void handleOi3Send(void);
  void DxcLoop(void);
  void dxcHandleRawClient(void);
  bool DxcConfigReady(void);
  void DxcDisconnectTelnet(void);
  void DxcDisconnectWebSocket(void);
  void DxcRequestReconnect(void);
  void DxcUpdateTelnetStatus(bool connected, bool forceSend = false);
  void DxcSendTelnetStatus(void);
  bool DxcConnectTelnet(void);
  bool DxcSendWebSocketFrame(uint8_t opcode, const uint8_t* payload, size_t length);
  bool DxcSendWebSocketText(const char* text);
  bool DxcSendWebSocketText(const String& text);
  bool DxcHandleWebSocketUpgrade(WiFiClient& webClient, const String& request);
  void DxcHandleWebSocketClient(void);
  void DxcHandleTelnetClient(void);
  String ExtractHttpHeader(const String& request, const String& headerName);
  String DxcComputeWebSocketAccept(const String& secKey);
  String Base64Encode(const uint8_t* data, size_t length);
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
    if (configuredType == "IC-7610-CI-V") {
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
  File file = SPIFFS.open(MEMORY_CONFIG_PATH, "w");
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

// Parse log-config.json into g_lc* variables so setupTemplateProcessor can serve them.
void loadLogConfigVars(void){
  if (!SPIFFS.exists(LOG_CONFIG_PATH)) return;
  File f = SPIFFS.open(LOG_CONFIG_PATH, "r");
  if (!f) return;
  String j = f.readString();
  f.close();
  j.trim();
  if (!j.startsWith("{")) return;

  String v;
  v = extractJsonString(j, "trx1Label"); if (v.length() > 0) g_lcTrx1Label = v;
  v = extractJsonString(j, "trx2Label"); if (v.length() > 0) g_lcTrx2Label = v;
  v = extractJsonString(j, "trx3Label"); if (v.length() > 0) g_lcTrx3Label = v;
  v = extractJsonString(j, "trx2Civ");   g_lcTrx2Civ = v;
  v = extractJsonString(j, "trx3Civ");   g_lcTrx3Civ = v;
  v = extractJsonString(j, "trx2Ip");    g_lcTrx2Ip  = v;
  v = extractJsonString(j, "trx3Ip");    g_lcTrx3Ip  = v;
  v = extractJsonString(j, "rstSsb");    if (v.length() > 0) g_lcRstSsb = v;
  v = extractJsonString(j, "rstCwRtty"); if (v.length() > 0) g_lcRstCwRtty = v;
  v = extractJsonString(j, "blockedDxcc"); g_lcBlockedDxcc = v;
  g_lcManualModeForPhone = extractJsonBool(j, "manualModeForPhone", true);
  // booleans
  int idx2 = j.indexOf("\"trx2Oi3\"");
  if (idx2 >= 0) { int c = j.indexOf(':', idx2)+1; while(c<(int)j.length()&&j[c]==' ')c++; g_lcTrx2Oi3 = j.substring(c,c+4)=="true"; }
  int idx3 = j.indexOf("\"trx3Oi3\"");
  if (idx3 >= 0) { int c = j.indexOf(':', idx3)+1; while(c<(int)j.length()&&j[c]==' ')c++; g_lcTrx3Oi3 = j.substring(c,c+4)=="true"; }
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
        switch (c) {
          case 'n':  out += '\n'; break;
          case 'r':  out += '\r'; break;
          case 't':  out += '\t'; break;
          case '\\': out += '\\'; break;
          case '"':  out += '"';  break;
          default:   out += c;    break;
        }
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

static String readLogConfigJson() {
  if (!SPIFFS.exists(LOG_CONFIG_PATH)) {
    return String();
  }
  File f = SPIFFS.open(LOG_CONFIG_PATH, "r");
  if (!f) {
    return String();
  }
  String json = f.readString();
  f.close();
  json.trim();
  return json;
}

static String buildLogConfigJson(
  const String &existingJson,
  const String &trx1Label,
  const String &trx2Label,
  const String &trx3Label,
  const String &trx2Civ,
  const String &trx3Civ,
  const String &trx2Ip,
  const String &trx3Ip,
  bool trx2Oi3,
  bool trx3Oi3,
  const String &rstSsb,
  const String &rstCwRtty,
  bool manualModeForPhone,
  const String &blockedDxcc
) {
  String json;
  json.reserve(512);
  json += "{";
  json += "\"trx1Label\":\""; json += jsonEscape(trx1Label); json += "\"";
  json += ",\"trx2Label\":\""; json += jsonEscape(trx2Label); json += "\"";
  json += ",\"trx3Label\":\""; json += jsonEscape(trx3Label); json += "\"";
  json += ",\"trx2Civ\":\"";   json += jsonEscape(trx2Civ);   json += "\"";
  json += ",\"trx3Civ\":\"";   json += jsonEscape(trx3Civ);   json += "\"";
  json += ",\"trx2Ip\":\"";    json += jsonEscape(trx2Ip);    json += "\"";
  json += ",\"trx3Ip\":\"";    json += jsonEscape(trx3Ip);    json += "\"";
  json += ",\"trx2Oi3\":";     json += trx2Oi3 ? "true" : "false";
  json += ",\"trx3Oi3\":";     json += trx3Oi3 ? "true" : "false";
  json += ",\"rstSsb\":\"";    json += jsonEscape(rstSsb);    json += "\"";
  json += ",\"rstCwRtty\":\""; json += jsonEscape(rstCwRtty); json += "\"";
  json += ",\"manualModeForPhone\":"; json += manualModeForPhone ? "true" : "false";
  json += ",\"blockedDxcc\":\""; json += jsonEscape(blockedDxcc); json += "\"";
  json += "}";
  return json;
}

static bool saveLogConfigJson(const String &json) {
  File f = SPIFFS.open(LOG_CONFIG_PATH, "w");
  if (!f) {
    return false;
  }
  f.print(json);
  f.close();
  loadLogConfigVars();
  return true;
}

String extractJsonObject(const String &json, const char *key) {
  String token = "\"" + String(key) + "\":";
  int start = json.indexOf(token);
  if (start < 0) return String();
  start += token.length();
  while (start < (int)json.length() && (json[start] == ' ' || json[start] == '\t')) start++;
  if (start >= (int)json.length() || json[start] != '{') return String();
  int depth = 0, end = start;
  for (; end < (int)json.length(); end++) {
    if (json[end] == '{') depth++;
    else if (json[end] == '}') { if (--depth == 0) { end++; break; } }
  }
  return json.substring(start, end);
}

int extractJsonInt(const String &j, const String &key) {
  String needle = "\"" + key + "\":";
  int idx = j.indexOf(needle);
  if (idx < 0) return 0;
  return j.substring(idx + needle.length()).toInt();
}

void bdLoadDefaults() {
  const uint32_t fMin[BD_ROWS] = {
    1810, 3500, 5351, 7000, 10100, 14000, 18068,
    21000, 24890, 28000, 50000, 70000, 144000, 430000, 0, 0
  };
  const uint32_t fMax[BD_ROWS] = {
    2000, 3800, 5367, 7200, 10150, 14350, 18168,
    21450, 24990, 29700, 54000, 70500, 146000, 440000, 0, 0
  };
  for (int i = 0; i < BD_ROWS; i++) {
    bdRows[i].fMin    = fMin[i];
    bdRows[i].fMax    = fMax[i];
    bdRows[i].outputs = (i < 14) ? (uint16_t)(1 << i) : 0;
  }
  bdSource = 1;
}

void bdLoadConfig() {
  if (!SPIFFS.exists(BD_CONFIG_PATH)) { bdLoadDefaults(); return; }
  File f = SPIFFS.open(BD_CONFIG_PATH, FILE_READ);
  if (!f) { bdLoadDefaults(); return; }
  String j = f.readString();
  f.close();
  int si = j.indexOf("\"source\":");
  if (si >= 0) bdSource = j.substring(si + 9).toInt();
  int arrStart = j.indexOf("\"rows\":[");
  if (arrStart < 0) { bdLoadDefaults(); return; }
  arrStart += 8;
  for (int i = 0; i < BD_ROWS; i++) {
    int ob = j.indexOf('{', arrStart);
    int cb = j.indexOf('}', ob);
    if (ob < 0 || cb < 0) break;
    String row = j.substring(ob, cb + 1);
    bdRows[i].fMin    = (uint32_t)extractJsonInt(row, "fMin");
    bdRows[i].fMax    = (uint32_t)extractJsonInt(row, "fMax");
    bdRows[i].outputs = (uint16_t)extractJsonInt(row, "outputs");
    arrStart = cb + 1;
  }
}

void bdSaveConfig() {
  File f = SPIFFS.open(BD_CONFIG_PATH, "w");
  if (!f) return;
  f.print("{\"source\":"); f.print(bdSource);
  f.print(",\"rows\":[");
  for (int i = 0; i < BD_ROWS; i++) {
    if (i > 0) f.print(",");
    f.print("{\"fMin\":"); f.print(bdRows[i].fMin);
    f.print(",\"fMax\":"); f.print(bdRows[i].fMax);
    f.print(",\"outputs\":"); f.print(bdRows[i].outputs);
    f.print("}");
  }
  f.print("]}");
  f.close();
}

void bdWriteOutputs(uint16_t mask) {
  uint8_t hi = (mask >> 8) & 0xFF;
  uint8_t lo = mask & 0xFF;
  digitalWrite(BD_LATCH_PIN, LOW);
  shiftOut(BD_DATA_PIN, BD_CLOCK_PIN, MSBFIRST, hi);
  shiftOut(BD_DATA_PIN, BD_CLOCK_PIN, MSBFIRST, lo);
  digitalWrite(BD_LATCH_PIN, HIGH);
  bdCurrentOutputs = mask;
}

void bdInit() {
  if (!bdEnabled) return;
  pinMode(BD_CLOCK_PIN, OUTPUT); digitalWrite(BD_CLOCK_PIN, LOW);
  pinMode(BD_LATCH_PIN, OUTPUT); digitalWrite(BD_LATCH_PIN, HIGH);
  pinMode(BD_DATA_PIN,  OUTPUT); digitalWrite(BD_DATA_PIN,  LOW);
  bdWriteOutputs(0);
}

void bdUpdate(uint32_t freqHz) {
  if (!bdEnabled) return;
  uint32_t freqKhz = freqHz / 1000;
  uint16_t mask = 0;
  for (int i = 0; i < BD_ROWS; i++) {
    if (bdRows[i].fMin == 0 && bdRows[i].fMax == 0) continue;
    if (freqKhz >= bdRows[i].fMin && freqKhz <= bdRows[i].fMax) {
      mask |= bdRows[i].outputs;
    }
  }
  if (mask != bdCurrentOutputs) bdWriteOutputs(mask);
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
  setupUdpPortErr = "";
  setupUdpCatPortErr = "";
  setupCivAddrErr = "";
  setupSaveOk = false;
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
  if (key == "TRX2_MQTT_ROOT") return TRX2_MQTT_ROOT;
  if (key == "TRX3_MQTT_ROOT") return TRX3_MQTT_ROOT;
  if (key == "TRX1_LABEL") return g_lcTrx1Label;
  if (key == "TRX2_LABEL") return g_lcTrx2Label;
  if (key == "TRX3_LABEL") return g_lcTrx3Label;
  if (key == "TRX2_CIV")   return g_lcTrx2Civ;
  if (key == "TRX3_CIV")   return g_lcTrx3Civ;
  if (key == "TRX2_IP")    return g_lcTrx2Ip;
  if (key == "TRX3_IP")    return g_lcTrx3Ip;
  if (key == "TRX2_OI3_CHK") return g_lcTrx2Oi3 ? "checked" : "";
  if (key == "TRX3_OI3_CHK") return g_lcTrx3Oi3 ? "checked" : "";
  if (key == "TRX2_OI3_STYLE") return g_lcTrx2Oi3 ? "display:block" : "display:none";
  if (key == "TRX3_OI3_STYLE") return g_lcTrx3Oi3 ? "display:block" : "display:none";
  if (key == "RST_SSB") return g_lcRstSsb;
  if (key == "RST_CW_RTTY") return g_lcRstCwRtty;
  if (key == "MANUAL_MODE_FOR_PHONE_CHK") return g_lcManualModeForPhone ? "checked" : "";
  if (key == "BLOCKED_DXCC") return g_lcBlockedDxcc;
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
  if (key == "UDP_PORT_ERR") return setupUdpPortErr;
  if (key == "UDP_CAT_PORT_ERR") return setupUdpCatPortErr;
  if (key == "CIV_ADDR") {
    String addr = String(configuredCivAddress, HEX);
    addr.toUpperCase();
    if (configuredCivAddress < 16) {
      addr = "0" + addr;
    }
    return addr;
  }
  if (key == "CIV_ADDR_ERR") return setupCivAddrErr;
  if (key == "TRX_IC705_SEL") return transceiverType == "IC-705-BT" ? "selected" : "";
  if (key == "TRX_IC7610_SEL") return transceiverType == "IC-7610-CI-V" ? "selected" : "";
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
  if (key == "DXC_HOST") return DxcHost;
  if (key == "DXC_PORT") return DxcPort > 0 ? String(DxcPort) : "";
  if (key == "DXC_CALL") return DxcCallsign;
  if (key == "DXC_LOCATOR") return DxcLocator;
  if (key == "BT_NAME") return BT_NAME;
  return String();
}

void renderSetupPage(){
  if (!SPIFFS.exists("/setup.html")) {
    webServer.send(500, "text/plain", "Missing /setup.html in SPIFFS");
    return;
  }
  File file = SPIFFS.open("/setup.html", "r");
  if (!file) { webServer.send(500, "text/plain", "File open failed"); return; }
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(200, "text/html", "");
  String buf;
  buf.reserve(1536);
  while (file.available()) {
    #if defined(WDT)
      esp_task_wdt_reset();
    #endif
    #if defined(MQTT)
      if (mqttEnable && mqttClient.connected()) mqttClient.loop();
    #endif
    String line = file.readStringUntil('\n');
    int start = 0;
    while (true) {
      int p1 = line.indexOf('%', start);
      if (p1 < 0) { buf += line.substring(start); break; }
      int p2 = line.indexOf('%', p1 + 1);
      if (p2 < 0) { buf += line.substring(start); break; }
      buf += line.substring(start, p1);
      buf += setupTemplateProcessor(line.substring(p1 + 1, p2));
      start = p2 + 1;
    }
    buf += '\n';
    if (buf.length() >= 1024) { webServer.sendContent(buf); buf = ""; }
  }
  file.close();
  if (buf.length() > 0) webServer.sendContent(buf);
  webServer.sendContent("");
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
  String buf;
  buf.reserve(1536);
  while (file.available()) {
    #if defined(WDT)
      esp_task_wdt_reset();
    #endif
    #if defined(MQTT)
      if (mqttEnable && mqttClient.connected()) mqttClient.loop();
    #endif
    String line = file.readStringUntil('\n');
    int start = 0;
    while (true) {
      int p1 = line.indexOf('%', start);
      if (p1 < 0) { buf += line.substring(start); break; }
      int p2 = line.indexOf('%', p1 + 1);
      if (p2 < 0) { buf += line.substring(start); break; }
      buf += line.substring(start, p1);
      buf += setupTemplateProcessor(line.substring(p1 + 1, p2));
      start = p2 + 1;
    }
    buf += '\n';
    if (buf.length() >= 1024) { webServer.sendContent(buf); buf = ""; }
  }
  file.close();
  if (buf.length() > 0) webServer.sendContent(buf);
  webServer.sendContent("");
}

void buildStateJson(char *buf, size_t bufSize){
  char modesSnapshot[sizeof(modes)];
  copyModesText(modesSnapshot, sizeof(modesSnapshot));
  char addrStr[5];
  snprintf(addrStr, sizeof(addrStr), "0x%02X", radio_address);
  const char *btStat = !btClientConnected ? "BT idle" :
                       (radio_address == 0x00 ? "BT linked | searching CI-V" : "BT linked");
  const char *wifiStat = APmode ? "WiFi AP" :
                         (WiFi.status() == WL_CONNECTED ? "WiFi STA" : "WiFi down");
  int rssi = (APmode || WiFi.status() != WL_CONNECTED) ? -999 : (int)WiFi.RSSI();
  snprintf(buf, bufSize,
    "{\"connected\":%s,\"btStatus\":\"%s\",\"wifiStatus\":\"%s\","
    "\"wifiRssi\":%d,\"fwRev\":\"%u\",\"power\":%s,"
    "\"frequency\":%u,\"mode\":\"%s\",\"filter\":%u,"
    "\"radioAddress\":\"%s\",\"transceiverType\":\"%s\",\"tx\":%s,\"ritRaw\":%u,"
    "\"smeterRaw\":%u,\"powerMeterRaw\":%u,"
    "\"afGain\":%u,\"keySpeed\":%u,\"rfPower\":%u,"
    "\"supplyVolts\":%.2f,\"swr\":%.2f,"
    "\"preamp\":%u,\"vox\":%u,\"dxcConnected\":%s}",
    btClientConnected ? "true" : "false", btStat, wifiStat,
    rssi, (unsigned)REV, statusPower ? "true" : "false",
    (unsigned)frequency, modesSnapshot, (unsigned)stateFilter,
    addrStr, transceiverType.c_str(), stateTx ? "true" : "false", (unsigned)stateRitRaw,
    (unsigned)stateSmeterRaw, (unsigned)statePowerMeterRaw,
    (unsigned)stateAfGain, (unsigned)stateKeySpeed, (unsigned)stateRfPower,
    stateSupplyVolts, stateSwr,
    (unsigned)statePreampMode, (unsigned)stateVoxMode,
    DxcTelnetStatus ? "true" : "false"
  );
}

void handleGetState(){
  static char stateBuf[640];
  buildStateJson(stateBuf, sizeof(stateBuf));
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send(200, "application/json", stateBuf);
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
  bool isStatic = false;
  if (path.endsWith(".html")) contentType = "text/html";
  else if (path.endsWith(".css"))  { contentType = "text/css";                  isStatic = true; }
  else if (path.endsWith(".js"))   { contentType = "application/javascript";    isStatic = true; }
  else if (path.endsWith(".ico"))  { contentType = "image/x-icon";              isStatic = true; }
  else if (path.endsWith(".png"))  { contentType = "image/png";                 isStatic = true; }
  // Prefer pre-compressed .gz variant if available
  String gzPath = path + ".gz";
  bool useGz = SPIFFS.exists(gzPath);
  String servePath = useGz ? gzPath : path;
  if (!useGz && !SPIFFS.exists(path)) return false;
  File f = SPIFFS.open(servePath, "r");
  if (!f) return false;
  if (isStatic) {
    webServer.sendHeader("Cache-Control", "public, max-age=3600");
  } else {
    webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    webServer.sendHeader("Pragma", "no-cache");
  }
  if (useGz) webServer.sendHeader("Content-Encoding", "gzip");
  webServer.setContentLength(f.size());
  webServer.send(200, contentType, "");
  static uint8_t buf[1024];
  while (f.available()) {
    #if defined(WDT)
      esp_task_wdt_reset();
    #endif
    #if defined(MQTT)
      if (mqttEnable && mqttClient.connected()) mqttClient.loop();
    #endif
    size_t n = f.read(buf, sizeof(buf));
    if (n > 0) webServer.sendContent((const char*)buf, n);
  }
  f.close();
  return true;
}

void handlePostCmd(){
  String body = webServer.arg("plain");
  if (body.length() == 0) { webServer.send(400, "application/json", "{\"error\":\"empty body\"}"); return; }
  String type = extractJsonString(body, "type");

  if (!btClientConnected || radio_address == 0x00) {
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

void handleConfigDownload() {
  char civHex[3];
  snprintf(civHex, sizeof(civHex), "%02X", configuredCivAddress);
  String j;
  j.reserve(2048);
  j += "{\"fwRev\":";           j += (unsigned)REV;
  j += ",\"ssid\":\"";          j += configJsonEscape(SSID);          j += "\"";
  j += ",\"pswd\":\"";          j += configJsonEscape(PSWD);          j += "\"";
  j += ",\"ssid2\":\"";         j += configJsonEscape(SSID2);         j += "\"";
  j += ",\"pswd2\":\"";         j += configJsonEscape(PSWD2);         j += "\"";
  j += ",\"mqttip0\":";         j += mqttBroker[0];
  j += ",\"mqttip1\":";         j += mqttBroker[1];
  j += ",\"mqttip2\":";         j += mqttBroker[2];
  j += ",\"mqttip3\":";         j += mqttBroker[3];
  j += ",\"mqttport\":";        j += MQTT_PORT;
  j += ",\"mqtttopic\":\"";      j += configJsonEscape(MQTT_TOPIC);        j += "\"";
  j += ",\"mqtttopicrx\":\"";   j += configJsonEscape(MQTT_TOPIC_RX);     j += "\"";
  j += ",\"trx2mqttroot\":\"";  j += configJsonEscape(TRX2_MQTT_ROOT);    j += "\"";
  j += ",\"trx3mqttroot\":\"";  j += configJsonEscape(TRX3_MQTT_ROOT);    j += "\"";
  j += ",\"httpcatport\":";     j += HTTP_CAT_PORT;
  j += ",\"udpport\":";         j += udpPort;
  j += ",\"udpcatport\":";      j += udpCatPort;
  j += ",\"baudrate\":";        j += BaudRate;
  j += ",\"trxprofile\":\"";    j += configJsonEscape(transceiverType); j += "\"";
  j += ",\"civaddr\":\"";       j += civHex;                          j += "\"";
  j += ",\"cwIpOnConnect\":";  j += cwIpOnConnect ? "true" : "false";
  for (uint8_t i = 0; i < CW_MEMORY_COUNT; i++) {
    j += ",\"cwmem"; j += (i + 1); j += "\":\""; j += configJsonEscape(cwMemoryText[i]); j += "\"";
  }
  for (uint8_t i = 0; i < FREQ_MEMORY_COUNT; i++) {
    j += ",\"freqmem"; j += (i + 1); j += "\":\""; j += configJsonEscape(freqMemoryText[i]); j += "\"";
  }
  j += ",\"dxchost\":\"";    j += configJsonEscape(DxcHost);     j += "\"";
  j += ",\"dxcport\":";      j += DxcPort;
  j += ",\"dxccall\":\"";    j += configJsonEscape(DxcCallsign); j += "\"";
  j += ",\"dxclocator\":\""; j += configJsonEscape(DxcLocator);  j += "\"";
  j += ",\"btname\":\"";     j += configJsonEscape(BT_NAME);     j += "\"";
  String lc = readLogConfigJson();
  if (lc.startsWith("{")) {
    j += ",\"logConfig\":";
    j += lc;
  }
  if (bdEnabled) {
    j += ",\"bd\":{\"source\":"; j += bdSource;
    j += ",\"rows\":[";
    for (int i = 0; i < BD_ROWS; i++) {
      if (i > 0) j += ",";
      j += "{\"fMin\":"; j += bdRows[i].fMin;
      j += ",\"fMax\":"; j += bdRows[i].fMax;
      j += ",\"outputs\":"; j += bdRows[i].outputs;
      j += "}";
    }
    j += "]}";
  }
  j += "}";
  webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  webServer.sendHeader("Pragma", "no-cache");
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

  if (body.indexOf("\"ssid2\"") >= 0) {
    String ssid2 = extractJsonString(body, "ssid2");
    if (ssid2.length() <= 20) { SSID2 = ssid2; eepromWriteStr(SSID2, 76, 20); }
  }

  if (body.indexOf("\"pswd2\"") >= 0) {
    String pswd2 = extractJsonString(body, "pswd2");
    if (pswd2.length() <= 18) { PSWD2 = pswd2; eepromWriteStr(PSWD2, 97, 18); }
  }

  String mqtttopic = extractJsonString(body, "mqtttopic");
  if (mqtttopic.length() >= 1 && mqtttopic.length() <= 20) { MQTT_TOPIC = mqtttopic; eepromWriteStr(MQTT_TOPIC, 47, 20); }

  if (body.indexOf("\"mqtttopicrx\"") >= 0) {
    String mqtttopicrx = extractJsonString(body, "mqtttopicrx");
    if (mqtttopicrx.length() <= 20) { MQTT_TOPIC_RX = mqtttopicrx; eepromWriteStr(MQTT_TOPIC_RX, 115, 21); }
  }

  if (body.indexOf("\"trx2mqttroot\"") >= 0) {
    String trx2mqttroot = extractJsonString(body, "trx2mqttroot");
    if (trx2mqttroot.length() <= 20) { TRX2_MQTT_ROOT = trx2mqttroot; eepromWriteStr(TRX2_MQTT_ROOT, 225, 21); }
  }

  if (body.indexOf("\"trx3mqttroot\"") >= 0) {
    String trx3mqttroot = extractJsonString(body, "trx3mqttroot");
    if (trx3mqttroot.length() <= 20) { TRX3_MQTT_ROOT = trx3mqttroot; eepromWriteStr(TRX3_MQTT_ROOT, 246, 21); }
  }

  String trx = extractJsonString(body, "trxprofile");
  if (trx == "IC-7610-CI-V") transceiverType = trx;
  else if (trx.length() > 0) transceiverType = "IC-705-BT";

  String civStr = extractJsonString(body, "civaddr");
  uint8_t civAddr = configuredCivAddress;
  if (parseHexByteString(civStr, civAddr)) configuredCivAddress = civAddr;

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


  // Integers to EEPROM
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

  int v;
  v = parseField("mqttip0", 0, 255); if (v >= 0) { mqttBroker[0] = v; EEPROM.writeByte(41, v); mqttEnable = (v != 0); }
  v = parseField("mqttip1", 0, 255); if (v >= 0) { mqttBroker[1] = v; EEPROM.writeByte(42, v); }
  v = parseField("mqttip2", 0, 255); if (v >= 0) { mqttBroker[2] = v; EEPROM.writeByte(43, v); }
  v = parseField("mqttip3", 0, 255); if (v >= 0) { mqttBroker[3] = v; EEPROM.writeByte(44, v); }
  v = parseField("mqttport", 1, 65534); if (v >= 0) { MQTT_PORT = v; EEPROM.writeUShort(45, v); }
  v = parseField("httpcatport", 1, 65534); if (v >= 0) { HTTP_CAT_PORT = v; EEPROM.writeUShort(68, v); }
  v = parseField("udpport", 1, 65534); if (v >= 0) { udpPort = v; EEPROM.writeUShort(70, v); }
  v = parseField("udpcatport", 1, 65534); if (v >= 0) { udpCatPort = v; EEPROM.writeUShort(72, v); }
  v = parseField("baudrate", 1200, 115200); if (v > 0) { BaudRate = v; EEPROM.writeUShort(74, v); }

  for (uint8_t i = 0; i < CW_MEMORY_COUNT; i++) {
    char key[10]; snprintf(key, sizeof(key), "cwmem%u", i + 1);
    cwMemoryText[i] = trimMemoryValue(extractJsonString(body, key), CW_MEMORY_MAX_LEN);
  }
  for (uint8_t i = 0; i < FREQ_MEMORY_COUNT; i++) {
    char key[12]; snprintf(key, sizeof(key), "freqmem%u", i + 1);
    freqMemoryText[i] = trimMemoryValue(extractJsonString(body, key), FREQ_MEMORY_MAX_LEN);
  }

  if (body.indexOf("\"dxchost\"") >= 0) {
    String dxchost = extractJsonString(body, "dxchost");
    if(dxchost.length() <= 64){ DxcHost = dxchost; eepromWriteStr(DxcHost, 137, 64); }
  }
  v = parseField("dxcport", 1, 65534); if(v >= 0){ DxcPort = v; EEPROM.writeUShort(201, v); }
  if (body.indexOf("\"dxccall\"") >= 0) {
    String dxccall = extractJsonString(body, "dxccall");
    if(dxccall.length() <= 16){ DxcCallsign = dxccall; eepromWriteStr(DxcCallsign, 203, 16); }
  }
  if (body.indexOf("\"dxclocator\"") >= 0) {
    String dxclocator = extractJsonString(body, "dxclocator");
    if(dxclocator.length() <= 6){ DxcLocator = dxclocator; eepromWriteStr(DxcLocator, 219, 6); }
  }

  if (body.indexOf("\"btname\"") >= 0) {
    String btname = extractJsonString(body, "btname");
    if (btname.length() >= 1 && btname.length() <= 20) { BT_NAME = btname; eepromWriteStr(BT_NAME, 267, 21); }
  }

  {
    String logCfg = extractJsonObject(body, "logConfig");
    if (logCfg.length() > 0 && logCfg.length() <= 2048) {
      saveLogConfigJson(logCfg);
    }
  }

  if (bdEnabled) {
    int bdIdx = body.indexOf("\"bd\":{");
    if (bdIdx >= 0) {
      String bdSection = body.substring(bdIdx + 5);
      int si = bdSection.indexOf("\"source\":");
      if (si >= 0) bdSource = bdSection.substring(si + 9).toInt();
      int arrStart = bdSection.indexOf("\"rows\":[");
      if (arrStart >= 0) {
        arrStart += 8;
        for (int i = 0; i < BD_ROWS; i++) {
          int ob = bdSection.indexOf('{', arrStart);
          int cb = bdSection.indexOf('}', ob);
          if (ob < 0 || cb < 0) break;
          String row = bdSection.substring(ob, cb + 1);
          bdRows[i].fMin    = (uint32_t)extractJsonInt(row, "fMin");
          bdRows[i].fMax    = (uint32_t)extractJsonInt(row, "fMax");
          bdRows[i].outputs = (uint16_t)extractJsonInt(row, "outputs");
          arrStart = cb + 1;
        }
      }
      bdSaveConfig();
    }
  }

  EEPROM.writeBool(0, false);
  EEPROM.commit();
  saveMemoryConfig();

  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleGetLogConfig() {
  if (!SPIFFS.exists(LOG_CONFIG_PATH)) {
    webServer.send(200, "application/json", "{}");
    return;
  }
  File f = SPIFFS.open(LOG_CONFIG_PATH, "r");
  if (!f) { webServer.send(500, "application/json", "{\"error\":\"open\"}"); return; }
  webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  webServer.sendHeader("Pragma", "no-cache");
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
  loadLogConfigVars();
  webServer.send(200, "application/json", "{\"ok\":true}");
}

// Translate OI3 mode number (from k3ng MQTT) to mode string.
// Returns nullptr when the value should be ignored (0=No mode, 8=Tune).
static const char* oi3ModeToString(int modeNum) {
  switch (modeNum) {
    case 1: return "CW";
    case 2: return "SSB";
    case 3: return "CW";
    case 4: return "RTTY";
    case 5: return "AM";
    case 6: return "RTTY";
    case 7: return "CW";   // CW-R
    case 9: return "RTTY"; // RTTY-R
    default: return nullptr; // 0=No mode, 8=Tune — keep last value
  }
}

void handleOi3State() {
  String trxArg = webServer.arg("trx");
  int idx = trxArg.toInt() - 2; // trx=2 → 0, trx=3 → 1
  if (idx < 0 || idx > 1) {
    webServer.send(400, "application/json", "{\"error\":\"invalid_trx\"}");
    return;
  }
  char jbuf[64];
  snprintf(jbuf, sizeof(jbuf), "{\"connected\":%s,\"frequency\":%ld,\"mode\":\"%s\"}",
    g_mqttHasData[idx] ? "true" : "false", g_mqttFreq[idx], g_mqttMode[idx]);
  webServer.send(200, "application/json", jbuf);
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
  webServer.on("/state", HTTP_GET, handleGetState);
  webServer.on("/cmd", HTTP_POST, handlePostCmd);
  webServer.on("/config/download", HTTP_GET,  handleConfigDownload);
  webServer.on("/config/upload",   HTTP_POST, handleConfigUpload);
  webServer.on("/log-config", HTTP_GET,  handleGetLogConfig);
  webServer.on("/log-config", HTTP_POST, handlePostLogConfig);
  webServer.on("/oi3/state", HTTP_GET,  handleOi3State);
  webServer.on("/oi3/send",  HTTP_POST, handleOi3Send);

  webServer.on("/", HTTP_GET, [](){
    if (!SPIFFS.exists("/index.html")) { renderSetupPage(); return; }
    renderIndexPage();
  });

  webServer.on("/setup/save", HTTP_POST, [](){
    handleSet();
    if (setupSaveOk) {
      webServer.send(200, "application/json", "{\"ok\":true}");
    } else {
      webServer.send(400, "application/json", "{\"ok\":false}");
    }
  });
  webServer.on("/setup",   HTTP_GET,  [](){ renderSetupPage(); });
  webServer.on("/setup",   HTTP_POST, [](){ handleSet(); renderSetupPage(); });
  webServer.on("/restart", HTTP_POST, [](){
    webServer.send(200, "application/json", "{\"ok\":true}");
    delay(500);
    ESP.restart();
  });
  webServer.on("/ws-cat",   HTTP_GET,  [](){ handleFileFromSPIFFS("/ws-cat.html"); });
  webServer.on("/log",      HTTP_GET,  [](){ handleFileFromSPIFFS("/log.html"); });
  webServer.on("/datasync", HTTP_GET,  [](){ handleFileFromSPIFFS("/datasync.html"); });

  webServer.on("/dxcinfo", HTTP_GET, [](){
    String j = "{\"locator\":\"" + DxcLocator + "\",\"callsign\":\"" + DxcCallsign + "\"}";
    webServer.send(200, "application/json", j);
  });

  webServer.on("/pairing/offer",  HTTP_OPTIONS, handlePairingOptions);
  webServer.on("/pairing/offer",  HTTP_POST,    handlePairingOfferPost);
  webServer.on("/pairing/offer",  HTTP_GET,     handlePairingOfferGet);
  webServer.on("/pairing/answer", HTTP_OPTIONS, handlePairingOptions);
  webServer.on("/pairing/answer", HTTP_POST,    handlePairingAnswerPost);
  webServer.on("/pairing/answer", HTTP_GET,     handlePairingAnswerGet);
  webServer.on("/pairing/reject", HTTP_OPTIONS, handlePairingOptions);
  webServer.on("/pairing/reject", HTTP_POST,    handlePairingReject);

  // Band Decoder
  webServer.on("/bd", HTTP_GET, [](){
    if (!bdEnabled) {
      webServer.send(200, "text/html",
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<title>Band Decoder</title>"
        "<link rel='stylesheet' href='/app.css'></head>"
        "<body><p style='margin:2em;font-family:sans-serif'>"
        "Band Decoder is available from RemoteQTH interface HW rev 04.</p></body></html>");
      return;
    }
    handleFileFromSPIFFS("/bd.html");
  });
  webServer.on("/api/bd-config", HTTP_GET, [](){
    if (!bdEnabled) { webServer.send(403, "application/json", "{\"error\":\"hw\"}"); return; }
    File f = SPIFFS.open(BD_CONFIG_PATH, FILE_READ);
    if (!f) { webServer.send(404, "application/json", "{}"); return; }
    webServer.streamFile(f, "application/json");
    f.close();
  });
  webServer.on("/api/bd-config", HTTP_POST, [](){
    if (!bdEnabled) { webServer.send(403, "application/json", "{\"error\":\"hw\"}"); return; }
    String body = webServer.arg("plain");
    int si = body.indexOf("\"source\":");
    if (si >= 0) bdSource = body.substring(si + 9).toInt();
    int arrStart = body.indexOf("\"rows\":[");
    if (arrStart >= 0) {
      arrStart += 8;
      for (int i = 0; i < BD_ROWS; i++) {
        int ob = body.indexOf('{', arrStart);
        int cb = body.indexOf('}', ob);
        if (ob < 0 || cb < 0) break;
        String row = body.substring(ob, cb + 1);
        bdRows[i].fMin    = (uint32_t)extractJsonInt(row, "fMin");
        bdRows[i].fMax    = (uint32_t)extractJsonInt(row, "fMax");
        bdRows[i].outputs = (uint16_t)extractJsonInt(row, "outputs");
        arrStart = cb + 1;
      }
    }
    bdSaveConfig();
    uint32_t f = (bdSource == 1) ? frequency :
                 (bdSource == 2) ? (uint32_t)g_mqttFreq[0] :
                                   (uint32_t)g_mqttFreq[1];
    bdUpdate(f);
    webServer.send(200, "application/json", "{\"ok\":true}");
  });
  webServer.on("/api/status", HTTP_GET, [](){
    String j = "{";
    j += "\"hwrev\":"; j += HardwareRev;
    j += ",\"trx1Label\":\""; j += g_lcTrx1Label; j += "\"";
    j += ",\"trx2Label\":\""; j += g_lcTrx2Label; j += "\"";
    j += ",\"trx3Label\":\""; j += g_lcTrx3Label; j += "\"";
    j += ",\"trx2freq\":"; j += g_mqttFreq[0];
    j += ",\"trx3freq\":"; j += g_mqttFreq[1];
    j += "}";
    webServer.sendHeader("Cache-Control", "no-cache");
    webServer.send(200, "application/json", j);
  });

  webServer.onNotFound([](){
    String path = webServer.uri();
    if (!handleFileFromSPIFFS(path)) webServer.send(404, "text/plain", "Not found");
  });

  webServer.begin();
}


bool catWriteFrame(const uint8_t *frame, size_t frameLen, bool broadcastTx){
  #if defined(BLUETOOTH)
    if (!btClientConnected) {
      return false;
    }
    for (size_t i = 0; i < frameLen; i++) {
      CAT.write(frame[i]);
    }
    return true;
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

    // 225-245 TRX2_MQTT_ROOT
    if(EEPROM.read(225)!=0xff){
      for (int i=225; i<246; i++){
        if(EEPROM.read(i)!=0xff){
          TRX2_MQTT_ROOT=TRX2_MQTT_ROOT+char(EEPROM.read(i));
        }
      }
    }

    // 246-266 TRX3_MQTT_ROOT
    if(EEPROM.read(246)!=0xff){
      for (int i=246; i<267; i++){
        if(EEPROM.read(i)!=0xff){
          TRX3_MQTT_ROOT=TRX3_MQTT_ROOT+char(EEPROM.read(i));
        }
      }
    }

    // 267-287 BT_NAME (21B)
    if(EEPROM.read(267)!=0xff){
      for (int i=267; i<288; i++){
        if(EEPROM.read(i)!=0xff){
          BT_NAME=BT_NAME+char(EEPROM.read(i));
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

    if(EEPROM.read(136) != 0xff){
      cwIpOnConnect = EEPROM.readBool(136);
    }

    // 137-200 DXC host (64B)
    if(EEPROM.read(137) != 0xff){
      for(int i=137; i<201; i++){
        if(EEPROM.read(i) != 0xff) DxcHost += char(EEPROM.read(i));
      }
    }

    // 201-202 DXC port (UShort)
    if(EEPROM.read(201) != 0xff){
      DxcPort = EEPROM.readUShort(201);
      if(DxcPort < 1) DxcPort = 7300;
    }

    // 203-218 DXC callsign (16B)
    if(EEPROM.read(203) != 0xff){
      for(int i=203; i<219; i++){
        if(EEPROM.read(i) != 0xff) DxcCallsign += char(EEPROM.read(i));
      }
    }

    // 219-224 DXC locator (6B)
    if(EEPROM.read(219) != 0xff){
      for(int i=219; i<225; i++){
        if(EEPROM.read(i) != 0xff) DxcLocator += char(EEPROM.read(i));
      }
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
     }else if(HWidValue>406 && HWidValue<=693){
       HardwareRev=3;  // 560
    }else if(HWidValue>693 && HWidValue<=900){
      HardwareRev=4;  // 827
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

  #if defined(WIFI)
    if (!SPIFFS.begin(true)) {
      Serial.println("SPIFFS| mount failed");
    } else {
      Serial.println("SPIFFS| mounted");
      loadMemoryConfig();
      loadLogConfigVars();
      bdEnabled = (HardwareRev >= 4);
      if (bdEnabled) {
        bdLoadConfig();
        bdInit();
        Serial.println("BD   | enabled");
      }
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
      dxcRawServer.begin();
      Serial.println("HTTP| web server started");
      Serial.println("HTTP| DXC WS server started on port 82");
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

    #if defined(BLUETOOTH)
      configRadioBaud(0);
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
  static bool serialFlushed = false;
  if (!serialFlushed && millis() > 2000) {
    while (Serial.available()) Serial.read();
    serialFlushed = true;
  }
  #define LOOP_WARN_MS 200
  #define _TIMED(name, call) { unsigned long _t = millis(); call; unsigned long _d = millis()-_t; if(_d > LOOP_WARN_MS) { Serial.print("LOOP| slow: " name " "); Serial.print(_d); Serial.println("ms"); } }
  _TIMED("Watchdog",        Watchdog())
  _TIMED("Mqtt",            Mqtt())
  _TIMED("httpCAT",         httpCAT())
  _TIMED("UdpToCwFsk",      UdpToCwFsk())
  _TIMED("UdpToCat",        UdpToCat())
  _TIMED("CLI",             CLI())
  #if defined(RTLE)
    _TIMED("rtleserver",    rtleserver.handleClient())
  #endif
  _TIMED("webServer",       webServer.handleClient())
  _TIMED("DxcLoop",         DxcLoop())
  _TIMED("dxcRaw",          dxcHandleRawClient())

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

  if (btConnectPending == true) {
    btConnectPending = false;
    btClientConnected = true;
    radio_address = configuredCivAddress;
    frequency = 0;
    frequencyTmp = 0;
    if (TrxSetupDone == false) {
      TrxNeedSet = 1;
      if (cwIpOnConnect) cwIpSendPending = true;
    }
    Serial.println("    | Client Connected");
  }

  if (btDisconnectPending == true) {
    btDisconnectPending = false;
    btClientConnected = false;
    radio_address = 0x00;
    TrxNeedSet = 0;
    powerTimer = 0;
    frequency = 0;
    frequencyTmp = 0;
    setModesText("OFF");
    mqttPostponeStatus = 1;
    Serial.println("    | Client Disconnected");
    if (statusPower == 1 && mqttEnable == true) {
      MqttPubString(MQTT_TOPIC, "0", 0);
    }
  }
}

void pollRadio(){
  static unsigned long catPollTimer = 0;
  static unsigned long auxPollTimer = 0;
  static uint8_t auxPollIndex = 0;

  if (!btClientConnected) return;

  if (millis() - catPollTimer > 200) {
    catPollTimer = millis();
    if (radio_address == 0x00) {
      if (Debug == true) Serial.println("CAT | searching radio...");
      searchRadio();
    } else {
      sendCatRequest(CMD_READ_FREQ);
      sendCatRequest(CMD_READ_MODE);
      processCatMessages();
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
    processCatMessages();
    auxPollIndex++;
  }
}

void Watchdog(){
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

    // Band Decoder update on TRX1 frequency change
    if (bdEnabled && bdSource == 1) bdUpdate(frequency);

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
    if(millis()-WdtTimer > 5000){
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
void FallbackToAPmode() {
  APmode = true;
  EEPROM.writeBool(0, true);
  EEPROM.commit();
  delay(500);
  ESP.restart();
}

//-------------------------------------------------------------------------------------------------------
void ConnectWiFiAlternating() {
  WiFi.mode(WIFI_STA);

  if (WifiProfileConfigured(0) == false && WifiProfileConfigured(1) == false) {
    Serial.println("WIFI| no configured SSID, staying in AP mode");
    FallbackToAPmode();
  }

  // Scan before connecting — avoids infinite loop when SSIDs are configured but not in range
  Serial.println("WIFI| scanning for configured networks...");
  int n = WiFi.scanNetworks();
  bool anyVisible = false;
  for (int i = 0; i < n; i++) {
    String scanned = WiFi.SSID(i);
    if ((WifiProfileConfigured(0) && scanned == WifiProfileSSID(0)) ||
        (WifiProfileConfigured(1) && scanned == WifiProfileSSID(1))) {
      anyVisible = true;
      break;
    }
  }
  WiFi.scanDelete();

  if (!anyVisible) {
    Serial.println("WIFI| no configured SSID found in scan, switching to AP mode");
    FallbackToAPmode();
  }

  byte wifiProfile = 0;
  if (WifiProfileConfigured(0) == false && WifiProfileConfigured(1) == true) {
    wifiProfile = 1;
  }

  WifiTimer = millis();
  int attempts = 0;
  const int maxAttempts = 4;
  while (WiFi.status() != WL_CONNECTED) {
    if (ConnectWiFiProfile(wifiProfile, wifi_max_try)) {
      break;
    }
    attempts++;
    if (attempts >= maxAttempts) {
      Serial.println("WIFI| max retries reached, switching to AP mode");
      FallbackToAPmode();
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
void processCatMessages(){
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
      if (radio_address == 0x00) {
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

      if (read_buffer[2] == BROADCAST_ADDRESS || read_buffer[2] == CONTROLLER_ADDRESS) {
        switch (read_buffer[4]) {
          case CMD_TRANS_FREQ:
          case CMD_READ_FREQ:
            if (len >= 11) printFrequency();
            break;

          case CMD_TRANS_MODE:
          case CMD_READ_MODE:
            if (len >= 7) {
              printMode();
              if (len >= 8) stateFilter = read_buffer[6];
            }
            break;
        }
      }

      // Update state variables from decoded CIV frames
      if (len >= 7) {
        const uint8_t cmd = read_buffer[4];
        const uint8_t *pl = read_buffer + 5;
        const size_t plLen = len - 6;
        if (cmd == 0x15 && plLen >= 2) {
          uint32_t raw = decodeCivBcdBytes(pl + 1, plLen - 1);
          if (pl[0] == 0x02) stateSmeterRaw = raw;
          else if (pl[0] == 0x11) statePowerMeterRaw = raw;
          else if (pl[0] == 0x12) stateSwr = 1.0f + ((float)raw * 3.0f / 120.0f);
          else if (pl[0] == 0x15) stateSupplyVolts = ((float)raw * 16.0f) / 241.0f;
        }
        if (cmd == 0x14 && plLen >= 2) {
          uint32_t raw = decodeCivBcdBytes(pl + 1, plLen - 1);
          if (pl[0] == 0x01) stateAfGain = (uint8_t)raw;
          else if (pl[0] == 0x0C) stateKeySpeed = (uint8_t)raw;
          else if (pl[0] == 0x0A) stateRfPower = (uint8_t)raw;
        }
        if (cmd == 0x21 && plLen >= 4 && pl[0] == 0x00) {
          stateRitRaw = decodeCivBcdBytesLsb(pl + 1, 3); // LSB-first, 3 bytes only, sign byte excluded
        }
        if (cmd == 0x1C && plLen >= 2 && pl[0] == 0x00) {
          stateTx = (pl[1] == 0x01);
        }
        if (cmd == 0x04 && plLen >= 2) {
          stateFilter = (uint8_t)pl[1];
        }
        if (cmd == 0x11 && plLen >= 1) {
          stateAttOn = pl[0] ? 1 : 0;
          if (stateAttOn) statePreampMode = 3;
          else if (statePreampMode == 3) statePreampMode = 0;
        }
        if (cmd == 0x16 && plLen >= 2) {
          if (pl[0] == 0x02 && !stateAttOn) {
            statePreampMode = pl[1]; // 0=OFF, 1=AMP1, 2=AMP2 direct from radio
          }
          if (pl[0] == 0x47) {
            stateVoxMode = pl[1];
          }
        }
      }

      powerTimer = millis();
    }
  #endif
}

//-------------------------------------------------------------------------------------------------------
// call back to get info about connection
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  (void)param;
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    btConnectPending = true;
    btStateBroadcastPending = true;
  }
  if (event == ESP_SPP_CLOSE_EVT) {
    btDisconnectPending = true;
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

    esp_bt_mem_release(ESP_BT_MODE_BLE);
    if (BT_NAME.length() == 0) {
      uint8_t mac[6];
      WiFi.macAddress(mac);
      char defName[22];
      snprintf(defName, sizeof(defName), "IC705-%02X%02X%02X", mac[3], mac[4], mac[5]);
      BT_NAME = String(defName);
    }
    if (!CAT.begin(BT_NAME)) {
      Serial.println(" BT | An error occurred initializing Bluetooth");
    } else {
      CAT.register_callback(callback);
      Serial.println(" BT | Initialized");
      Serial.println("    | -----------------------------------------------------------------------------");
      Serial.println(" BT | The device started, now you MUST PAIR it with Bluetooth name " + BT_NAME);
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
          // Subscribe to TRX2/TRX3 OI3 MQTT topics (wildcard)
          auto mqttSubscribeOi3Root = [&](const String &root) {
            if (root.length() == 0) return;
            String r = root;
            if (r[r.length()-1] != '/') r += '/';
            String tWild = r + "#";
            tWild.toCharArray(mqttPath, sizeof(mqttPath));
            mqttClient.subscribe(mqttPath);
            Serial.print("MQTT| subscribe "); Serial.println(mqttPath);
          };
          mqttSubscribeOi3Root(TRX2_MQTT_ROOT);
          mqttSubscribeOi3Root(TRX3_MQTT_ROOT);
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
      char payloadBuf[32];
      unsigned int copyLen = length < sizeof(payloadBuf)-1 ? length : sizeof(payloadBuf)-1;
      memcpy(payloadBuf, payload, copyLen);
      payloadBuf[copyLen] = '\0';

      String rxTopic = String(topic);

      // Check TRX2 / TRX3 OI3 MQTT topics first
      auto checkOi3Root = [&](const String &root, int idx) -> bool {
        if (root.length() == 0) return false;
        String r = root;
        if (r[r.length()-1] != '/') r += '/';
        if (rxTopic == r + "hz") {
          long freq = atol(payloadBuf);
          if (freq > 0) {
            g_mqttFreq[idx] = freq;
            g_mqttHasData[idx] = true;
            Serial.printf("OI3 | TRX%d freq=%ld\n", idx+2, freq);
            if (bdEnabled && bdSource == idx + 2) bdUpdate((uint32_t)freq);
          }
          return true;
        }
        if (rxTopic == r + "mode") {
          int modeNum = atoi(payloadBuf);
          const char *modeStr = oi3ModeToString(modeNum);
          if (modeStr != nullptr) {
            strlcpy(g_mqttMode[idx], modeStr, sizeof(g_mqttMode[idx]));
            g_mqttHasData[idx] = true;
            Serial.printf("OI3 | TRX%d mode=%s\n", idx+2, modeStr);
          }
          return true;
        }
        return false;
      };

      if (checkOi3Root(TRX2_MQTT_ROOT, 0)) return;
      if (checkOi3Root(TRX3_MQTT_ROOT, 1)) return;

      // MQTT_TOPIC_RX: set TRX1 VFO frequency
      if (MQTT_TOPIC_RX.length() == 0) return;
      if (rxTopic != MQTT_TOPIC_RX) return;

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
    Serial.println("  Bluetooth: "+BT_NAME);
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

    // 70-71 udpPort
    if ( requestArg("udpport").length()<1 || requestArg("udpport").toInt()<1 || requestArg("udpport").toInt()>65534){
      setupUdpPortErr = "Out of range number 1-65534";
      ERRdetect=1;
    }else{
      if(udpPort == requestArg("udpport").toInt()){
        setupUdpPortErr = "";
      }else{
        setupUdpPortErr = "";
        udpPort = requestArg("udpport").toInt();
        EEPROM.writeUShort(70, udpPort);
      }
    }

    // 72-73 udpCatPort
    if ( requestArg("udpcatport").length()<1 || requestArg("udpcatport").toInt()<1 || requestArg("udpcatport").toInt()>65534){
      setupUdpCatPortErr = "Out of range number 1-65534";
      ERRdetect=1;
    }else{
      if(udpCatPort == requestArg("udpcatport").toInt()){
        setupUdpCatPortErr = "";
      }else{
        setupUdpCatPortErr = "";
        udpCatPort = requestArg("udpcatport").toInt();
        EEPROM.writeUShort(72, udpCatPort);
      }
    }

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

    // 225-245 TRX2_MQTT_ROOT
    if (requestArg("trx2mqttroot").length() > 20) {
      ERRdetect=1;
    } else {
      String str = requestArg("trx2mqttroot");
      if (TRX2_MQTT_ROOT != str) {
        TRX2_MQTT_ROOT = str;
        eepromWriteStr(TRX2_MQTT_ROOT, 225, 21);
      }
    }

    // 246-266 TRX3_MQTT_ROOT
    if (requestArg("trx3mqttroot").length() > 20) {
      ERRdetect=1;
    } else {
      String str = requestArg("trx3mqttroot");
      if (TRX3_MQTT_ROOT != str) {
        TRX3_MQTT_ROOT = str;
        eepromWriteStr(TRX3_MQTT_ROOT, 246, 21);
      }
    }

    // 267-287 BT_NAME (21B)
    {
      String str = requestArg("btname");
      if (str.length() > 20) {
        ERRdetect=1;
      } else if (str.length() >= 1 && BT_NAME != str) {
        BT_NAME = str;
        eepromWriteStr(BT_NAME, 267, 21);
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

    String nextTransceiverType = trimMemoryValue(requestArg("trxprofile"), 16);
    if (nextTransceiverType != "IC-7610-CI-V") {
      nextTransceiverType = "IC-705-BT";
    }
    transceiverType = nextTransceiverType;

    uint8_t nextCivAddress = configuredCivAddress;
    if (!parseHexByteString(requestArg("civaddr"), nextCivAddress)) {
      setupCivAddrErr = "Use 2-digit hex value 00-FF";
      ERRdetect = 1;
    } else {
      configuredCivAddress = nextCivAddress;
    }

    cwIpOnConnect = requestHasArg("cwIpOnConnect");
    EEPROM.writeBool(136, cwIpOnConnect);

    {
      String dxchost = requestArg("dxchost");
      if(dxchost.length() <= 64){ DxcHost = dxchost; eepromWriteStr(DxcHost, 137, 64); }
      String dxcportStr = requestArg("dxcport");
      if(dxcportStr.length() > 0){ int p = dxcportStr.toInt(); if(p >= 1 && p <= 65534){ DxcPort = p; EEPROM.writeUShort(201, p); } }
      String dxccall = requestArg("dxccall");
      if(dxccall.length() <= 16){ DxcCallsign = dxccall; eepromWriteStr(DxcCallsign, 203, 16); }
      String dxclocator = requestArg("dxclocator");
      if(dxclocator.length() <= 6){ DxcLocator = dxclocator; eepromWriteStr(DxcLocator, 219, 6); }
    }

    {
      String trx1Label = trimMemoryValue(requestArg("trx1label"), 10);
      if (trx1Label.length() == 0) trx1Label = "IC-705";
      String trx2Label = trimMemoryValue(requestArg("trx2label"), 10);
      if (trx2Label.length() == 0) trx2Label = "TRX2";
      String trx3Label = trimMemoryValue(requestArg("trx3label"), 10);
      if (trx3Label.length() == 0) trx3Label = "TRX3";
      String trx2Civ = trimMemoryValue(requestArg("trx2civ"), 2);
      trx2Civ.toUpperCase();
      String trx3Civ = trimMemoryValue(requestArg("trx3civ"), 2);
      trx3Civ.toUpperCase();
      String trx2Ip = trimMemoryValue(requestArg("trx2ip"), 32);
      String trx3Ip = trimMemoryValue(requestArg("trx3ip"), 32);
      bool trx2Oi3 = requestHasArg("trx2Oi3");
      bool trx3Oi3 = requestHasArg("trx3Oi3");
      String rstSsb = trimMemoryValue(requestArg("rstSsb"), 3);
      if (rstSsb.length() == 0) rstSsb = "59";
      String rstCwRtty = trimMemoryValue(requestArg("rstCwRtty"), 3);
      if (rstCwRtty.length() == 0) rstCwRtty = "599";
      bool manualModeForPhone = requestHasArg("manualModeForPhone");
      String blockedDxcc = requestArg("blockedDxcc");

      g_lcTrx1Label = trx1Label;
      g_lcTrx2Label = trx2Label;
      g_lcTrx3Label = trx3Label;
      g_lcTrx2Civ = trx2Civ;
      g_lcTrx3Civ = trx3Civ;
      g_lcTrx2Ip = trx2Ip;
      g_lcTrx3Ip = trx3Ip;
      g_lcTrx2Oi3 = trx2Oi3;
      g_lcTrx3Oi3 = trx3Oi3;
      g_lcRstSsb = rstSsb;
      g_lcRstCwRtty = rstCwRtty;
      g_lcManualModeForPhone = manualModeForPhone;
      g_lcBlockedDxcc = blockedDxcc;

      String nextLogConfig = buildLogConfigJson(
        readLogConfigJson(),
        trx1Label, trx2Label, trx3Label,
        trx2Civ, trx3Civ,
        trx2Ip, trx3Ip,
        trx2Oi3, trx3Oi3,
        rstSsb, rstCwRtty,
        manualModeForPhone, blockedDxcc
      );
      if (!saveLogConfigJson(nextLogConfig)) {
        ERRdetect = 1;
      }
    }

    // Only overwrite memories when they are actually present in the request
    // (CW/freq memory inputs live outside the EEPROM <form> — absent = empty string from requestArg).
    if (requestHasArg("cwmem1")) {
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
    }


    if(ERRdetect==0){
      setupSaveOk = true;
      // // APmode
      EEPROM.writeBool(0, false);
      EEPROM.commit();
      if (requestHasArg("noRestart")) {
        return;
      }
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

// ---- DXC telnet proxy -------------------------------------------------------

String ExtractHttpHeader(const String& request, const String& headerName){
  String needle = "\n" + headerName + ":";
  int start = request.indexOf(needle);
  if(start < 0){
    String lowerRequest = request;
    lowerRequest.toLowerCase();
    String lowerNeedle = needle;
    lowerNeedle.toLowerCase();
    start = lowerRequest.indexOf(lowerNeedle);
    if(start < 0) return "";
  }
  start = request.indexOf(':', start);
  if(start < 0) return "";
  start++;
  int end = request.indexOf('\n', start);
  if(end < 0) end = request.length();
  String value = request.substring(start, end);
  value.trim();
  return value;
}

String Base64Encode(const uint8_t* data, size_t length){
  static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String encoded = "";
  encoded.reserve(((length + 2) / 3) * 4);
  for(size_t i = 0; i < length; i += 3){
    uint32_t block = uint32_t(data[i]) << 16;
    bool hasSecond = (i + 1) < length;
    bool hasThird  = (i + 2) < length;
    if(hasSecond) block |= uint32_t(data[i + 1]) << 8;
    if(hasThird)  block |= uint32_t(data[i + 2]);
    encoded += alphabet[(block >> 18) & 0x3F];
    encoded += alphabet[(block >> 12) & 0x3F];
    encoded += hasSecond ? alphabet[(block >> 6) & 0x3F] : '=';
    encoded += hasThird  ? alphabet[block & 0x3F] : '=';
  }
  return encoded;
}

String DxcComputeWebSocketAccept(const String& secKey){
  String source = secKey;
  source += "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  uint8_t digest[20];
  mbedtls_sha1(reinterpret_cast<const unsigned char*>(source.c_str()), source.length(), digest);
  return Base64Encode(digest, sizeof(digest));
}

bool DxcConfigReady(){
  return DxcHost.length() > 0 && DxcPort > 0 && DxcCallsign.length() > 0;
}

void DxcUpdateTelnetStatus(bool connected, bool forceSend){
  if(!forceSend && DxcTelnetStatus == connected) return;
  DxcTelnetStatus = connected;
  DxcSendTelnetStatus();
}

void DxcSendTelnetStatus(){
  if(!DxcWsClient.connected()) return;
  String payload = String("{\"telnet\":") + (DxcTelnetStatus ? "true" : "false") + "}";
  DxcSendWebSocketText(payload);
}

void DxcDisconnectTelnet(){
  if(DxcTelnetClient.connected()) DxcTelnetClient.stop();
  DxcTelnetLoginPending = false;
  DxcUpdateTelnetStatus(false);
}

void DxcDisconnectWebSocket(){
  if(DxcWsClient.connected()) DxcWsClient.stop();
  DxcWsStatus = false;
  DxcDisconnectTelnet();
}

void DxcRequestReconnect(){
  DxcDisconnectTelnet();
  DxcReconnectTimer = millis() + 250;
}

bool DxcConnectTelnet(){
  if(!DxcWsClient.connected() || !DxcConfigReady()){
    DxcUpdateTelnetStatus(false);
    return false;
  }
  if(DxcTelnetClient.connected()) return true;
  WiFiClient newClient;
  newClient.setNoDelay(true);
  if(!newClient.connect(DxcHost.c_str(), DxcPort)){
    DxcReconnectTimer = millis() + 5000;
    DxcUpdateTelnetStatus(false);
    return false;
  }
  DxcTelnetClient = newClient;
  DxcTelnetLoginPending = true;
  DxcUpdateTelnetStatus(true, true);
  return true;
}

bool DxcSendWebSocketFrame(uint8_t opcode, const uint8_t* payload, size_t length){
  if(!DxcWsClient.connected()) return false;
  uint8_t header[10];
  size_t headerLen = 0;
  header[headerLen++] = 0x80 | (opcode & 0x0F);
  if(length < 126){
    header[headerLen++] = uint8_t(length);
  }else if(length <= 0xFFFF){
    header[headerLen++] = 126;
    header[headerLen++] = uint8_t((length >> 8) & 0xFF);
    header[headerLen++] = uint8_t(length & 0xFF);
  }else{
    header[headerLen++] = 127;
    for(int shift = 56; shift >= 0; shift -= 8)
      header[headerLen++] = uint8_t((uint64_t(length) >> shift) & 0xFF);
  }
  if(DxcWsClient.write(header, headerLen) != headerLen){ DxcDisconnectWebSocket(); return false; }
  if(length > 0 && payload != nullptr){
    if(DxcWsClient.write(payload, length) != length){ DxcDisconnectWebSocket(); return false; }
  }
  return true;
}

bool DxcSendWebSocketText(const char* text){
  if(text == nullptr) return DxcSendWebSocketFrame(0x1, nullptr, 0);
  return DxcSendWebSocketFrame(0x1, reinterpret_cast<const uint8_t*>(text), strlen(text));
}

bool DxcSendWebSocketText(const String& text){
  return DxcSendWebSocketFrame(0x1, reinterpret_cast<const uint8_t*>(text.c_str()), text.length());
}

bool DxcHandleWebSocketUpgrade(WiFiClient& webClient, const String& request){
  String secKey    = ExtractHttpHeader(request, "Sec-WebSocket-Key");
  String upgrade   = ExtractHttpHeader(request, "Upgrade");
  String connection = ExtractHttpHeader(request, "Connection");
  upgrade.toLowerCase();
  connection.toLowerCase();
  if(secKey.length() == 0 || upgrade != "websocket" || connection.indexOf("upgrade") < 0){
    webClient.println(F("HTTP/1.1 400 Bad Request\r\nContent-Type: text/plain\r\nConnection: close\r\n"));
    webClient.println(F("Invalid WebSocket handshake"));
    return false;
  }
  if(DxcWsClient.connected()) DxcDisconnectWebSocket();
  String accept = DxcComputeWebSocketAccept(secKey);
  webClient.println(F("HTTP/1.1 101 Switching Protocols"));
  webClient.println(F("Upgrade: websocket"));
  webClient.println(F("Connection: Upgrade"));
  webClient.print(F("Sec-WebSocket-Accept: "));
  webClient.println(accept);
  webClient.println();
  DxcWsClient = webClient;
  DxcWsClient.setNoDelay(true);
  DxcWsStatus = true;
  DxcUpdateTelnetStatus(DxcTelnetClient.connected(), true);
  DxcRequestReconnect();
  return true;
}

void DxcHandleWebSocketClient(){
  if(!DxcWsClient.connected()){
    if(DxcWsStatus){ DxcWsStatus = false; DxcDisconnectTelnet(); }
    return;
  }
  while(DxcWsClient.available() >= 2){
    uint8_t hdr[2];
    if(DxcWsClient.read(hdr, 2) != 2){ DxcDisconnectWebSocket(); return; }
    uint8_t opcode = hdr[0] & 0x0F;
    bool masked = (hdr[1] & 0x80) != 0;
    uint64_t payloadLen = hdr[1] & 0x7F;
    if(payloadLen == 126){
      uint8_t ext[2];
      while(DxcWsClient.connected() && DxcWsClient.available() < 2) delay(1);
      if(DxcWsClient.read(ext, 2) != 2){ DxcDisconnectWebSocket(); return; }
      payloadLen = (uint16_t(ext[0]) << 8) | uint16_t(ext[1]);
    }else if(payloadLen == 127){
      uint8_t ext[8];
      while(DxcWsClient.connected() && DxcWsClient.available() < 8) delay(1);
      if(DxcWsClient.read(ext, 8) != 8){ DxcDisconnectWebSocket(); return; }
      payloadLen = 0;
      for(int i = 0; i < 8; i++) payloadLen = (payloadLen << 8) | ext[i];
    }
    uint8_t maskKey[4] = {0,0,0,0};
    if(masked){
      while(DxcWsClient.connected() && DxcWsClient.available() < 4) delay(1);
      if(DxcWsClient.read(maskKey, 4) != 4){ DxcDisconnectWebSocket(); return; }
    }
    if(payloadLen > 2048){ DxcDisconnectWebSocket(); return; }
    static uint8_t payload[2048];
    size_t needed = size_t(payloadLen);
    while(DxcWsClient.connected() && DxcWsClient.available() < int(needed)) delay(1);
    if(needed > 0 && DxcWsClient.read(payload, needed) != int(needed)){ DxcDisconnectWebSocket(); return; }
    if(masked){ for(size_t i = 0; i < needed; i++) payload[i] ^= maskKey[i % 4]; }
    if(opcode == 0x8){ DxcDisconnectWebSocket(); return; }
    if(opcode == 0x9){ DxcSendWebSocketFrame(0xA, payload, needed); continue; }
    if(opcode != 0x1) continue;
    String command = "";
    command.reserve(needed + 1);
    for(size_t i = 0; i < needed; i++){ if(payload[i] != '\0') command += char(payload[i]); }
    command.trim();
    if(command.length() == 0) continue;
    if(command == "@reconnect"){ DxcRequestReconnect(); continue; }
    if(!DxcTelnetClient.connected()){ DxcRequestReconnect(); continue; }
    DxcTelnetClient.print(command);
    DxcTelnetClient.print("\r\n");
  }
}

void DxcHandleTelnetClient(){
  if(!DxcWsClient.connected()){
    if(DxcTelnetClient.connected()) DxcDisconnectTelnet();
    return;
  }
  if(!DxcTelnetClient.connected()){ DxcUpdateTelnetStatus(false); return; }
  if(DxcTelnetLoginPending && DxcCallsign.length() > 0){
    DxcTelnetClient.print(DxcCallsign);
    DxcTelnetClient.print("\r\n");
    DxcTelnetLoginPending = false;
  }
  static uint8_t telnetBuffer[1024];
  while(DxcTelnetClient.available()){
    int chunk = DxcTelnetClient.read(telnetBuffer, sizeof(telnetBuffer));
    if(chunk <= 0) break;
    if(!DxcSendWebSocketFrame(0x1, telnetBuffer, size_t(chunk))) return;
  }
}

void DxcLoop(){
  DxcHandleWebSocketClient();
  if(!DxcWsClient.connected()){ DxcDisconnectTelnet(); return; }
  if(!DxcTelnetClient.connected() && DxcConfigReady() && millis() >= DxcReconnectTimer) DxcConnectTelnet();
  DxcHandleTelnetClient();
}

void dxcHandleRawClient(){
  WiFiClient client = dxcRawServer.available();
  if(!client) return;
  String request = "";
  unsigned long timeout = millis() + 2000;
  while(client.connected() && millis() < timeout){
    while(client.available()){
      char c = client.read();
      request += c;
      if(request.endsWith("\r\n\r\n")) goto done;
    }
    delay(1);
  }
  done:
  // check URI is /dxcws
  int uriStart = request.indexOf(' ') + 1;
  int uriEnd   = request.indexOf(' ', uriStart);
  String uri   = request.substring(uriStart, uriEnd);
  if(uri != "/dxcws"){
    client.println(F("HTTP/1.1 404 Not Found\r\nConnection: close\r\n"));
    client.stop();
    return;
  }
  if(!DxcHandleWebSocketUpgrade(client, request)){
    client.stop();
  }
  // on success, client is stored in DxcWsClient — do NOT stop it
}
