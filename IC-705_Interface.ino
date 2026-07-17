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

  Manual workflow
  - afer standart edit data/*.html/css/js,
  - before manual SPIFFS upload run tools/gzip-assets.sh
  - continue with Arduino IDE

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
// TrxNet config — NET_ID 0x00 = disabled sentinel (TrxNet not activated)
byte     TRXNET_ID      = 0x01;   // own device NET_ID → device name "705.01"
byte     TRX2_NET_ID    = 0xff;   // peer NET_ID for TRX2 Band Decoder slot (0x00 = disabled)
byte     TRX3_NET_ID    = 0x00;   // peer NET_ID for TRX3 Band Decoder slot (0x00 = disabled)
byte     TRX2_CONN_TYPE = 0x00;   // 0=TrxNet, 1=CI-V (EEPROM byte 44)
byte     TRX3_CONN_TYPE = 0x00;   // 0=TrxNet, 1=CI-V (EEPROM byte 47)
byte     TRX2_CIV_ADDR  = 0x00;   // CI-V address of TRX2 when CONN_TYPE=CI-V (EEPROM byte 48; 0x00=unset)
byte     TRX3_CIV_ADDR  = 0x00;   // CI-V address of TRX3 when CONN_TYPE=CI-V (EEPROM byte 49; 0x00=unset)
uint16_t TRXNET_PORT    = 5683;   // CoAP/discovery UDP port (CoAP default)
int BaudRate        = 9600;
// char* BTname        = "";
// const char* BTname  = "IC705-interface";
String BT_NAME;  // loaded from EEPROM; default IC705-XXXXXX from MAC
bool Debug          = false;
bool cwIpOnConnect  = true;       // announce WiFi IP via CW on first BT connect
volatile bool cwIpSendPending = false;

#define LOOP_WARN_MS 200
#define REV 20260717
#define WIFI
#define UDP_TO_FSK
#define WDT         // watchdog timer
#define CIV_OUT     // send freq to CIV out with BaudRate
#define BLUETOOTH   // BT
// #define RTLE     // not work now | credit OK2CQR https://github.com/ok2cqr/rtle/tree/master
// #define RESET_AFTER_DISCONNECT  // enable reset after each disconnect + short CW msg

#include "EEPROM.h"
#define EEPROM_SIZE 360
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
  41 TRXNET_ID       (own NET_ID, 0x00=disabled → device name "705.XX")
  42 TRX2_NET_ID     (peer NET_ID for TRX2 BD slot, 0x00=disabled)
  43 TRX3_NET_ID     (peer NET_ID for TRX3 BD slot, 0x00=disabled)
  44 TRX2_CONN_TYPE (0=TrxNet, 1=CI-V; 0xff=unprogrammed → default 0x00)
  45-46 TRXNET_PORT  (was MQTT_PORT)
  47 TRX3_CONN_TYPE (0=TrxNet, 1=CI-V; 0xff=unprogrammed → default 0x00)
  48 TRX2_CIV_ADDR  (CI-V address of TRX2; 0x00/0xff = unset)
  49 TRX3_CIV_ADDR  (CI-V address of TRX3; 0x00/0xff = unset)
  50-67 FREE         (was MQTT_TOPIC 21B)
  68-69 FREE         (was HTTP_CAT_PORT)
  70-71 FREE         (was udpPort)
  72-73 FREE         (was udpCatPort)
  74-75 BaudRate
  76-95 SSID2
  97-114 PSWD2
  115-135 FREE       (was MQTT_TOPIC_RX 21B)
  136 cwIpOnConnect
  137-200 DXC host (64B)
  201-202 DXC port (UShort)
  203-218 DXC callsign (16B)
  219-224 DXC locator (6B)
  225-245 FREE       (was TRX2 MQTT root topic 21B)
  246-266 FREE       (was TRX3 MQTT root topic 21B)
  267-287 BT_NAME (21B)
  288 TRXNET_PRIO flag (0xff=unprogrammed → default "OI3 ANT"; 0x01=user set → read string)
  289-359 TRXNET_PRIO priority prefixes string (space-separated, 71B; empty = priority off)

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

#if defined(WDT)
  // 73 seconds WDT (WatchDogTimer)
  #include <esp_task_wdt.h>
  #define WDT_TIMEOUT 73
  long WdtTimer=0;
#endif

#if defined(WIFI)
  #include <WiFi.h>
  #include <esp_wifi.h>
  #include "esp_coexist.h"
  #include <FS.h>
  #include <SPIFFS.h>
  #define HTTP_MAX_DATA_WAIT 1000
  #include <WebServer.h>
  #include <mbedtls/sha1.h>
  // #include <ETH.h>
  // int SsidPassSize = (sizeof(SsidPass)/sizeof(char *))/2; //array size
  // int SelectSsidPass = -1;
  #define wifi_max_try 40             // Number of try
  unsigned long WifiTimer = 0;
  unsigned long WifiReconnect = 5000;
  unsigned long WifiDownSince = 0;        // 0 = station link OK
  bool WifiHardResetDone = false;         // one radio off/on per outage — repeating it wedges the driver
  bool WifiCoexPreferred = false;         // coex arbiter tilted to WiFi for the duration of an outage
  #define WIFI_HARD_RESET_AFTER_MS 10000  // radio off/on cycle when down longer than this
  #define WIFI_RESTART_AFTER_MS    60000  // ESP.restart() when down longer than this
  // Last known AP — reconnect targets channel+BSSID directly, because a full
  // scan often finds nothing while the BT Classic (SPP) link is active (coex).
  int32_t wifiLastChannel = 0;
  uint8_t wifiLastBssid[6];
  bool    wifiLastBssidValid = false;
  String  wifiLastSsid;
  unsigned long webQuietUntil = 0;
  byte ActiveWifiProfile = 0;
  String MACString;

  #include <ESPmDNS.h>
  #include <DNSServer.h>

  const char* ssidAP     = "IC705-if";
  const char* passwordAP = "remoteqth";
  bool APmode = false;
  // WebServer discards the client before handleClient() returns, so the slow-loop
  // diagnostic can't sample it from outside. Response writes (where stalls to a dead
  // client block the loop) go through this virtual, so capture peer+uri here.
  class DiagWebServer : public WebServer {
  public:
    using WebServer::WebServer;
    IPAddress lastPeer;
    String lastUri;
  protected:
    size_t _currentClientWrite(const char* b, size_t l) override {
      if (_currentClient.connected()) {
        lastPeer = _currentClient.remoteIP();
        lastUri = _currentUri;
      }
      return WebServer::_currentClientWrite(b, l);
    }
  };
  DiagWebServer webServer(80);
  const byte DNS_PORT = 53;
  DNSServer dnsServer;             // captive portal in AP mode
#endif
  #if defined(RTLE)
    #ifndef HTTP_MAX_DATA_WAIT
      #define HTTP_MAX_DATA_WAIT 1000
    #endif
    #include <WebServer.h>
    #include "rtle.h"  //Web page header file
    WebServer rtleserver(88);
  #endif


// uint8_t CwMsg[36] = "";
char CwMsg[37] = "";

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
  volatile bool abortFskTransmission = false;
#endif


// TrxNet — P2P telemetry, replaces PubSubClient/MQTT
// TRXNET_ID == 0x00 acts as "disabled" sentinel — net.begin() is not called.
// NOTE: never #define TRXNET_MAX_PENDING/TRXNET_MAX_PEERS here — TrxNet.cpp is
// compiled separately without sketch defines, so the class layout would differ
// between translation units (ODR violation → global memory corruption → boot
// crash in first nvs_open). Library per-board defaults apply (ESP32: 24).
#include <WiFiUdp.h>
#include <TrxNet.h>
#include "icomLanClient.h"      // LAN CI-V transport (alternative to BT)
IcomLanClient lanClient;
bool    lanMode = true;         // true when transceiverType == "IC-705-LAN" (LAN is the default)
String  lanRadioIp = "";        // radio IP for LAN mode
String  lanUser = "";           // ICOM network username
String  lanPass = "";           // ICOM network password
uint32_t lanFreqTmp = 0;        // last freq published to TrxNet (change detect)
WiFiUDP trxUdp;
TrxNet  net(trxUdp);
char    trxDeviceName[TRXNET_MAX_DEVICE_NAME];
bool    trxNetEnabled = false; // set true after net.begin() succeeds

// TrxNet priority prefixes — protect these peers when the peer table fills.
// EEPROM 288 flag + 289-359 string. Limits: 8 tokens x 8 chars (see setPriorityPrefixes()).
#define TRXNET_PRIO_MAX_TOKENS 8
#define TRXNET_PRIO_MAX_TOKLEN 8
#define TRXNET_PRIO_STR_MAX    71   // 8*8 + 7 separators
String      TRXNET_PRIO = "OI3 ANT";                 // canonical (normalized) value for UI/save
char        trxPrioBuf[TRXNET_PRIO_STR_MAX + 1];     // tokenized: spaces -> '\0' (library holds a pointer)
const char* trxPrioPtrs[TRXNET_PRIO_MAX_TOKENS];     // pointers into trxPrioBuf, valid for object lifetime
uint8_t     trxPrioCount = 0;

// TrxNet pending state — set in callbacks, processed in loop()
volatile uint32_t trxPendingHz   = 0;
volatile uint8_t  trxPendingMode = 0; // CI-V byte
volatile bool     trxFreqPending = false;
volatile bool     trxModePending = false;

int incomingByte = 0;   // for incoming serial data

#if defined(WIFI)
  WiFiServer dxcRawServer(82);
  WiFiClient DxcTelnetClient;
  WiFiClient DxcWsClient;
  String DxcHost = "";
  uint16_t DxcPort = 7300;
  IPAddress DxcHostIp;                // cached resolve — hostByName blocks the loop for seconds
  String DxcHostResolved = "";        // host the cache belongs to
  #define DXC_CONNECT_TIMEOUT_MS 1500
  String DxcCallsign = "";
  String DxcLocator = "";

  static const char* LOG_CONFIG_PATH = "/log-config.json";

  // In-memory cache of log-config.json fields used by setupTemplateProcessor.
  String g_lcTrx1Label = "IC-705";
  String g_lcTrx2Label = "TRX2";
  String g_lcTrx3Label = "TRX3";
  String g_lcRstSsb    = "59";
  String g_lcRstCwRtty = "599";
  bool   g_lcManualModeForPhone = true;
  String g_lcBlockedDxcc = "Russia\nBelarus\nKaliningrad";

  // TrxNet peer cache for TRX2 (index 0) and TRX3 (index 1) — written by onHz/onMode callbacks
  static volatile long g_trxFreq[2]    = {0, 0};
  static char          g_trxMode[2][8] = {"USB", "USB"};
  static volatile bool g_trxHasData[2] = {false, false};

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
  // BT CAT poll cadence (used by pollRadio) — slow when idle, fast while the
  // CAT page holds it via /state?fast=1
  #define CAT_POLL_MS       1000  // slow: freq poll interval
  #define CAT_MODE_EVERY    2     // slow: mode polled every N-th freq tick (2 s)
  #define AUX_POLL_MS       160   // slow: aux round-robin step (each of 10 items every 1.6 s)
  #define CAT_POLL_FAST_MS  200   // fast: freq+mode poll interval (original cadence)
  #define AUX_POLL_FAST_MS  80    // fast: aux round-robin step
  #define CAT_FAST_HOLD_MS  3000  // fast mode lingers this long after the last ?fast=1 poll
  #define CAT_POLL_OUTAGE_MS 3000 // WiFi outage: freq-only poll — BT must go near-silent
                                  // so the SPP link can sniff and WiFi can re-associate
  unsigned long catFastUntil = 0;
  String setupSsidErr = "";
  String setupPswdErr = "";
  String setupSsid2Err = "";
  String setupPswd2Err = "";
  String setupCivAddrErr = "";
  bool setupSaveOk = false;
  String transceiverType = "IC-705-LAN";  // LAN is the recommended default (BT deprecated)
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
  String configJsonEscape(const String &s);
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
  bool sendTemplatedHtml(const char *path);
  void handleSetupData(void);
  void handleTrxNetPeers(void);
  void handleWebServerLoop(void);
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
  void handleOi3SetHz(void);
  void TrxNetLoop(void);
  void onTrxHz(const char* from, const uint8_t* data, size_t len);
  void onTrxMode(const char* from, const uint8_t* data, size_t len);
  void onTrxSetHz(const char* from, const uint8_t* data, size_t len);
  void DxcLoop(void);
  void dxcHandleRawClient(void);
  bool DxcConfigReady(void);
  void DxcDisconnectTelnet(void);
  void DxcDisconnectWebSocket(void);
  void DxcRequestReconnect(void);
  void DxcUpdateTelnetStatus(bool connected, bool forceSend = false);
  void DxcSendTelnetStatus(void);
  bool DxcConnectTelnet(void);
  bool WiFiStationReady(void);
  void WiFiRetryActiveProfile(const char *reason);
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
    if (configuredType == "IC-7610-CI-V" || configuredType == "IC-705-LAN") {
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

  // LAN transport config (backward compatible: absent lines -> empty)
  if (file.available()) lanRadioIp = trimMemoryValue(file.readStringUntil('\n'), 15);
  if (file.available()) lanUser    = trimMemoryValue(file.readStringUntil('\n'), 16);
  if (file.available()) lanPass    = trimMemoryValue(file.readStringUntil('\n'), 16);

  lanMode = (transceiverType == "IC-705-LAN");

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
  file.println(lanRadioIp);
  file.println(lanUser);
  file.println(lanPass);

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
  v = extractJsonString(j, "rstSsb");    if (v.length() > 0) g_lcRstSsb = v;
  v = extractJsonString(j, "rstCwRtty"); if (v.length() > 0) g_lcRstCwRtty = v;
  if (j.indexOf("\"blockedDxcc\"") >= 0) g_lcBlockedDxcc = extractJsonString(j, "blockedDxcc");
  g_lcManualModeForPhone = extractJsonBool(j, "manualModeForPhone", true);
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
          case 'u':  {
            // \uXXXX — decode 4 hex digits to char (BMP only, basic ASCII range)
            if (i + 4 < (int)json.length()) {
              char h[5] = { json.charAt(i+1), json.charAt(i+2), json.charAt(i+3), json.charAt(i+4), '\0' };
              unsigned int cp = (unsigned int)strtoul(h, nullptr, 16);
              if (cp < 0x80) out += (char)cp;
              i += 4;
            }
            break;
          }
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
  const String &rstSsb,
  const String &rstCwRtty,
  bool manualModeForPhone,
  const String &blockedDxcc
) {
  String json;
  json.reserve(256);
  json += "{";
  json += "\"trx1Label\":\""; json += jsonEscape(trx1Label); json += "\"";
  json += ",\"trx2Label\":\""; json += jsonEscape(trx2Label); json += "\"";
  json += ",\"trx3Label\":\""; json += jsonEscape(trx3Label); json += "\"";
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
  if (key == "TRXNET_PORT") return String(TRXNET_PORT);
  if (key == "TRXNET_ID") { char h[3]; snprintf(h, sizeof(h), "%02x", TRXNET_ID); return String(h); }
  if (key == "TRX2_NET_ID") { char h[3]; snprintf(h, sizeof(h), "%02x", TRX2_NET_ID); return String(h); }
  if (key == "TRX3_NET_ID") { char h[3]; snprintf(h, sizeof(h), "%02x", TRX3_NET_ID); return String(h); }
  if (key == "TRXNET_DEVICE_NAME") return TRXNET_ID != 0x00 ? String(trxDeviceName) : String("disabled");
  if (key == "TRXNET_AP_NOTE") return APmode
    ? "<span class=\"ap-note\">TrxNet is not active in AP mode — requires WiFi station mode.</span>"
    : "";
  if (key == "TRX1_LABEL") return g_lcTrx1Label;
  if (key == "TRX2_LABEL") return g_lcTrx2Label;
  if (key == "TRX3_LABEL") return g_lcTrx3Label;
  if (key == "RST_SSB") return g_lcRstSsb;
  if (key == "RST_CW_RTTY") return g_lcRstCwRtty;
  if (key == "MANUAL_MODE_FOR_PHONE_CHK") return g_lcManualModeForPhone ? "checked" : "";
  if (key == "BLOCKED_DXCC") return g_lcBlockedDxcc;
  if (key == "SSID_ERR") return setupSsidErr;
  if (key == "PSWD_ERR") return setupPswdErr;
  if (key == "SSID2_ERR") return setupSsid2Err;
  if (key == "PSWD2_ERR") return setupPswd2Err;
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

bool sendTemplatedHtml(const char *path){
  webQuietUntil = millis() + 1500;
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
  if (!SPIFFS.exists(path)) {
    String msg = "Missing ";
    msg += path;
    msg += " in SPIFFS";
    webServer.send(500, "text/plain", msg);
    return false;
  }
  File file = SPIFFS.open(path, "r");
  if (!file) {
    webServer.send(500, "text/plain", "File open failed");
    return false;
  }

  String page;
  page.reserve(file.size() + 1024);
  String line;
  line.reserve(256);
  file.setTimeout(10);

  while (file.available()) {
    #if defined(WDT)
      esp_task_wdt_reset();
    #endif
    line = file.readStringUntil('\n');
    int start = 0;
    while (true) {
      int p1 = line.indexOf('%', start);
      if (p1 < 0) { page += line.substring(start); break; }
      int p2 = line.indexOf('%', p1 + 1);
      if (p2 < 0) { page += line.substring(start); break; }
      page += line.substring(start, p1);
      page += setupTemplateProcessor(line.substring(p1 + 1, p2));
      start = p2 + 1;
    }
    if (file.available()) page += '\n';
  }
  file.close();

  webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  webServer.sendHeader("Pragma", "no-cache");
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
  webServer.setContentLength(page.length());
  webServer.send(200, "text/html", page);
  webQuietUntil = millis() + 1500;
  return true;
}

void renderSetupPage(){
  handleFileFromSPIFFS("/setup.html");
}

void renderIndexPage(){
  if (!sendTemplatedHtml("/index.html")) {
    return;
  }
}

void handleSetupData(){
  char civHex[3];
  snprintf(civHex, sizeof(civHex), "%02X", configuredCivAddress);
  char trxnetHex[3], trx2Hex[3], trx3Hex[3];
  snprintf(trxnetHex, sizeof(trxnetHex), "%02x", TRXNET_ID);
  snprintf(trx2Hex, sizeof(trx2Hex), "%02x", TRX2_NET_ID);
  snprintf(trx3Hex, sizeof(trx3Hex), "%02x", TRX3_NET_ID);

  int baudSelect = 3;
  if (BaudRate == 1200) baudSelect = 0;
  else if (BaudRate == 2400) baudSelect = 1;
  else if (BaudRate == 4800) baudSelect = 2;
  else if (BaudRate == 9600) baudSelect = 3;
  else if (BaudRate == 115200) baudSelect = 4;

  uint8_t ipLastOctet = APmode ? 0 : (uint8_t)WiFi.localIP()[3];
  bool trxnetidIsDefault = (EEPROM.read(41) == 0xff);

  String j;
  j.reserve(2400);
  j += "{\"fwRev\":"; j += (unsigned)REV;
  j += ",\"apMode\":"; j += APmode ? "true" : "false";
  j += ",\"apModeText\":\""; j += APmode ? "AP mode ON" : "AP mode OFF"; j += "\"";
  j += ",\"mac\":\""; j += configJsonEscape(MACString); j += "\"";
  j += ",\"hwRev\":"; j += HardwareRev;
  j += ",\"ipLastOctet\":"; j += ipLastOctet;
  j += ",\"trxnetidIsDefault\":"; j += trxnetidIsDefault ? "true" : "false";
  j += ",\"trxnetDeviceName\":\""; j += configJsonEscape(TRXNET_ID != 0x00 ? String(trxDeviceName) : String("disabled")); j += "\"";
  j += ",\"trxnetApNote\":\""; j += APmode ? "TrxNet is not active in AP mode - requires WiFi station mode." : ""; j += "\"";
  j += ",\"ssid\":\""; j += configJsonEscape(SSID); j += "\"";
  j += ",\"pswd\":\""; j += configJsonEscape(PSWD); j += "\"";
  j += ",\"ssid2\":\""; j += configJsonEscape(SSID2); j += "\"";
  j += ",\"pswd2\":\""; j += configJsonEscape(PSWD2); j += "\"";
  j += ",\"trxnetid\":\""; j += trxnetHex; j += "\"";
  j += ",\"trxnetport\":\""; j += TRXNET_PORT; j += "\"";
  j += ",\"trxnetprio\":\""; j += configJsonEscape(TRXNET_PRIO); j += "\"";
  j += ",\"baud\":\""; j += baudSelect; j += "\"";
  j += ",\"trx1label\":\""; j += configJsonEscape(g_lcTrx1Label); j += "\"";
  j += ",\"trx1transport\":\""; j += (transceiverType == "IC-705-LAN") ? "lan" : "bluetooth"; j += "\"";
  j += ",\"civaddr\":\""; j += civHex; j += "\"";
  j += ",\"lanip\":\"";   j += configJsonEscape(lanRadioIp); j += "\"";
  j += ",\"lanuser\":\""; j += configJsonEscape(lanUser); j += "\"";
  j += ",\"lanpass\":\""; j += configJsonEscape(lanPass); j += "\"";
  j += ",\"btname\":\""; j += configJsonEscape(BT_NAME); j += "\"";
  j += ",\"cwIpOnConnect\":"; j += cwIpOnConnect ? "true" : "false";
  j += ",\"trx2label\":\""; j += configJsonEscape(g_lcTrx2Label); j += "\"";
  j += ",\"trx2netid\":\""; j += trx2Hex; j += "\"";
  j += ",\"trx2conntype\":"; j += TRX2_CONN_TYPE;
  { char h[3]; snprintf(h, sizeof(h), "%02x", TRX2_CIV_ADDR); j += ",\"trx2civaddr\":\""; j += h; j += "\""; }
  j += ",\"trx3label\":\""; j += configJsonEscape(g_lcTrx3Label); j += "\"";
  j += ",\"trx3netid\":\""; j += trx3Hex; j += "\"";
  j += ",\"trx3conntype\":"; j += TRX3_CONN_TYPE;
  { char h[3]; snprintf(h, sizeof(h), "%02x", TRX3_CIV_ADDR); j += ",\"trx3civaddr\":\""; j += h; j += "\""; }
  j += ",\"dxchost\":\""; j += configJsonEscape(DxcHost); j += "\"";
  j += ",\"dxcport\":\""; j += DxcPort > 0 ? String(DxcPort) : ""; j += "\"";
  j += ",\"dxccall\":\""; j += configJsonEscape(DxcCallsign); j += "\"";
  j += ",\"dxclocator\":\""; j += configJsonEscape(DxcLocator); j += "\"";
  j += ",\"rstSsb\":\""; j += configJsonEscape(g_lcRstSsb); j += "\"";
  j += ",\"rstCwRtty\":\""; j += configJsonEscape(g_lcRstCwRtty); j += "\"";
  j += ",\"manualModeForPhone\":"; j += g_lcManualModeForPhone ? "true" : "false";
  j += ",\"blockedDxcc\":\""; j += configJsonEscape(g_lcBlockedDxcc); j += "\"";
  for (uint8_t i = 0; i < CW_MEMORY_COUNT; i++) {
    j += ",\"cwmem"; j += (i + 1); j += "\":\""; j += configJsonEscape(cwMemoryText[i]); j += "\"";
  }
  for (uint8_t i = 0; i < FREQ_MEMORY_COUNT; i++) {
    j += ",\"freqmem"; j += (i + 1); j += "\":\""; j += configJsonEscape(freqMemoryText[i]); j += "\"";
  }
  j += "}";
  webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  webServer.sendHeader("Pragma", "no-cache");
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
  webServer.send(200, "application/json", j);
}

// Live list of visible TrxNet devices (peer table) for the setup page.
// state: "ap" (AP mode) / "disabled" (NET_ID 0x00) / "ok". prio computed server-side.
void handleTrxNetPeers(){
  String j;
  j.reserve(1024);
  j += "{";
  if (APmode) {
    j += "\"state\":\"ap\",\"self\":\"\",\"peers\":[]";
  } else if (!trxNetEnabled) {
    j += "\"state\":\"disabled\",\"self\":\"\",\"peers\":[]";
  } else {
    j += "\"state\":\"ok\",\"self\":\""; j += configJsonEscape(String(trxDeviceName)); j += "\",\"peers\":[";
    uint32_t now = millis();
    int count = net.peerCount();
    bool first = true;
    for (int i = 0; i < count; i++) {
      const TrxPeer* p = net.peer(i);
      if (!p || !p->active) continue;
      if (!first) j += ",";
      first = false;
      uint32_t age = (now - p->lastSeen) / 1000;
      j += "{\"name\":\""; j += configJsonEscape(String(p->name)); j += "\"";
      j += ",\"ip\":\"";   j += p->ip.toString(); j += "\"";
      j += ",\"age\":";    j += age;
      j += ",\"prio\":";   j += trxIsPriorityName(p->name) ? "true" : "false";
      j += "}";
    }
    j += "]";
  }
  j += "}";
  webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
  webServer.send(200, "application/json", j);
}

void handleWebServerLoop(){
  unsigned long start = millis();
  if (APmode) dnsServer.processNextRequest();   // captive portal
  webServer.handleClient();
  unsigned long elapsed = millis() - start;
  if (elapsed > LOOP_WARN_MS) {
    // lastUri/lastPeer = request whose response was written most recently; for a
    // write stall (the 10s blocks) that is exactly the connection that stalled
    Serial.print("LOOP| slow: webServer ");
    Serial.print(elapsed);
    Serial.print("ms uri=");
    Serial.print(webServer.lastUri);
    Serial.print(" peer=");
    Serial.print(webServer.lastPeer);
    Serial.print(" wifi=");
    Serial.print((int)WiFi.status());
    Serial.print(" ip=");
    Serial.println(WiFi.localIP());
  }
}

void buildStateJson(char *buf, size_t bufSize){
  char modesSnapshot[sizeof(modes)];
  copyModesText(modesSnapshot, sizeof(modesSnapshot));
  char addrStr[5];
  snprintf(addrStr, sizeof(addrStr), "0x%02X", radio_address);
  bool radioLinked = lanMode ? lanClient.connected() : btClientConnected;
  const char *btStat = lanMode ? (lanClient.connected() ? "LAN linked" : "LAN connecting") :
                       (!btClientConnected ? "BT idle" :
                       (radio_address == 0x00 ? "BT linked | searching CI-V" : "BT linked"));
  const char *wifiStat = APmode ? "WiFi AP" :
                         (WiFiStationReady() ? "WiFi STA" : "WiFi down");
  int rssi = (APmode || !WiFiStationReady()) ? -999 : (int)WiFi.RSSI();
  snprintf(buf, bufSize,
    "{\"connected\":%s,\"btStatus\":\"%s\",\"wifiStatus\":\"%s\","
    "\"wifiRssi\":%d,\"fwRev\":\"%u\",\"power\":%s,"
    "\"frequency\":%u,\"mode\":\"%s\",\"filter\":%u,"
    "\"radioAddress\":\"%s\",\"transceiverType\":\"%s\",\"tx\":%s,\"ritRaw\":%u,"
    "\"smeterRaw\":%u,\"powerMeterRaw\":%u,"
    "\"afGain\":%u,\"keySpeed\":%u,\"rfPower\":%u,"
    "\"supplyVolts\":%.2f,\"swr\":%.2f,"
    "\"preamp\":%u,\"vox\":%u,\"dxcConnected\":%s}",
    radioLinked ? "true" : "false", btStat, wifiStat,
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
  // CAT page polls /state?fast=1 — hold the fast BT poll cadence while it's open
  if (webServer.arg("fast") == "1") catFastUntil = millis() + CAT_FAST_HOLD_MS;
  static char stateBuf[640];
  buildStateJson(stateBuf, sizeof(stateBuf));
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
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
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
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
  webQuietUntil = millis() + 1500;
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
  webServer.sendHeader("Connection", "close");
  webServer.setContentLength(f.size());
  webServer.send(200, contentType, "");

  static uint8_t buf[1024];
  while (f.available()) {
    #if defined(WDT)
      esp_task_wdt_reset();
    #endif
    size_t n = f.read(buf, sizeof(buf));
    if (n > 0) webServer.sendContent((const char*)buf, n);
  }
  f.close();
  webQuietUntil = millis() + 1500;
  return true;
}

void handlePostCmd(){
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
  String body = webServer.arg("plain");
  if (body.length() == 0) { webServer.send(400, "application/json", "{\"error\":\"empty body\"}"); return; }
  String type = extractJsonString(body, "type");

  if (type == "abortCw") {
    #if defined(UDP_TO_FSK)
    char modesSnapshot[sizeof(modes)];
    copyModesText(modesSnapshot, sizeof(modesSnapshot));
    if (strcmp(modesSnapshot, "CW") == 0 && radioLinkUp() && radio_address != 0x00) {
      uint8_t frame[] = {START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS,
                         CMD_SEND_CW_MSG, 0xFF, STOP_BYTE};
      catWriteFrame(frame, sizeof(frame), true);
    } else if (strcmp(modesSnapshot, "RTTY") == 0) {
      abortFskTransmission = true;
    }
    #else
    if (radioLinkUp() && radio_address != 0x00) {
      char modesSnapshot[sizeof(modes)];
      copyModesText(modesSnapshot, sizeof(modesSnapshot));
      if (strcmp(modesSnapshot, "CW") == 0) {
        uint8_t frame[] = {START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS,
                           CMD_SEND_CW_MSG, 0xFF, STOP_BYTE};
        catWriteFrame(frame, sizeof(frame), true);
      }
    }
    #endif
    webServer.send(200, "application/json", "{\"ok\":true}");
    return;
  }

  if (!radioLinkUp() || radio_address == 0x00) {
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

String configJsonEscape(const String &s) {
  String out;
  out.reserve(s.length() + 4);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"')       out += "\\\"";
    else if (c == '\\') out += "\\\\";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else if ((uint8_t)c < 0x20) {
      char esc[7];
      snprintf(esc, sizeof(esc), "\\u%04x", (uint8_t)c);
      out += esc;
    } else {
      out += c;
    }
  }
  return out;
}

static void eepromWriteStr(const String &str, int addr, int maxLen) {
  for (int i = 0; i < maxLen; i++) {
    EEPROM.write(addr + i, (i < (int)str.length()) ? (uint8_t)str[i] : 0xff);
  }
}

// Normalize raw space-separated priority prefixes: uppercase, whitespace-collapsed,
// clamped to TRXNET_PRIO_MAX_TOKENS tokens x TRXNET_PRIO_MAX_TOKLEN chars. No globals touched.
static String trxNormalizePrio(const String &raw) {
  String out;
  uint8_t tokens = 0;
  int i = 0, n = raw.length();
  while (i < n && tokens < TRXNET_PRIO_MAX_TOKENS) {
    while (i < n && raw[i] == ' ') i++;
    if (i >= n) break;
    String tok;
    while (i < n && raw[i] != ' ') {
      if (tok.length() < TRXNET_PRIO_MAX_TOKLEN) tok += (char)toupper((unsigned char)raw[i]);
      i++;   // overflow chars of this token are dropped
    }
    if (tok.length() == 0) continue;
    if (out.length()) out += ' ';
    out += tok;
    tokens++;
  }
  return out;
}

// Fill the live token buffers from a normalized string (boot only — the library keeps
// a pointer into trxPrioBuf, so this must not run while net is actively using it).
static void trxLoadPrioBuffers(const String &canonical) {
  strncpy(trxPrioBuf, canonical.c_str(), TRXNET_PRIO_STR_MAX);
  trxPrioBuf[TRXNET_PRIO_STR_MAX] = '\0';
  trxPrioCount = 0;
  bool inTok = false;
  for (int k = 0; trxPrioBuf[k] != '\0'; k++) {
    if (trxPrioBuf[k] == ' ') { trxPrioBuf[k] = '\0'; inTok = false; }
    else if (!inTok) {
      if (trxPrioCount < TRXNET_PRIO_MAX_TOKENS) trxPrioPtrs[trxPrioCount++] = &trxPrioBuf[k];
      inTok = true;
    }
  }
}

// True if a device name matches any active priority prefix (same strncmp
// prefix semantics as TrxNet::setPriorityPrefixes()).
static bool trxIsPriorityName(const char* name) {
  for (uint8_t i = 0; i < trxPrioCount; i++) {
    if (trxPrioPtrs[i] && strncmp(name, trxPrioPtrs[i], strlen(trxPrioPtrs[i])) == 0)
      return true;
  }
  return false;
}

// Persist a normalized prefix string: flag 0x01 + string (empty string = priority off).
static void eepromWriteTrxPrio(const String &s) {
  EEPROM.writeByte(288, 0x01);
  for (int i = 0; i < TRXNET_PRIO_STR_MAX; i++) {
    EEPROM.write(289 + i, (i < (int)s.length()) ? (uint8_t)s[i] : 0xff);
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
  j += ",\"trxnetid\":";        j += (unsigned)TRXNET_ID;
  j += ",\"trxnetport\":";      j += TRXNET_PORT;
  j += ",\"trxnetprio\":\"";    j += configJsonEscape(TRXNET_PRIO); j += "\"";
  j += ",\"trx2netid\":";       j += (unsigned)TRX2_NET_ID;
  j += ",\"trx2conntype\":";    j += TRX2_CONN_TYPE;
  { char h[3]; snprintf(h, sizeof(h), "%02x", TRX2_CIV_ADDR); j += ",\"trx2civaddr\":\""; j += h; j += "\""; }
  j += ",\"trx3netid\":";       j += (unsigned)TRX3_NET_ID;
  j += ",\"trx3conntype\":";    j += TRX3_CONN_TYPE;
  { char h[3]; snprintf(h, sizeof(h), "%02x", TRX3_CIV_ADDR); j += ",\"trx3civaddr\":\""; j += h; j += "\""; }
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
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
  webServer.send(200, "application/json", j);
}

void handleConfigUpload() {
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
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
  v = parseField("trxnetid",  0, 255); if (v >= 0) { TRXNET_ID   = (byte)v; EEPROM.writeByte(41, v); }
  v = parseField("trx2netid", 0, 255); if (v >= 0) { TRX2_NET_ID = (byte)v; EEPROM.writeByte(42, v); }
  v = parseField("trx3netid", 0, 255); if (v >= 0) { TRX3_NET_ID = (byte)v; EEPROM.writeByte(43, v); }
  // EEPROM 44 TRX2_CONN_TYPE
  v = parseField("trx2conntype", 0, 1); if (v >= 0) { TRX2_CONN_TYPE = (byte)v; EEPROM.writeByte(44, v); }
  v = parseField("trxnetport", 1, 65534); if (v >= 0) { TRXNET_PORT = (uint16_t)v; EEPROM.writeUShort(45, v); }
  // EEPROM 288 flag + 289-359 TRXNET_PRIO
  if (body.indexOf("\"trxnetprio\"") >= 0) {
    TRXNET_PRIO = trxNormalizePrio(extractJsonString(body, "trxnetprio"));
    eepromWriteTrxPrio(TRXNET_PRIO);
  }
  // EEPROM 47 TRX3_CONN_TYPE
  v = parseField("trx3conntype", 0, 1); if (v >= 0) { TRX3_CONN_TYPE = (byte)v; EEPROM.writeByte(47, v); }
  // EEPROM 48/49 TRX2/3 CI-V address (hex string)
  {
    uint8_t a;
    String s2 = extractJsonString(body, "trx2civaddr");
    if (s2.length() > 0 && parseHexByteString(s2, a)) { TRX2_CIV_ADDR = a; EEPROM.writeByte(48, a); }
    String s3 = extractJsonString(body, "trx3civaddr");
    if (s3.length() > 0 && parseHexByteString(s3, a)) { TRX3_CIV_ADDR = a; EEPROM.writeByte(49, a); }
  }
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
  String spiffsJson = "{}";
  if (SPIFFS.exists(LOG_CONFIG_PATH)) {
    File f = SPIFFS.open(LOG_CONFIG_PATH, "r");
    if (f) { spiffsJson = f.readString(); f.close(); spiffsJson.trim(); }
  }
  // Inject EEPROM TrxNet peer IDs + CI-V conn type so frontend can derive whether
  // TRX2/3 is a remote (non-local) TRX, regardless of transport.
  String inject;
  inject += ",\"trx2netid\":"; inject += (unsigned)TRX2_NET_ID;
  inject += ",\"trx3netid\":"; inject += (unsigned)TRX3_NET_ID;
  inject += ",\"trx2conntype\":"; inject += (unsigned)TRX2_CONN_TYPE;
  inject += ",\"trx3conntype\":"; inject += (unsigned)TRX3_CONN_TYPE;
  String out;
  out.reserve(spiffsJson.length() + inject.length() + 4);
  if (spiffsJson.startsWith("{") && spiffsJson.length() > 2) {
    out = spiffsJson.substring(0, spiffsJson.length() - 1);
    out += inject;
    out += "}";
  } else {
    out = "{"; out += inject.substring(1); out += "}";
  }
  webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  webServer.sendHeader("Pragma", "no-cache");
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
  webServer.send(200, "application/json", out);
}

void handlePostLogConfig() {
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
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


void handleOi3State() {
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
  String trxArg = webServer.arg("trx");
  int idx = trxArg.toInt() - 2; // trx=2 → 0, trx=3 → 1
  if (idx < 0 || idx > 1) {
    webServer.send(400, "application/json", "{\"error\":\"invalid_trx\"}");
    return;
  }
  char jbuf[96];
  snprintf(jbuf, sizeof(jbuf), "{\"connected\":%s,\"frequency\":%ld,\"mode\":\"%s\",\"dxcConnected\":%s}",
    g_trxHasData[idx] ? "true" : "false", g_trxFreq[idx], g_trxMode[idx],
    DxcTelnetStatus ? "true" : "false");
  webServer.send(200, "application/json", jbuf);
}

void handleOi3Send() {
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
  String body = webServer.arg("plain");
  String trxArg = extractJsonString(body, "trx");
  String text   = extractJsonString(body, "text");
  if (text.length() == 0) {
    webServer.send(400, "application/json", "{\"error\":\"missing\"}");
    return;
  }
  int trxN = trxArg.toInt();  // 2 or 3
  byte peerNetId = (trxN == 3) ? TRX3_NET_ID : TRX2_NET_ID;
  if (peerNetId == 0x00 || !trxNetEnabled) {
    webServer.send(503, "application/json", "{\"error\":\"unavailable\"}");
    return;
  }
  char peerName[TRXNET_MAX_DEVICE_NAME];
  snprintf(peerName, sizeof(peerName), "OI3.%02x", peerNetId);
  // Legacy abort: old clients sent text='\x03' (ETX); after extractJsonString fix it
  // arrives as a single 0x03 byte — treat it as abort, not CW text.
  if (text.length() == 1 && (uint8_t)text[0] == 0x03) {
    uint8_t stopByte = 0xFF;
    net.publishTo(peerName, "/s-cw", &stopByte, 1, TRX_CON);
    webServer.send(200, "application/json", "{\"ok\":true}");
    return;
  }
  if (!net.publishTo(peerName, "/s-cw", (const uint8_t*)text.c_str(), text.length(), TRX_CON)) {
    webServer.send(503, "application/json", "{\"error\":\"peer_unavailable\"}");
    return;
  }
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleOi3AbortCw() {
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
  String body = webServer.arg("plain");
  String trxArg = extractJsonString(body, "trx");
  int trxN = trxArg.toInt();  // 2 or 3
  byte peerNetId = (trxN == 3) ? TRX3_NET_ID : TRX2_NET_ID;
  if (peerNetId == 0x00 || !trxNetEnabled) {
    webServer.send(503, "application/json", "{\"error\":\"unavailable\"}");
    return;
  }
  char peerName[TRXNET_MAX_DEVICE_NAME];
  snprintf(peerName, sizeof(peerName), "OI3.%02x", peerNetId);
  uint8_t stopByte = 0xFF;
  if (!net.publishTo(peerName, "/s-cw", &stopByte, 1, TRX_CON)) {
    webServer.send(503, "application/json", "{\"error\":\"peer_unavailable\"}");
    return;
  }
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleOi3SetHz() {
  webServer.sendHeader("Connection", "close");
  webServer.client().setNoDelay(true);
  String body = webServer.arg("plain");
  String trxArg = extractJsonString(body, "trx");
  int trxN = trxArg.toInt();  // 2 or 3
  // hz comes as a number in JSON — parse manually
  int hzIdx = body.indexOf("\"hz\":");
  if (hzIdx < 0) { webServer.send(400, "application/json", "{\"error\":\"missing\"}"); return; }
  int start = body.indexOf(':', hzIdx) + 1;
  while (start < (int)body.length() && body[start] == ' ') start++;
  int end = start;
  while (end < (int)body.length() && isDigit(body[end])) end++;
  uint32_t hz = (uint32_t)body.substring(start, end).toInt();
  if (hz == 0) { webServer.send(400, "application/json", "{\"error\":\"bad_hz\"}"); return; }

  byte connType = (trxN == 3) ? TRX3_CONN_TYPE : TRX2_CONN_TYPE;
  if (connType == 1) {
    // CI-V transport: write frequency directly on the serial bus
    uint8_t civAddr = (trxN == 3) ? TRX3_CIV_ADDR : TRX2_CIV_ADDR;
    if (civAddr == 0x00) {
      webServer.send(503, "application/json", "{\"error\":\"unavailable\"}");
      return;
    }
    civWriteFreq(civAddr, hz);
    webServer.send(200, "application/json", "{\"ok\":true}");
    return;
  }

  byte peerNetId = (trxN == 3) ? TRX3_NET_ID : TRX2_NET_ID;
  if (peerNetId == 0x00 || !trxNetEnabled) {
    webServer.send(503, "application/json", "{\"error\":\"unavailable\"}");
    return;
  }
  char peerName[TRXNET_MAX_DEVICE_NAME];
  snprintf(peerName, sizeof(peerName), "OI3.%02x", peerNetId);
  if (!net.publishTo(peerName, "/s-hz", (const uint8_t*)&hz, sizeof(hz))) {
    webServer.send(503, "application/json", "{\"error\":\"peer_unavailable\"}");
    return;
  }
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void setupWebServer(void){
  webServer.on("/state", HTTP_GET, handleGetState);
  webServer.on("/cmd", HTTP_POST, handlePostCmd);
  webServer.on("/config/download", HTTP_GET,  handleConfigDownload);
  webServer.on("/config/upload",   HTTP_POST, handleConfigUpload);
  webServer.on("/log-config", HTTP_GET,  handleGetLogConfig);
  webServer.on("/log-config", HTTP_POST, handlePostLogConfig);
  webServer.on("/oi3/state",    HTTP_GET,  handleOi3State);
  webServer.on("/oi3/send",     HTTP_POST, handleOi3Send);
  webServer.on("/oi3/abort-cw", HTTP_POST, handleOi3AbortCw);
  webServer.on("/oi3/set-hz",   HTTP_POST, handleOi3SetHz);

  webServer.on("/", HTTP_GET, [](){
    if (!SPIFFS.exists("/index.html")) { renderSetupPage(); return; }
    renderIndexPage();
  });

  webServer.on("/setup/save", HTTP_POST, [](){
    webServer.sendHeader("Connection", "close");
    webServer.client().setNoDelay(true);
    handleSet();
    if (setupSaveOk) {
      webServer.send(200, "application/json", "{\"ok\":true}");
    } else {
      webServer.send(400, "application/json", "{\"ok\":false}");
    }
  });
  webServer.on("/setup",   HTTP_GET,  [](){ renderSetupPage(); });
  webServer.on("/setup",   HTTP_POST, [](){ handleSet(); renderSetupPage(); });
  webServer.on("/setup-data.json", HTTP_GET, handleSetupData);
  webServer.on("/trxnet-peers.json", HTTP_GET, handleTrxNetPeers);
  webServer.on("/restart", HTTP_POST, [](){
    webServer.sendHeader("Connection", "close");
    webServer.client().setNoDelay(true);
    webServer.send(200, "application/json", "{\"ok\":true}");
    delay(500);
    ESP.restart();
  });
  webServer.on("/ws-cat",   HTTP_GET,  [](){ handleFileFromSPIFFS("/ws-cat.html"); });
  webServer.on("/log",      HTTP_GET,  [](){ handleFileFromSPIFFS("/log.html"); });
  webServer.on("/datasync", HTTP_GET,  [](){ handleFileFromSPIFFS("/datasync.html"); });

  webServer.on("/dxcinfo", HTTP_GET, [](){
    webServer.sendHeader("Cache-Control", "no-cache");
    webServer.sendHeader("Connection", "close");
    webServer.client().setNoDelay(true);
    String j = "{\"locator\":\"" + DxcLocator + "\",\"callsign\":\"" + DxcCallsign
             + "\",\"trx2netid\":" + String((unsigned)TRX2_NET_ID)
             + ",\"trx3netid\":" + String((unsigned)TRX3_NET_ID) + "}";
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
    webServer.sendHeader("Connection", "close");
    webServer.client().setNoDelay(true);
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
    webServer.sendHeader("Connection", "close");
    webServer.client().setNoDelay(true);
    if (!bdEnabled) { webServer.send(403, "application/json", "{\"error\":\"hw\"}"); return; }
    File f = SPIFFS.open(BD_CONFIG_PATH, FILE_READ);
    if (!f) { webServer.send(404, "application/json", "{}"); return; }
    webServer.streamFile(f, "application/json");
    f.close();
  });
  webServer.on("/api/bd-config", HTTP_POST, [](){
    webServer.sendHeader("Connection", "close");
    webServer.client().setNoDelay(true);
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
                 (bdSource == 2) ? (uint32_t)g_trxFreq[0] :
                                   (uint32_t)g_trxFreq[1];
    bdUpdate(f);
    webServer.send(200, "application/json", "{\"ok\":true}");
  });
  webServer.on("/api/status", HTTP_GET, [](){
    webServer.sendHeader("Connection", "close");
    webServer.client().setNoDelay(true);
    String j = "{";
    j += "\"hwrev\":"; j += HardwareRev;
    j += ",\"trx1Label\":\""; j += g_lcTrx1Label; j += "\"";
    j += ",\"trx2Label\":\""; j += g_lcTrx2Label; j += "\"";
    j += ",\"trx3Label\":\""; j += g_lcTrx3Label; j += "\"";
    j += ",\"trx2freq\":"; j += g_trxFreq[0];
    j += ",\"trx3freq\":"; j += g_trxFreq[1];
    j += "}";
    webServer.sendHeader("Cache-Control", "no-cache");
    webServer.send(200, "application/json", j);
  });

  webServer.onNotFound([](){
    String path = webServer.uri();
    if (!handleFileFromSPIFFS(path)) {
      if (APmode) {   // captive portal: send every unknown request to the setup page
        webServer.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/setup", true);
        webServer.send(302, "text/plain", "");
        return;
      }
      webServer.sendHeader("Connection", "close");
      webServer.client().setNoDelay(true);
      webServer.send(404, "text/plain", "Not found");
    }
  });

  webServer.begin();
}


// True when the primary radio link is up, whichever transport it uses.
bool radioLinkUp(){
  return lanMode ? lanClient.connected() : btClientConnected;
}

bool catWriteFrame(const uint8_t *frame, size_t frameLen, bool broadcastTx){
  (void)broadcastTx;
  if (lanMode) {
    // frame = FE FE <radio> <ctrl> <cmd> <payload> FD. Strip the wrapper and
    // hand the body (cmd+payload) to the LAN client, which re-wraps with 0xE1.
    if (frameLen < 6) return false;
    return lanClient.sendCommand(frame + 4, frameLen - 5);
  }
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

    // 41 TRXNET_ID  (own NET_ID; 0x00=disabled; 0xff=unprogrammed → default 0x01)
    TRXNET_ID = (EEPROM.read(41) == 0xff) ? 0x01 : EEPROM.readByte(41);

    // 42 TRX2_NET_ID (peer for TRX2 BD slot; 0x00=disabled; 0xff=unprogrammed → default 0xff)
    TRX2_NET_ID = (EEPROM.read(42) == 0xff) ? 0xff : EEPROM.readByte(42);

    // 43 TRX3_NET_ID (peer for TRX3 BD slot; 0x00=disabled; 0xff=unprogrammed → default 0x00)
    TRX3_NET_ID = (EEPROM.read(43) == 0xff) ? 0x00 : EEPROM.readByte(43);

    // 44 TRX2_CONN_TYPE (0=TrxNet, 1=CI-V; 0xff=unprogrammed → default 0x00)
    TRX2_CONN_TYPE = (EEPROM.read(44) == 0xff) ? 0x00 : EEPROM.readByte(44);
    // 45-46   TRXNET_PORT
    TRXNET_PORT = (EEPROM.read(45) == 0xff) ? 5683 : EEPROM.readUShort(45);
    // 47 TRX3_CONN_TYPE (0=TrxNet, 1=CI-V; 0xff=unprogrammed → default 0x00)
    TRX3_CONN_TYPE = (EEPROM.read(47) == 0xff) ? 0x00 : EEPROM.readByte(47);
    // 48 TRX2_CIV_ADDR (CI-V address of TRX2; 0xff=unprogrammed → 0x00 unset)
    TRX2_CIV_ADDR = (EEPROM.read(48) == 0xff) ? 0x00 : EEPROM.readByte(48);
    // 49 TRX3_CIV_ADDR (CI-V address of TRX3; 0xff=unprogrammed → 0x00 unset)
    TRX3_CIV_ADDR = (EEPROM.read(49) == 0xff) ? 0x00 : EEPROM.readByte(49);
    // 50-67   FREE (was MQTT_TOPIC)
    // 115-135 FREE (was MQTT_TOPIC_RX)
    // 225-245 FREE (was TRX2_MQTT_ROOT)
    // 246-266 FREE (was TRX3_MQTT_ROOT)

    // 267-287 BT_NAME (21B)
    if(EEPROM.read(267)!=0xff){
      for (int i=267; i<288; i++){
        if(EEPROM.read(i)!=0xff){
          BT_NAME=BT_NAME+char(EEPROM.read(i));
        }
      }
    }

    // 288 TRXNET_PRIO flag / 289-359 string (0xff flag = tovární default "OI3 ANT"; else read, "" = off)
    if (EEPROM.read(288) == 0xff) {
      TRXNET_PRIO = "OI3 ANT";
    } else {
      TRXNET_PRIO = "";
      for (int i = 289; i < 360; i++) {
        uint8_t c = EEPROM.read(i);
        if (c == 0xff || c == 0x00) break;
        TRXNET_PRIO += char(c);
      }
    }
    TRXNET_PRIO = trxNormalizePrio(TRXNET_PRIO);   // canonical form
    trxLoadPrioBuffers(TRXNET_PRIO);               // fill trxPrioPtrs/trxPrioCount


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
      WiFi.softAP(ssidAP, passwordAP, 1, 0, 4); // ch1, visible, max 4 clients
      { // WPA/WPA2 mixed mode — fixes auth failure on modern clients (Win11/Android12+/iOS16+)
        wifi_config_t conf;
        esp_wifi_get_config(WIFI_IF_AP, &conf);
        conf.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
        esp_wifi_set_config(WIFI_IF_AP, &conf);
      }
      IPAddress IP = WiFi.softAPIP();
      MACString = WiFi.softAPmacAddress();
      Serial.print(" AP | IP address: ");
      Serial.println(IP);

      dnsServer.start(DNS_PORT, "*", IP);   // captive portal: resolve every host to us

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

      // Prewarm the DXC DNS cache while nothing time-critical runs yet — the
      // first hostByName may block for seconds on a slow resolver, which would
      // otherwise freeze the loop on the first telnet connect.
      if (DxcHost.length() > 0) {
        IPAddress dxcIp;
        if (WiFi.hostByName(DxcHost.c_str(), dxcIp)) {
          DxcHostIp = dxcIp;
          DxcHostResolved = DxcHost;
          Serial.print(" DXC| resolved ");
          Serial.print(DxcHost);
          Serial.print(" to ");
          Serial.println(dxcIp);
        } else {
          Serial.print(" DXC| DNS prewarm failed for ");
          Serial.println(DxcHost);
        }
      }

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



    #if defined(UDP_TO_FSK)
      pinMode(PTT,  OUTPUT);
        digitalWrite(PTT, LOW);
      pinMode(FSK_OUT,  OUTPUT);
        digitalWrite(FSK_OUT, LOW);
    #endif

    #if defined(BLUETOOTH)
      esp_bt_mem_release(ESP_BT_MODE_BLE);
      if (lanMode) {
        // LAN transport: never start the BT controller (no WiFi/BT coex at all)
        IPAddress rip;
        Serial.println("LAN | cfg ip=" + lanRadioIp + " user='" + lanUser +
                       "' passlen=" + String(lanPass.length()));
        if (!rip.fromString(lanRadioIp)) {
          Serial.println("LAN | bad radio IP in config — LAN not started");
        } else if (lanUser.length() == 0 || lanPass.length() == 0) {
          Serial.println("LAN | empty user/pass in config — use CLI 'L' to set them");
        } else {
          lanClient.begin(rip, 50001, lanUser.c_str(), lanPass.c_str(), configuredCivAddress);
          Serial.println("LAN | transport active (BT not started)");
        }
      } else {
        configRadioBaud(0);
      }
    #endif

    // TrxNet init — after WiFi is up; NET_ID 0x00 = disabled
    if (TRXNET_ID != 0x00) {
      snprintf(trxDeviceName, sizeof(trxDeviceName), "705.%02x", TRXNET_ID);
      net.setPort(TRXNET_PORT);
      net.setPriorityPrefixes(trxPrioCount ? trxPrioPtrs : NULL, trxPrioCount);  // before begin()
      net.begin(trxDeviceName);
      net.subscribe("/hz",   onTrxHz);
      net.subscribe("/mode", onTrxMode);
      net.subscribe("/s-hz", onTrxSetHz);
      trxNetEnabled = true;
      Serial.print("TRXNET| begin ");
      Serial.println(trxDeviceName);
    } else {
      Serial.println("TRXNET| disabled (NET_ID=0x00)");
    }

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
  #define _TIMED(name, call) { unsigned long _t = millis(); call; unsigned long _d = millis()-_t; if(_d > LOOP_WARN_MS) { Serial.print("LOOP| slow: " name " "); Serial.print(_d); Serial.println("ms"); } }
  _TIMED("Watchdog",        Watchdog())
  _TIMED("LanClient",       lanClientLoop())
  _TIMED("CLI",             serialPump())
  _TIMED("CIV",             civPollTick())
  #if defined(RTLE)
    _TIMED("rtleserver",    rtleserver.handleClient())
  #endif
  handleWebServerLoop();
  _TIMED("TrxNet",          TrxNetLoop())
  _TIMED("DxcLoop",         DxcLoop())
  _TIMED("dxcRaw",          dxcHandleRawClient())

  // TrxNet: process pending /s-hz command (set TRX1 VFO via CI-V)
  if (trxFreqPending) {
    trxFreqPending = false;
    uint32_t f = trxPendingHz;
    if (f > 0) {
      if (radioSetFrequency(f)) {
        if (Debug) Serial.printf("CAT | set VFO %lu\n", (unsigned long)f);
      } else {
        if (Debug) Serial.println("CAT | set VFO skipped, TRX not connected");
      }
    }
  }

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
    Serial.println("    | Client Disconnected");
    if (statusPower == 1 && trxNetEnabled) {
      uint32_t zero = 0;
      net.publish("/hz", (uint8_t*)&zero, sizeof(zero));
    }
  }
}

// BT CAT poll cadence. Every frame keeps the SPP link out of sniff mode and the
// coex arbiter then starves WiFi (TCP stalls → AP drops us), so idle cadence is
// slow. The CAT page requests /state?fast=1, which holds the fast cadence while
// it stays open; the log page and everything else run on the slow one.
void pollRadio(){
  static unsigned long catPollTimer = 0;
  static unsigned long auxPollTimer = 0;
  static uint8_t auxPollIndex = 0;
  static uint8_t modePollDivider = 0;

  if (!btClientConnected) return;
  bool fastCat = (long)(catFastUntil - millis()) > 0;
  // During a WiFi outage every BT frame keeps the SPP link out of sniff mode and
  // re-association then never succeeds (even with ESP_COEX_PREFER_WIFI). Web and
  // TrxNet are unreachable anyway — keep only a slow freq poll for the band decoder.
  bool wifiOutage = false;
  #if defined(WIFI)
    wifiOutage = !APmode && (WifiDownSince != 0);
  #endif

  unsigned long freqInterval = wifiOutage ? CAT_POLL_OUTAGE_MS
                                          : (fastCat ? CAT_POLL_FAST_MS : CAT_POLL_MS);
  if (millis() - catPollTimer > freqInterval) {
    catPollTimer = millis();
    if (radio_address == 0x00) {
      if (Debug == true) Serial.println("CAT | searching radio...");
      searchRadio();
    } else {
      sendCatRequest(CMD_READ_FREQ);
      if (!wifiOutage && (fastCat || ++modePollDivider >= CAT_MODE_EVERY)) {
        modePollDivider = 0;
        sendCatRequest(CMD_READ_MODE);
      }
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

  if (!wifiOutage && radio_address != 0x00 && millis() - auxPollTimer > (fastCat ? AUX_POLL_FAST_MS : AUX_POLL_MS)) {
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
  if (!lanMode) { handleBtEvents(); pollRadio(); }

  static unsigned long mqttFreqTimer = 0;

  // In LAN mode the PWR/freq state is owned by lanClientLoop(); skip the
  // BT-driven power/publish logic below (powerTimer is never fed over LAN).
  if (!lanMode) {
  // Power OUT/LED
  // powerTimer==0 => no radio data yet (fresh boot / disconnected): force OFF.
  // Without this, at boot millis() is still < 3000 so millis()-powerTimer would
  // read as "recent activity" and spuriously pulse PWR ON then OFF after 3s.
  if(powerTimer==0 || millis()-powerTimer > 3000){
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
    }
  }

  if(statusPower==1 && btClientConnected==true && radio_address!=0x00 && millis()-mqttFreqTimer > 2000 && frequencyTmp!=frequency){
    // TrxNet publish /hz on frequency change (2s throttle)
    if(trxNetEnabled){
      uint32_t f = (uint32_t)frequency;
      net.publish("/hz", (uint8_t*)&f, sizeof(f));
    }
    frequencyTmp=frequency;
    mqttFreqTimer=millis();

    // Band Decoder update on TRX1 frequency change
    if (bdEnabled && bdSource == 1) bdUpdate(frequency);
  }
  } // end if(!lanMode)

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
      if (WiFiStationReady()) {
        WifiDownSince = 0;
        WifiHardResetDone = false;
        if (WifiCoexPreferred) {
          esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
          WifiCoexPreferred = false;
          Serial.print(millis());
          Serial.println(" WIFI| coex back to balance");
        }
        uint8_t *bssid = WiFi.BSSID();
        if (bssid && (!wifiLastBssidValid || memcmp(wifiLastBssid, bssid, 6) != 0
                      || wifiLastChannel != WiFi.channel())) {
          memcpy(wifiLastBssid, bssid, 6);
          wifiLastChannel = WiFi.channel();
          wifiLastSsid = WiFi.SSID();
          wifiLastBssidValid = true;
        }
        WifiTimer = currentMillis;
      } else {
        if (WifiDownSince == 0) WifiDownSince = currentMillis;
        unsigned long downFor = currentMillis - WifiDownSince;
        wl_status_t st = WiFi.status();

        // With the BT link up the coex arbiter starves WiFi so badly that
        // re-association never succeeds — tilt RF time to WiFi until we are back.
        if (!WifiCoexPreferred && btClientConnected) {
          esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
          WifiCoexPreferred = true;
          Serial.print(millis());
          Serial.println(" WIFI| coex prefer WiFi (BT active)");
        }

        // status 255 = driver no longer initialized; only a reboot brings it back
        if ((downFor >= WIFI_RESTART_AFTER_MS || st == WL_NO_SHIELD) && !stateTx) {
          Serial.print(millis());
          Serial.println(" WIFI| unrecoverable, restarting ESP");
          delay(100);
          ESP.restart();
        }
        // Radio off/on at most once per outage, and never while the BT SPP link is
        // up — esp_wifi_stop() then times out ("wifi un-init") and repeating the
        // cycle kills the driver (rx buffer init errors, status=255).
        bool hardReset = false;
        if (downFor >= WIFI_HARD_RESET_AFTER_MS && !WifiHardResetDone) {
          WifiHardResetDone = true;
          hardReset = !btClientConnected;
        }

        if (st == WL_DISCONNECTED || st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL) {
          ActiveWifiProfile = NextWifiProfile(ActiveWifiProfile);
          WiFiRetryActiveProfile("status", hardReset);
        } else {
          WiFiRetryActiveProfile("no ip", hardReset);
        }
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
// LAN transport service loop. Non-blocking; drives lanClient and mirrors the
// decoded frequency/mode into the same globals the BT path fills, so the web UI,
// TrxNet publish and band decoder work unchanged. Auto-reconnects on failure.
void lanClientLoop(){
  if (!lanMode) return;
  static uint32_t lanRetryAt = 0;
  static uint32_t lanBackoff = 3000;
  lanClient.loop();

  if (lanClient.connected()) {
    lanBackoff = 3000;   // healthy link resets the reconnect backoff
    if (statusPower == 0) {
      statusPower = 1;
      digitalWrite(PowerOnPin, HIGH);
      Serial.println(" PWR| ON (LAN)");
    }
    // frequency/mode/meters are written directly by lanCivFrameHandler (shared
    // parser). Here we only propagate a frequency change to TrxNet + band decoder.
    if (trxNetEnabled && frequency != 0 && (uint32_t)frequency != lanFreqTmp) {
      uint32_t f = (uint32_t)frequency;
      net.publish("/hz", (uint8_t*)&f, sizeof(f));
      lanFreqTmp = (uint32_t)frequency;
      if (bdEnabled && bdSource == 1) bdUpdate(frequency);
    }
  } else {
    if (statusPower == 1) {
      statusPower = 0;
      digitalWrite(PowerOnPin, LOW);
      Serial.println(" PWR| OFF (LAN)");
      frequency = 0; frequencyTmp = 0; lanFreqTmp = 0;
      setModesText("OFF");
    }
    // On failure: release the radio session IMMEDIATELY (single-session radio
    // must free the old one before we retry), then wait an escalating backoff
    // so we don't hammer a radio that still holds a zombie session.
    if (lanClient.failed()) {
      lanClient.stop();                 // sends disconnect/token-release -> state IDLE
      lanRetryAt = millis() + lanBackoff;
      if (lanBackoff < 20000) lanBackoff *= 2;
      Serial.print("LAN | reconnect in "); Serial.print(lanBackoff/1000); Serial.println("s");
    } else if (lanClient.status() == IcomLanClient::LAN_IDLE && lanRetryAt && millis() >= lanRetryAt) {
      lanRetryAt = 0;
      IPAddress rip;
      if (rip.fromString(lanRadioIp))
        lanClient.begin(rip, 50001, lanUser.c_str(), lanPass.c_str(), configuredCivAddress);
    }
  }
}

//-------------------------------------------------------------------------------------------------------

void ServiceBackgroundTasks(unsigned long waitMs) {
  unsigned long start = millis();
  while (millis() - start < waitMs) {
    if (trxNetEnabled) net.loop();
    if (APmode) dnsServer.processNextRequest();   // captive portal
    webServer.handleClient();
    #if defined(WDT)
      esp_task_wdt_reset();
    #endif
    delay(10);
  }
}

//-------------------------------------------------------------------------------------------------------
bool WiFiStationReady() {
  if (APmode) return true;
  if (WiFi.status() != WL_CONNECTED) return false;
  return (uint32_t)WiFi.localIP() != 0;
}

//-------------------------------------------------------------------------------------------------------
void WiFiRetryActiveProfile(const char *reason, bool hardReset) {
  if (WifiProfileConfigured(ActiveWifiProfile) == false) {
    ActiveWifiProfile = 0;
  }
  String wifiSsid = WifiProfileSSID(ActiveWifiProfile);
  String wifiPswd = WifiProfilePSWD(ActiveWifiProfile);
  bool targeted = !hardReset && wifiLastBssidValid && wifiSsid == wifiLastSsid;
  Serial.print(millis());
  Serial.print(" WIFI| reconnecting ");
  Serial.print(reason);
  if (hardReset) Serial.print(" [radio reset]");
  else if (targeted) { Serial.print(" [bssid ch"); Serial.print(wifiLastChannel); Serial.print("]"); }
  Serial.print(" status=");
  Serial.print((int)WiFi.status());
  Serial.print(" ip=");
  Serial.print(WiFi.localIP());
  Serial.print(" ssid ");
  Serial.println(wifiSsid);
  if (hardReset) {
    // full radio off/on clears wedged driver state, then full-scan connect
    WiFi.disconnect(true, false);
    delay(200);
    WiFi.mode(WIFI_STA);
    delay(100);
    WiFi.begin(wifiSsid.c_str(), wifiPswd.c_str());
  } else {
    WiFi.disconnect(false, false);
    delay(100);
    if (targeted) {
      WiFi.begin(wifiSsid.c_str(), wifiPswd.c_str(), wifiLastChannel, wifiLastBssid);
    } else {
      WiFi.begin(wifiSsid.c_str(), wifiPswd.c_str());
    }
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
    if (WiFiStationReady()) {
      Serial.println();
      ActiveWifiProfile = profile;
      return true;
    }
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  print_wifi_error();
  return WiFiStationReady();
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
  while (!WiFiStationReady()) {
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

      processCivBuffer(len);
      powerTimer = millis();
    }
  #endif
}

//-------------------------------------------------------------------------------------------------------
// Parse one CI-V frame already in read_buffer (FE FE <to> <from> <cmd> payload FD)
// into the state globals. Transport-agnostic — used by BT (processCatMessages)
// and LAN (lanCivFrameHandler). Caller ensures the frame is from the radio.
void processCivBuffer(uint8_t len) {
  // to-addr: broadcast (00), BT controller (0xE0) or LAN controller (0xE1).
  // LAN poll replies are addressed to 0xE1 — without it, freq/mode from a poll
  // (as opposed to a transceive broadcast) would be dropped.
  if (read_buffer[2] == BROADCAST_ADDRESS || read_buffer[2] == CONTROLLER_ADDRESS
      || read_buffer[2] == 0xE1) {
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
}

//-------------------------------------------------------------------------------------------------------
// LAN transport hook: route a received CI-V frame through the shared parser so
// LAN gets the same full CAT state as BT (freq, mode, S-meter, TX, power, ...).
void lanCivFrameHandler(const uint8_t *frame, size_t len) {
  if (len < 6 || len > sizeof(read_buffer)) return;
  memcpy(read_buffer, frame, len);
  if (radio_address == 0x00) radio_address = frame[3];  // from-addr = the radio
  processCivBuffer((uint8_t)len);
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
    catWriteFrame(req, sizeof(req), true);
  #endif
}
//-------------------------------------------------------------------------------------------------------
bool radioSetFrequency(uint32_t freqHz){
  #if defined(BLUETOOTH)
    bool linkUp = lanMode ? lanClient.connected() : btClientConnected;
    if (!linkUp || radio_address == 0x00) {
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

    catWriteFrame(req, sizeof(req), true);
    return true;
  #endif
  return false;
}
//-------------------------------------------------------------------------------------------------------
void sendCatRequest(uint8_t requestCode){
  #if defined(BLUETOOTH)
    uint8_t req[] = {START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS, requestCode, STOP_BYTE};
    catWriteFrame(req, sizeof(req), true);
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
        frequency += (read_buffer[9 - i] >> 4) * decMulti[i * 2];
        frequency += (read_buffer[9 - i] & 0x0F) * decMulti[i * 2 + 1];
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
//-------------------------------------------------------------------------------------------------------
// TrxNet loop — replaces Mqtt(). Handles net.loop() and WiFi reconnect detection.
// net.begin() is re-called on WiFi reconnect to re-broadcast discovery probe.
void TrxNetLoop(){
  if (APmode || TRXNET_ID == 0x00) return;
  if ((long)(millis() - webQuietUntil) < 0) return;
  static bool prevWifiConnected = WiFiStationReady();
  bool wifiConnected = WiFiStationReady();
  if (wifiConnected && !prevWifiConnected) {
    // WiFi reconnected — re-announce to network
    net.begin(trxDeviceName);
    trxNetEnabled = true;
    Serial.print("TRXNET| reconnect begin ");
    Serial.println(trxDeviceName);
  }
  prevWifiConnected = wifiConnected;
  if (trxNetEnabled && wifiConnected) net.loop();
}

//-------------------------------------------------------------------------------------------------------
// TrxNet callbacks — called from net.loop(), must be short and non-blocking.

// Helper: match sender device name against a configured peer NET_ID.
// Returns peer slot index (0=TRX2, 1=TRX3) or -1 if no match.
static int trxnetPeerSlot(const char* from) {
  char expected[TRXNET_MAX_DEVICE_NAME];
  if (TRX2_NET_ID != 0x00) {
    snprintf(expected, sizeof(expected), "OI3.%02x", TRX2_NET_ID);
    if (strcmp(from, expected) == 0) return 0;
  }
  if (TRX3_NET_ID != 0x00) {
    snprintf(expected, sizeof(expected), "OI3.%02x", TRX3_NET_ID);
    if (strcmp(from, expected) == 0) return 1;
  }
  return -1;
}

// Convert CI-V mode byte to display string for Band Decoder / web UI.
// Returns nullptr for unknown/ignored values.
static const char* trxnetModeToString(uint8_t civMode) {
  switch (civMode) {
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
    default:   return nullptr;
  }
}

// Receive /hz from any peer — update TRX2/TRX3 Band Decoder slot.
void onTrxHz(const char* from, const uint8_t* data, size_t len) {
  if (len < sizeof(uint32_t)) return;
  uint32_t freq;
  memcpy(&freq, data, sizeof(freq));
  int idx = trxnetPeerSlot(from);
  if (idx < 0) return;
  g_trxFreq[idx] = (long)freq;
  g_trxHasData[idx] = true;
  if (Debug) Serial.printf("TRXN| TRX%d freq=%lu\n", idx+2, (unsigned long)freq);
  if (bdEnabled && bdSource == idx + 2) bdUpdate(freq);
}

// Receive /mode from any peer — update TRX2/TRX3 mode string for Band Decoder / web UI.
void onTrxMode(const char* from, const uint8_t* data, size_t len) {
  if (len < sizeof(uint8_t)) return;
  int idx = trxnetPeerSlot(from);
  if (idx < 0) return;
  const char* modeStr = trxnetModeToString(data[0]);
  if (modeStr != nullptr) {
    strlcpy(g_trxMode[idx], modeStr, sizeof(g_trxMode[idx]));
    g_trxHasData[idx] = true;
    if (Debug) Serial.printf("TRXN| TRX%d mode=%s\n", idx+2, modeStr);
  }
}

// Receive /s-hz — remote command to set IC-705 VFO frequency via CI-V.
void onTrxSetHz(const char* from, const uint8_t* data, size_t len) {
  if (len < sizeof(uint32_t)) return;
  uint32_t newFreq;
  memcpy(&newFreq, data, sizeof(newFreq));
  if (newFreq == 0) return;
  trxPendingHz   = newFreq;
  trxFreqPending = true;
}

//-----------------------------------------------------------------------------------
// Serial CI-V driver for TRX2/TRX3 (shares UART0 with CLI/debug, gated by CIVmutePin)
//
// TRX2/3 can be reached either via TrxNet (CONN_TYPE=0) or via CI-V on the serial
// bus (CONN_TYPE=1). When CI-V is selected we poll freq (03) and mode (04) and also
// passively accept Transceive broadcasts; results land in the same g_trxFreq /
// g_trxMode / g_trxHasData slots used by the TrxNet path, so band decoder, web UI
// and the PHP log (port 81) are transport-agnostic.

static const uint32_t CIV_POLL_INTERVAL_MS = 750;  // per full round (TRX2+TRX3, freq+mode)
static const uint32_t CIV_REPLY_TIMEOUT_MS = 250;  // wait for a single reply
static const uint32_t CIV_GAP_MS           = 40;   // bus turnaround between messages
static const uint8_t  CIV_MAX_MISS         = 3;    // consecutive timeouts -> slot stale

enum CivPollState { CIV_IDLE, CIV_WAIT };
static CivPollState civState   = CIV_IDLE;
static uint8_t  civSeq         = 0;        // schedule index 0..3 (see civSeqSlot/Cmd)
static uint32_t civStateT0     = 0;
static uint32_t civNextRun     = 0;
static uint8_t  civMiss[2]     = {0, 0};
static uint8_t  civAwaitSlot   = 0xFF;     // slot we sent the current query to
static uint8_t  civAwaitCmd    = 0;
static bool     civGotReply    = false;

// frame parser
static uint8_t  civRxBuf[20];
static uint8_t  civRxLen       = 0;
static bool     civInFrame     = false;
static uint8_t  civPreCount    = 0;        // consecutive START_BYTE seen

static inline uint8_t civSlotAddr(uint8_t idx) {
  return (idx == 0) ? TRX2_CIV_ADDR : TRX3_CIV_ADDR;
}
static inline bool civSlotEnabled(uint8_t idx) {
  if (idx == 0) return TRX2_CONN_TYPE == 1 && TRX2_CIV_ADDR != 0x00;
  return TRX3_CONN_TYPE == 1 && TRX3_CIV_ADDR != 0x00;
}
static inline bool civAnyEnabled() { return civSlotEnabled(0) || civSlotEnabled(1); }

// schedule mapping: seq 0..3 -> (slot, cmd)
static inline uint8_t civSeqSlot(uint8_t seq) { return seq >> 1; }          // 0,0,1,1
static inline uint8_t civSeqCmd(uint8_t seq)  { return (seq & 1) ? CMD_READ_MODE : CMD_READ_FREQ; }

// Send a CI-V frame: FE FE <toAddr> E0 <body...> FD, gated by MUTE so debug never
// leaks onto the bus. Synchronous and short (a few bytes).
static void civSend(uint8_t toAddr, const uint8_t* body, size_t bodyLen) {
  Serial.flush();                 // drain pending debug bytes before opening the gate
  digitalWrite(CIVmutePin, LOW);
  delay(2);
  Serial.write(START_BYTE);
  Serial.write(START_BYTE);
  Serial.write(toAddr);
  Serial.write(CONTROLLER_ADDRESS);
  for (size_t i = 0; i < bodyLen; i++) Serial.write(body[i]);
  Serial.write(STOP_BYTE);
  Serial.flush();
  delay(2);
  digitalWrite(CIVmutePin, HIGH);
}

static void civQuery(uint8_t slot, uint8_t cmd) {
  uint8_t body = cmd;
  civAwaitSlot = slot;
  civAwaitCmd  = cmd;
  civGotReply  = false;
  civSend(civSlotAddr(slot), &body, 1);
}

// Write frequency (cmd 05) to a CI-V TRX. Frequency is 5 BCD bytes, LSB-first.
void civWriteFreq(uint8_t addr, uint32_t hz) {
  if (addr == 0x00 || hz == 0) return;
  String s = IntToTenString((int)hz);     // 10 digits, MSB-first
  String pairs[5];
  SplitString(s, pairs);                   // pairs[0]=MS .. pairs[4]=LS
  uint8_t body[6];
  body[0] = CMD_WRITE_FREQ;
  for (int i = 0; i < 5; i++) body[1 + i] = stringToByte(pairs[4 - i]);  // LSB-first
  civSend(addr, body, 6);
}

// Decode a completed CI-V frame held in civRxBuf (length civRxLen, last byte = FD).
// Layout after the FE FE preamble: [0]=toAddr [1]=fromAddr [2]=cmd [payload...] [FD].
static void civHandleFrame() {
  if (civRxLen < 4) return;                          // need toAddr,fromAddr,cmd,FD
  uint8_t toAddr   = civRxBuf[0];
  uint8_t fromAddr = civRxBuf[1];
  uint8_t cmd      = civRxBuf[2];
  // Accept only frames addressed to the controller or broadcast; drop our own echo.
  if (toAddr != CONTROLLER_ADDRESS && toAddr != BROADCAST_ADDRESS) return;

  int idx = -1;
  if (civSlotEnabled(0) && fromAddr == TRX2_CIV_ADDR) idx = 0;
  else if (civSlotEnabled(1) && fromAddr == TRX3_CIV_ADDR) idx = 1;
  if (idx < 0) return;

  const uint8_t* pl = civRxBuf + 3;
  size_t plLen = (size_t)civRxLen - 4;              // minus toAddr,fromAddr,cmd,FD

  if ((cmd == CMD_READ_FREQ || cmd == CMD_TRANS_FREQ) && plLen >= 5) {
    uint32_t freq = decodeCivFrequencyBytes(pl, 5);
    g_trxFreq[idx]    = (long)freq;
    g_trxHasData[idx] = true;
    civMiss[idx]      = 0;
    if (bdEnabled && bdSource == idx + 2) bdUpdate(freq);
    if (civAwaitSlot == idx && civAwaitCmd == CMD_READ_FREQ) civGotReply = true;
    if (Debug) Serial.printf("CIV| TRX%d freq=%lu\n", idx + 2, (unsigned long)freq);
  } else if ((cmd == CMD_READ_MODE || cmd == CMD_TRANS_MODE) && plLen >= 1) {
    const char* modeStr = trxnetModeToString(pl[0]);
    if (modeStr != nullptr) {
      strlcpy(g_trxMode[idx], modeStr, sizeof(g_trxMode[idx]));
      g_trxHasData[idx] = true;
      civMiss[idx]      = 0;
      if (civAwaitSlot == idx && civAwaitCmd == CMD_READ_MODE) civGotReply = true;
      if (Debug) Serial.printf("CIV| TRX%d mode=%s\n", idx + 2, modeStr);
    }
  }
}

// Feed one received byte to the CI-V framer. Returns true if the byte was consumed
// as part of a CI-V frame (FE FE .. FD); false means it belongs to the CLI.
bool civParserFeed(uint8_t b) {
  if (!civInFrame) {
    if (b == START_BYTE) {
      if (civPreCount < 2) civPreCount++;
      if (civPreCount == 2) { civInFrame = true; civRxLen = 0; }
      return true;                                  // 0xFE is never a CLI command
    }
    civPreCount = 0;
    return false;                                   // hand to CLI
  }
  // inside a frame
  if (civRxLen < sizeof(civRxBuf)) {
    civRxBuf[civRxLen++] = b;
  } else {
    civInFrame = false; civPreCount = 0; civRxLen = 0;  // overflow, drop
    return true;
  }
  if (b == STOP_BYTE) {
    civHandleFrame();
    civInFrame = false; civPreCount = 0; civRxLen = 0;
  }
  return true;
}

// Non-blocking polling state machine. Sends one query per visit and lets the parser
// (driven by serialPump) collect the reply across subsequent loop iterations.
void civPollTick() {
  if (!civAnyEnabled()) { civState = CIV_IDLE; return; }
  uint32_t now = millis();

  if (civState == CIV_WAIT) {
    if (civGotReply) {
      civState   = CIV_IDLE;
      civNextRun = now + CIV_GAP_MS;
    } else if (now - civStateT0 >= CIV_REPLY_TIMEOUT_MS) {
      uint8_t slot = civAwaitSlot;
      if (slot < 2) {
        if (civMiss[slot] < 255) civMiss[slot]++;
        if (civMiss[slot] >= CIV_MAX_MISS && g_trxHasData[slot]) {
          g_trxHasData[slot] = false;
          g_trxFreq[slot]    = 0;
        }
      }
      civState   = CIV_IDLE;
      civNextRun = now + CIV_GAP_MS;
    }
    return;
  }

  // CIV_IDLE
  if ((int32_t)(now - civNextRun) < 0) return;

  // advance to the next enabled slot in the schedule
  for (uint8_t tries = 0; tries < 4; tries++) {
    uint8_t slot = civSeqSlot(civSeq);
    if (civSlotEnabled(slot)) break;
    civSeq = (civSeq + 1) & 0x03;
  }

  uint8_t slot = civSeqSlot(civSeq);
  uint8_t cmd  = civSeqCmd(civSeq);
  bool roundEnd = (civSeq == 3) || (civSeq == 1 && !civSlotEnabled(1));

  civQuery(slot, cmd);
  civStateT0 = now;
  civState   = CIV_WAIT;

  civSeq = (civSeq + 1) & 0x03;
  if (roundEnd) civNextRun = now + CIV_POLL_INTERVAL_MS;  // pace whole round
}

//-----------------------------------------------------------------------------------

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

    if (Debug) {
      Serial.print("CW ");
      for (uint8_t i = 0; i < frameLen; i++) {
        Serial.print(frame[i], HEX);
        Serial.print(" ");
      }
    }
    catWriteFrame(frame, frameLen, true);
    if (Debug) Serial.println();
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
    abortFskTransmission = false;
    digitalWrite(PTT, HIGH);          // PTT ON
    delay(PTTlead);                   // PTT lead delay
    // ch = ' '; Serial.print(ch); chTable(); sendFsk();   // Space before sending
    // while (Serial.available()) {
    if (Debug) Serial.print("FSK ");
    bool fskAborted = false;
    for (int i = 0; i < TheEnd+1; i++) {
        if (abortFskTransmission) { fskAborted = true; break; }
        ch = toUpperCase(static_cast<char>(CwMsg[i]));
        if (Debug) { Serial.print(String(ch)); Serial.print("|"); }
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
    if (Debug) Serial.println();
    abortFskTransmission = false;
    // ch = ' '; Serial.print(ch); chTable(); sendFsk();   // Space after sending
    if (!fskAborted) delay(PTTtail);
    digitalWrite(PTT, LOW);
    if (Debug) Serial.println();
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
// Handle a single CLI command byte (driven by serialPump). 0xFE/0xFD framing bytes
// never reach here — they are consumed by the CI-V parser first.
void cliHandleByte(uint8_t b){
  {
    incomingByte = b;
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

    // L — LAN CI-V test (step 3): read "IP user pass", probe the radio over UDP
    }else if(incomingByte==76 || incomingByte==108){
      runLanCivTest();

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
// Single RX pump for UART0: route each byte to the CI-V framer (FE FE..FD) or, if it
// is not part of a CI-V frame, to the CLI command handler.
void serialPump(){
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    if (civParserFeed(b)) continue;
    cliHandleByte(b);
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
// Blocking line reader for CLI prompts. Returns trimmed line, "" on timeout.
String EnterLine(const char* prompt){
  Serial.print(prompt);
  String s = "";
  unsigned long t0 = millis();
  while (millis() - t0 < 30000) {
    #if defined(WDT)
      esp_task_wdt_reset();
    #endif
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r' || c == '\n') { s.trim(); if (s.length()) { Serial.println(s); return s; } }
      else { s += c; t0 = millis(); }
    }
    delay(5);
  }
  Serial.println("(timeout)");
  return "";
}

//-------------------------------------------------------------------------------------------------------
// LAN transport config menu (CLI 'L'). Enter "IP user pass" (or empty to keep
// stored values), then choose: (t)est once, (s)ave+reboot into LAN mode, or
// (b)ack to Bluetooth. Persisted in memories.cfg alongside transceiverType.
void runLanCivTest(){
  Serial.println("");
  Serial.print("-- LAN transport -- current mode: ");
  Serial.println(lanMode ? "LAN" : "Bluetooth");
  if (lanRadioIp.length()) {
    Serial.println("   stored: " + lanRadioIp + " user=" + lanUser +
                   " pass=" + String(lanPass.length() ? "(set)" : "(none)"));
  }
  Serial.println("   enter <radioIP> <user> <pass>  (empty = keep stored)");
  String line = EnterLine(" LAN> ");

  String ipStr = lanRadioIp, user = lanUser, pass = lanPass;
  if (line.length() > 0) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);
    if (sp1 < 0 || sp2 < 0) { Serial.println("LAN | need: IP user pass"); return; }
    ipStr = line.substring(0, sp1);
    user  = line.substring(sp1 + 1, sp2);
    pass  = line.substring(sp2 + 1);
  }
  IPAddress rip;
  if (!rip.fromString(ipStr)) { Serial.println("LAN | bad/empty IP"); return; }

  Serial.println("   action: (t)est now  (s)ave + reboot to LAN  (b)ack to Bluetooth");
  EnterChar();
  char act = (char)incomingByte;

  if (act == 't' || act == 'T') {
    if (lanMode) { Serial.println("LAN | already in LAN mode — client is live, watch the log"); return; }
    Serial.println("LAN | testing (BT stays as is)...");
    lanClient.begin(rip, 50001, user.c_str(), pass.c_str(), configuredCivAddress);
    unsigned long t0 = millis();
    while (millis() - t0 < 20000) {
      lanClient.loop();
      #if defined(WDT)
        esp_task_wdt_reset();
      #endif
      if (lanClient.connected() && millis() - t0 > 2000) break;
      if (lanClient.failed()) break;
      delay(5);
    }
    if (lanClient.connected()) {
      char m[sizeof(modes)]; copyModesText(m, sizeof(m));
      Serial.println("LAN | RESULT: SUCCESS, freq " + String((unsigned long)frequency) +
                     " Hz mode " + String(m));
    } else {
      Serial.println("LAN | RESULT: FAILED (state " + String((int)lanClient.status()) + ")");
    }
    lanClient.stop();

  } else if (act == 's' || act == 'S') {
    if (user.length() == 0 || pass.length() == 0) {
      Serial.println("LAN | refusing to save: user/pass empty (type: IP user pass)");
      return;
    }
    lanRadioIp = ipStr; lanUser = user; lanPass = pass;
    transceiverType = "IC-705-LAN";
    saveMemoryConfig();
    Serial.println("LAN | saved (user='" + lanUser + "' passlen=" + String(lanPass.length()) +
                   "). Rebooting into LAN mode (BT off)...");
    delay(1500);
    ESP.restart();

  } else if (act == 'b' || act == 'B') {
    transceiverType = "IC-705-BT";
    saveMemoryConfig();
    Serial.println("LAN | reverting to Bluetooth. Rebooting...");
    delay(1500);
    ESP.restart();

  } else {
    Serial.println("LAN | aborted");
  }
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
    Serial.print("  WIFI-status ");
    Serial.print((int)WiFi.status());
    Serial.print(" | IP ");
    Serial.println(WiFi.localIP());
    Serial.println("  WIFI-MAC "+String(MACString) );
    Serial.println("  WIFI-dBm: "+String(WiFi.RSSI()) );
    Serial.println("----------------------------------------------------------------------------" );
    Serial.println("  For setup OPEN url http://ic705.local or http://"+String(WiFi.localIP()[0])+"."+String(WiFi.localIP()[1])+"."+String(WiFi.localIP()[2])+"."+String(WiFi.localIP()[3]) );
    Serial.println("----------------------------------------------------------------------------" );
    if(TRXNET_ID != 0x00){
      Serial.println("  TrxNet device: "+String(trxDeviceName)+" port:"+String(TRXNET_PORT));
      if(TRX2_NET_ID != 0x00) Serial.println("  TrxNet TRX2 peer: OI3."+String(TRX2_NET_ID, HEX));
      if(TRX3_NET_ID != 0x00) Serial.println("  TrxNet TRX3 peer: OI3."+String(TRX3_NET_ID, HEX));
    }else{
      Serial.println("  TrxNet DISABLE (NET_ID=0x00)" );
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


    // 41 TRXNET_ID (hex string "01".."ff"; 0x00 = disabled)
    { long id = strtol(requestArg("trxnetid").c_str(), nullptr, 16);
      if (id < 0 || id > 255) id = 0x01;
      if (TRXNET_ID != (byte)id) { TRXNET_ID = (byte)id; EEPROM.writeByte(41, (byte)id); } }

    // Per-TRX enable: when the box is unchecked the checkbox is absent, so we
    // zero NET_ID and CI-V address -> the slot is disabled.
    bool trx2en = requestHasArg("trx2enable");
    bool trx3en = requestHasArg("trx3enable");

    // 42 TRX2_NET_ID (hex string; 0x00 = disabled)
    { long id = trx2en ? strtol(requestArg("trx2netid").c_str(), nullptr, 16) : 0;
      if (id < 0 || id > 255) id = 0x00;
      if (TRX2_NET_ID != (byte)id) { TRX2_NET_ID = (byte)id; EEPROM.writeByte(42, (byte)id); } }

    // 43 TRX3_NET_ID (hex string; 0x00 = disabled)
    { long id = trx3en ? strtol(requestArg("trx3netid").c_str(), nullptr, 16) : 0;
      if (id < 0 || id > 255) id = 0x00;
      if (TRX3_NET_ID != (byte)id) { TRX3_NET_ID = (byte)id; EEPROM.writeByte(43, (byte)id); } }

    // 44 TRX2_CONN_TYPE (0=TrxNet, 1=CI-V)
    { int ct = requestArg("trx2conntype").toInt(); if (ct < 0 || ct > 1) ct = 0;
      if (TRX2_CONN_TYPE != (byte)ct) { TRX2_CONN_TYPE = (byte)ct; EEPROM.writeByte(44, (byte)ct); } }
    // 45-46 TRXNET_PORT
    { int p = requestArg("trxnetport").toInt(); if (p >= 1 && p <= 65534 && TRXNET_PORT != (uint16_t)p) { TRXNET_PORT = (uint16_t)p; EEPROM.writeUShort(45, p); } }
    // 47 TRX3_CONN_TYPE (0=TrxNet, 1=CI-V)
    { int ct = requestArg("trx3conntype").toInt(); if (ct < 0 || ct > 1) ct = 0;
      if (TRX3_CONN_TYPE != (byte)ct) { TRX3_CONN_TYPE = (byte)ct; EEPROM.writeByte(47, (byte)ct); } }
    // 48 TRX2_CIV_ADDR (hex string; 0x00 = unset)
    { uint8_t a = TRX2_CIV_ADDR;
      String s = trx2en ? requestArg("trx2civaddr") : String("");
      if (s.length() == 0) { if (TRX2_CIV_ADDR != 0x00) { TRX2_CIV_ADDR = 0x00; EEPROM.writeByte(48, 0x00); } }
      else if (parseHexByteString(s, a)) { if (TRX2_CIV_ADDR != a) { TRX2_CIV_ADDR = a; EEPROM.writeByte(48, a); } } }
    // 49 TRX3_CIV_ADDR (hex string; 0x00 = unset)
    { uint8_t a = TRX3_CIV_ADDR;
      String s = trx3en ? requestArg("trx3civaddr") : String("");
      if (s.length() == 0) { if (TRX3_CIV_ADDR != 0x00) { TRX3_CIV_ADDR = 0x00; EEPROM.writeByte(49, 0x00); } }
      else if (parseHexByteString(s, a)) { if (TRX3_CIV_ADDR != a) { TRX3_CIV_ADDR = a; EEPROM.writeByte(49, a); } } }
    // 48-67 FREE (was MQTT_TOPIC)
    // 115-135 FREE (was MQTT_TOPIC_RX)
    // 225-245 FREE (was TRX2_MQTT_ROOT)
    // 246-266 FREE (was TRX3_MQTT_ROOT)

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

    // TRX1 transport: new setup field trx1transport (lan/bluetooth) is the
    // authority; legacy trxprofile kept as fallback for older clients.
    String trx1transport = trimMemoryValue(requestArg("trx1transport"), 12);
    if (trx1transport == "lan") {
      transceiverType = "IC-705-LAN";
      String ip = trimMemoryValue(requestArg("lanip"), 15);
      String u  = trimMemoryValue(requestArg("lanuser"), 16);
      String pw = trimMemoryValue(requestArg("lanpass"), 16);
      if (ip.length()) lanRadioIp = ip;
      if (u.length())  lanUser = u;
      if (pw.length()) lanPass = pw;   // blank = keep stored password
    } else if (trx1transport == "bluetooth") {
      transceiverType = "IC-705-BT";
    } else {
      String nextTransceiverType = trimMemoryValue(requestArg("trxprofile"), 16);
      if (nextTransceiverType != "IC-7610-CI-V") nextTransceiverType = "IC-705-BT";
      transceiverType = nextTransceiverType;
    }
    lanMode = (transceiverType == "IC-705-LAN");

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

    // TrxNet priority prefixes — takes effect after the restart that follows this save.
    if (requestHasArg("trxnetprio")) {
      TRXNET_PRIO = trxNormalizePrio(requestArg("trxnetprio"));
      eepromWriteTrxPrio(TRXNET_PRIO);
    }

    {
      String trx1Label = trimMemoryValue(requestArg("trx1label"), 10);
      if (trx1Label.length() == 0) trx1Label = "IC-705";
      String trx2Label = trimMemoryValue(requestArg("trx2label"), 10);
      if (trx2Label.length() == 0) trx2Label = "TRX2";
      String trx3Label = trimMemoryValue(requestArg("trx3label"), 10);
      if (trx3Label.length() == 0) trx3Label = "TRX3";
      String rstSsb = trimMemoryValue(requestArg("rstSsb"), 3);
      if (rstSsb.length() == 0) rstSsb = "59";
      String rstCwRtty = trimMemoryValue(requestArg("rstCwRtty"), 3);
      if (rstCwRtty.length() == 0) rstCwRtty = "599";
      bool manualModeForPhone = requestHasArg("manualModeForPhone");
      String blockedDxcc = requestArg("blockedDxcc");

      g_lcTrx1Label = trx1Label;
      g_lcTrx2Label = trx2Label;
      g_lcTrx3Label = trx3Label;
      g_lcRstSsb = rstSsb;
      g_lcRstCwRtty = rstCwRtty;
      g_lcManualModeForPhone = manualModeForPhone;
      g_lcBlockedDxcc = blockedDxcc;

      String nextLogConfig = buildLogConfigJson(
        readLogConfigJson(),
        trx1Label, trx2Label, trx3Label,
        rstSsb, rstCwRtty,
        manualModeForPhone, blockedDxcc
      );
      if (!saveLogConfigJson(nextLogConfig)) {
        ERRdetect = 1;
      }
    }

    // Only overwrite memories when they are actually present in the request
    // (CW/freq memory inputs live outside the EEPROM <form> — absent = empty string from requestArg).
    // The in-RAM memory arrays are loaded at boot, so calling saveMemoryConfig()
    // unconditionally below preserves them while persisting transceiverType/civaddr/LAN.
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
    }
    saveMemoryConfig();   // persists transceiverType, civaddr, LAN ip/user/pass, memories


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
  // resolve once and cache — hostByName blocks for seconds when DNS packets get lost
  if((uint32_t)DxcHostIp == 0 || DxcHostResolved != DxcHost){
    IPAddress ip;
    if(!WiFi.hostByName(DxcHost.c_str(), ip)){
      DxcReconnectTimer = millis() + 5000;
      DxcUpdateTelnetStatus(false);
      return false;
    }
    DxcHostIp = ip;
    DxcHostResolved = DxcHost;
  }
  WiFiClient newClient;
  if(!newClient.connect(DxcHostIp, DxcPort, DXC_CONNECT_TIMEOUT_MS)){
    DxcHostIp = IPAddress();   // stale address? re-resolve on the next attempt
    DxcReconnectTimer = millis() + 5000;
    DxcUpdateTelnetStatus(false);
    return false;
  }
  newClient.setNoDelay(true);
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
