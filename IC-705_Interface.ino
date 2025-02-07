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
byte mqttBroker[4]  = {0,0,0,0};
int MQTT_PORT       = 0;
String MQTT_TOPIC   = "";
int HTTP_CAT_PORT   = 0;
int udpPort         = 0; // UDP port | echo -n "cq de ok1hra ok1hra test k;" | nc -u -w1 192.168.1.19 89
int udpCatPort      = 0;
int BaudRate        = 9600;
// char* BTname        = "";
const char* BTname  = "IC705-interface";
bool Debug          = false;

#define REV 20250207
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
#define EEPROM_SIZE 97
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
  1-21 - SSID
  22-40 - PSWD
  41-44 mqttBroker[4]
  45-46 MQTT_PORT
  47-67 MQTT_TOPIC
  68-69 HTTP_CAT_PORT
  70-71 udpPort
  72-73 udpCatPort
  74-75 BaudRate
  76-96 BTname

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
  uint32_t  readtimeout;      //Serial port read timeout
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
  // #include <ETH.h>
  // int SsidPassSize = (sizeof(SsidPass)/sizeof(char *))/2; //array size
  // int SelectSsidPass = -1;
  #define wifi_max_try 20             // Number of try
  unsigned long WifiTimer = 0;
  unsigned long WifiReconnect = 30000;
  String MACString;

  #include <ESPmDNS.h>

  const char* ssidAP     = "IC705-if-AP";
  const char* passwordAP = "remoteqth";
  bool APmode = false;
  #include <WebServer.h>
  WebServer ajaxserver(80);
#endif
  #if defined(RTLE)
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
char CwMsg[36] = "";
uint8_t CwCat[36] = "";

#if defined(UDP_TO_FSK)
  #define FSK_OUT  33                      // TTL LEVEL pin OUTPUT
  #define PTT      32                      // PTT pin OUTPUT
  #define FMARK    LOW                    // FSK mark level [LOW/HIGH]
  #define FSPACE   HIGH                     // FSK space level [LOW/HIGH]
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
  long MqttStatusTimer[2]{1500,1000};
#endif

int incomingByte = 0;   // for incoming serial data

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

    // 76-96 BTname
    // if(EEPROM.read(76)==0xff){
    //   BTname="IC705-interface";
    // }else{
    //   String str;
    //   for (int i=76; i<97; i++){
    //     if(EEPROM.read(i)!=0xff){
    //       str=str+String(char(EEPROM.read(i)));
    //     }
    //   }
    //   int str_len = str.length();
    //   char char_array[str_len];
    //   str.toCharArray(char_array, str_len+1);
    //   BTname = char_array;
    //   Serial.println(BTname);
    // }
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
    if(APmode==true){
      // WiFi.softAP(ssid, password); remove the password parameter, if you want the AP (Access Point) to be open
      WiFi.softAP(ssidAP, passwordAP);
      IPAddress IP = WiFi.softAPIP();
      Serial.print(" AP | IP address: ");
      Serial.println(IP);

      // serverForm.begin();

      // ajax
      // ajaxserver.on("/",HTTP_POST, handlePostRot);
      ajaxserver.on("/", handleSet);      //This is display page
      // ajaxserver.on("/", handleRoot);      //This is display page
      // ajaxserver.on("/set", handleSet);
      ajaxserver.begin();                  //Start server
      Serial.println("HTTP| ajax server started");

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

      Serial.println("mDNS| responder started");
      APcliAlert();
      Serial.println("SETTINGS  press key to select");
      Serial.println("       ?  list refresh");
      Serial.println("       E  erase whole eeprom and restart");
      Serial.println("       @  restart device");
      Serial.print( " > " );

    }else{
      // WiFi.disconnect(true);
      WiFi.mode(WIFI_STA);
      WiFi.begin(SSID.c_str(), PSWD.c_str());
      Serial.print("WIFI| Connecting ssid "+String(SSID)+" ");
      int count_try = 0;
      while(WiFi.status() != WL_CONNECTED){
        delay(500);
        Serial.print(".");
        count_try++;    // Increase try counter
        if ( count_try >= wifi_max_try ) {
          Serial.println();
          Serial.println("    | Impossible to establish WiFi connexion");
          print_wifi_error();

          EEPROM.writeBool(0, true);
          EEPROM.commit();
          Serial.println("    | Interface will be restarted to AP mode...");
          delay(3000);
          ESP.restart();
        }
      }
      Serial.println("");
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

      // ajax
      // ajaxserver.on("/",HTTP_POST, handlePostRot);
      ajaxserver.on("/", handleSet);      //This is display page
      ajaxserver.begin();                  //Start server
      Serial.println("HTTP| ajax server started");

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
            Serial.println("MQTT| Callback");
            lastMqttReconnectAttempt = 0;

            char charbuf[50];
            WiFi.macAddress().toCharArray(charbuf, 17);
              Serial.print("MQTT| maccharbuf ");
              Serial.println(charbuf);
              mqttReconnect();
        }
      }
    #endif

    #if defined(BLUETOOTH)
      while (radio_address == 0x00) {
        if (!searchRadio()) {
          if(Debug==true){
            Serial.println("Radio not found");
          }
        } else {
          if(Debug==true){
            Serial.print("Radio found at ");
            Serial.print(radio_address, HEX);
            Serial.println();
          }
        }
      }
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
    ajaxserver.handleClient();
    #if defined(RTLE)
      rtleserver.handleClient();
    #endif
    CLI();

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
    ajaxserver.handleClient();
    #if defined(RTLE)
      rtleserver.handleClient();
    #endif
  }
}

// SUBROUTINES -------------------------------------------------------------------------------------------------------

void Watchdog(){

  // set TRX after BT connect
  static bool onlyOne = false;
  if(TrxNeedSet==1 && onlyOne == false){
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
    delay(500);

    // Enable RIT
    Serial.print("|RIT-ON");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x21;
    CatMsg[1] = 0x01;
    CatMsg[2] = 0x01;
    sendCat();
    delay(500);

    // Disable BK-IN
    Serial.print("|BK-IN-OFF");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x16;
    CatMsg[1] = 0x47;
    CatMsg[2] = 0x00;
    sendCat();
    delay(500);

    // Set mode CW
    Serial.print("|MODE-CW");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x06;
    CatMsg[1] = 0x03;
    CatMsg[2] = 0x03;
    sendCat();
    delay(500);

    // Set CW 28 WPM - 0000=6 WPM ~ 0255=48 WPM (6 per one wpm)
    Serial.print("|CW-WPM-28");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x14;
    CatMsg[1] = 0x0C;
    CatMsg[2] = 0x01;
    CatMsg[3] = 0x38;
    sendCat();
    delay(500);

    // Set AFgain to 10
    Serial.print("|AFgain-10");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x14;
    CatMsg[1] = 0x01;
    CatMsg[2] = 0x00;
    CatMsg[3] = 0x25;
    sendCat();
    delay(500);

    // Set RFgain to 0
    Serial.print("|RFgain-0");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x14;
    CatMsg[1] = 0x02;
    CatMsg[2] = 0x00;
    CatMsg[3] = 0x00;
    sendCat();
    delay(500);

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
      delay(2000);
    #else
      delay(11000);
    #endif

    // Enable BK-IN
    Serial.print("|BK-IN-ON");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x16;
    CatMsg[1] = 0x47;
    CatMsg[2] = 0x01;
    sendCat();
    delay(500);

    // Set RFgain to 100
    Serial.print("|RFgain-100");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x14;
    CatMsg[1] = 0x02;
    CatMsg[2] = 0x02;
    CatMsg[3] = 0x55;
    sendCat();
    delay(500);

    Serial.println();
    onlyOne=true;
  }

  // BT CAT
  static long requestTimer = 0;
  if(millis()-requestTimer > 1000){
    // Serial.println("req "+String(millis()/1000));
    sendCatRequest(CMD_READ_FREQ);
    sendCatRequest(CMD_READ_MODE);
    requestTimer=millis();
  }
  processCatMessages();

  // Power OUT/LED
  if(millis()-powerTimer > 3000){
    if(statusPower==1){
      digitalWrite(PowerOnPin, LOW);
      Serial.println(" PWR| OFF");
      statusPower = 0;
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
  if(millis()-mqttPostponeTimer > 11000 && mqttPostponeStatus==0){
    mqttPostponeStatus = 1;
    MqttPubString(MQTT_TOPIC, String(frequency), 0);
  }

  static long catTimer = 0;
  if(millis()-catTimer > 2000 && frequencyTmp!=frequency){
  // MQTT
    if(mqttEnable==true){
      // Serial.println("MQTT>"+String(frequency)+"Hz "+String(modes) );
      MqttPubString(MQTT_TOPIC, String(frequency), 0);
    }
    frequencyTmp=frequency;
    catTimer=millis();
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
    if ((WiFi.status() != WL_CONNECTED) && (currentMillis - WifiTimer >=WifiReconnect)) {
      Serial.print(millis());
      Serial.println("cReconnecting...");
      WiFi.disconnect();
      WiFi.reconnect();
      WifiTimer = currentMillis;
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
void SelectANT(){
  #if defined(SELECT_ANT)
    TRXselectANT = 42;
    for(int ant=0; ant<8; ant++){
      if(ANTrange[ant][0]<frequency && frequency<ANTrange[ant][1]){
        TRXselectANT=ant;
      }else{
      }
    }
    ShiftOutByte=0x00;
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
void processCatMessages(){
  #if defined(BLUETOOTH)
    /*
      <FE FE E0 42 04 00 01 FD  - LSB
      <FE FE E0 42 03 00 00 58 45 01 FD  -145.580.000

      FE FE - start bytes
      00/E0 - target address (broadcast/controller)
      42 - source address
      00/03 - data type
      <data>
      FD - stop byte
    */

    while (CAT.available()) {
      uint8_t knowncommand = 1;
      uint8_t r;
      if (readLine() > 0) {
        if (read_buffer[0] == START_BYTE && read_buffer[1] == START_BYTE) {
          if (read_buffer[3] == radio_address) {
            if (read_buffer[2] == BROADCAST_ADDRESS) {
              switch (read_buffer[4]) {
                case CMD_TRANS_FREQ:
                  printFrequency();
                  break;
                case CMD_TRANS_MODE:
                  printMode();
                  break;
                default:
                  knowncommand = false;
              }
            } else if (read_buffer[2] == CONTROLLER_ADDRESS) {
              switch (read_buffer[4]) {
                case CMD_READ_FREQ:
                  printFrequency();
                  break;
                case CMD_READ_MODE:
                  printMode();
                  break;
                default:
                  knowncommand = false;
              }
            }
          } else {
            // if(Debug==true){
            //   Serial.print(read_buffer[3]);
            //   Serial.println(" also on-line?!");
            // }
          }
        }
        powerTimer=millis(); // RX BT

      }

      // if(Debug==true){
      //   if(!knowncommand){
      //   Serial.print("<");
      //     if(read_buffer[10] == STOP_BYTE){
      //       memcpy(read_buffer_snapshot, read_buffer, sizeof(read_buffer));
      //       for (uint8_t i = 0; i < sizeof(read_buffer); i++){
      //         if (read_buffer[i] < 16)Serial.print("0");
      //         Serial.print(read_buffer[i], HEX);
      //         Serial.print(" ");
      //         if (read_buffer[i] == STOP_BYTE)break;
      //       }
      //       Serial.println();
      //     }
      //   }
      // }
    }

  // #ifdef MIRRORCAT
  //   while (Serial2.available()) {
  //     CAT.print((byte)Serial2.read());
  //   }
  // #endif
  #endif
}
//-------------------------------------------------------------------------------------------------------
// call back to get info about connection
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  #if defined(BLUETOOTH)
    if (event == ESP_SPP_SRV_OPEN_EVT) {
      Serial.println("    | Client Connected");
      Serial.println("    | CHANGE frequency on IC-705, for initialize CAT");
      Serial.println("    | -----------------------------------------------------------------------------");
      frequencyTmp=0;
      TrxNeedSet=1;
    }
  #endif
}
//-------------------------------------------------------------------------------------------------------
void configRadioBaud(uint16_t  baudrate){
  #if defined(BLUETOOTH)
    if (!CAT.begin(BTname)) //Bluetooth device name
    {
      Serial.println(" BT | An error occurred initializing Bluetooth");

    } else {
      CAT.register_callback(callback);
      Serial.println(" BT | Initialized");
      Serial.println("    | -----------------------------------------------------------------------------");
      Serial.println(" BT | The device started, now you MUST PAIR it with Bluetooth name "+String(BTname));
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
      CAT.write(req[i]);
      if(Debug==true){
        if (req[i] < 16)Serial.print("0");
        Serial.print(req[i], HEX);
        Serial.print(" ");
      }
    }
    if(Debug==true){
      Serial.println();
    }
  #endif
}
//-------------------------------------------------------------------------------------------------------
void sendCatRequest(uint8_t requestCode){
  #if defined(BLUETOOTH)
      uint8_t req[] = {START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS, requestCode, STOP_BYTE};
    if(Debug==true){
      Serial.print("CAT TX >");
    }
      for (uint8_t i = 0; i < sizeof(req); i++) {
        CAT.write(req[i]);
        if(Debug==true){
          if (req[i] < 16)Serial.print("0");
          Serial.print(req[i], HEX);
          Serial.print(" ");
        }
      }
      if(Debug==true){
        Serial.println();
      }
  #endif
}
//-------------------------------------------------------------------------------------------------------
bool searchRadio(){
  #if defined(BLUETOOTH)
    for (uint8_t baud = 0; baud < BAUD_RATES_SIZE; baud++) {
      if(Debug==true){
        Serial.print("Try baudrate ");
        Serial.println(baudRates[baud]);
      }
        configRadioBaud(baudRates[baud]);
        sendCatRequest(CMD_READ_FREQ);

        if (readLine() > 0)
        {
          if (read_buffer[0] == START_BYTE && read_buffer[1] == START_BYTE) {
            radio_address = read_buffer[3];
          }
          return true;
        }
    }

      radio_address = 0xFF;
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
void printMode(void){
  #if defined(BLUETOOTH)
    //FE FE E0 42 04 <00 01> FD
    if(Debug==true){
      Serial.println(mode[read_buffer[5]]);
    }
    modes = mode[read_buffer[5]];
    //read_buffer[6] -> 01 - Wide, 02 - Medium, 03 - Narrow
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
        char charbuf[50];
        WiFi.macAddress().toCharArray(charbuf, 17);
        if (mqttClient.connect(charbuf)) {
          Serial.println("MQTT| mqttReconnect-connected");

          // String topic = String(ROT_TOPIC) + "mainHWdeviceSelect";
          // topic.reserve(50);
          // const char *cstrr = topic.c_str();
          // if(mqttClient.subscribe(cstrr)==true){
          //   Serial.print("mqttReconnect-subscribe ");
          //   Serial.println(String(cstrr));
          // }
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
      String CheckTopicBase;
      CheckTopicBase.reserve(100);
      byte* p = (byte*)malloc(length);
      memcpy(p,payload,length);
      // static bool HeardBeatStatus;
      Serial.print("RXmqtt < ");

        // CheckTopicBase = String(ROT_TOPIC) + "AzimuthStop";
        // if ( CheckTopicBase.equals( String(topic) )){
        //   Azimuth=(int)payloadToFloat(payload, length);
        //   Serial.println(String(Azimuth)+"°");
        //   RxMqttTimer=millis();
        //   if( (AzimuthTmp!=Azimuth && abs(Azimuth-AzimuthTmp)>3) || eInkOfflineDetect==true){
        //     eInkNeedRefresh=true;
        //     AzimuthTmp=Azimuth;
        //   }
        //   WDTimer();
        // }
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
      char charbuf[50];
      // memcpy( charbuf, mac, 6);
      WiFi.macAddress().toCharArray(charbuf, 17);
      // if(EnableEthernet==1 && MQTT_ENABLE==1 && EthLinkStatus==1 && mqttClient.connected()==true){
      if(mqttClient.connected()==true){
        if (mqttClient.connect(charbuf)) {
          Serial.print("MQTT| ");
          String topic = String(TOPICEND);
          topic.toCharArray( mqttPath, 50 );
          DATA.toCharArray( mqttTX, 50 );
          mqttClient.publish(mqttPath, mqttTX, RETAIN);
          Serial.print(mqttPath);
          Serial.print(" ");
          Serial.println(mqttTX);
        }
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
      while (webCatClient.connected()) {
        if (webCatClient.available()) {
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
    memset(CwMsg, 0, 36);

    //processing incoming packet, must be called before reading the buffer
    udp.parsePacket();
    //receive response from server, it will be HELLO WORLD
    if(udp.read(CwMsg, 36) > 0){
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

    int TheEnd = 99;
    for (int i = 9; i>-1; i--) {
      if(CatMsg[i]!=0xff){
          CatMsg[i+4]=CatMsg[i];
        if(TheEnd==99){
          TheEnd=i+5;
        }
      }
    }

    // Serial.print("TheEnd: ");
    // Serial.println(TheEnd);
    // Serial.print("To CAT: ");
    // Serial.println((char *)CwMsg);

    // CAT -----------------
    CatMsg[0] = START_BYTE;
    CatMsg[1] = START_BYTE;
    CatMsg[2] = radio_address;
    CatMsg[3] = CONTROLLER_ADDRESS;

    CatMsg[TheEnd] = STOP_BYTE;
    for (uint8_t i = 0; i < sizeof(CatMsg); i++) {
      if (i <= TheEnd){
        CAT.write(CatMsg[i]);

        // if (CatMsg[i] < 16){
        //   Serial.print("0");
        // }
        // Serial.print(CatMsg[i], HEX);
        // Serial.print(" ");
      }
    }
    // Serial.println();  
  #endif
}
//-------------------------------------------------------------------------------------------------------
void sendCW(){
  int TheEnd = 99;
  for (int i = 30; i>-1; i--) {
    if(CwMsg[i]!=0){
      if(modes=="CW"){
        CwMsg[i+5]=CwMsg[i];
      }
      if(TheEnd==99){
        if(modes=="CW"){
          TheEnd=i+6;
        }else{
          TheEnd=i;
        }
      }
    }
  }
  // Serial.print("TX cat: ");
  // Serial.println((char *)CwMsg);

  if(modes=="CW"){  // CAT -----------------
    CwMsg[0] = START_BYTE;
    CwMsg[1] = START_BYTE;
    CwMsg[2] = radio_address;
    CwMsg[3] = CONTROLLER_ADDRESS;
    CwMsg[4] = CMD_SEND_CW_MSG;
    CwMsg[TheEnd] = STOP_BYTE;
    Serial.print("CW ");
    for (uint8_t i = 0; i < sizeof(CwMsg); i++) {
      if (i <= TheEnd){
        CAT.write(CwMsg[i]);
        // if (CwMsg[i] < 16){
        //   Serial.print("0");
        // }
        Serial.print(CwMsg[i], HEX);
        Serial.print(" ");
      }
    }
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
        digitalWrite(FSK_OUT, FSPACE); delay(OneBit);
        //--bit1
        if(d1 == 1){digitalWrite(FSK_OUT, FMARK); }
        else       {digitalWrite(FSK_OUT, FSPACE); } delay(OneBit);
        //--bit2
        if(d2 == 1){digitalWrite(FSK_OUT, FMARK); }
        else       {digitalWrite(FSK_OUT, FSPACE); } delay(OneBit);
        //--bit3
        if(d3 == 1){digitalWrite(FSK_OUT, FMARK); }
        else       {digitalWrite(FSK_OUT, FSPACE); } delay(OneBit);
        //--bit4
        if(d4 == 1){digitalWrite(FSK_OUT, FMARK); }
        else       {digitalWrite(FSK_OUT, FSPACE); } delay(OneBit);
        //--bit5
        if(d5 == 1){digitalWrite(FSK_OUT, FMARK); }
        else       {digitalWrite(FSK_OUT, FSPACE); } delay(OneBit);
        //--stop bit
        digitalWrite(FSK_OUT, FMARK); delay(OneBit*StopBit);
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
      Serial.println("  IP MqttTopic: "+String(MQTT_TOPIC));
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
  Serial.println("---------------------------------------------------------------");
  Serial.println();
}

//-------------------------------------------------------------------------------------------------------

// ajax rx
void handleSet() {

  String ssidERR="red;'>";
  String pswdERR="red;'>";
  String baudSELECT0= "";
  String baudSELECT1= "";
  String baudSELECT2= "";
  String baudSELECT3= "";
  String baudSELECT4= "";
  String mqttERR= "";
  String mqttERR1= "";
  String mqttERR2= "";
  String mqttERR3= "";
  String mqttERR4= "";
  String mqttportERR= "";
  String mqtttopicERR= "";
  String httpcatportERR= "";
  String udpportERR= "";
  String udpcatportERR= "";
  String btnameERR= "";
  bool ERRdetect=0;

  if(mqttEnable==false){
    mqttERR= "<span style='color:#0c0;'> MQTT disabled</span>";
  }

  if ( ajaxserver.hasArg("ssid") == false \
    && ajaxserver.hasArg("pswd") == false \
  ) {
    // Serial.println("Form NOT valid");
  }else{
    // Serial.println("Form VALID");

    // 1-21 - SSID*
    if ( ajaxserver.arg("ssid").length()<1 || ajaxserver.arg("ssid").length()>20){
      ssidERR= "red;'> Out of range 1-20 characters";
      ERRdetect=1;
    }else{
      String str = String(ajaxserver.arg("ssid"));
      if(SSID == str){
        ssidERR="red;'>";
      }else{
        ssidERR="orange;'> Warning: SSID has changed.";
        SSID = String(ajaxserver.arg("ssid"));

        int str_len = str.length();
        char char_array[str_len];
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

    // 22-40 - PSWD*
    if ( ajaxserver.arg("pswd").length()<1 || ajaxserver.arg("pswd").length()>18){
      pswdERR= "red;'> Out of range 1-18 characters";
      ERRdetect=1;
    }else{
      String str = String(ajaxserver.arg("pswd"));
      if(PSWD == str){
        pswdERR="red;'>";
      }else{
        pswdERR="orange;'> Warning: PSWD has changed.";
        PSWD = String(ajaxserver.arg("pswd"));

        int str_len = str.length();
        char char_array[str_len];
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

    // 76-96 BTname
    // if ( ajaxserver.arg("btname").length()<1 || ajaxserver.arg("btname").length()>20){
    //   btnameERR= " Out of range 1-20 characters";
    //   ERRdetect=1;
    // }else{
    //   String str = String(ajaxserver.arg("btname"));
    //   if(String(BTname) == str){
    //     btnameERR="";
    //   }else{
    //     btnameERR="";

    //     int str_len = str.length();
    //     char char_array[str_len];
    //     str.toCharArray(char_array, str_len+1);

    //     BTname = char_array;
    //     // str.toCharArray(BTname, str_len+1);
    //     for (int i=0; i<20; i++){
    //       if(i < str_len){
    //         EEPROM.write(76+i, char_array[i]);
    //       }else{
    //         EEPROM.write(76+i, 0xff);
    //       }
    //     }
    //   }
    // }

    // 68-69 HTTP_CAT_PORT
    if ( ajaxserver.arg("httpcatport").length()<1 || ajaxserver.arg("httpcatport").toInt()<1 || ajaxserver.arg("httpcatport").toInt()>65534){
      httpcatportERR= " Out of range number 1-65534";
      ERRdetect=1;
    }else{
      if(HTTP_CAT_PORT == ajaxserver.arg("httpcatport").toInt()){
        httpcatportERR="";
      }else{
        httpcatportERR="";
        HTTP_CAT_PORT = ajaxserver.arg("httpcatport").toInt();
        EEPROM.writeUShort(68, HTTP_CAT_PORT);
      }
    }

    // 70-71 udpPort
    if ( ajaxserver.arg("udpport").length()<1 || ajaxserver.arg("udpport").toInt()<1 || ajaxserver.arg("udpport").toInt()>65534){
      udpportERR= " Out of range number 1-65534";
      ERRdetect=1;
    }else{
      if(udpPort == ajaxserver.arg("udpport").toInt()){
        udpportERR="";
      }else{
        udpportERR="";
        udpPort = ajaxserver.arg("udpport").toInt();
        EEPROM.writeUShort(70, udpPort);
      }
    }

    // 72-73 udpCatPort
    if ( ajaxserver.arg("udpcatport").length()<1 || ajaxserver.arg("udpcatport").toInt()<1 || ajaxserver.arg("udpcatport").toInt()>65534){
      udpcatportERR= " Out of range number 1-65534";
      ERRdetect=1;
    }else{
      if(41 == ajaxserver.arg("udpcatport").toInt()){
        udpcatportERR="";
      }else{
        udpcatportERR="";
        udpCatPort = ajaxserver.arg("udpcatport").toInt();
        EEPROM.writeUShort(72, udpCatPort);
      }
    }

    // 41-44 mqttBroker[4]
    if ( ajaxserver.arg("mqttip0").length()<1 || ajaxserver.arg("mqttip0").toInt()>255){
      mqttERR1= " IP1: Out of range 0-255";
      ERRdetect=1;
    }else{
      if(mqttBroker[0] == byte(ajaxserver.arg("mqttip0").toInt()) ){
        mqttERR1="";
      }else{
        mqttERR1="";
        mqttBroker[0] = byte(ajaxserver.arg("mqttip0").toInt()) ;
        EEPROM.writeByte(41, mqttBroker[0]);
      }
      if(mqttBroker[0]==0x00){
        mqttEnable = false;
      }else{
        mqttEnable = true;
      }
    }

    if ( ajaxserver.arg("mqttip1").length()<1 || ajaxserver.arg("mqttip1").toInt()>255){
      mqttERR2= " IP2: Out of range 0-255";
      ERRdetect=1;
    }else{
      if(mqttBroker[1] == byte(ajaxserver.arg("mqttip1").toInt()) ){
        mqttERR2="";
      }else{
        mqttERR2="";
        mqttBroker[1] = byte(ajaxserver.arg("mqttip1").toInt()) ;
        EEPROM.writeByte(42, mqttBroker[1]);
      }
    }

    if ( ajaxserver.arg("mqttip2").length()<1 || ajaxserver.arg("mqttip2").toInt()>255){
      mqttERR3= " IP3: Out of range 0-255";
      ERRdetect=1;
    }else{
      if(mqttBroker[2] == byte(ajaxserver.arg("mqttip2").toInt()) ){
        mqttERR3="";
      }else{
        mqttERR3="";
        mqttBroker[2] = byte(ajaxserver.arg("mqttip2").toInt()) ;
        EEPROM.writeByte(43, mqttBroker[2]);
      }
    }

    if ( ajaxserver.arg("mqttip3").length()<1 || ajaxserver.arg("mqttip3").toInt()>255){
      mqttERR4= " IP4: Out of range 0-255";
      ERRdetect=1;
    }else{
      if(mqttBroker[3] == byte(ajaxserver.arg("mqttip3").toInt()) ){
        mqttERR4="";
      }else{
        mqttERR4="";
        mqttBroker[3] = byte(ajaxserver.arg("mqttip3").toInt()) ;
        EEPROM.writeByte(44, mqttBroker[3]);
      }
    }

    // 45-46 MQTT_PORT
    if ( ajaxserver.arg("mqttport").length()<1 || ajaxserver.arg("mqttport").toInt()<1 || ajaxserver.arg("mqttport").toInt()>65534){
      mqttportERR= " Out of range number 1-65534";
      ERRdetect=1;
    }else{
      if(MQTT_PORT == ajaxserver.arg("mqttport").toInt()){
        mqttportERR="";
      }else{
        mqttportERR="";
        MQTT_PORT = ajaxserver.arg("mqttport").toInt();
        EEPROM.writeUShort(45, MQTT_PORT);
      }
    }

    // 47-67 MQTT_TOPIC
    if ( ajaxserver.arg("mqtttopic").length()<1 || ajaxserver.arg("mqtttopic").length()>20){
      mqtttopicERR= " Out of range 1-20 characters";
      ERRdetect=1;
    }else{
      String str = String(ajaxserver.arg("mqtttopic"));
      if(MQTT_TOPIC == str){
        mqtttopicERR="";
      }else{
        mqtttopicERR="";
        MQTT_TOPIC = String(ajaxserver.arg("mqtttopic"));

        int str_len = str.length();
        char char_array[str_len];
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

    // 74-75 BaudRate *
    static int BaudRateTmp=115200;
    switch (ajaxserver.arg("baud").toInt()) {
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


    // // APmode
    EEPROM.writeBool(0, false);
    EEPROM.commit();
    Serial.println("Interface will be restarted...");
    delay(3000);
    ESP.restart();
    // Serial.println("ERRdetect = "+String(ERRdetect) );

  } // else form valid

  baudSELECT0= "";
  baudSELECT1= "";
  baudSELECT2= "";
  baudSELECT3= "";
  baudSELECT4= "";
  switch (BaudRate) {
    case 1200: {baudSELECT0= " selected"; break; }
    case 2400: {baudSELECT1= " selected"; break; }
    case 4800: {baudSELECT2= " selected"; break; }
    case 9600: {baudSELECT3= " selected"; break; }
    case 115200: {baudSELECT4= " selected"; break; }
  }
  String MQTTstyle;
  if(mqttEnable==true){
    MQTTstyle="tdr";
  }else{
    MQTTstyle="tdg";
  }

  String HtmlSrc = "<!DOCTYPE html><html><head><title>SETUP IC705 IP interface</title>\n";
  HtmlSrc +="<meta http-equiv='Content-Type' content='text/html; charset=UTF-8'>\n";
  // <meta http-equiv = 'refresh' content = '600; url = /'>\n";
  HtmlSrc +="<style type='text/css'> button#go {background-color: #ccc; padding: 5px 20px 5px 20px; border: none; -webkit-border-radius: 5px; -moz-border-radius: 5px; border-radius: 5px;} button#go:hover {background-color: orange;} ";
  HtmlSrc +=" table, th, td {color: #fff; border-collapse: collapse; border:0px } .tdg {color: #ccc; height: 40px; text-align: right; vertical-align: middle; padding-right: 15px} .tdr {color: #0c0; height: 40px; text-align: right; vertical-align: middle; padding-right: 15px} html,body {background-color: #333; text-color: #ccc; font-family: sans-serif,Arial,Tahoma,Verdana;} a:hover {color: #fff;} a { color: #ccc; text-decoration: underline; } ";
  HtmlSrc +=".b {border-top: 1px dotted #666;} .tooltip-text {visibility: hidden; position: absolute; z-index: 1; width: 300px; color: white; font-size: 12px; background-color: #DE3163; border-radius: 10px; padding: 10px 15px 10px 15px; } .hover-text:hover .tooltip-text { visibility: visible; } #right { top: -30px; left: 200%; } #top { top: -60px; left: -150%; } #left { top: -8px; right: 120%;} ";
  HtmlSrc +=".hover-text {position: relative; background: #888; padding: 5px 12px; margin: 5px; font-size: 15px; border-radius: 100%; color: #FFF; display: inline-block; text-align: center; }</style>\n";
  HtmlSrc +="</head><body>\n";
  HtmlSrc +="<H1 style='color: #666; text-align: center;'>Setup IC-705 IP interface</H1>";

  HtmlSrc +="<div style='display: flex; justify-content: center;'><table><form action='/' method='post' style='color: #ccc; margin: 50 0 0 0; text-align: center;'>\n";
  HtmlSrc +="<tr class='b'><td class='tdr'></td><td><span style='color: #666;'>";
  if(APmode==true){
    HtmlSrc +="AP mode <span style='color: #0c0; '>ON</span>";
  }else{
    HtmlSrc +="AP mode OFF";
  }
  HtmlSrc +="|MAC ";
  HtmlSrc +=MACString;
  HtmlSrc +="|FW ";
  HtmlSrc +=REV;
  HtmlSrc +="|HW ";
  HtmlSrc +=String(HardwareRev);
  HtmlSrc +="</span><span style='color: #333;'>";
  HtmlSrc +=String(HWidValue);
  HtmlSrc +="</span></td></tr>\n";

  HtmlSrc +="<tr class='b'><td class='tdr'><label for='ssid'>Conect to WiFi SSID:</label></td><td><input type='text' id='ssid' name='ssid' size='20' value='";
  HtmlSrc += SSID;
  HtmlSrc +="'><span style='color:";
  HtmlSrc += ssidERR;
  HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 200px;'>After reboot, IC705 interface<br>connect to this WiFi SSID</span></td></tr>\n";
  // HtmlSrc +="<tr><td class='tdr'><label for='pswd'>WiFi Password:</label></td><td><input type='text' id='pswd' name='pswd' size='18' value='";
  HtmlSrc +="<tr><td class='tdr'><label for='pswd'>WiFi Password:</label></td><td><input type='password' id='pswd' name='pswd' size='18' value='";
  HtmlSrc += PSWD;
  HtmlSrc +="'><span style='color:";
  HtmlSrc += pswdERR;
  HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 200px;'>WiFi acces password</span></td></tr>\n";

  // HtmlSrc +="<tr class='b'><td class='tdr'><label for='btname'>Bluetooth name:</label></td><td><input type='text' id='btname' name='btname' size='20' value='";
  // HtmlSrc += BTname;
  // HtmlSrc +="'><span style='color:red;'>";
  // HtmlSrc += btnameERR;
  // HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 200px;'>For pairing bluetooth</span></td></tr>\n";

  HtmlSrc +="<tr class='b'><td class='tdr'><label for='httpcatport'>http CAT ip port:</label></td><td>";
  HtmlSrc +="<input type='text' id='httpcatport' name='httpcatport' size='4' value='" + String(HTTP_CAT_PORT) + "'>\n";
  HtmlSrc +="<span style='color:red;'>";
  HtmlSrc += httpcatportERR;
  HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 150px;'>Default http port 81</span></span></td></tr>\n";
  HtmlSrc +="<tr><td class='tdr'><label for='udpport'>CW/FSK udp port:</label></td><td>";
  HtmlSrc +="<input type='text' id='udpport' name='udpport' size='4' value='" + String(udpPort) + "'>\n";
  HtmlSrc +="<span style='color:red;'>";
  HtmlSrc += udpportERR;
  HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 150px;'>Default udp port 89</span></span></td></tr>\n";
  HtmlSrc +="<tr><td class='tdr'><label for='udpcatport'>CAT udp port:</label></td><td>";
  HtmlSrc +="<input type='text' id='udpcatport' name='udpcatport' size='4' value='" + String(udpCatPort) + "'>\n";
  HtmlSrc +="<span style='color:red;'>";
  HtmlSrc += udpcatportERR;
  HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 250px;'>Default udp port 90<br><hr>Send only the data part<br>of the CI-V packet to the UDP port,<br>Interface will add addresses,<br>start and stop bytes</span></span></td></tr>\n";

  HtmlSrc +="<tr class='b'><td class='"+String(MQTTstyle)+"'><label for='mqttip0'>MQTT broker IP:</label></td><td>";
  HtmlSrc +="<input type='text' id='mqttip0' name='mqttip0' size='1' value='" + String(mqttBroker[0]) + "'>&nbsp;.&nbsp;<input type='text' id='mqttip1' name='mqttip1' size='1' value='" + String(mqttBroker[1]) + "'>&nbsp;.&nbsp;<input type='text' id='mqttip2' name='mqttip2' size='1' value='" + String(mqttBroker[2]) + "'>&nbsp;.&nbsp;<input type='text' id='mqttip3' name='mqttip3' size='1' value='" + String(mqttBroker[3]) + "'>";
  HtmlSrc += mqttERR;
  HtmlSrc +="<span style='color:red;'>";
  HtmlSrc += mqttERR1;
  HtmlSrc += mqttERR2;
  HtmlSrc += mqttERR3;
  HtmlSrc += mqttERR4;
  HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 250px;'>Default public broker 54.38.157.134<br>If the first digit is zero, MQTT is disabled</span></span></td></tr>\n";
  HtmlSrc +="<tr><td class='"+String(MQTTstyle)+"'><label for='mqttport'>MQTT broker PORT:</label></td><td>";
  HtmlSrc +="<input type='text' id='mqttport' name='mqttport' size='4' value='" + String(MQTT_PORT) + "'>\n";
  HtmlSrc +="<span style='color:red;'>";
  HtmlSrc += mqttportERR;
  HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 150px;'>Default public broker port 1883</span></span></td></tr>\n";
  HtmlSrc +="<tr><td class='"+String(MQTTstyle)+"'><label for='mqtttopic'>MQTT topic:</label></td><td><input type='text' id='mqtttopic' name='mqtttopic' size='20' value='";
  HtmlSrc += MQTT_TOPIC;
  HtmlSrc +="'><span style='color:red;'>";
  HtmlSrc += mqtttopicERR;
  HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 200px;'>Topic path, for example:<br>CALL/IC705/1/hz</span></td></tr>\n";

  HtmlSrc +="<tr class='b'><td class='tdr'><label for='baud'>USB serial BAUDRATE:</label></td><td><select name='baud' id='baud'><option value='0'";
  HtmlSrc += baudSELECT0;
  HtmlSrc +=">1200</option><option value='1'";
  HtmlSrc += baudSELECT1;
  HtmlSrc +=">2400</option><option value='2'";
  HtmlSrc += baudSELECT2;
  HtmlSrc +=">4800</option><option value='3'";
  HtmlSrc += baudSELECT3;
  HtmlSrc +=">9600</option><option value='4'";
  HtmlSrc += baudSELECT4;
  HtmlSrc +=">115200</option></select><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 150px;'>For CI-V output, use<br>speed 9600 or lower</span></span></td></tr>\n";

  #if defined(RESET_AFTER_DISCONNECT)
    HtmlSrc +="<tr class='b'><td class='tdr'>RESET after TRX disconnect:</td><td>ENABLE <span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 150px;'>Compile-defined functionality<br>see <a href='https://github.com/ok1hra/IC-705_Interface/blob/main/IC-705_Interface.ino#L85' target='_blank'>GitHub &#10138;</a></span></span></td></tr>\n";
  #endif

  HtmlSrc +="<tr class='b'><td class='tdr'></td><td><button id='go'>&#10004; Save and Restart</button> - Be patient, saving is slow ;)</form>";
  HtmlSrc +="</td></tr>\n";

  // HtmlSrc +="<tr><td class='tdr'></td><td style='height: 42px;'></td></tr>\n";
  // HtmlSrc +="<tr><td class='tdr'></td><td style='height: 42px;'></td></tr>";
  // HtmlSrc +="<tr><td class='tdr'><a href='/'><button id='go'>&#8617; Back to Control</button></a></td><td class='tdl'><a href='/cal' onclick=\"window.open( this.href, this.href, 'width=700,height=715,left=0,top=0,menubar=no,location=no,status=no' ); return false;\"><button id='go'>Calibrate &#8618;</button></a></td></tr>";
  HtmlSrc +="<tr><td class='tdr'></td><td class='tdl'><span style='color: #666;'>After the reboot, the wifi switches to client mode<br>and if it fails to connect, it returns to AP mode.<br>Debug is available in the serial console on USB-C.</span><br><a href='https://github.com/ok1hra/IC-705_Interface' target='_blank'>More on GitHub &#10138;</a></td></tr></table>\n";
  HtmlSrc +="</body></html>\n";

  ajaxserver.send(200, "text/html", HtmlSrc); //Send web page
}

#if defined(RTLE)
  void handleRTLE() {
    String HtmlSrc = "";
    String s = RTLE_page; //Read HTML contents
    HtmlSrc +=s;
    rtleserver.send(200, "text/html", HtmlSrc); //Send web page
  }
#endif