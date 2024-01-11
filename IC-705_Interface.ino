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
  Using library BluetoothSerial at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/BluetoothSerial 
  Using library WiFi at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/WiFi 
  Using library ESPmDNS at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/ESPmDNS 
  Using library PubSubClient at version 2.8 in folder: /home/dan/Arduino/libraries/PubSubClient 

  Features
  + Connecting the IC-705 via Bluetooth and sending the frequency to MQTT
  + Frequency and mode for PHP log available on http port 81 (address http://ic705.local:81)
  + UDP port 89 receives ascii characters, which it sends via Bluetooth to the IC-705, and transmit them as a CW message
  + UDP port 89 receives ascii characters, which it sends in RTTY mode by keying FSK and PTT TRX inputs
  + Status LED
    - ON after start
    - OFF if cononnect Wifi
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

  ToDo
  - CLI for setting wifi

//--------------------------------------------------------------------
----------------- CONFIGURE ------------------------------------------*/
const String SSID         = "SSID";             // Wifi SSID
const String PSWD         = "PASSWORD";         // Wifi password
const byte mqttBroker[4]  = {54.38.157.134};    // MQTT broker IP address (54.38.157.134 public broker RemoteQTH.com)
const int MQTT_PORT       = 1883;               // MQTT broker port
const String MQTT_TOPIC   = "CALL/IC705/1/hz";  // MQTT topic for send frequency
const int HTTP_CAT_PORT   = 81;                 // http IP port for get frequency and mode
const int udpPort         = 89;                 // UDP port CW | echo -n "cq de ok1hra ok1hra test k;" | nc -u -w1 192.168.1.19 89
const int udpCatPort      = 90;                 // UDP port for CAT command
const int CivOutBaud      = 9600;               // Baudrate terminal and CI-V output (in range 9600 4800 2400 1200)
const char* BTname        = "IC-705-CAT";       //Bluetooth device name
//--------------------------------------------------------------------
//--------------------------------------------------------------------

#define REV 20240111
#define WIFI
#define MQTT
#define UDP_TO_CW
#define UDP_TO_FSK
#define SELECT_ANT  // enable external shift registrer antenna switching (not tested)
#define HTTP        // http propagation freq|mode|
#define WDT         // watchdog timer
#define CIV_OUT     // send freq to CIV out with CivOutBaud
#define UDP_TO_CAT  // data command from udpCatPort send to TRX | FE FE A4 E0 <command> FD

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

const char* mode[] = {"LSB", "USB", "AM", "CW", "FSK", "FM", "WFM"};
#define MODE_TYPE_LSB   0x00
#define MODE_TYPE_USB   0x01
#define MODE_TYPE_AM    0x02
#define MODE_TYPE_CW    0x03
#define MODE_TYPE_RTTY  0x04
#define MODE_TYPE_FM    0x05

String modes;
BluetoothSerial CAT;

short HardwareRev = 99;
const int HWidPin       = 34;  // analog
int HWidValue           = 0;

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
  #define wifi_max_try 1000             // Number of try
  unsigned long WifiTimer = 0;
  unsigned long WifiReconnect = 30000;
  // String SSID="";                // (all) Wifi SSID (max 20 characters)
  // String PSWD="";       // (all) Wifi password (max 20 characters)

  #include <ESPmDNS.h>
#endif

#if defined(HTTP)
  WiFiServer server(HTTP_CAT_PORT);
  char linebuf[80];
  int charcount=0;
#endif

#if defined(UDP_TO_CW)
  #include <WiFiUdp.h>
  WiFiUDP udp;
#endif

#if defined(UDP_TO_CAT)
  WiFiUDP udpCat;
#endif

uint8_t CwMsg[36] = "";
uint8_t CwCat[36] = "";

#if defined(UDP_TO_FSK)
  #define FSK_OUT  33                      // TTL LEVEL pin OUTPUT
  #define PTT      32                      // PTT pin OUTPUT
  #define FMARK    LOW                    // FSK mark level [LOW/HIGH]
  #define FSPACE   HIGH                     // FSK space level [LOW/HIGH]
  #define BaudRate 45.45                   // RTTY baud rate
  #define StopBit  1.5                     // stop bit long
  #define PTTlead  400                     // PTT lead delay ms
  #define PTTtail  200                     // PTT tail delay ms
  int     OneBit = 1/BaudRate*1000;
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
  #include <PubSubClient.h>
  // byte mqttBroker[4]={54,38,157,134}; // MQTT broker IP address
  // byte mqttBroker[4]={192,168,1,200}; // MQTT broker IP address
  // int MQTT_PORT = 1883;         // MQTT broker port
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

//-------------------------------------------------------------------------------------------------------

void setup(){
  Serial.begin(CivOutBaud);
  Serial.println();
  Serial.print("FW rev. ");
  Serial.println(REV);

  pinMode(HWidPin, INPUT);
    // HWidValue = readADC_Cal(analogRead(HWidPin));
    HWidValue = analogRead(HWidPin);
    if(HWidValue<=150){
      HardwareRev=0;
    // }else if(HWidValue>150 && HWidValue<=450){
    //   HardwareRev=3;  // 319
    // }else if(HWidValue>450 && HWidValue<=700){
    //   HardwareRev=4;  // 604
    // }else if(HWidValue>700 && HWidValue<=900){
    //   HardwareRev=5;  // 807
    // }else if(HWidValue>900){
    //   HardwareRev=6;  // 1036
    }
  Serial.print("HW rev. [");
  Serial.print(HWidValue);
  Serial.print("] ");
  Serial.println(HardwareRev);

  pinMode(StatusPin, OUTPUT);
    digitalWrite(StatusPin, LOW);
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
    // WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID.c_str(), PSWD.c_str());
    Serial.print("WIFI-Connecting ssid "+String(SSID)+" ");
    int count_try = 0;
    while(WiFi.status() != WL_CONNECTED){
      delay(500);
      Serial.print(".");
      count_try++;    // Increase try counter
      if ( count_try >= wifi_max_try ) {
        Serial.println("\n");
        Serial.println("Impossible to establish WiFi connexion");
        print_wifi_error();
      }
    }
    Serial.println("");
    Serial.print("WIFI-connected with IP ");
    Serial.println(WiFi.localIP());
    Serial.print("WIFI-dBm: ");
    Serial.println(WiFi.RSSI());
    digitalWrite(StatusPin, HIGH);

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
    Serial.println("mDNS responder started");
  #endif

  #if defined(HTTP)
    server.begin();

    // Add service to MDNS-SD
    MDNS.addService("http", "tcp", 81);
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
    if (MQTT_LOGIN == true){
    // if (mqttClient.connect("esp32gwClient", MQTT_USER, MQTT_PASS)){
      //   AfterMQTTconnect();
      // }
    }else{
        mqttClient.setServer(mqtt_server_ip, MQTT_PORT);
        Serial.println("MQTT-Client");
        mqttClient.setCallback(MqttRx);
        Serial.println("MQTT-Callback");
        lastMqttReconnectAttempt = 0;

        char charbuf[50];
        WiFi.macAddress().toCharArray(charbuf, 17);
          Serial.print("MQTT-maccharbuf ");
          Serial.println(charbuf);
          mqttReconnect();
    }
  #endif

  while (radio_address == 0x00) {
    if (!searchRadio()) {
      #ifdef DEBUG
      Serial.println("Radio not found");
      #endif
    } else {
      #ifdef DEBUG
      Serial.print("Radio found at ");
      Serial.print(radio_address, HEX);
      Serial.println();
      #endif
    }
  }

  #if defined(WDT)
    // WDT
    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
    WdtTimer=millis();
  #endif

}
//-------------------------------------------------------------------------------------------------------

void loop(){
  Watchdog();
  Mqtt();
  httpCAT();
  UdpToCwFsk();
  UdpToCat();
}

// SUBROUTINES -------------------------------------------------------------------------------------------------------

void Watchdog(){

  // set TRX after BT connect
  if(TrxNeedSet==1){
    TrxNeedSet=0;
    /*
    FE FE A4 E0 <command> FD
      - enable CI-V transceive after BT connect, 1A 05 01 31 01
      - enable RIT 21 01 01|clear RIT 21 00 00 00 00 (RIT freq to 0)
      - IP to CW after connect, 16 47 00 (BK-IN OFF), 16 47 01 (BK-IN ON)
    */

    // Enable CI-V transceive
    Serial.print("Enable| CI-V transceive");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x1A;
    CatMsg[1] = 0x05;
    CatMsg[2] = 0x01;
    CatMsg[3] = 0x31;
    CatMsg[4] = 0x01;
    sendCat();
    delay(500);

    // Enable RIT
    Serial.print(" | RIT");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x21;
    CatMsg[1] = 0x01;
    CatMsg[2] = 0x01;
    sendCat();
    delay(500);

    // Enable BK-IN
    Serial.print(" | BK-IN");
    memset(CatMsg, 0xff, 10);
    CatMsg[0] = 0x16;
    CatMsg[1] = 0x47;
    CatMsg[2] = 0x01;
    sendCat();
    delay(500);

    Serial.println();
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
      Serial.println(" PWR OFF");
      statusPower = 0;
    }
  }else{
    if(statusPower==0){
      digitalWrite(PowerOnPin, HIGH);
      Serial.println(" PWR ON");
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
    // Serial.print(frequency);
    // Serial.print("|");
    // Serial.print(modes);
    // Serial.println("|");
    MqttPubString(MQTT_TOPIC, String(frequency), 0);
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
        buffer[i] = stringToByte(SplitStrFreq[4-(i-5)]);
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

      // for (int i = 0; i < 11; i++) {
      //   Serial.print(buffer[i], HEX);
      //   Serial.print(" ");
      // }
      // Serial.println();
      /*
      TXmqtt > OK1HRA/OI3/1/hz 21243820
      FE FE 0 42 0 20 38 24 21 0 FD 
      */
     #endif
  }

  // WIFI status
  #if defined(WIFI)
    unsigned long currentMillis = millis();
    // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
    if ((WiFi.status() != WL_CONNECTED) && (currentMillis - WifiTimer >=WifiReconnect)) {
      Serial.print(millis());
      Serial.println("WiFi - Reconnecting...");
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
    case WL_IDLE_STATUS : Serial.println("WL_IDLE_STATUS"); break;
    case WL_NO_SSID_AVAIL : Serial.println("WL_NO_SSID_AVAIL"); break;
    case WL_CONNECT_FAILED : Serial.println("WL_CONNECT_FAILED"); break;
    case WL_DISCONNECTED : Serial.println("WL_DISCONNECTED"); break;
    default : Serial.printf("No know WiFi error"); break;
  }
}
//-------------------------------------------------------------------------------------------------------
void processCatMessages(){
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
          #ifdef DEBUG
            Serial.print(read_buffer[3]);
            Serial.println(" also on-line?!");
          #endif
        }
      }
      powerTimer=millis(); // RX BT

    }

// #ifdef DEBUG
    //if(!knowncommand){
    // Serial.print("<");
    // if(read_buffer[10] == STOP_BYTE){
    //     memcpy(read_buffer_snapshot, read_buffer, sizeof(read_buffer));
      // for (uint8_t i = 0; i < sizeof(read_buffer); i++){
      //   if (read_buffer[i] < 16)Serial.print("0");
      //   Serial.print(read_buffer[i], HEX);
      //   Serial.print(" ");
      //   if (read_buffer[i] == STOP_BYTE)break;
      // }
      // Serial.println();
    // }
    //}
// #endif
  }

// #ifdef MIRRORCAT
//   while (Serial2.available()) {
//     CAT.print((byte)Serial2.read());
//   }
// #endif
}
//-------------------------------------------------------------------------------------------------------
// call back to get info about connection
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("Client Connected");
    frequencyTmp=0;
    TrxNeedSet=1;
  }

}
//-------------------------------------------------------------------------------------------------------
void configRadioBaud(uint16_t  baudrate){
  if (!CAT.begin(BTname)) //Bluetooth device name
  {
    Serial.println(" BT -An error occurred initializing Bluetooth");

  } else {
    CAT.register_callback(callback);
    Serial.println(" BT -Initialized");
    Serial.println(" BT -The device started, now you can pair it with bluetooth");
  }
}
//-------------------------------------------------------------------------------------------------------
uint8_t readLine(void){
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
}
//-------------------------------------------------------------------------------------------------------
void radioSetMode(uint8_t modeid, uint8_t modewidth){
  uint8_t req[] = {START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS, CMD_WRITE_MODE, modeid, modewidth, STOP_BYTE};
#ifdef DEBUG
  Serial.print(">");
#endif
  for (uint8_t i = 0; i < sizeof(req); i++) {
    CAT.write(req[i]);
#ifdef DEBUG
    if (req[i] < 16)Serial.print("0");
    Serial.print(req[i], HEX);
    Serial.print(" ");
#endif
  }
#ifdef DEBUG
  Serial.println();
#endif
}
//-------------------------------------------------------------------------------------------------------
void sendCatRequest(uint8_t requestCode){
  uint8_t req[] = {START_BYTE, START_BYTE, radio_address, CONTROLLER_ADDRESS, requestCode, STOP_BYTE};
#ifdef DEBUG
  Serial.print(">");
#endif
  for (uint8_t i = 0; i < sizeof(req); i++) {
    CAT.write(req[i]);
#ifdef DEBUG
    if (req[i] < 16)Serial.print("0");
    Serial.print(req[i], HEX);
    Serial.print(" ");
#endif
  }
#ifdef DEBUG
  Serial.println();
#endif
}
//-------------------------------------------------------------------------------------------------------
bool searchRadio(){
  for (uint8_t baud = 0; baud < BAUD_RATES_SIZE; baud++) {
#ifdef DEBUG
    Serial.print("Try baudrate ");
    Serial.println(baudRates[baud]);
#endif
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
}
//-------------------------------------------------------------------------------------------------------
void printFrequency(void){
  frequency = 0;
  //FE FE E0 42 03 <00 00 58 45 01> FD ic-820
  //FE FE 00 40 00 <00 60 06 14> FD ic-732
  for (uint8_t i = 0; i < 5; i++) {
    if (read_buffer[9 - i] == 0xFD)continue; //spike
#ifdef DEBUG
    if (read_buffer[9 - i] < 16)Serial.print("0");
    Serial.print(read_buffer[9 - i], HEX);
#endif

    frequency += (read_buffer[9 - i] >> 4) * decMulti[i * 2];
    frequency += (read_buffer[9 - i] & 0x0F) * decMulti[i * 2 + 1];
  }
#ifdef DEBUG
  Serial.println();
#endif
}
//-------------------------------------------------------------------------------------------------------
void printMode(void){
  //FE FE E0 42 04 <00 01> FD
#ifdef DEBUG
  Serial.println(mode[read_buffer[5]]);
#endif
  modes = mode[read_buffer[5]];
  //read_buffer[6] -> 01 - Wide, 02 - Medium, 03 - Narrow
}
//-------------------------------------------------------------------------------------------------------
void Mqtt(){
  #if defined(MQTT)
    if (millis()-MqttStatusTimer[0]>MqttStatusTimer[1]){
      if(!mqttClient.connected()){
        long now = millis();
        if (now - lastMqttReconnectAttempt > 10000) {
          lastMqttReconnectAttempt = now;
          Serial.print("Attempt to MQTT reconnect | ");
          Serial.println(millis()/1000);
          if (mqttReconnect()) {
            lastMqttReconnectAttempt = 0;
          }
        }
      }else{
        // Client connected
        mqttClient.loop();
      }
      MqttStatusTimer[0]=millis();
    }
  #endif
}

//-------------------------------------------------------------------------------------------------------

#if defined(MQTT)
bool mqttReconnect() {
    char charbuf[50];
    WiFi.macAddress().toCharArray(charbuf, 17);
    if (mqttClient.connect(charbuf)) {
      Serial.println("MQTT-mqttReconnect-connected");

      // String topic = String(ROT_TOPIC) + "mainHWdeviceSelect";
      // topic.reserve(50);
      // const char *cstrr = topic.c_str();
      // if(mqttClient.subscribe(cstrr)==true){
      //   Serial.print("mqttReconnect-subscribe ");
      //   Serial.println(String(cstrr));
      // }

    }
    return mqttClient.connected();
}
#endif
//------------------------------------------------------------------------------------
void MqttRx(char *topic, byte *payload, unsigned int length) {
  #if defined(MQTT)
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

  #endif
} // MqttRx END
//-----------------------------------------------------------------------------------
void MqttPubString(String TOPICEND, String DATA, bool RETAIN){
  #if defined(MQTT)
    digitalWrite(StatusPin, LOW);
    char charbuf[50];
     // memcpy( charbuf, mac, 6);
     WiFi.macAddress().toCharArray(charbuf, 17);
    // if(EnableEthernet==1 && MQTT_ENABLE==1 && EthLinkStatus==1 && mqttClient.connected()==true){
    if(mqttClient.connected()==true){
      if (mqttClient.connect(charbuf)) {
        Serial.print("TXmqtt > ");
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
    digitalWrite(StatusPin, HIGH);
  #endif
}
//-----------------------------------------------------------------------------------

void httpCAT(){
  #if defined(HTTP)
  // listen for incoming clients
  WiFiClient webCatClient = server.available();
  if (webCatClient) {
      // Debugging("WebCat new client");
      memset(linebuf,0,sizeof(linebuf));
      charcount=0;
      // an http request ends with a blank line
      boolean currentLineIsBlank = true;
      while (webCatClient.connected()) {
        if (webCatClient.available()) {
          char c = webCatClient.read();
          // Debugging(c);
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
      // Debugging("WebCat client disconnected");
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
      digitalWrite(StatusPin, LOW);
      // Serial.print("UDP rx: ");
      // Serial.println((char *)CwMsg);
      if(statusPower==1){
        send();
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
void send(){
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
    for (uint8_t i = 0; i < sizeof(CwMsg); i++) {
      if (i <= TheEnd){
        CAT.write(CwMsg[i]);
        // if (CwMsg[i] < 16){
        //   Serial.print("0");
        // }
        // Serial.print(CwMsg[i], HEX);
        // Serial.print(" ");
      }
    }
    // Serial.println();
    delay(100);
    digitalWrite(StatusPin, HIGH);
    delay(100);
    digitalWrite(StatusPin, LOW);
    delay(100);
    digitalWrite(StatusPin, HIGH);
  }else if(modes=="FSK"){ // GPIO -----------------
    digitalWrite(PTT, HIGH);          // PTT ON
    delay(PTTlead);                   // PTT lead delay
    // ch = ' '; Serial.print(ch); chTable(); sendFsk();   // Space before sending
    // while (Serial.available()) {
    for (int i = 0; i < TheEnd+1; i++) {
        ch = toUpperCase(static_cast<char>(CwMsg[i]));
        // Serial.print(String(ch));
        // Serial.print("|");
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
    digitalWrite(StatusPin, HIGH);
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