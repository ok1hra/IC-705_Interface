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

  Použití knihovny BluetoothSerial ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/BluetoothSerial
  Použití knihovny WiFi ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/WiFi
  Použití knihovny PubSubClient ve verzi 2.8 v adresáři: /home/dan/Arduino/libraries/PubSubClient

  + BT CAT to MQTT via WiFi
  + http CAT for óm
  + UDP to BT-CAT CW

  ToDo
  - CI-V output to serial2 for PA
  - UDP RTTY + PTT to GPIO
  - status/live LED
  - ON/OFF gpio output if cat data available (trx off)
  - GPIO out by band or BCD
  - clear RIT, 21 00  00 00 00 (RIT freq to 0) new CMD UDP port?
  - IP to CW after connect, 16 47 00 (BK-IN OFF), 16 47 01 (BK-IN ON)
  - mDNS
  - Watchdog
//--------------------------------------------------------------------
----------------- CONFIGURE ------------------------------------------*/
const String SSID         = "";                // Wifi SSID
const String PSWD         = "";       // Wifi password
const byte mqttBroker[4]  = {192,168,1,200};    // MQTT broker IP address (54.38.157.134 public broker RemoteQTH.com)
const int MQTT_PORT       = 1883;               // MQTT broker port
const String MQTT_TOPIC   = "OK1HRA/OI3/1/hz";  // MQTT topic for send frequency
const int HTTP_CAT_PORT   = 81;                 // http IP port for send frequency and mode
const int udpPort         = 89;                 // UDP port | echo -n "cq de ok1hra ok1hra test k;" | nc -u -w1 192.168.1.19 89
const int CivOutBaud      = 9600;               // Baudrate optional CI-V serial output
//--------------------------------------------------------------------
//--------------------------------------------------------------------

#define REV 20231029
#define WIFI
#define MQTT
#define HTTP
#define UDP_TO_CW
// #define CIV_OUT //     Serial2.write(byte);

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
#endif

#if defined(HTTP)
  WiFiServer server(HTTP_CAT_PORT);
  char linebuf[80];
  int charcount=0;
#endif

#if defined(UDP_TO_CW)
  #include <WiFiUdp.h>
  WiFiUDP udp;
  uint8_t CwMsg[36] = "";
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
  Serial.begin(115200);
  Serial.println();
  Serial.print("FW rev. ");
  Serial.println(REV);

  #if defined(CIV_OUT)
    Serial2.begin(CivOutBaud, SERIAL_8N1, 26, 0);
  #endif

  #if defined(WIFI)
    // WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID.c_str(), PSWD.c_str());
    Serial.print("WIFI-Connecting ssid "+String(SSID)+" ");
    int count_try = 0;
    while(WiFi.status() != WL_CONNECTED) {
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
  #endif

  #if defined(HTTP)
    server.begin();
  #endif

  #if defined(UDP_TO_CW)
    udp.begin(udpPort);
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
        if (mqttClient.connect(charbuf)){
          Serial.print("MQTT-maccharbuf ");
          Serial.println(charbuf);
          mqttReconnect();
        }
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

}
//-------------------------------------------------------------------------------------------------------

void loop(){
  double qrg = 0;

  static long requestTimer = 0;
  if(millis()-requestTimer > 1000){
    // Serial.println("req "+String(millis()/1000));
    sendCatRequest(CMD_READ_FREQ);
    sendCatRequest(CMD_READ_MODE);
    requestTimer=millis();
  }
  processCatMessages();

  static long catTimer = 0;
  if(millis()-catTimer > 2000 && frequencyTmp!=frequency){
    // Serial.print(frequency);
    // Serial.print("|");
    // Serial.print(modes);
    // Serial.println("|");
    MqttPubString(MQTT_TOPIC, String(frequency), 0);
    frequencyTmp=frequency;
    catTimer=millis();
  }

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

  Mqtt();
  httpCAT();
  UdpToCwFsk();

}
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
    }

#ifdef DEBUG
    //if(!knowncommand){
    Serial.print("<");
    for (uint8_t i = 0; i < sizeof(read_buffer); i++) {
      if (read_buffer[i] < 16)Serial.print("0");
      Serial.print(read_buffer[i], HEX);
      Serial.print(" ");
      if (read_buffer[i] == STOP_BYTE)break;
    }
    Serial.println();
    //}
#endif
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
  }

}
//-------------------------------------------------------------------------------------------------------
void configRadioBaud(uint16_t  baudrate){
  if (!CAT.begin("IC-705-CAT")) //Bluetooth device name
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

            webCatClient.print(String(frequency)+"|"+String(modes)+"|");
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
      Serial.print("UDP rx: ");
      Serial.println((char *)CwMsg);
      send();
    }

  #endif
}
//-------------------------------------------------------------------------------------------------------
void send(){
  int TheEnd = 99;
  for (int i = 30; i>-1; i--) {
    if(CwMsg[i]!=0){
      CwMsg[i+5]=CwMsg[i];
      if(TheEnd==99){
        TheEnd=i+6;
      }
    }
  }
  CwMsg[0] = START_BYTE;
  CwMsg[1] = START_BYTE;
  CwMsg[2] = radio_address;
  CwMsg[3] = CONTROLLER_ADDRESS;
  CwMsg[4] = CMD_SEND_CW_MSG;
  CwMsg[TheEnd] = STOP_BYTE;

  // Serial.print("TX cat: ");
  // Serial.println((char *)CwMsg);

  if(modes=="CW"){  // CAT
    for (uint8_t i = 0; i < sizeof(CwMsg); i++) {
      if (i <= TheEnd){
        CAT.write(CwMsg[i]);
        if (CwMsg[i] < 16){
          Serial.print("0");
        }
        Serial.print(CwMsg[i], HEX);
        Serial.print(" ");
      }
    }
    Serial.println();
  }else if(modes=="FSK"){ // GPIO
    Serial.println("FSK not implemented yet ;)");
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
