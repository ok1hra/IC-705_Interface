# IC-705_Interface
Functionality
- Connect to IC-705 via **Bluetooth**
- Connect to **WiFi**
- Connect to **MQTT** broker
- Send **frequency to MQTT**
- Show CAT **frequency and mode on http** port 81
- Transfer message **from open UDP port 89 to CW** (via CAT)

# Configure
- In head of .ino file
  - Variables
  - You can disable non use functionality with add // before #define ..

# Compile and upload
1.  **Install [Arduino IDE](https://www.arduino.cc/en/software)** rev 1.8.19
1.  **Install support [for ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)**
1.  **Install** these **libraries** in the versions listed
  * BluetoothSerial rev 2.0.0
  * WiFi rev 2.0.0
  * PubSubClient rev 2.8
1. **Select board** 'ESP32 Dev Module'
1. **Connect** the ESP32 with a **USB** cable and select the corresponding port in the arduino IDE
1. Now you can **compile and upload** code using USB

# Connect and debug
- Open terminal in Arduino IDE
- Set Baudrate to 115200
- Reset ESP32
- Read IP address from terminal debug
