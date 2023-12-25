# IC-705 IP Interface

## Functionality
- Support [Óm - simple contest PHP log](https://github.com/ok1hra/Om)
- Connect to IC-705 via **Bluetooth**
- Connect to **WiFi** AP
- Connect to **MQTT** broker and send **frequency**
- CAT **frequency and mode available on http** port 81

  <img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/http-cat.png" height="100">
  
- Transfer message **from UDP port 89 to CW/FSK**
- **CI-V isolated output** for PA or other devices

  <img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/hw/IC-705-interface-01-CAT.jpg" height="400">

## PCB
<img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/hw/IC-705-interface-01.png" height="300"><img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/hw/IC-705-interface-01-PCB.jpg" height="300">

## 3D prit
<img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/3Dprint/preview.png" height="200">

## Block diagram
<img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/hw/hw-block.png" height="250">

<img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/hw/sw-block.png">


## Configure
- In head of .ino file
  - Variables
  - You can disable non use functionality with add // before #define ..

## Compile and upload
1.  **Install [Arduino IDE](https://www.arduino.cc/en/software)** rev 1.8.19
1.  **Install support [for ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)**
1.  **Install** these **libraries** in the versions listed
  * BluetoothSerial rev 2.0.0
  * WiFi rev 2.0.0
  * PubSubClient rev 2.8
1. **Select board** 'ESP32 Dev Module'
1. **Connect** the ESP32 with a **USB** cable and select the corresponding port in the arduino IDE
1. Now you can **compile and upload** code using USB

## Connect and debug
- Open terminal in Arduino IDE
- Set Baudrate to 9600
- Reset ESP32
- Read IP address from terminal debug
