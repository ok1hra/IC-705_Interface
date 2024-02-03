# IC-705 IP Interface

## Functionality
- **WiFi AP mode with Setup html form** available on address http://ic705.local

    <img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/setup_form.png" height="300">

- Connecting the IC-705 via Bluetooth and **sending the frequency to MQTT**
- **Frequency and mode** for [Óm - simple contest PHP log](https://github.com/ok1hra/Om) available on **http port 81** (address http://ic705.local:81)

    <img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/http-cat.png" height="100">

- **UDP port 89** receives ascii characters, which it sends via Bluetooth to the IC-705, and *transmit them as a CW message**
- **UDP port 89** receives ascii characters, which it sends **in RTTY mode by keying FSK and PTT TRX inputs**

    [![RTTY + PTT keying](https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/hw/rtty-key.png)](https://youtube.com/shorts/b0uTiIwEsbw)

- **Status LED**
    - ON after start
    - OFF if cononnect Wifi
    - FLASH send MQTT freq
    - DOUBLE FLASH receive CW via UDP
    - FLASH+PTT receive RTTY via UDP
- **mDNS** - to easily find IP devices in the network, using the command **"ping ic705.local"**
- **Watchdog -** resets the device after more than 73 seconds of inactivity
- **Output signal POWER-OUT** (13.8V/0.5A) with LED activates after connecting BT (can turn on your hamshack)
- **Galvanically isolated CI-V output** for connecting PA or other devices

    <img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/hw/IC-705-interface-01-CAT.jpg" height="400">

- **CIV-MUTE** on gpio16 allow send to CI-V output only commands with frequency (not debug messages)
- **UDP port for CAT command (clear RIT) from log**
- After BT connect, **set TRX to enable: CI-V transceive + enable RIT + enable BK-IN**
- Support external shift register control switch by frequency (not tested)
- Detect PCB hardware ID

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
1. Select Partition Scheme: 'No OTA (2MB APP/2MB SPIFFS)'
1. **Connect** the ESP32 with a **USB-C** cable and select the corresponding port in the arduino IDE
1. Now you can **compile and upload** code using USB

## Connect and debug
- Open terminal in Arduino IDE
- Set Baudrate to 9600
- Reset ESP32
- Read IP address from terminal debug

## Find IP adress
- Turn on the interface
- Wait for the Status LED to turn off, which signals the Wifi connection
- In the terminal window, use the command ```ping ic705.local``` - the output will show the IP address that the interface received from the DHCP server
