# ESP32 QRPlog for IC-705

## Key features
- running on ESP32
- connected to IC-705 via bluetooth (no wire)
- QRPlog web logbook
- logbook database stored directly in web browser
- works decentralized/offline, without internet usable on portable
- logbook import/export function
- network synchronization between web browsers in local network - same database on multiple devices (phone, tablet, pc)
- integrated DX cluster client
- web CAT interface
- option to backup settings or entire logbook database

<img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/docs/CAT.png" height="700">

<img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/docs/LOG.png" height="400"><img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/docs/DXC.png" height="400">

<img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/docs/SETUP.png" height="400"><img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/docs/LOGSYNC.png" height="400">

## Hardware required
- ICOM IC-705
- any ESP32 or RemoteQTH interface for extended functions
- web client (phone/tablet/pc)

## Firmware web installer
- First plug USB-C between ESP32 and PC
- Then open the Firmware page and follow the instructions https://ok1hra.github.io/IC-705_Interface/

## Quick start guide
- upload firmware
- connect PC to WIFI SSID ```IC705-if``` access point
- [Find IP address](#find-ip-address)
- open IP address in web browser
- select ```SETUP``` page and set Wii SSID and password
- after reboot device connect to your Wifi access point
- [Find IP address](#find-ip-address) again or open http://ic705.local in web browser
- in IC-705 select ```MENU / SET / Bluetooth Set / Pairing/Connect / Device Search``` and select ```IC705-interface```

## Find IP address

### via Arduino IDE
- Open terminal in Arduino IDE
- Set Baudrate to 9600
- Press ? and Enter
- Read IP address from terminal debug
<img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/docs/cli.png" height="500">

### via Terminal
- Turn on the interface
- Wait for the Status LED to turn off, which signals the Wifi connection
- In the terminal window, use the command ```ping ic705.local``` - the output will show the IP address that the interface received from the DHCP server

## WEB app user Manual
- [docs/user-manual.md](docs/user-manual.md) — setup, web UI, CW/RTTY, MQTT, troubleshooting
<img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/hw/sw-block.png">

## Hardware
- **Output signal POWER-OUT** (13.8V/0.5A) with LED activates after connecting BT (can turn on your hamshack)
- **Galvanically isolated CI-V output** for connecting PA or other devices
- Power consumption < 1W
- RTTY operation

[![RTTY + PTT keying](https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/hw/rtty-key.png)](https://youtube.com/shorts/b0uTiIwEsbw)

### Status LED
- Fade in/out - WiFi in AP mode
- WiFi in client mode
    - ON waiting connected to WiFi
    - OFF Wifi connected to AP
    - FLASH send MQTT freq
    - DOUBLE FLASH receive CW via UDP
    - FLASH+PTT receive RTTY via UDP

### Connection
<img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/hw/hw-block.png" height="250">

### Connectors
- 13,8V DC jack
- KEY stereo jack
- SEND/ALC stereo jack
- USB-C
- ACC RJ45

### PCB
- [Schematic rev3 PDF](https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/hw/IC-705-interface-03.pdf)
- [BOM rev3 html](https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/hw/IC-705-interface-ibom-03.html)

### 3D prit case
<img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/3Dprint/preview.png" height="200"><img src="https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/3Dprint/preview-mountpoint.png" height="200">

- [Source rev3 OpenScad](https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/3Dprint/ic-705-interface-3.scad)
- [rev3 STL](https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/3Dprint/ic-705-interface-3.stl)
- [rev3 3MF](https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/3Dprint/ic-705-interface-3.3mf)
- [With mountpoint rev3 STL](https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/3Dprint/ic-705-interface-3-mountpoint.stl)
- [With mountpoint rev3 3MF](https://raw.githubusercontent.com/ok1hra/IC-705_Interface/main/3Dprint/ic-705-interface-3-mountpoint.3mf)
