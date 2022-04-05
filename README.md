# Scientisst - Sense Firmware

The firmware for the Scientisst - Sense development board

---

## Repository structure

```
- deps              :   External Dependencies
- main              :   Firmware Source Files
- sdkconfig         :   ESP32 Hardware Configurations
- www               :   Configuration HTML Page
``` 

## Installing
### 1) Prerequisites
Install the ESP-IDF prerequisites following espressif's guide https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/#step-1-install-prerequisites



### 2) Getting this repository 

```sh
# Getting this repository 
git clone --recursive https://github.com/scientisst/scientisst-sense-firmware.git
```
### 3) Install Xtensa's toolchain

#### Linux/MacOS:
```
./deps/esp-idf/install.sh esp32
```

#### Windows:
```
deps/esp-idf/install.bat esp32
```

## Flash Firmware (Linux/MacOS)
```sh
#Export idf.py
ESP_LOCATION=./deps  #Your esp-idf installation location
. $ESP_LOCATION/esp-idf/export.sh

#Flash Firmware
cd scientisst-sense-firmware 
idf.py flash
```

## Serial Monitor (Linux/MacOS)
```sh
#Export idf.py
ESP_LOCATION=./deps  #Your esp-idf installation location
. $ESP_LOCATION/esp-idf/export.sh

#Serial monitor
cd scientisst-sense-firmware 
idf.py monitor
```
If you're having trouble with the default command, try specifying the USB port:

```
idf.py monitor -p '/dev/ttyUSB0'
```
To exit the serial monitor, press `CTRL+T` followed by `CTRL+X`
