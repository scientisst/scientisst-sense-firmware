# Scientisst - Sense Firmware

The firmware for the Scientisst - Sense development board


## Repository structure

```
- deps              :   External Dependencies
- main              :   Firmware Source Files
- sdkconfig         :   ESP32 Hardware Configurations
```
## Installing
```sh
# Getting this repository 
git clone --recursive https://github.com/scientisst/scientisst-sense-firmware.git
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