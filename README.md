# Scientisst - Sense Firmware

The firmware for the Scientisst - Sense development board


## Repository structure

```
- main              : Firmware Source Files
- sdkconfig         : ESP32 Hardware Configurations
```
## Dependencies
- The ESP32 programming framework, esp-idf, needs to be installed in order to interface with the Scientisst-Sense Board.
Follow [these installation instructions](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)

## Installing
```sh
# Getting this repository 
git clone https://github.com/scientisst/scientisst-sense-firmware.git
```

## Flash Firmware (Linux/MacOS)
```sh
#Export idf.py
ESP_LOCATION= #Insert here your esp-idf installation location. Example: ESP_LOCATION=~/esp
. $ESP_LOCATION/esp-idf/export.sh

#Flash Firmware
cd scientisst-sense-firmware 
idf.py flash
```


## Serial Monitor (Linux/MacOS)
```sh
#Export idf.py
ESP_LOCATION= #Insert here your esp-idf installation location. Example: ~/esp
. $ESP_LOCATION/esp-idf/export.sh

#Serial monitor
cd scientisst-sense-firmware 
idf.py monitor
```
