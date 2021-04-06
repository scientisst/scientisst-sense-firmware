# Scientisst - Sense Firmware

The firmware for the Scientisst - Sense development board


## Repository structure

```
- main
  - main.cpp        : A test example source file that uses the scientisst class to perform a live mode acquisition
  - scientisst.cpp  : The scientisst class source file
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
cd scientisst-sense-firmware 
idf.py flash
```


## Serial Monitor (Linux/MacOS)
```sh
cd scientisst-sense-firmware 
idf.py monitor
```
