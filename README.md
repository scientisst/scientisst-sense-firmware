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
cd scientisst-sense-firmware

#Export idf.py
ESP_LOCATION=./deps  #Your esp-idf installation location
. $ESP_LOCATION/esp-idf/export.sh

#Flash Firmware. Sense must be in flash mode: press MODE (without releasing), press RESET (without releasing), release RESET, release MODE.
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

## Disclaimer
This is not a medical device certified for diagnosis or treatment. It is provided to you as is only for research and educational purposes.

## Acknowledgments
This work was partially supported by Fundação para a Ciência e Tecnologia (FCT) under the projects’ UIDB/50008/2020 and PCIF/SSO/0163/2019 (SafeFire), and by the Active Assisted Living Programme Collaborative Project AAL-2020-7-237-CP (FORTO 2.0) through IT—Instituto de Telecomunicações, which is gratefully acknowledged.