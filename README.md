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

## scientisst-firmware.py usage

```
ScientISST Firmware Tool

optional arguments:
  -h, --help            show this help message and exit
  -i, --install         Install esp-idf tools in order to build firmware
  -b, --build           Build firmware
  -c, --clean           Clean build outputs
  -f, --flash           Flash firmware
  -v, --version         Show firmware version
  -m [MONITOR], --monitor [MONITOR]
                        Open serial monitor in <MONITOR> port. Example: scientisst-firmware.py --monitor /dev/ttyUSB0
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

```
python scientisst-firmware.py --install
```

## Flash Firmware (Linux/MacOS)
```sh
#Flash Firmware. Sense must be in flash mode: press MODE (without releasing), press RESET (without releasing), release RESET, release MODE.
python scientisst-firmware.py --flash
```

### Troubleshooting
####  Cannot open /dev/ttyUSB0: Permission denied
This is not an issue with the ScientISST SENSE, and its not actually a bug at all, its just part of Linux. Its best practice to not change permissions in /dev unless as a last resort. What you want to do instead is to add yourself to the group which would give you permission to access the tty ports.

To see the groups you are in simply type:

```
groups
```

To see all available groups type:

```
compgen -g
```

Most of them are self-explanatory, in this case you want to add yourself to either the tty group, or dialout, which you would do by:

```
sudo usermod -a -G tty yourname
```

Then your user should have access to tty without use of sudo.

## Serial Monitor (Linux/MacOS)
```sh
python scientisst-firmware.py --monitor
```
To exit the serial monitor, press `CTRL+T` followed by `CTRL+X`

## Disclaimer
This is not a medical device certified for diagnosis or treatment. It is provided to you as is only for research and educational purposes.

## Acknowledgments
This work was partially supported by Fundação para a Ciência e Tecnologia (FCT) under the projects’ UIDB/50008/2020 and PCIF/SSO/0163/2019 (SafeFire), and by the Active Assisted Living Programme Collaborative Project AAL-2020-7-237-CP (FORTO 2.0) through IT—Instituto de Telecomunicações, which is gratefully acknowledged.